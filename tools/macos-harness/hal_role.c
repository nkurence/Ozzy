/* SPDX-License-Identifier: MIT */
#include "hal_role.h"
#include "../../common/devices/ploytec/ploytec_protocol.h"
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define WAV_FIFO_BYTES (64u << 20)   /* 2 channels at 96 kHz: about two minutes; 16: about 14 s */

static void *wav_writer(void *arg)
{
	struct wav *w = arg;
	/* test: OZZY_WAV_STALL_MS=n blocks the writer once for n ms after its first 4 MiB, a disk that
	 * stalls; the reader must lose nothing while the FIFO has room */
	const char *stall_env = getenv("OZZY_WAV_STALL_MS");
	unsigned int stall = stall_env ? (unsigned int)atoi(stall_env) : 0;
	for (;;) {
		if (stall && atomic_load_explicit(&w->tail, memory_order_relaxed) > (4u << 20)) {
			fprintf(stderr, "WAV writer: stalling %u ms (OZZY_WAV_STALL_MS)\n", stall);
			usleep(stall * 1000);
			stall = 0;
		}
		uint64_t tail = atomic_load_explicit(&w->tail, memory_order_relaxed);
		uint64_t head = atomic_load_explicit(&w->head, memory_order_acquire);
		if (head == tail) {
			/* closing is set after the last frame went in: read head again once it is */
			if (atomic_load(&w->closing) && atomic_load(&w->head) == tail)
				break;
			usleep(10000);
			continue;
		}
		uint64_t off = tail % WAV_FIFO_BYTES, n = head - tail;
		if (n > WAV_FIFO_BYTES - off)
			n = WAV_FIFO_BYTES - off;
		w->bytes += (uint32_t)fwrite(w->fifo + off, 1, (size_t)n, w->f);
		atomic_store_explicit(&w->tail, tail + n, memory_order_release);
	}
	return NULL;
}

bool wav_open(struct wav *w, const char *name, uint32_t rate, uint32_t channels)
{
	w->fifo = malloc(WAV_FIFO_BYTES);
	if (!w->fifo)
		return false;
	memset(w->fifo, 0, WAV_FIFO_BYTES);   /* its pages in now, not as faults on the real-time reader */
	w->f = fopen(name, "wb");
	if (!w->f) {
		free(w->fifo);
		w->fifo = NULL;
		return false;
	}
	w->channels = channels;
	uint8_t h[44] = { 'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E', 'f', 'm', 't', ' ', 16, 0, 0, 0, 1, 0 };
	uint32_t block_align = channels * 3, byte_rate = rate * block_align;
	h[22] = (uint8_t)channels;
	memcpy(h + 24, &rate, 4);
	memcpy(h + 28, &byte_rate, 4);
	h[32] = (uint8_t)block_align;
	h[34] = 24;
	memcpy(h + 36, "data", 4);
	fwrite(h, 1, sizeof(h), w->f);
	if (pthread_create(&w->writer, NULL, wav_writer, w) != 0) {
		fclose(w->f);
		w->f = NULL;
		free(w->fifo);
		w->fifo = NULL;
		return false;
	}
	return true;
}

void wav_write(struct wav *w, const uint8_t *s24, uint32_t frames)
{
	uint64_t n = (uint64_t)frames * w->channels * 3;
	uint64_t head = atomic_load_explicit(&w->head, memory_order_relaxed);
	uint64_t tail = atomic_load_explicit(&w->tail, memory_order_acquire);
	if (head - tail + n > WAV_FIFO_BYTES) {
		w->dropped += n;
		return;
	}
	for (uint64_t i = 0; i < n;) {
		uint64_t off = (head + i) % WAV_FIFO_BYTES, c = n - i;
		if (c > WAV_FIFO_BYTES - off)
			c = WAV_FIFO_BYTES - off;
		memcpy(w->fifo + off, s24 + i, (size_t)c);
		i += c;
	}
	atomic_store_explicit(&w->head, head + n, memory_order_release);
}

void wav_close(struct wav *w)
{
	if (!w->f)
		return;
	atomic_store(&w->closing, true);
	pthread_join(w->writer, NULL);
	if (w->dropped)
		fprintf(stderr, "frames the WAV writer had no room for: %llu\n",
			(unsigned long long)(w->dropped / (w->channels * 3)));
	free(w->fifo);
	w->fifo = NULL;
	uint32_t riff = 36 + w->bytes;
	fseek(w->f, 4, SEEK_SET);
	fwrite(&riff, 4, 1, w->f);
	fseek(w->f, 40, SEEK_SET);
	fwrite(&w->bytes, 4, 1, w->f);
	fclose(w->f);
	w->f = NULL;
}

/* Frame @f of the output ring: sub-packets of 10 frames end to end in the device's layout */
static uint8_t *out_frame(struct ozzy_shared_memory *shm, enum ploytec_out_layout layout, uint32_t f)
{
	return (uint8_t *)shm->audio.output_buffer + (f / PLOYTEC_FRAMES_PER_OUT_SUBPKT) * ploytec_out_subpkt_size(layout) +
	       ploytec_out_frame_offset(layout, f % PLOYTEC_FRAMES_PER_OUT_SUBPKT);
}

double fill_tone(struct ozzy_shared_memory *shm, uint32_t channels, uint32_t rate)
{
	const uint32_t ring_frames = shm->audio.output_ring_frames;
	ploytec_frame_f32_enc_fn encode = ploytec_frame_encoder_f32(channels);
	if (!encode || !ring_frames)
		return 0;
	enum ploytec_out_layout layout = (shm->audio.device_flags & OZZY_DEVICE_FLAG_PLOYTEC_BULK) ? PLOYTEC_OUT_LAYOUT_BULK
												: PLOYTEC_OUT_LAYOUT_INT;
	const uint32_t cycles = 48;
	float frame[16] = { 0 };
	for (uint32_t f = 0; f < ring_frames; f++) {
		float v = 0.125f * (float)sin(2.0 * 3.14159265358979 * cycles * f / ring_frames);
		frame[0] = frame[1] = v;
		encode(out_frame(shm, layout, f), frame);
	}
	return (double)cycles * rate / ring_frames;
}

/* The IN transfers run alongside the OUT ones; the ring: one PLOYTEC_IN_FRAME_SIZE frame after
 * another */
void input_reader_run(struct input_reader *r, struct ozzy_shared_memory *shm, struct wav *w, uint64_t sample_time)
{
	const uint32_t in_frames = shm->audio.input_ring_frames ? shm->audio.input_ring_frames : 1;
	const uint32_t behind = shm->audio.input_safety_offset_frames;
	uint64_t end = sample_time > behind ? sample_time - behind : 0;
	if (r->resync) {   /* take up from here (after a restart the gap shows in the WAV) */
		r->next_frame = end;
		r->resync = false;
	}
	if (end > r->next_frame + in_frames / 2) {
		r->lost += end - in_frames / 2 - r->next_frame;
		r->next_frame = end - in_frames / 2;
	}
	uint8_t s24[16 * 3];
	for (; r->next_frame < end; r->next_frame++) {
		uint32_t f = (uint32_t)(r->next_frame % in_frames);
		r->decode(s24, (const uint8_t *)shm->audio.input_buffer + f * PLOYTEC_IN_FRAME_SIZE);
		wav_write(w, s24 + (r->ch_first - 1) * 3, 1);   /* the channels are contiguous */
	}
}

void input_reader_make_realtime(void)
{
	mach_timebase_info_data_t tb;
	mach_timebase_info(&tb);
	const uint64_t period_ns = 20000000, compute_ns = 2000000;
	thread_time_constraint_policy_data_t pol = {
		.period = (uint32_t)(period_ns * tb.denom / tb.numer),
		.computation = (uint32_t)(compute_ns * tb.denom / tb.numer),
		.constraint = (uint32_t)(period_ns * tb.denom / tb.numer),
		.preemptible = 1,
	};
	kern_return_t kr = thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY, (thread_policy_t)&pol,
					     THREAD_TIME_CONSTRAINT_POLICY_COUNT);
	if (kr != KERN_SUCCESS)
		fprintf(stderr, "reader: real-time policy refused: 0x%08x\n", kr);
}
