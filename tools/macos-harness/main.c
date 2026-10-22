/* SPDX-License-Identifier: MIT */
/* ozzy-harness (macOS): runs an Ozzy engine against a device from user space, through IOUSBLib,
 * with no kext and SIP untouched. Protocol work on a Mac, and the measurement for a userspace
 * backend. It plays the HAL's part of the shared memory: a test tone in the output ring, the
 * input ring to a WAV file, MIDI in to the console. A port of tools/winusb-harness/main.c.
 *
 *   ozzy-harness [-l] [-d N] [-R rate] [-s seconds] [-r capture.wav] [-q]
 *     -l  list the supported devices found
 *     -d  the Nth of them (default 0)
 *     -R  sample rate in Hz (default: the device's default)
 *     -s  stop after this many seconds (default: Ctrl+C)
 *     -r  record the input to a 24-bit WAV file
 *     -q  silence instead of the test tone
 */

#include "iousb_bus.h"
#include "../../engine/core/ozzy_engine.h"
#include "../../common/devices/ploytec/ploytec_protocol.h"
#include "../../engine/shared/ozzy_shared_data.h"
#include "../../common/devices/ploytec/ploytec_codec.h"
#include "../../common/ozzy_version.h"
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

enum { MAX_FOUND = 16 };

static volatile sig_atomic_t quit;

static void on_signal(int sig)
{
	(void)sig;
	quit = 1;
}

struct found_device {
	const struct ozzy_device *model;
	io_service_t service;
	uint32_t location;
};

static int registry_int(io_service_t s, CFStringRef key)
{
	int v = -1;
	CFTypeRef ref = IORegistryEntryCreateCFProperty(s, key, kCFAllocatorDefault, 0);
	if (ref) {
		if (CFGetTypeID(ref) == CFNumberGetTypeID())
			CFNumberGetValue(ref, kCFNumberIntType, &v);
		CFRelease(ref);
	}
	return v;
}

/* Present USB devices that are in common/ozzy_devices.def, at most @max into @found */
static int find_devices(struct found_device *found, int max)
{
	io_iterator_t it = 0;
	if (IOServiceGetMatchingServices(MACH_PORT_NULL, IOServiceMatching("IOUSBHostDevice"), &it) != KERN_SUCCESS)
		return 0;
	int count = 0;
	io_service_t s;
	while ((s = IOIteratorNext(it))) {
		int vid = registry_int(s, CFSTR("idVendor")), pid = registry_int(s, CFSTR("idProduct"));
		const struct ozzy_device *model = vid >= 0 && pid >= 0 ? ozzy_device_lookup((uint16_t)vid, (uint16_t)pid) : NULL;
		if (!model || count >= max) {
			IOObjectRelease(s);
			continue;
		}
		found[count].model = model;
		found[count].service = s;
		found[count++].location = (uint32_t)registry_int(s, CFSTR("locationID"));
	}
	IOObjectRelease(it);
	return count;
}

/* 24-bit WAV writer (sizes patched on close) */
struct wav {
	FILE *f;
	uint32_t channels, bytes;
};

static bool wav_open(struct wav *w, const char *name, uint32_t rate, uint32_t channels)
{
	w->f = fopen(name, "wb");
	if (!w->f)
		return false;
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
	return true;
}

static void wav_write(struct wav *w, const uint8_t *s24, uint32_t frames)
{
	w->bytes += (uint32_t)fwrite(s24, 1, frames * w->channels * 3, w->f);
}

static void wav_close(struct wav *w)
{
	if (!w->f)
		return;
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

/* A -18 dBFS sine on outputs 1 and 2 with a whole number of cycles in the output ring, as the
 * WinUSB harness */
static double fill_tone(struct ozzy_shared_memory *shm, uint32_t channels, uint32_t rate)
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

static uint64_t now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

static void print_stats(const struct ozzy_stats *st)
{
	static const char *names[2] = { "out", "in" };
	for (int d = 0; d < 2; d++)
		fprintf(stderr, "  %-3s transfers %llu, errors %u, short %u, late %u, longest gap %u us\n", names[d],
			(unsigned long long)st->dir[d].transfers, st->dir[d].errors, st->dir[d].short_transfers,
			st->dir[d].late, st->dir[d].max_gap_us);
	fprintf(stderr, "  recoveries %u (failed %u)\n", st->recoveries, st->recovery_failures);
}

int main(int argc, char **argv)
{
	bool list = false, quiet = false;
	int index = 0, seconds = 0;
	uint32_t want_rate = 0;
	const char *record = NULL;
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-l"))
			list = true;
		else if (!strcmp(argv[i], "-q"))
			quiet = true;
		else if (!strcmp(argv[i], "-d") && i + 1 < argc)
			index = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-R") && i + 1 < argc)
			want_rate = (uint32_t)atoi(argv[++i]);
		else if (!strcmp(argv[i], "-s") && i + 1 < argc)
			seconds = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-r") && i + 1 < argc)
			record = argv[++i];
		else {
			fprintf(stderr, "usage: ozzy-harness [-l] [-d N] [-R rate] [-s seconds] [-r capture.wav] [-q]\n");
			return 2;
		}
	}
	fprintf(stderr, "ozzy-harness for macOS (build %s)\n", OZZY_GIT_VERSION);

	static struct found_device devices[MAX_FOUND];
	int device_count = find_devices(devices, MAX_FOUND);
	for (int i = 0; list && i < device_count; i++)
		fprintf(stderr, "%d: %s (%04x:%04x), location 0x%08x%s\n", i, devices[i].model->name, devices[i].model->vid,
			devices[i].model->pid, devices[i].location,
			devices[i].model->status == OZZY_STATUS_EXPERIMENTAL ? ", experimental" : "");
	if (list)
		return 0;
	if (index < 0 || index >= device_count) {
		fprintf(stderr, "no supported device found (-l lists them)\n");
		return 1;
	}
	const struct found_device *found = &devices[index];
	const struct ozzy_device *model = found->model;
	if (model->family == OZZY_FAMILY_PLOYTEC_ISOC) {
		fprintf(stderr, "%s streams isochronous OUT, which this harness doesn't do yet\n", model->name);
		return 1;
	}

	size_t engine_size = ozzy_engine_size(model);
	struct ozzy_engine *engine = ozzy_engine_init(engine_size ? calloc(1, engine_size) : NULL, model);
	if (!engine)
		return 1;

	static struct iousb_bus bus;
	if (!ibus_open(&bus, found->service, engine)) {
		ibus_close(&bus);
		return 1;
	}

	/* The shared memory the kext would map for the HAL; here both sides are in this process */
	struct ozzy_shared_memory *shm = mmap(NULL, sizeof(*shm), PROT_READ | PROT_WRITE, MAP_ANON | MAP_PRIVATE, -1, 0);
	if (shm == MAP_FAILED)
		return 1;
	ibus_get_string(&bus, bus.i_manufacturer, shm->manufacturer_name, sizeof(shm->manufacturer_name));
	ibus_get_string(&bus, bus.i_product, shm->product_name, sizeof(shm->product_name));
	ibus_get_string(&bus, bus.i_serial, shm->serial_number, sizeof(shm->serial_number));
	fprintf(stderr, "device: %s %s (%s), %04x:%04x\n", shm->manufacturer_name, shm->product_name, model->name,
		model->vid, model->pid);

	if (!ibus_configure(&bus)) {
		ibus_close(&bus);
		return 1;
	}
	ozzy_engine_attach(engine, &bus.bus, shm);
	ozzy_engine_set_usb_speed(engine, ibus_speed(&bus));
	ozzy_engine_set_anchors(engine, true);   /* the clock statistics and the input reader below */
	if (want_rate)
		ozzy_engine_set_rate(engine, want_rate);
	if (!ozzy_engine_start(engine)) {
		fprintf(stderr, "engine start failed\n");
		ozzy_engine_stop(engine);
		ibus_close(&bus);
		return 1;
	}

	uint32_t rate = shm->audio.sample_rate, in = shm->audio.input_channels, out = shm->audio.output_channels;
	fprintf(stderr, "running at %u Hz, %u in / %u out\n", rate, in, out);
	if (want_rate && want_rate != rate)
		fprintf(stderr, "note: asked for %u Hz\n", want_rate);
	if (out && !quiet)
		fprintf(stderr, "tone: %.2f Hz on outputs 1/2 at -18 dBFS\n", fill_tone(shm, out, rate));

	struct wav wav = { 0 };
	ploytec_frame_fn decode = in ? ploytec_frame_decoder(in) : NULL;
	if (record && decode && !wav_open(&wav, record, rate, in))
		fprintf(stderr, "can't write %s\n", record);

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	const uint64_t hz = bus.bus.ops->time_hz(&bus);
	uint64_t first_sample = 0, first_host = 0, next_frame = 0, lost = 0;
	bool have_first = false;
	uint64_t start = now_ms(), last_stats = start;
	uint32_t last_late[2] = { 0, 0 };

	while (!quit && (!seconds || now_ms() - start < (uint64_t)seconds * 1000)) {
		usleep(20000);

		/* MIDI in */
		uint8_t midi[256];
		uint32_t n = ozzy_midi_ring_pop(&shm->midi_in, midi, sizeof(midi));
		if (n) {
			fprintf(stderr, "midi in:");
			for (uint32_t i = 0; i < n; i++)
				fprintf(stderr, " %02x", midi[i]);
			fprintf(stderr, "\n");
		}

		uint64_t sample_time, host_time;
		if (!ozzy_timestamp_read(&shm->audio.timestamp, &sample_time, &host_time) || !sample_time)
			continue;
		if (!have_first) {
			have_first = true;
			first_sample = sample_time;
			first_host = host_time;
		}

		/* Input up to the input safety offset before the anchor (the IN transfers run alongside
		 * the OUT ones); the ring: one PLOYTEC_IN_FRAME_SIZE frame after another */
		if (wav.f) {
			const uint32_t in_frames = shm->audio.input_ring_frames ? shm->audio.input_ring_frames : 1;
			const uint32_t behind = shm->audio.input_safety_offset_frames;
			uint64_t end = sample_time > behind ? sample_time - behind : 0;
			if (end > next_frame + in_frames / 2) {
				lost += end - in_frames / 2 - next_frame;
				next_frame = end - in_frames / 2;
			}
			uint8_t s24[16 * 3];
			for (; next_frame < end; next_frame++) {
				uint32_t f = (uint32_t)(next_frame % in_frames);
				decode(s24, (const uint8_t *)shm->audio.input_buffer + f * PLOYTEC_IN_FRAME_SIZE);
				wav_write(&wav, s24, 1);
			}
		}

		if (now_ms() - last_stats >= 1000) {
			last_stats = now_ms();
			struct ibus_stats s = ibus_take_stats(&bus);
			double secs = hz ? (double)(host_time - first_host) / (double)hz : 0;
			uint32_t late_out = shm->stats.dir[OZZY_PCM_OUT].late, late_in = shm->stats.dir[OZZY_PCM_IN].late;
			fprintf(stderr, "clock %.1f Hz | out %u/s, in %u/s, errors %u | late out +%u in +%u%s\n",
				secs > 0 ? (sample_time - first_sample) / secs : 0.0, s.out_done, s.in_done, s.errors,
				late_out - last_late[0], late_in - last_late[1], lost ? " | input frames lost" : "");
			last_late[0] = late_out;
			last_late[1] = late_in;
		}
	}

	fprintf(stderr, "stopping\n");
	ozzy_engine_stop(engine);
	struct ozzy_stats final_stats = shm->stats;
	ibus_close(&bus);
	wav_close(&wav);
	fprintf(stderr, "engine stats:\n");
	print_stats(&final_stats);
	if (lost)
		fprintf(stderr, "input frames lost by the WAV reader: %llu\n", (unsigned long long)lost);
	free(engine);
	munmap(shm, sizeof(*shm));
	for (int i = 0; i < device_count; i++)
		IOObjectRelease(devices[i].service);
	return 0;
}
