/* SPDX-License-Identifier: MIT */
#ifndef HAL_ROLE_H
#define HAL_ROLE_H

/* The CoreAudio HAL's part of the shared memory, as the macOS dev tools play it: a test tone in
 * the output ring and the input ring to a 24-bit WAV file. ozzy-harness runs the engine in the
 * same process, ozzyd-client against ozzyd (macos/Backends/OzzyDaemon); both use this, so they
 * measure the same way. */

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "../../engine/shared/ozzy_shared_data.h"
#include "../../common/devices/ploytec/ploytec_codec.h"

/*
 * 24-bit WAV writer (sizes patched on close). The reader hands it frames through a FIFO in memory
 * and a thread of its own writes them to the file, so a disk that stalls can't make the reader
 * fall behind the input ring (at 96 kHz the ring holds 169 ms; a write that blocked longer than
 * half of that once cost 4,700 frames under load). A writer more than the FIFO behind drops
 * frames, counted apart from the reader's. Zero it, then wav_open.
 */
struct wav {
	FILE *f;
	uint32_t channels, bytes;      /* bytes: the writer thread's, read after it has finished */
	uint8_t *fifo;
	_Atomic uint64_t head, tail;   /* bytes put in by the reader, taken out by the writer, since open */
	atomic_bool closing;
	uint64_t dropped;              /* bytes the FIFO had no room for (the reader's) */
	pthread_t writer;
};

bool wav_open(struct wav *w, const char *name, uint32_t rate, uint32_t channels);
/* The reader's side: into the FIFO, no system call */
void wav_write(struct wav *w, const uint8_t *s24, uint32_t frames);
void wav_close(struct wav *w);

/* A -18 dBFS sine on outputs 1 and 2 with a whole number of cycles in the output ring, as the
 * WinUSB harness; its frequency in Hz. The engine lays the ring out again at every start, so
 * fill it again after one. */
double fill_tone(struct ozzy_shared_memory *shm, uint32_t channels, uint32_t rate);

/* The input ring into a WAV, frame by frame, as far as the clock says it is complete. Zero it,
 * then set decode and ch_first. */
struct input_reader {
	ploytec_frame_fn decode;
	unsigned int ch_first;   /* the first input recorded, 1-based (the WAV's channels follow it) */
	uint64_t next_frame;     /* the next frame to read, in sample time */
	uint64_t lost;           /* frames the reader fell more than half a ring behind on */
	bool resync;             /* take up from the current position (after a restart, or on joining) */
};

/* Up to the input safety offset before @sample_time (the latest clock anchor) */
void input_reader_run(struct input_reader *r, struct ozzy_shared_memory *shm, struct wav *w, uint64_t sample_time);

/* The calling thread, the reader's, under the time-constraint policy: 20 ms apart, 2 ms to copy
 * what came in, so a loaded machine can't make it fall half a ring behind and drop frames (which
 * shows in the WAV as a discontinuity that is not the device's) */
void input_reader_make_realtime(void);

#endif
