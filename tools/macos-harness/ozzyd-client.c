/* SPDX-License-Identifier: MIT */
/* ozzyd-client (macOS): a test client for ozzyd, the userspace backend (macos/Backends/OzzyDaemon).
 * It plays the HAL's part of a device's shared memory the way ozzy-harness plays it in its own
 * process, with the same code (hal_role.c): a test tone in the output ring, the input ring to a
 * WAV file. So the two measure the same stream, once with the engine in the harness's process
 * and once in the daemon's.
 *
 * Settings reach the daemon as OzzyHAL's would, as OZZY_METHOD_SET_RATE and _APPLY_SETTING calls,
 * each a restart of the engine (none for a value it runs with already). The daemon logs the
 * stream's per-second line (ozzyd -v); this prints the reader's side and, at the end, what the
 * engine counted while it ran.
 *
 *   ozzyd-client [-l] [-d N] [-R rate] [-s seconds] [-r capture.wav] [-c first[-last]] [-q]
 *                [-X ms] [-F frames] [-Q in_flight] [-O out_flight]
 *     the harness's options, the same meanings; -X asks the daemon for the stall, which it
 *     takes only when it runs with -T
 */

#include "hal_role.h"
#include "../../macos/Shared/OzzyDaemonClient.h"
#include "../../engine/shared/ozzy_shared_data.h"
#include "../../engine/core/ozzy_bus.h"   /* enum ozzy_pcm_dir, which indexes the stats */
#include "../../common/devices/ploytec/ploytec_codec.h"
#include <mach/mach_time.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t quit;
static atomic_bool daemon_gone;

static void on_signal(int sig)
{
	(void)sig;
	quit = 1;
}

static uint64_t now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

static double host_hz(void)
{
	mach_timebase_info_data_t tb;
	mach_timebase_info(&tb);
	return 1e9 * tb.denom / tb.numer;
}

/* A setting as OzzyHAL changes it: the daemon restarts the engine with it, then answers */
static bool apply(xpc_connection_t c, uint64_t id, uint64_t method, uint64_t a0, uint64_t a1, const char *what)
{
	int64_t ret = OzzyDaemonCall(c, id, method, a0, a1);
	if (ret)
		fprintf(stderr, "%s: the daemon answered 0x%08x\n", what, (unsigned int)ret);
	return ret == 0;
}

int main(int argc, char **argv)
{
	bool list = false, quiet = false;
	unsigned int stall_ms = 0, xfer_frames = 0, in_queue = 0, out_queue = 0;
	int index = 0, seconds = 0;
	uint32_t want_rate = 0;
	unsigned int ch_first = 1, ch_last = 0;   /* 0: up to the device's last input */
	const char *record = NULL;
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-l"))
			list = true;
		else if (!strcmp(argv[i], "-q"))
			quiet = true;
		else if (!strcmp(argv[i], "-X") && i + 1 < argc)
			stall_ms = (unsigned int)atoi(argv[++i]);
		else if (!strcmp(argv[i], "-F") && i + 1 < argc) {
			xfer_frames = (unsigned int)atoi(argv[++i]);
			if (!xfer_frames || xfer_frames % 40 || xfer_frames > 200) {
				fprintf(stderr, "-F: a multiple of 40 up to 200\n");
				return 2;
			}
		} else if (!strcmp(argv[i], "-Q") && i + 1 < argc)
			in_queue = (unsigned int)atoi(argv[++i]);   /* the daemon checks it */
		else if (!strcmp(argv[i], "-O") && i + 1 < argc)
			out_queue = (unsigned int)atoi(argv[++i]);
		else if (!strcmp(argv[i], "-d") && i + 1 < argc)
			index = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-R") && i + 1 < argc)
			want_rate = (uint32_t)atoi(argv[++i]);
		else if (!strcmp(argv[i], "-s") && i + 1 < argc)
			seconds = atoi(argv[++i]);
		else if (!strcmp(argv[i], "-r") && i + 1 < argc)
			record = argv[++i];
		else if (!strcmp(argv[i], "-c") && i + 1 < argc) {
			if (sscanf(argv[++i], "%u-%u", &ch_first, &ch_last) == 1)
				ch_last = ch_first;
		} else {
			fprintf(stderr, "usage: ozzyd-client [-l] [-d N] [-R rate] [-s seconds] [-r capture.wav] [-c first[-last]] [-q] [-X ms] [-F frames] [-Q in_flight] [-O out_flight]\n");
			return 2;
		}
	}

	dispatch_queue_t queue = dispatch_queue_create("ozzyd-client", DISPATCH_QUEUE_SERIAL);
	xpc_connection_t c = OzzyDaemonConnect(queue, ^(xpc_object_t error) {
		if (error == XPC_ERROR_CONNECTION_INTERRUPTED || error == XPC_ERROR_CONNECTION_INVALID)
			atomic_store(&daemon_gone, true);
	});
	xpc_object_t devices = c ? OzzyDaemonList(c) : NULL;
	if (!devices) {
		fprintf(stderr, "no answer from %s (is ozzyd running? macos/Backends/OzzyDaemon/ozzyd-agent.sh load)\n",
			OZZYD_SERVICE);
		return 1;
	}
	size_t count = xpc_array_get_count(devices);
	for (size_t i = 0; list && i < count; i++) {
		xpc_object_t e = xpc_array_get_value(devices, i);
		fprintf(stderr, "%llu: %s (%04llx:%04llx), location 0x%08llx, session 0x%08llx\n",
			xpc_dictionary_get_uint64(e, OZZYD_KEY_ID), xpc_dictionary_get_string(e, OZZYD_KEY_NAME),
			xpc_dictionary_get_uint64(e, OZZYD_KEY_VID), xpc_dictionary_get_uint64(e, OZZYD_KEY_PID),
			xpc_dictionary_get_uint64(e, OZZYD_KEY_LOCATION), xpc_dictionary_get_uint64(e, OZZYD_KEY_SESSION));
	}
	if (list)
		return 0;
	if (index < 0 || (size_t)index >= count) {
		fprintf(stderr, "the daemon drives %zu device%s (-l lists them)\n", count, count == 1 ? "" : "s");
		return 1;
	}
	xpc_object_t entry = xpc_array_get_value(devices, (size_t)index);
	const uint64_t id = xpc_dictionary_get_uint64(entry, OZZYD_KEY_ID);
	const uint64_t session = xpc_dictionary_get_uint64(entry, OZZYD_KEY_SESSION);

	/* The device's shared memory, as OzzyHAL would map it from the kext */
	size_t size = 0;
	struct ozzy_shared_memory *shm = OzzyDaemonAttach(c, id, &size);
	if (!shm) {
		fprintf(stderr, "attach: no shared memory from the daemon\n");
		return 1;
	}
	if (size < sizeof(*shm) || shm->magic != OZZY_MAGIC || shm->version != OZZY_SHARED_VERSION ||
	    shm->session_id != session) {
		fprintf(stderr, "attach: %zu bytes, magic 0x%08x, version %u, session 0x%08x (want at least %zu, 0x%08x, %u, "
			"0x%08llx)\n", size, shm->magic, shm->version, shm->session_id, sizeof(*shm), OZZY_MAGIC,
			OZZY_SHARED_VERSION, (unsigned long long)session);
		return 1;
	}
	fprintf(stderr, "attached: %s %s, %04x:%04x, %zu bytes mapped, session 0x%08x\n", shm->manufacturer_name,
		shm->product_name, shm->vendor_id, shm->product_id, size, shm->session_id);

	/* Settings, each a restart of the engine (settings values: out (v + 1) x 10 frames, in (v + 1) x 8) */
	bool ok = true;
	if (want_rate)
		ok &= apply(c, id, OZZY_METHOD_SET_RATE, want_rate, 0, "rate");
	if (xfer_frames) {
		ok &= apply(c, id, OZZY_METHOD_APPLY_SETTING, OZZY_SETTING_PCM_OUT_FRAMES, xfer_frames / 10 - 1, "out frames");
		ok &= apply(c, id, OZZY_METHOD_APPLY_SETTING, OZZY_SETTING_PCM_IN_FRAMES, xfer_frames / 8 - 1, "in frames");
	}
	if (in_queue)
		ok &= apply(c, id, OZZY_METHOD_APPLY_SETTING, OZZY_SETTING_PCM_IN_QUEUE, in_queue - 1, "in queue");
	if (out_queue)
		ok &= apply(c, id, OZZY_METHOD_APPLY_SETTING, OZZY_SETTING_PCM_OUT_QUEUE, out_queue - 1, "out queue");
	if (!ok)
		return 1;
	if (!shm->audio.hardware_present || !shm->audio.driver_ready) {
		fprintf(stderr, "the device isn't streaming (hardware_present %d, driver_ready %d)\n",
			(int)shm->audio.hardware_present, (int)shm->audio.driver_ready);
		return 1;
	}

	const uint32_t rate = shm->audio.sample_rate, in = shm->audio.input_channels, out = shm->audio.output_channels;
	{
		const struct ozzy_info *info = &shm->info;
		uint32_t q_out = info->pipes[OZZY_INFO_PIPE_PCM_OUT].in_flight, q_in = info->pipes[OZZY_INFO_PIPE_PCM_IN].in_flight;
		uint32_t f_out = info->transfer_frames[OZZY_PCM_OUT], f_in = info->transfer_frames[OZZY_PCM_IN];
		fprintf(stderr, "running at %u Hz, %u in / %u out; in flight: out %u x %u frames (%.1f ms), in %u x %u frames "
			"(%.1f ms)\n", rate, in, out, q_out, f_out, rate ? 1000.0 * q_out * f_out / rate : 0.0, q_in, f_in,
			rate ? 1000.0 * q_in * f_in / rate : 0.0);
	}
	if (want_rate && want_rate != rate)
		fprintf(stderr, "note: asked for %u Hz\n", want_rate);
	if (out && !quiet)
		fprintf(stderr, "tone: %.2f Hz on outputs 1/2 at -18 dBFS\n", fill_tone(shm, out, rate));

	struct wav wav = { 0 };
	ploytec_frame_fn decode = in ? ploytec_frame_decoder(in) : NULL;
	struct input_reader reader = { .decode = decode, .ch_first = ch_first, .resync = true };   /* joining a running stream */
	if (!ch_last)
		ch_last = in;
	if (record && (ch_first < 1 || ch_last < ch_first || ch_last > in)) {
		fprintf(stderr, "-c %u-%u: the device has inputs 1-%u\n", ch_first, ch_last, in);
		record = NULL;
	}
	if (record && decode && !wav_open(&wav, record, rate, ch_last - ch_first + 1))
		fprintf(stderr, "can't write %s\n", record);
	else if (record && decode && (ch_first > 1 || ch_last < in))
		fprintf(stderr, "recording inputs %u-%u\n", ch_first, ch_last);

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	if (wav.f)
		input_reader_make_realtime();
	const double hz = host_hz();
	const struct ozzy_stats at_start = shm->stats;
	uint32_t last_late[2] = { at_start.dir[OZZY_PCM_OUT].late, at_start.dir[OZZY_PCM_IN].late };
	uint32_t seen_recoveries = at_start.recoveries + at_start.recovery_failures;
	uint64_t first_sample = 0, first_host = 0, last_sample = 0;
	bool have_first = false, present = true;
	unsigned int restarts = 0, tick = 0;
	uint64_t start = now_ms(), last_stats = start;

	while (!quit && (!seconds || now_ms() - start < (uint64_t)seconds * 1000)) {
		usleep(20000);
		if (atomic_load(&daemon_gone) || shm->magic != OZZY_MAGIC) {
			fprintf(stderr, "the daemon %s\n", atomic_load(&daemon_gone) ? "went away" : "dropped the device");
			break;
		}
		if (present != (bool)shm->audio.hardware_present) {
			present = !present;
			fprintf(stderr, "hardware_present %d\n", present);
		}

		/* A restart the daemon made (a recovery) lays the output ring out again and starts the
		 * clock from 0: the tone again, and the reader takes up from the current position */
		uint32_t recoveries = shm->stats.recoveries + shm->stats.recovery_failures;
		uint64_t sample_time = 0, host_time = 0;
		bool have_time = ozzy_timestamp_read(&shm->audio.timestamp, &sample_time, &host_time) && sample_time;
		if (recoveries != seen_recoveries || (have_time && sample_time < last_sample)) {
			seen_recoveries = recoveries;
			restarts++;
			struct timespec wall;
			clock_gettime(CLOCK_REALTIME, &wall);
			fprintf(stderr, "%ld.%03ld restart seen (%u)\n", (long)wall.tv_sec, wall.tv_nsec / 1000000, restarts);
			if (out && !quiet)
				fill_tone(shm, out, rate);
			reader.resync = true;
			have_first = false;
			last_sample = 0;
		}
		if (!have_time)
			continue;
		last_sample = sample_time;
		if (!have_first) {
			have_first = true;
			first_sample = sample_time;
			first_host = host_time;
		}
		if (wav.f)
			input_reader_run(&reader, shm, &wav, sample_time);

		if (now_ms() - last_stats >= 1000) {
			last_stats = now_ms();
			double secs = (double)(host_time - first_host) / hz;
			uint32_t late_out = shm->stats.dir[OZZY_PCM_OUT].late, late_in = shm->stats.dir[OZZY_PCM_IN].late;
			struct timespec wall;
			clock_gettime(CLOCK_REALTIME, &wall);
			fprintf(stderr, "%ld.%03ld reader | clock %.1f Hz | late out +%u in +%u%s%s\n", (long)wall.tv_sec,
				wall.tv_nsec / 1000000, secs > 0 ? (sample_time - first_sample) / secs : 0.0, late_out - last_late[0],
				late_in - last_late[1], reader.lost ? " | input frames lost" : "",
				wav.dropped ? " | WAV writer behind, frames dropped" : "");
			last_late[0] = late_out;
			last_late[1] = late_in;
			if (stall_ms && ++tick % 10 == 5) {
				fprintf(stderr, "STALL: asking the daemon to freeze its completion thread for %u ms\n", stall_ms);
				OzzyDaemonSend(c, id, OZZYD_METHOD_TEST_STALL, stall_ms, 0);   /* one-way: the reader doesn't wait */
			}
		}
	}

	fprintf(stderr, "stopping\n");
	wav_close(&wav);
	const struct ozzy_stats st = shm->stats;
	static const char *names[2] = { "out", "in" };
	fprintf(stderr, "engine stats while this ran:\n");
	for (int d = 0; d < 2; d++)
		fprintf(stderr, "  %-3s transfers %llu, errors %u, short %u, late %u; longest gap since the daemon started %u us\n",
			names[d], (unsigned long long)(st.dir[d].transfers - at_start.dir[d].transfers),
			st.dir[d].errors - at_start.dir[d].errors, st.dir[d].short_transfers - at_start.dir[d].short_transfers,
			st.dir[d].late - at_start.dir[d].late, st.dir[d].max_gap_us);
	fprintf(stderr, "  recoveries %u (failed %u); restarts seen here %u\n", st.recoveries - at_start.recoveries,
		st.recovery_failures - at_start.recovery_failures, restarts);
	if (reader.lost)
		fprintf(stderr, "input frames lost by the WAV reader: %llu\n", (unsigned long long)reader.lost);
	munmap(shm, size);
	xpc_release(devices);
	xpc_connection_cancel(c);
	return 0;
}
