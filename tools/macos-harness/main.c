/* SPDX-License-Identifier: MIT */
/* ozzy-harness (macOS): runs an Ozzy engine against a device from user space, through IOUSBLib,
 * with no kext and SIP untouched. Protocol work on a Mac, and the measurement for a userspace
 * backend. It plays the HAL's part of the shared memory: a test tone in the output ring, the
 * input ring to a WAV file, MIDI in to the console. A port of tools/winusb-harness/main.c.
 *
 *   ozzy-harness [-l] [-d N] [-R rate] [-s seconds] [-r capture.wav] [-c first[-last]] [-q]
 *     -l  list the supported devices found
 *     -d  the Nth of them (default 0)
 *     -R  sample rate in Hz (default: the device's default)
 *     -s  stop after this many seconds (default: Ctrl+C)
 *     -r  record the input to a 24-bit WAV file
 *     -c  record only these input channels, 1-based (e.g. -c 15-16), for long runs
 *     -q  silence instead of the test tone
 *     -W  stop when a PCM direction stops completing for 3 s (a wedge), instead of running on
 *     -X  test: every 10 s, freeze the completion thread for this many ms (a host that stops
 *         serving the device), to find whether and when a service gap wedges it
 *     -F  frames per PCM transfer, both ways (a multiple of 40, up to 200; default 80)
 *     -Q  PCM IN transfers in flight (2 to 8 in this build; default 4). In, every one but the
 *         first is margin, not latency: a gap in service longer than Q x frames overflows the
 *         device and loses input (a click), and that is all it does
 *     -O  PCM OUT transfers in flight (2 to 8 in this build; default 4), each one latency. A gap
 *         longer than about O x frames runs the device dry, which on the WZ4 also stops its IN
 *         side for good (the wedge; -A recovers it): measured, IN 8 / OUT 2 wedged 6 of 6, IN 2 /
 *         OUT 4 at the same gaps 0 of 6
 *     -P  the completion thread's real-time policy: "off", or period_us:compute_us
 *         (default 1000:250), for measuring how the policy shapes the gaps
 *     -A  recover a wedge: a PCM direction with no completions for 0.5 s suspends the engine,
 *         selects the interfaces' alternate setting again (which resets the endpoints on the
 *         device side) and starts it again with its handshake; each one is logged as RECOVERED
 *         or RECOVERY FAILED with how long it took, and counted in the engine's stats
 *     -K  test: every 10 s, restart the engine as a rate or setting change does, one of three ways:
 *         "plain" (ozzy_engine_restart, the kext's: suspend, start), "reselect" (suspend, the
 *         interfaces' alternate setting 0 and then theirs, start: ozzyd's) or "reconfig" (suspend,
 *         SET_CONFIGURATION 0 and 1 and the interfaces again, start: the official driver's, from
 *         the DB4 capture). A restart after which both directions aren't completing within 3 s is
 *         logged as wedged (with -A the watchdog then recovers it)
 *
 * With -r the reader that copies the input ring to the WAV runs under a real-time policy too,
 * so a loaded machine can't make it fall half a ring behind and drop frames (which shows in the
 * WAV as a discontinuity that is not the device's).
 *
 * Every second it prints the wall-clock time, the device clock, transfers per second, errors,
 * the engine's late counts and the longest gap between two completions of each PCM pipe. A
 * direction with no completions for 3 s is reported once as WEDGED: the engine's late count
 * can't see that, because it is measured when a completion closes a gap and a stopped pipe
 * never closes one. Without -A it stays wedged (the Windows driver resets the port).
 */

#include "../../macos/Backends/OzzyDaemon/iousb_bus.h"
#include "hal_role.h"
#include "../../engine/core/ozzy_engine.h"
#include "../../engine/shared/ozzy_shared_data.h"
#include "../../common/devices/ploytec/ploytec_codec.h"
#include "../../common/ozzy_version.h"
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
	bool list = false, quiet = false, stop_on_wedge = false, auto_recover = false;
	unsigned int stall_ms = 0, xfer_frames = 0, in_queue = 0, out_queue = 0, rt_period = 0, rt_compute = 0;
	bool rt_off = false;
	enum { RESTART_NONE, RESTART_PLAIN, RESTART_RESELECT, RESTART_RECONFIG } restart_mode = RESTART_NONE;
	static const char *restart_names[] = { "", "plain", "reselect", "reconfig" };
	int index = 0, seconds = 0;
	uint32_t want_rate = 0;
	unsigned int ch_first = 1, ch_last = 0;   /* 0: up to the device's last input */
	const char *record = NULL;
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-l"))
			list = true;
		else if (!strcmp(argv[i], "-q"))
			quiet = true;
		else if (!strcmp(argv[i], "-W"))
			stop_on_wedge = true;
		else if (!strcmp(argv[i], "-A"))
			auto_recover = true;
		else if (!strcmp(argv[i], "-X") && i + 1 < argc)
			stall_ms = (unsigned int)atoi(argv[++i]);
		else if (!strcmp(argv[i], "-K") && i + 1 < argc) {
			const char *m = argv[++i];
			restart_mode = !strcmp(m, "plain") ? RESTART_PLAIN : !strcmp(m, "reselect") ? RESTART_RESELECT
				     : !strcmp(m, "reconfig") ? RESTART_RECONFIG : RESTART_NONE;
			if (restart_mode == RESTART_NONE) {
				fprintf(stderr, "-K: plain, reselect or reconfig\n");
				return 2;
			}
		}
		else if (!strcmp(argv[i], "-P") && i + 1 < argc) {
			const char *p = argv[++i];
			if (!strcmp(p, "off"))
				rt_off = true;
			else if (sscanf(p, "%u:%u", &rt_period, &rt_compute) != 2 || !rt_period || !rt_compute ||
				 rt_compute > rt_period) {
				fprintf(stderr, "-P: off, or period_us:compute_us with compute <= period\n");
				return 2;
			}
		} else if (!strcmp(argv[i], "-F") && i + 1 < argc) {
			xfer_frames = (unsigned int)atoi(argv[++i]);
			if (!xfer_frames || xfer_frames % 40 || xfer_frames > 200) {
				fprintf(stderr, "-F: a multiple of 40 up to 200\n");
				return 2;
			}
		} else if (!strcmp(argv[i], "-Q") && i + 1 < argc) {
			in_queue = (unsigned int)atoi(argv[++i]);
			if (in_queue < OZZY_PCM_QUEUE_MIN + 1 || in_queue > OZZY_NUM_PCM_IN_QUEUES) {
				fprintf(stderr, "-Q: %u to %u\n", OZZY_PCM_QUEUE_MIN + 1, OZZY_NUM_PCM_IN_QUEUES);
				return 2;
			}
		} else if (!strcmp(argv[i], "-O") && i + 1 < argc) {
			out_queue = (unsigned int)atoi(argv[++i]);
			if (out_queue < OZZY_PCM_QUEUE_MIN + 1 || out_queue > OZZY_NUM_PCM_OUT_QUEUES) {
				fprintf(stderr, "-O: %u to %u\n", OZZY_PCM_QUEUE_MIN + 1, OZZY_NUM_PCM_OUT_QUEUES);
				return 2;
			}
		}
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
			fprintf(stderr, "usage: ozzy-harness [-l] [-d N] [-R rate] [-s seconds] [-r capture.wav] [-c first[-last]] [-q] [-W] [-A] [-X ms] [-K plain|reselect|reconfig] [-F frames] [-Q in_flight] [-O out_flight] [-P off|period:compute]\n");
			return 2;
		}
	}
	fprintf(stderr, "ozzy-harness for macOS (build %s)\n", OZZY_GIT_VERSION);

	static struct ibus_found devices[MAX_FOUND];
	int device_count = ibus_find_supported(devices, MAX_FOUND);
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
	const struct ibus_found *found = &devices[index];
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
	bus.rt_off = rt_off;
	bus.rt_period_us = rt_period;
	bus.rt_compute_us = rt_compute;
	fprintf(stderr, "completion thread: %s\n", rt_off ? "no real-time policy" : "real-time policy");
	if (rt_period)
		fprintf(stderr, "  period %u us, computation %u us\n", rt_period, rt_compute);
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
	if (xfer_frames) {   /* settings values: out (v + 1) x 10 frames, in (v + 1) x 8 (ozzy_pcm_frames) */
		ozzy_engine_set_setting(engine, OZZY_SETTING_PCM_OUT_FRAMES, (uint8_t)(xfer_frames / 10 - 1));
		ozzy_engine_set_setting(engine, OZZY_SETTING_PCM_IN_FRAMES, (uint8_t)(xfer_frames / 8 - 1));
	}
	if (in_queue)
		ozzy_engine_set_setting(engine, OZZY_SETTING_PCM_IN_QUEUE, (uint8_t)(in_queue - 1));
	if (out_queue)
		ozzy_engine_set_setting(engine, OZZY_SETTING_PCM_OUT_QUEUE, (uint8_t)(out_queue - 1));
	if (!ozzy_engine_start(engine)) {
		fprintf(stderr, "engine start failed\n");
		ozzy_engine_stop(engine);
		ibus_close(&bus);
		return 1;
	}

	uint32_t rate = shm->audio.sample_rate, in = shm->audio.input_channels, out = shm->audio.output_channels;
	fprintf(stderr, "running at %u Hz, %u in / %u out\n", rate, in, out);
	{
		/* As the engine runs them (it may have taken fewer than asked: its places) */
		uint32_t q_out = engine->report.in_flight[OZZY_PIPE_PCM_OUT], q_in = engine->report.in_flight[OZZY_PIPE_PCM_IN];
		uint32_t f_out = ozzy_pcm_frames(false, engine->settings.value[OZZY_SETTING_PCM_OUT_FRAMES]);
		uint32_t f_in = ozzy_pcm_frames(true, engine->settings.value[OZZY_SETTING_PCM_IN_FRAMES]);
		fprintf(stderr, "in flight: out %u x %u frames (%.1f ms), in %u x %u frames (%.1f ms)\n", q_out, f_out,
			rate ? 1000.0 * q_out * f_out / rate : 0.0, q_in, f_in, rate ? 1000.0 * q_in * f_in / rate : 0.0);
		if (in_queue && q_in != in_queue)
			fprintf(stderr, "note: asked for %u in flight\n", in_queue);
		if (out_queue && q_out != out_queue)
			fprintf(stderr, "note: asked for %u out in flight\n", out_queue);
	}
	if (want_rate && want_rate != rate)
		fprintf(stderr, "note: asked for %u Hz\n", want_rate);
	if (out && !quiet)
		fprintf(stderr, "tone: %.2f Hz on outputs 1/2 at -18 dBFS\n", fill_tone(shm, out, rate));

	struct wav wav = { 0 };
	ploytec_frame_fn decode = in ? ploytec_frame_decoder(in) : NULL;
	struct input_reader reader = { .decode = decode, .ch_first = ch_first };
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
	const uint64_t hz = bus.bus.ops->time_hz(&bus);
	uint64_t first_sample = 0, first_host = 0;
	bool have_first = false;
	uint64_t start = now_ms(), last_stats = start;
	uint32_t last_late[2] = { 0, 0 };
	unsigned int idle_secs[2] = { 0, 0 };   /* consecutive seconds without completions, out / in */
	bool wedged[2] = { false, false };
	/* the recovery watchdog: when each direction last made progress */
	uint64_t seen_done[2] = { atomic_load(&bus.total_done[0]), atomic_load(&bus.total_done[1]) };
	uint64_t seen_at[2] = { start, start };
	unsigned int recovered = 0, recovery_failed = 0;
	uint64_t recovery_ms_max = 0, recovery_ms_sum = 0;
	unsigned int restarts = 0, restarts_streaming = 0, restarts_failed = 0;

	while (!quit && (!seconds || now_ms() - start < (uint64_t)seconds * 1000)) {
		usleep(20000);

		/* Recovery watchdog: a PCM direction that made no progress for 0.5 s */
		uint64_t t_now = now_ms();
		for (int d = 0; d < 2; d++) {
			uint64_t n = atomic_load(&bus.total_done[d]);
			if (n != seen_done[d]) {
				seen_done[d] = n;
				seen_at[d] = t_now;
			}
		}
		if (auto_recover && (t_now - seen_at[0] >= 500 || t_now - seen_at[1] >= 500)) {
			struct timespec wall;
			clock_gettime(CLOCK_REALTIME, &wall);
			fprintf(stderr, "%ld.%03ld RECOVER: PCM %s stopped (%llu ms without a completion)\n",
				(long)wall.tv_sec, wall.tv_nsec / 1000000, t_now - seen_at[1] >= 500 ? "in" : "out",
				(unsigned long long)(t_now - (seen_at[1] < seen_at[0] ? seen_at[1] : seen_at[0])));
			uint64_t t0 = now_ms();
			/* Device or host: reads the device stopped answering are still in flight, and the
			 * reset has to abort them; a completion the host lost would never come back at all
			 * (the abort then reports transfers still pending) */
			int fly_out = atomic_load(&bus.pcm_in_flight[0]), fly_in = atomic_load(&bus.pcm_in_flight[1]);
			unsigned int ab_out = atomic_load(&bus.pcm_aborted[0]), ab_in = atomic_load(&bus.pcm_aborted[1]);
			ozzy_engine_suspend(engine);
			fprintf(stderr, "  in flight at the wedge: out %d, in %d; aborted by the reset: out %u, in %u\n", fly_out,
				fly_in, atomic_load(&bus.pcm_aborted[0]) - ab_out, atomic_load(&bus.pcm_aborted[1]) - ab_in);
			bool ok = ibus_reselect(&bus) && ozzy_engine_start(engine);
			/* streaming again only once both directions complete */
			uint64_t base[2] = { atomic_load(&bus.total_done[0]), atomic_load(&bus.total_done[1]) };
			while (ok && now_ms() - t0 < 3000 &&
			       (atomic_load(&bus.total_done[0]) == base[0] || atomic_load(&bus.total_done[1]) == base[1]))
				usleep(1000);
			ok = ok && atomic_load(&bus.total_done[0]) != base[0] && atomic_load(&bus.total_done[1]) != base[1];
			uint64_t took = now_ms() - t0;
			ozzy_engine_stats_recovery(engine, ok);
			if (ok) {
				recovered++;
				recovery_ms_sum += took;
				if (took > recovery_ms_max)
					recovery_ms_max = took;
				if (out && !quiet)
					fill_tone(shm, out, rate);   /* the restart laid the output ring out again */
			} else {
				recovery_failed++;
			}
			fprintf(stderr, "%s after %llu ms\n", ok ? "RECOVERED" : "RECOVERY FAILED", (unsigned long long)took);
			seen_at[0] = seen_at[1] = now_ms();
			seen_done[0] = atomic_load(&bus.total_done[0]);
			seen_done[1] = atomic_load(&bus.total_done[1]);
			reader.resync = true;   /* the reader takes up from the current position */
			have_first = false;
			if (!ok)
				break;
		}

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

		/* Input up to the input safety offset before the anchor */
		if (wav.f)
			input_reader_run(&reader, shm, &wav, sample_time);

		if (now_ms() - last_stats >= 1000) {
			last_stats = now_ms();
			struct ibus_stats s = ibus_take_stats(&bus);
			double secs = hz ? (double)(host_time - first_host) / (double)hz : 0;
			uint32_t late_out = shm->stats.dir[OZZY_PCM_OUT].late, late_in = shm->stats.dir[OZZY_PCM_IN].late;
			struct timespec wall;
			clock_gettime(CLOCK_REALTIME, &wall);
			fprintf(stderr, "%ld.%03ld clock %.1f Hz | out %u/s, in %u/s, errors %u | late out +%u in +%u | "
				"max gap out %u us, in %u us%s%s\n",
				(long)wall.tv_sec, wall.tv_nsec / 1000000, secs > 0 ? (sample_time - first_sample) / secs : 0.0,
				s.out_done, s.in_done, s.errors, late_out - last_late[0], late_in - last_late[1],
				s.max_gap_out_us, s.max_gap_in_us, reader.lost ? " | input frames lost" : "",
				wav.dropped ? " | WAV writer behind, frames dropped" : "");
			last_late[0] = late_out;
			last_late[1] = late_in;
			uint32_t done[2] = { s.out_done, s.in_done };
			static const char *dir_name[2] = { "out", "in" };
			for (int d = 0; d < 2; d++) {
				idle_secs[d] = done[d] ? 0 : idle_secs[d] + 1;
				if (idle_secs[d] >= 3 && !wedged[d]) {
					wedged[d] = true;
					fprintf(stderr, "WEDGED: PCM %s stopped completing at about %ld (3 s with none)\n",
						dir_name[d], (long)wall.tv_sec - 3);
				}
			}
			if (stop_on_wedge && (wedged[0] || wedged[1]))
				quit = 1;
			static unsigned int tick;
			if (stall_ms && ++tick % 10 == 5) {
				fprintf(stderr, "STALL: freezing the completion thread for %u ms\n", stall_ms);
				atomic_store(&bus.stall_ms, stall_ms);
			}
			static unsigned int restart_tick;
			if (restart_mode && ++restart_tick % 10 == 5) {
				fprintf(stderr, "%ld.%03ld RESTART (%s)\n", (long)wall.tv_sec, wall.tv_nsec / 1000000,
					restart_names[restart_mode]);
				uint64_t t0 = now_ms();
				bool ok;
				if (restart_mode == RESTART_PLAIN) {
					ok = ozzy_engine_restart(engine);   /* stops the engine if the device didn't take it */
				} else {
					ozzy_engine_suspend(engine);
					ok = (restart_mode == RESTART_RESELECT ? ibus_reselect(&bus) : ibus_reconfigure(&bus)) &&
					     ozzy_engine_start(engine);
				}
				uint64_t base[2] = { atomic_load(&bus.total_done[0]), atomic_load(&bus.total_done[1]) };
				while (ok && now_ms() - t0 < 3000 &&
				       (atomic_load(&bus.total_done[0]) == base[0] || atomic_load(&bus.total_done[1]) == base[1]))
					usleep(1000);
				bool streaming = ok && atomic_load(&bus.total_done[0]) != base[0] &&
						 atomic_load(&bus.total_done[1]) != base[1];
				restarts++;
				restarts_streaming += streaming;
				restarts_failed += !ok;
				fprintf(stderr, "RESTART %s after %llu ms\n",
					streaming ? "streaming" : ok ? "NOT STREAMING (wedged)" : "FAILED",
					(unsigned long long)(now_ms() - t0));
				if (streaming) {   /* else the watchdog (-A) sees the old progress times and recovers */
					seen_at[0] = seen_at[1] = now_ms();
					seen_done[0] = atomic_load(&bus.total_done[0]);
					seen_done[1] = atomic_load(&bus.total_done[1]);
				}
				reader.resync = true;
				have_first = false;
				if (out && !quiet)
					fill_tone(shm, out, rate);   /* the start laid the output ring out again */
			}
		}
	}

	fprintf(stderr, "stopping\n");
	ozzy_engine_stop(engine);
	struct ozzy_stats final_stats = shm->stats;
	ibus_close(&bus);
	wav_close(&wav);
	fprintf(stderr, "engine stats:\n");
	print_stats(&final_stats);
	if (reader.lost)
		fprintf(stderr, "input frames lost by the WAV reader: %llu\n", (unsigned long long)reader.lost);
	if (restart_mode)
		fprintf(stderr, "restarts (%s): %u, streaming after %u, wedged after %u, failed %u\n",
			restart_names[restart_mode], restarts, restarts_streaming, restarts - restarts_streaming - restarts_failed,
			restarts_failed);
	if (auto_recover)
		fprintf(stderr, "recoveries: %u ok, %u failed; time to stream again: max %llu ms, mean %llu ms\n",
			recovered, recovery_failed, (unsigned long long)recovery_ms_max,
			(unsigned long long)(recovered ? recovery_ms_sum / recovered : 0));
	free(engine);
	munmap(shm, sizeof(*shm));
	for (int i = 0; i < device_count; i++)
		IOObjectRelease(devices[i].service);
	return 0;
}
