/* SPDX-License-Identifier: MIT */
/*
 * ozzyd (macOS): Ozzy's userspace backend, run by launchd. It drives the supported USB devices
 * from user space through IOUSBLib (iousb_bus.c: no kext, SIP untouched), one engine per device,
 * and hands each device's shared memory, the kext's layout unchanged, to OzzyHAL and OzzyMIDI
 * over XPC (the protocol and its client side: macos/Shared/OzzyDaemonClient.h).
 *
 * A device that completes no PCM transfer for 0.5 s is recovered the way the dev harness's -A
 * does it: suspend the engine, select the interfaces' alternate setting 0 and then theirs again
 * (which resets the endpoints on the device side), and start it again with its handshake. The
 * WZ4 needs it: when its output runs dry it stops answering both pipes until then (measured,
 * tools/macos-harness/main.c).
 *
 * So far: the devices present at launch, and the service in the launching user's domain (a
 * LaunchAgent, ozzyd-agent.sh), which a test client can reach but CoreAudio's driver helpers,
 * which run as _coreaudiod, can't: OzzyHAL needs it in the system domain, as a LaunchDaemon. Not
 * yet: devices that arrive or leave later, settings kept across restarts, checking who connects,
 * a kext that already holds a device.
 *
 *   ozzyd [-v] [-T]
 *     -v  every second, per device, the dev harness's line: the device clock, PCM transfers per
 *         second, errors, the engine's late counts, the longest gap between completions
 *     -T  take test requests (OZZYD_METHOD_TEST_STALL)
 */

#include "iousb_bus.h"
#include "../../Shared/OzzyDaemonClient.h"
#include "../../../engine/core/ozzy_engine.h"
#include "../../../engine/shared/ozzy_shared_data.h"
#include "../../../common/ozzy_version.h"
#include <mach/mach_time.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

enum {
	MAX_DEVICES = 8,
	WATCHDOG_MS = 500,    /* a PCM direction without a completion this long: recover */
	RETRY_MS = 2000,      /* after a start that failed, try again this often */
};

struct device {
	const struct ozzy_device *model;
	uint32_t location;
	struct iousb_bus bus;
	struct ozzy_engine *engine;
	struct ozzy_shared_memory *shm;
	size_t shm_size;
	xpc_object_t shm_xpc;              /* the shared memory as the clients get it */
	pthread_t monitor;
	bool monitor_running;

	/* Starts one at a time (a client's SET_RATE or APPLY_SETTING, a recovery); what follows is
	 * under it too */
	pthread_mutex_t lock;
	bool streaming;                    /* false: the last start failed, and the watchdog tries again */
	unsigned int starts;               /* engine starts since launch */
	uint64_t seen_done[2], seen_at[2]; /* the watchdog: when each PCM direction last made progress */
	uint64_t retry_at;
};

static struct device devices[MAX_DEVICES];
static int device_count;
static bool verbose, test_requests;
static atomic_bool quitting;

static uint64_t now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

/* A line to the log (launchd's StandardErrorPath), after the wall-clock time as the harness has it */
static void log_line(const char *fmt, ...) OZZY_PRINTF_FORMAT(1, 2);
static void log_line(const char *fmt, ...)
{
	struct timespec wall;
	clock_gettime(CLOCK_REALTIME, &wall);
	char text[512];
	va_list args;
	va_start(args, fmt);
	vsnprintf(text, sizeof(text), fmt, args);
	va_end(args);
	fprintf(stderr, "%ld.%03ld %s\n", (long)wall.tv_sec, wall.tv_nsec / 1000000, text);
}

/* Under d->lock: both directions count as making progress from now */
static void watchdog_reset(struct device *d)
{
	uint64_t t = now_ms();
	for (int dir = 0; dir < 2; dir++) {
		d->seen_done[dir] = atomic_load(&d->bus.total_done[dir]);
		d->seen_at[dir] = t;
	}
}

/* What the engine runs, as it runs it (it may have taken fewer transfers than asked: its places) */
static void log_running(const struct device *d)
{
	const struct ozzy_engine *e = d->engine;
	uint32_t rate = d->shm->audio.sample_rate;
	uint32_t q_out = e->report.in_flight[OZZY_PIPE_PCM_OUT], q_in = e->report.in_flight[OZZY_PIPE_PCM_IN];
	uint32_t f_out = ozzy_pcm_frames(false, e->settings.value[OZZY_SETTING_PCM_OUT_FRAMES]);
	uint32_t f_in = ozzy_pcm_frames(true, e->settings.value[OZZY_SETTING_PCM_IN_FRAMES]);
	log_line("%s: running at %u Hz, %u in / %u out; in flight: out %u x %u frames (%.1f ms), in %u x %u frames "
		 "(%.1f ms)", d->model->name, rate, (unsigned int)d->shm->audio.input_channels,
		 (unsigned int)d->shm->audio.output_channels, q_out, f_out, rate ? 1000.0 * q_out * f_out / rate : 0.0, q_in,
		 f_in, rate ? 1000.0 * q_in * f_in / rate : 0.0);
}

/* Under d->lock: streaming again only once both directions complete */
static bool wait_streaming(struct device *d, uint64_t t0)
{
	uint64_t base[2] = { atomic_load(&d->bus.total_done[0]), atomic_load(&d->bus.total_done[1]) };
	while (now_ms() - t0 < 3000 &&
	       (atomic_load(&d->bus.total_done[0]) == base[0] || atomic_load(&d->bus.total_done[1]) == base[1]))
		usleep(1000);
	return atomic_load(&d->bus.total_done[0]) != base[0] && atomic_load(&d->bus.total_done[1]) != base[1];
}

/* Under d->lock: a PCM direction stopped (the WZ4's wedge), or the last start failed */
static void recover(struct device *d, uint64_t t_now)
{
	struct iousb_bus *b = &d->bus;
	if (d->streaming) {
		log_line("RECOVER: PCM %s stopped (%llu ms without a completion)", t_now - d->seen_at[1] >= WATCHDOG_MS ? "in" : "out",
			 (unsigned long long)(t_now - (d->seen_at[1] < d->seen_at[0] ? d->seen_at[1] : d->seen_at[0])));
		/* Device or host: reads the device stopped answering are still in flight and the reset
		 * aborts them; a completion the host lost would never come back at all */
		int fly_out = atomic_load(&b->pcm_in_flight[0]), fly_in = atomic_load(&b->pcm_in_flight[1]);
		unsigned int ab_out = atomic_load(&b->pcm_aborted[0]), ab_in = atomic_load(&b->pcm_aborted[1]);
		ozzy_engine_suspend(d->engine);
		fprintf(stderr, "  in flight at the wedge: out %d, in %d; aborted by the reset: out %u, in %u\n", fly_out, fly_in,
			atomic_load(&b->pcm_aborted[0]) - ab_out, atomic_load(&b->pcm_aborted[1]) - ab_in);
	} else {
		log_line("RECOVER: starting again after a failed start");
	}
	uint64_t t0 = now_ms();
	bool ok = ibus_reselect(b) && ozzy_engine_start(d->engine);
	d->starts++;
	ok = ok && wait_streaming(d, t0);
	ozzy_engine_stats_recovery(d->engine, ok);
	if (!ok)
		ozzy_engine_stop(d->engine);   /* the HAL drops the device until a retry works */
	d->streaming = ok;
	fprintf(stderr, "%s after %llu ms\n", ok ? "RECOVERED" : "RECOVERY FAILED", (unsigned long long)(now_ms() - t0));
	watchdog_reset(d);
	d->retry_at = now_ms() + RETRY_MS;
}

/* Per device: the recovery watchdog every 20 ms and, with -v, the harness's line every second */
static void *monitor(void *arg)
{
	struct device *d = arg;
	const uint64_t hz = d->bus.bus.ops->time_hz(&d->bus);
	uint64_t first_sample = 0, first_host = 0, last_stats = now_ms();
	uint32_t last_late[2] = { d->shm->stats.dir[OZZY_PCM_OUT].late, d->shm->stats.dir[OZZY_PCM_IN].late };
	unsigned int seen_starts = 0, idle_secs[2] = { 0, 0 };
	bool have_first = false, wedged[2] = { false, false };
	static const char *dir_name[2] = { "out", "in" };

	while (!atomic_load(&quitting)) {
		usleep(20000);

		pthread_mutex_lock(&d->lock);
		uint64_t t = now_ms();
		for (int dir = 0; dir < 2; dir++) {
			uint64_t n = atomic_load(&d->bus.total_done[dir]);
			if (n != d->seen_done[dir]) {
				d->seen_done[dir] = n;
				d->seen_at[dir] = t;
			}
		}
		if ((t - d->seen_at[0] >= WATCHDOG_MS || t - d->seen_at[1] >= WATCHDOG_MS) &&
		    (d->streaming || t >= d->retry_at))
			recover(d, t);
		unsigned int starts = d->starts;
		pthread_mutex_unlock(&d->lock);

		if (starts != seen_starts) {   /* the clock starts from 0 again */
			seen_starts = starts;
			have_first = false;
		}
		uint64_t sample_time = 0, host_time = 0;
		bool have_time = ozzy_timestamp_read(&d->shm->audio.timestamp, &sample_time, &host_time) && sample_time;
		if (have_time && !have_first) {
			have_first = true;
			first_sample = sample_time;
			first_host = host_time;
		}
		if (!verbose || now_ms() - last_stats < 1000)
			continue;
		last_stats = now_ms();
		struct ibus_stats s = ibus_take_stats(&d->bus);
		double secs = have_time && hz ? (double)(host_time - first_host) / (double)hz : 0;
		uint32_t late_out = d->shm->stats.dir[OZZY_PCM_OUT].late, late_in = d->shm->stats.dir[OZZY_PCM_IN].late;
		log_line("clock %.1f Hz | out %u/s, in %u/s, errors %u | late out +%u in +%u | max gap out %u us, in %u us",
			 secs > 0 ? (sample_time - first_sample) / secs : 0.0, s.out_done, s.in_done, s.errors,
			 late_out - last_late[0], late_in - last_late[1], s.max_gap_out_us, s.max_gap_in_us);
		last_late[0] = late_out;
		last_late[1] = late_in;
		uint32_t done[2] = { s.out_done, s.in_done };
		for (int dir = 0; dir < 2; dir++) {
			idle_secs[dir] = done[dir] ? 0 : idle_secs[dir] + 1;
			if (idle_secs[dir] >= 3 && !wedged[dir]) {
				wedged[dir] = true;
				log_line("WEDGED: PCM %s stopped completing (3 s with none)", dir_name[dir]);
			} else if (!idle_secs[dir]) {
				wedged[dir] = false;
			}
		}
	}
	return NULL;
}

/*
 * Under d->lock: take a changed rate or device setting (the kext's RestartLocked). Not with
 * ozzy_engine_restart (suspend, then start) as the kext does: the handshake keeps the device's
 * output dry for about 0.7 s, and the WZ4 stops answering after that until its interfaces are
 * selected again (measured: 3 of 3 restarts that way left it silent until the watchdog's
 * recovery). So the way the recovery does it, which reselects them.
 */
static bool restart(struct device *d)
{
	uint64_t t0 = now_ms();
	ozzy_engine_suspend(d->engine);
	bool ok = ibus_reselect(&d->bus) && ozzy_engine_start(d->engine);
	d->starts++;
	ok = ok && wait_streaming(d, t0);
	if (!ok)
		ozzy_engine_stop(d->engine);   /* the HAL drops the device until the watchdog's retry works */
	d->streaming = ok;
	watchdog_reset(d);
	d->retry_at = now_ms() + RETRY_MS;
	if (ok)
		log_running(d);
	else
		log_line("%s: restart failed", d->model->name);
	log_line("restart took %llu ms", (unsigned long long)(now_ms() - t0));
	return ok;
}

/* A client's call: the kext's user client methods, and the test stall */
static int64_t device_call(struct device *d, uint64_t method, uint64_t a0, uint64_t a1)
{
	if (method == OZZY_METHOD_MIDI_OUT_PENDING) {
		/* On the completion thread, as the kext runs it on its work loop: never beside a completion */
		struct ozzy_engine *e = d->engine;
		CFRunLoopPerformBlock(d->bus.completion_loop, kCFRunLoopDefaultMode, ^{
			ozzy_engine_midi_out_pending(e);
		});
		CFRunLoopWakeUp(d->bus.completion_loop);
		return kIOReturnSuccess;
	}
	if (method == OZZYD_METHOD_TEST_STALL) {
		if (!test_requests)
			return kIOReturnUnsupported;
		fprintf(stderr, "STALL: freezing the completion thread for %u ms\n", (unsigned int)a0);
		atomic_store(&d->bus.stall_ms, (unsigned int)a0);
		return kIOReturnSuccess;
	}
	if (method != OZZY_METHOD_SET_RATE && method != OZZY_METHOD_APPLY_SETTING)
		return kIOReturnUnsupported;

	IOReturn ret = kIOReturnNotReady;
	pthread_mutex_lock(&d->lock);
	if (d->streaming) {
		enum ozzy_setting_change change;
		if (method == OZZY_METHOD_SET_RATE)
			change = ozzy_engine_change_rate(d->engine, (uint32_t)a0);
		else if (a0 >= OZZY_NUM_SETTINGS || ozzy_setting_descs[a0].kind != OZZY_SETTING_DEVICE)
			change = OZZY_SETTING_INVALID;   /* device settings only: the HAL applies routes itself */
		else
			change = ozzy_engine_change_setting(d->engine, (unsigned int)a0, (unsigned int)a1);
		ret = change == OZZY_SETTING_INVALID ? kIOReturnBadArgument
		    : change == OZZY_SETTING_RESTART ? (restart(d) ? kIOReturnSuccess : kIOReturnIOError)
						     : kIOReturnSuccess;
	}
	pthread_mutex_unlock(&d->lock);
	return ret;
}

static void handle_message(xpc_connection_t peer, xpc_object_t message)
{
	xpc_object_t reply = xpc_dictionary_create_reply(message);   /* NULL for a one-way message */
	uint64_t op = xpc_dictionary_get_uint64(message, OZZYD_KEY_OP);
	uint64_t id = xpc_dictionary_get_uint64(message, OZZYD_KEY_ID);
	struct device *d = id < (uint64_t)device_count ? &devices[id] : NULL;
	int64_t ret = kIOReturnSuccess;

	switch (op) {
	case OZZYD_OP_LIST: {
		xpc_object_t list = xpc_array_create(NULL, 0);
		for (int i = 0; i < device_count; i++) {
			const struct device *e = &devices[i];
			xpc_object_t entry = xpc_dictionary_create(NULL, NULL, 0);
			xpc_dictionary_set_uint64(entry, OZZYD_KEY_ID, (uint64_t)i);
			xpc_dictionary_set_uint64(entry, OZZYD_KEY_LOCATION, e->location);
			xpc_dictionary_set_uint64(entry, OZZYD_KEY_SESSION, e->shm->session_id);
			xpc_dictionary_set_bool(entry, OZZYD_KEY_AUDIO, e->model->in_channels || e->model->out_channels);
			xpc_dictionary_set_uint64(entry, OZZYD_KEY_VID, e->model->vid);
			xpc_dictionary_set_uint64(entry, OZZYD_KEY_PID, e->model->pid);
			xpc_dictionary_set_string(entry, OZZYD_KEY_NAME, e->model->name);
			xpc_array_append_value(list, entry);
			xpc_release(entry);
		}
		if (reply)
			xpc_dictionary_set_value(reply, OZZYD_KEY_DEVICES, list);
		xpc_release(list);
		break;
	}
	case OZZYD_OP_ATTACH:
		if (!d) {
			ret = kIOReturnNoDevice;
		} else if (reply) {
			xpc_dictionary_set_value(reply, OZZYD_KEY_SHM, d->shm_xpc);
			xpc_dictionary_set_uint64(reply, OZZYD_KEY_SIZE, d->shm_size);
		}
		break;
	case OZZYD_OP_CALL:
		ret = d ? device_call(d, xpc_dictionary_get_uint64(message, OZZYD_KEY_METHOD),
				      xpc_dictionary_get_uint64(message, OZZYD_KEY_A0),
				      xpc_dictionary_get_uint64(message, OZZYD_KEY_A1))
			: kIOReturnNoDevice;
		break;
	default:
		ret = kIOReturnBadArgument;
		break;
	}
	if (reply) {
		xpc_dictionary_set_int64(reply, OZZYD_KEY_RET, ret);
		xpc_connection_send_message(peer, reply);
		xpc_release(reply);
	}
}

/* The engine's stats since launch (ozzy_stats.h), as the harness prints them at the end */
static void log_stats(const struct device *d)
{
	static const char *names[2] = { "out", "in" };
	const struct ozzy_stats *st = &d->shm->stats;
	fprintf(stderr, "engine stats, %s:\n", d->model->name);
	for (int dir = 0; dir < 2; dir++)
		fprintf(stderr, "  %-3s transfers %llu, errors %u, short %u, late %u, longest gap %u us\n", names[dir],
			(unsigned long long)st->dir[dir].transfers, st->dir[dir].errors, st->dir[dir].short_transfers,
			st->dir[dir].late, st->dir[dir].max_gap_us);
	fprintf(stderr, "  recoveries %u (failed %u)\n", st->recoveries, st->recovery_failures);
}

static void close_device(struct device *d)
{
	if (d->monitor_running) {   /* quitting is set */
		pthread_join(d->monitor, NULL);
		d->monitor_running = false;
	}
	pthread_mutex_lock(&d->lock);
	if (d->shm)
		d->shm->magic = 0;   /* the clients see it gone at once, as with the kext */
	if (d->engine && d->engine->shm) {
		ozzy_engine_stop(d->engine);
		log_stats(d);
	}
	pthread_mutex_unlock(&d->lock);
	ibus_close(&d->bus);
	if (d->shm_xpc) {
		xpc_release(d->shm_xpc);
		d->shm_xpc = NULL;
	}
	if (d->shm) {
		munmap(d->shm, d->shm_size);
		d->shm = NULL;
	}
	free(d->engine);
	d->engine = NULL;
}

static bool open_device(struct device *d, const struct ibus_found *found)
{
	const struct ozzy_device *model = found->model;
	d->model = model;
	d->location = found->location;
	pthread_mutex_init(&d->lock, NULL);
	if (model->family == OZZY_FAMILY_PLOYTEC_ISOC) {
		log_line("%s streams isochronous OUT, which the IOUSBLib bus doesn't do yet", model->name);
		return false;
	}
	size_t engine_size = ozzy_engine_size(model);
	d->engine = ozzy_engine_init(engine_size ? calloc(1, engine_size) : NULL, model);
	if (!d->engine || !ibus_open(&d->bus, found->service, d->engine))
		return false;

	/* The shared memory: shared anonymous pages, so what a client maps are these pages and not
	 * a copy, in whole pages as xpc_shmem wants them */
	size_t page = (size_t)getpagesize();
	d->shm_size = (sizeof(struct ozzy_shared_memory) + page - 1) & ~(page - 1);
	void *shm = mmap(NULL, d->shm_size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANON, -1, 0);
	if (shm == MAP_FAILED)
		return false;
	d->shm = shm;
	memset(shm, 0, d->shm_size);   /* every page in now, not as faults on the completion thread */
	d->shm->session_id = (uint32_t)((uintptr_t)shm ^ mach_absolute_time());   /* as the kext */
	ibus_get_string(&d->bus, d->bus.i_manufacturer, d->shm->manufacturer_name, sizeof(d->shm->manufacturer_name));
	ibus_get_string(&d->bus, d->bus.i_product, d->shm->product_name, sizeof(d->shm->product_name));
	ibus_get_string(&d->bus, d->bus.i_serial, d->shm->serial_number, sizeof(d->shm->serial_number));
	log_line("device: %s %s (%s), %04x:%04x, location 0x%08x", d->shm->manufacturer_name, d->shm->product_name,
		 model->name, model->vid, model->pid, d->location);
	if (!ibus_configure(&d->bus))
		return false;
	d->shm_xpc = xpc_shmem_create(shm, d->shm_size);
	if (!d->shm_xpc)
		return false;

	ozzy_engine_attach(d->engine, &d->bus.bus, d->shm);
	ozzy_engine_set_usb_speed(d->engine, ibus_speed(&d->bus));
	ozzy_engine_set_anchors(d->engine, true);   /* the HAL's clock */
	ozzy_engine_set_driver_version(d->engine, "ozzyd " OZZY_GIT_VERSION);
	pthread_mutex_lock(&d->lock);
	d->streaming = ozzy_engine_start(d->engine);
	d->starts++;
	watchdog_reset(d);
	d->retry_at = now_ms() + RETRY_MS;
	if (d->streaming)
		log_running(d);
	else
		log_line("%s: start failed; trying again every %u ms", model->name, RETRY_MS);
	pthread_mutex_unlock(&d->lock);

	if (pthread_create(&d->monitor, NULL, monitor, d) != 0)
		return false;
	d->monitor_running = true;
	return true;
}

static void shut_down(int sig)
{
	log_line("signal %d: stopping", sig);
	atomic_store(&quitting, true);
	for (int i = 0; i < device_count; i++)
		close_device(&devices[i]);
	exit(0);
}

int main(int argc, char **argv)
{
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-v")) {
			verbose = true;
		} else if (!strcmp(argv[i], "-T")) {
			test_requests = true;
		} else {
			fprintf(stderr, "usage: ozzyd [-v] [-T]\n");
			return 2;
		}
	}
	log_line("ozzyd for macOS (build %s), pid %d%s", OZZY_GIT_VERSION, getpid(), test_requests ? ", test requests on" : "");

	struct ibus_found found[MAX_DEVICES];
	int n = ibus_find_supported(found, MAX_DEVICES);
	for (int i = 0; i < n; i++) {
		struct device *d = &devices[device_count];
		if (open_device(d, &found[i])) {
			device_count++;
		} else {
			log_line("%s: not driven", found[i].model->name);
			close_device(d);
			memset(d, 0, sizeof(*d));
		}
		IOObjectRelease(found[i].service);   /* ibus_open took its own reference */
	}
	log_line("%d device%s", device_count, device_count == 1 ? "" : "s");

	dispatch_queue_t queue = dispatch_queue_create("com.ozzy.daemon.xpc", DISPATCH_QUEUE_SERIAL);
	xpc_connection_t listener = xpc_connection_create_mach_service(OZZYD_SERVICE, queue,
								      XPC_CONNECTION_MACH_SERVICE_LISTENER);
	xpc_connection_set_event_handler(listener, ^(xpc_object_t peer) {
		if (xpc_get_type(peer) != XPC_TYPE_CONNECTION) {
			char *what = xpc_copy_description(peer);
			log_line("listener: %s", what);
			free(what);
			return;
		}
		pid_t pid = xpc_connection_get_pid(peer);
		log_line("client %d connected", pid);
		xpc_connection_set_target_queue(peer, queue);
		xpc_connection_set_event_handler(peer, ^(xpc_object_t message) {
			if (xpc_get_type(message) == XPC_TYPE_DICTIONARY)
				handle_message(peer, message);
			else if (message == XPC_ERROR_CONNECTION_INVALID)
				log_line("client %d gone", pid);   /* its mappings are its own to drop */
		});
		xpc_connection_resume(peer);
	});
	xpc_connection_resume(listener);

	/* launchctl bootout sends SIGTERM */
	static const int signals[] = { SIGTERM, SIGINT };
	for (unsigned int i = 0; i < sizeof(signals) / sizeof(signals[0]); i++) {
		int sig = signals[i];
		signal(sig, SIG_IGN);
		dispatch_source_t source = dispatch_source_create(DISPATCH_SOURCE_TYPE_SIGNAL, (uintptr_t)sig, 0,
								  dispatch_get_main_queue());
		dispatch_source_set_event_handler(source, ^{
			shut_down(sig);
		});
		dispatch_resume(source);
	}
	dispatch_main();
}
