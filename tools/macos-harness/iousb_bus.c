/* SPDX-License-Identifier: MIT */
#include "iousb_bus.h"
#include <IOKit/IOCFPlugIn.h>
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

struct ibus_transfer {
	struct iousb_bus *ib;
	uint8_t pipe;
	void *xfer;
};

static int ibus_pipe_slot(uint8_t address)
{
	return (address & 0x0F) | ((address & 0x80) >> 3);
}

void ibus_log(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);
	fputc('\n', stderr);
}

static uint64_t ns_to_abs(uint64_t ns)
{
	static mach_timebase_info_data_t tb;
	if (!tb.denom)
		mach_timebase_info(&tb);
	return ns * tb.denom / tb.numer;
}

/* Run the calling thread under the time-constraint (real-time) policy CoreAudio's IO threads
 * use: @period_us apart, needing up to @compute_us of it */
static void make_realtime(uint32_t period_us, uint32_t compute_us)
{
	thread_time_constraint_policy_data_t pol = {
		.period = (uint32_t)ns_to_abs(period_us * 1000ull),
		.computation = (uint32_t)ns_to_abs(compute_us * 1000ull),
		.constraint = (uint32_t)ns_to_abs(period_us * 1000ull),
		.preemptible = 1,
	};
	kern_return_t kr = thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY,
					     (thread_policy_t)&pol, THREAD_TIME_CONSTRAINT_POLICY_COUNT);
	if (kr != KERN_SUCCESS)
		ibus_log("[IOUSBBus] real-time policy refused: 0x%08x", kr);
}

static uint64_t ibus_time(void *ctx)
{
	(void)ctx;
	return mach_absolute_time();
}

static uint64_t ibus_time_hz(void *ctx)
{
	(void)ctx;
	mach_timebase_info_data_t tb;
	mach_timebase_info(&tb);
	return 1000000000ull * tb.denom / tb.numer;
}

static void ibus_sleep(void *ctx, unsigned int ms)
{
	(void)ctx;
	usleep(ms * 1000);
}

/* A transfer finished: on the completion thread, one at a time. An aborted pipe completes its
 * transfers with kIOReturnAborted, which the engines know as OZZY_STATUS_ABORTED. */
static void ibus_complete(void *refcon, IOReturn result, void *arg0)
{
	struct ibus_transfer *t = refcon;
	struct iousb_bus *ib = t->ib;
	uint32_t bytes = (uint32_t)(uintptr_t)arg0;
	uint8_t pipe = t->pipe;
	void *xfer = t->xfer;
	int status = result == kIOReturnSuccess ? 0 : (int)result;
	free(t);

	if (status == 0) {
		int dir = (pipe & 0x80) ? 1 : 0;
		uint64_t now = mach_absolute_time();
		atomic_fetch_add(dir ? &ib->in_done : &ib->out_done, 1);
		/* gaps between completions of the PCM pipes only (MIDI in completes on its own schedule) */
		if (pipe == 0x05 || pipe == 0x86) {
			if (ib->last_done[dir]) {
				uint64_t gap = now - ib->last_done[dir];
				if (gap > atomic_load(&ib->max_gap[dir]))
					atomic_store(&ib->max_gap[dir], gap);
			}
			ib->last_done[dir] = now;
		}
	} else if (result != kIOReturnAborted && atomic_fetch_add(&ib->errors, 1) < 10)
		ibus_log("[IOUSBBus] pipe 0x%02x error: 0x%08x (%u bytes)", pipe, result, bytes);

	if (ib->engine)
		ozzy_engine_complete(ib->engine, pipe, xfer, status, bytes);
	atomic_fetch_sub(&ib->pending, 1);
}

static bool ibus_submit(void *ctx, uint8_t pipe, void *buf, uint32_t size, void *xfer)
{
	struct iousb_bus *ib = ctx;
	int slot = ibus_pipe_slot(pipe);
	ibus_interface intf = ib->pipe_intf[slot];
	if (!intf || ib->pipe_addr[slot] != pipe)
		return false;

	struct ibus_transfer *t = calloc(1, sizeof(*t));
	if (!t)
		return false;
	t->ib = ib;
	t->pipe = pipe;
	t->xfer = xfer;
	atomic_fetch_add(&ib->pending, 1);
	IOReturn r = (pipe & 0x80)
		? (*intf)->ReadPipeAsync(intf, ib->pipe_ref[slot], buf, size, ibus_complete, t)
		: (*intf)->WritePipeAsync(intf, ib->pipe_ref[slot], buf, size, ibus_complete, t);
	if (r != kIOReturnSuccess) {
		ibus_log("[IOUSBBus] pipe 0x%02x submit failed: 0x%08x", pipe, r);
		atomic_fetch_sub(&ib->pending, 1);
		free(t);
		return false;
	}
	return true;
}

static bool ibus_control(void *ctx, uint8_t type, uint8_t req, uint16_t val, uint16_t idx, void *data,
			 uint16_t len)
{
	struct iousb_bus *ib = ctx;
	IOUSBDevRequestTO r = {
		.bmRequestType = type,
		.bRequest = req,
		.wValue = val,
		.wIndex = idx,
		.wLength = len,
		.pData = data,
		.noDataTimeout = 1000,
		.completionTimeout = 2000,
	};
	IOReturn kr = (*ib->dev)->DeviceRequestTO(ib->dev, &r);
	if (kr == kIOReturnSuccess)
		return true;
	ibus_log("[IOUSBBus] control request 0x%02x (type 0x%02x, wValue 0x%04x) failed: 0x%08x", req, type, val, kr);
	return false;
}

/* Clears the stall on the device's endpoint and the host side's data toggle */
static int ibus_clear_halt(void *ctx, uint8_t address)
{
	struct iousb_bus *ib = ctx;
	int slot = ibus_pipe_slot(address);
	ibus_interface intf = ib->pipe_intf[slot];
	if (!intf || ib->pipe_addr[slot] != address)
		return -ENODEV;
	IOReturn kr = (*intf)->ClearPipeStallBothEnds(intf, ib->pipe_ref[slot]);
	if (kr == kIOReturnSuccess)
		return 0;
	ibus_log("[IOUSBBus] clear halt on pipe 0x%02x failed: 0x%08x", address, kr);
	return -EIO;
}

/* ozzy_engine_timer every interval, on absolute deadlines (no drift); if we fell behind, skip
 * to the next deadline ahead, as KextBus does */
static void *ibus_timer_thread(void *arg)
{
	struct iousb_bus *ib = arg;
	make_realtime(ib->timer_interval_us, ib->timer_interval_us / 4 + 1);
	uint64_t interval = ns_to_abs(ib->timer_interval_us * 1000ull);
	uint64_t deadline = mach_absolute_time() + interval;
	while (atomic_load(&ib->timer_on)) {
		mach_wait_until(deadline);
		if (!atomic_load(&ib->timer_on))
			break;
		if (ib->engine)
			ozzy_engine_timer(ib->engine);
		deadline += interval;
		uint64_t now = mach_absolute_time();
		if (deadline <= now)
			deadline = now + interval;
	}
	return NULL;
}

static bool ibus_start_timer(void *ctx, uint32_t interval_us)
{
	struct iousb_bus *ib = ctx;
	if (ib->timer_running)
		return true;
	ib->timer_interval_us = interval_us ? interval_us : 1000;
	atomic_store(&ib->timer_on, true);
	if (pthread_create(&ib->timer_thread, NULL, ibus_timer_thread, ib) != 0) {
		atomic_store(&ib->timer_on, false);
		return false;
	}
	ib->timer_running = true;
	return true;
}

static void ibus_stop_timer(void *ctx)
{
	struct iousb_bus *ib = ctx;
	atomic_store(&ib->timer_on, false);
	if (!ib->timer_running)
		return;
	pthread_join(ib->timer_thread, NULL);   /* waits for a call in progress */
	ib->timer_running = false;
}

/* Abort what is in flight and wait until the completion thread has delivered it */
static void ibus_abort(void *ctx)
{
	struct iousb_bus *ib = ctx;
	for (int s = 0; s < IBUS_PIPE_SLOTS; s++) {
		if (ib->pipe_intf[s])
			(*ib->pipe_intf[s])->AbortPipe(ib->pipe_intf[s], ib->pipe_ref[s]);
	}
	for (int i = 0; i < 2000 && atomic_load(&ib->pending) > 0; i++)
		usleep(1000);
	int pending = atomic_load(&ib->pending);
	if (pending > 0)
		ibus_log("[IOUSBBus] %d transfers still pending after abort", pending);
}

/* No isochronous transfers and no PCM rings (the harness plays the HAL's part of the shared
 * memory), as the WinUSB harness */
static const struct ozzy_bus_ops ibus_ops = {
	.submit = ibus_submit,
	.control = ibus_control,
	.clear_halt = ibus_clear_halt,
	.start_timer = ibus_start_timer,
	.stop_timer = ibus_stop_timer,
	.sleep_ms = ibus_sleep,
	.time = ibus_time,
	.abort = ibus_abort,
	.time_hz = ibus_time_hz,
};

static void ibus_idle(CFRunLoopTimerRef timer, void *info)
{
	(void)timer;
	(void)info;
}

/* The completion thread: runs the interfaces' async event sources. A far-off timer keeps the
 * run loop alive before any source is added. */
static void *ibus_completion_thread(void *arg)
{
	struct iousb_bus *ib = arg;
	make_realtime(1000, 250);
	CFRunLoopTimerRef keepalive = CFRunLoopTimerCreate(NULL, CFAbsoluteTimeGetCurrent() + 1e9, 1e9, 0, 0,
							   ibus_idle, NULL);
	CFRunLoopAddTimer(CFRunLoopGetCurrent(), keepalive, kCFRunLoopDefaultMode);
	ib->completion_loop = CFRunLoopGetCurrent();
	CFRetain(ib->completion_loop);
	dispatch_semaphore_signal(ib->loop_ready);
	CFRunLoopRun();
	CFRunLoopTimerInvalidate(keepalive);
	CFRelease(keepalive);
	return NULL;
}

io_service_t ibus_find(uint16_t vid, uint16_t pid, int index)
{
	CFMutableDictionaryRef match = IOServiceMatching("IOUSBHostDevice");
	int v = vid, p = pid;
	CFNumberRef nv = CFNumberCreate(NULL, kCFNumberIntType, &v), np = CFNumberCreate(NULL, kCFNumberIntType, &p);
	CFDictionarySetValue(match, CFSTR("idVendor"), nv);
	CFDictionarySetValue(match, CFSTR("idProduct"), np);
	CFRelease(nv);
	CFRelease(np);
	io_iterator_t it = 0;
	if (IOServiceGetMatchingServices(MACH_PORT_NULL, match, &it) != KERN_SUCCESS)
		return 0;
	io_service_t s, found = 0;
	for (int i = 0; (s = IOIteratorNext(it)); i++) {
		if (i == index && !found)
			found = s;
		else
			IOObjectRelease(s);
	}
	IOObjectRelease(it);
	return found;
}

bool ibus_open(struct iousb_bus *ib, io_service_t service, struct ozzy_engine *engine)
{
	ib->bus.ops = &ibus_ops;
	ib->bus.ctx = ib;
	ib->engine = engine;
	ib->service = service;
	IOObjectRetain(service);

	IOCFPlugInInterface **plug = NULL;
	SInt32 score = 0;
	kern_return_t kr = IOCreatePlugInInterfaceForService(service, kIOUSBDeviceUserClientTypeID,
							     kIOCFPlugInInterfaceID, &plug, &score);
	if (kr != KERN_SUCCESS || !plug) {
		ibus_log("[IOUSBBus] no device plug-in: 0x%08x", kr);
		return false;
	}
	HRESULT hr = (*plug)->QueryInterface(plug, CFUUIDGetUUIDBytes(kIOUSBDeviceInterfaceID650), (LPVOID *)&ib->dev);
	IODestroyPlugInInterface(plug);
	if (hr || !ib->dev) {
		ibus_log("[IOUSBBus] no device interface: 0x%08x", (unsigned)hr);
		return false;
	}
	kr = (*ib->dev)->USBDeviceOpen(ib->dev);
	if (kr == kIOReturnExclusiveAccess) {
		ibus_log("[IOUSBBus] device is open elsewhere, seizing it");
		kr = (*ib->dev)->USBDeviceOpenSeize(ib->dev);
	}
	if (kr != kIOReturnSuccess) {
		ibus_log("[IOUSBBus] opening the device failed: 0x%08x", kr);
		(*ib->dev)->Release(ib->dev);
		ib->dev = NULL;
		return false;
	}
	(*ib->dev)->GetDeviceSpeed(ib->dev, &ib->speed);
	(*ib->dev)->USBGetManufacturerStringIndex(ib->dev, &ib->i_manufacturer);
	(*ib->dev)->USBGetProductStringIndex(ib->dev, &ib->i_product);
	(*ib->dev)->USBGetSerialNumberStringIndex(ib->dev, &ib->i_serial);

	ib->loop_ready = dispatch_semaphore_create(0);
	if (pthread_create(&ib->completion_thread, NULL, ibus_completion_thread, ib) != 0)
		return false;
	dispatch_semaphore_wait(ib->loop_ready, DISPATCH_TIME_FOREVER);
	return true;
}

void ibus_close(struct iousb_bus *ib)
{
	ibus_stop_timer(ib);

	/* Anything still in flight (ozzy_engine_stop has aborted what the engine had) */
	ibus_abort(ib);

	for (int i = 0; i < ib->intf_count; i++) {
		if (ib->intf_source[i] && ib->completion_loop)
			CFRunLoopRemoveSource(ib->completion_loop, ib->intf_source[i], kCFRunLoopDefaultMode);
		ib->intf_source[i] = NULL;   /* owned by the interface */
	}
	if (ib->completion_loop) {
		CFRunLoopStop(ib->completion_loop);
		CFRunLoopWakeUp(ib->completion_loop);
		pthread_join(ib->completion_thread, NULL);
		CFRelease(ib->completion_loop);
		ib->completion_loop = NULL;
	}
	for (int s = 0; s < IBUS_PIPE_SLOTS; s++)
		ib->pipe_intf[s] = NULL;
	for (int i = ib->intf_count - 1; i >= 0; i--) {
		if (ib->intf[i]) {
			(*ib->intf[i])->USBInterfaceClose(ib->intf[i]);
			(*ib->intf[i])->Release(ib->intf[i]);
		}
		ib->intf[i] = NULL;
	}
	ib->intf_count = 0;
	if (ib->dev) {
		(*ib->dev)->USBDeviceClose(ib->dev);
		(*ib->dev)->Release(ib->dev);
		ib->dev = NULL;
	}
	if (ib->service) {
		IOObjectRelease(ib->service);
		ib->service = 0;
	}
}

/* Every interface the device has at its current configuration, as interface interfaces. The
 * nubs appear a moment after SetConfiguration, so the caller retries. */
static int ibus_list_interfaces(struct iousb_bus *ib)
{
	IOUSBFindInterfaceRequest req = { kIOUSBFindInterfaceDontCare, kIOUSBFindInterfaceDontCare,
					  kIOUSBFindInterfaceDontCare, kIOUSBFindInterfaceDontCare };
	io_iterator_t it = 0;
	if ((*ib->dev)->CreateInterfaceIterator(ib->dev, &req, &it) != kIOReturnSuccess)
		return 0;
	io_service_t s;
	while ((s = IOIteratorNext(it)) && ib->intf_count < IBUS_MAX_INTERFACES) {
		IOCFPlugInInterface **plug = NULL;
		SInt32 score = 0;
		ibus_interface intf = NULL;
		if (IOCreatePlugInInterfaceForService(s, kIOUSBInterfaceUserClientTypeID, kIOCFPlugInInterfaceID,
						      &plug, &score) == KERN_SUCCESS && plug) {
			(*plug)->QueryInterface(plug, CFUUIDGetUUIDBytes(kIOUSBInterfaceInterfaceID650), (LPVOID *)&intf);
			IODestroyPlugInInterface(plug);
		}
		IOObjectRelease(s);
		if (!intf)
			continue;
		UInt8 number = 0xFF;
		(*intf)->GetInterfaceNumber(intf, &number);
		ib->intf[ib->intf_count] = intf;
		ib->intf_number[ib->intf_count++] = number;
	}
	IOObjectRelease(it);
	return ib->intf_count;
}

bool ibus_configure(struct iousb_bus *ib)
{
	const struct ozzy_device_profile *profile = &ib->engine->profile;

	UInt8 config = 0;
	(*ib->dev)->GetConfiguration(ib->dev, &config);
	if (config != 1) {
		IOReturn kr = (*ib->dev)->SetConfiguration(ib->dev, 1);
		if (kr != kIOReturnSuccess) {
			ibus_log("[IOUSBBus] SetConfiguration(1) failed: 0x%08x", kr);
			return false;
		}
		usleep(500000);   /* as OzzyKext */
	}

	/* The profile's interfaces, waiting up to 2 s for their nubs */
	ibus_interface by_index[4] = { 0 };
	for (int attempt = 0; attempt < 20; attempt++) {
		ibus_list_interfaces(ib);
		int missing = 0;
		for (int i = 0; i < profile->interface_count; i++) {
			by_index[i] = NULL;
			for (int k = 0; k < ib->intf_count; k++) {
				if (ib->intf_number[k] == profile->interfaces[i])
					by_index[i] = ib->intf[k];
			}
			missing += !by_index[i];
		}
		if (!missing)
			break;
		for (int k = 0; k < ib->intf_count; k++)
			(*ib->intf[k])->Release(ib->intf[k]);
		ib->intf_count = 0;
		usleep(100000);
	}
	for (int i = 0; i < profile->interface_count; i++) {
		if (!by_index[i]) {
			ibus_log("[IOUSBBus] interface %u not found", profile->interfaces[i]);
			return false;
		}
	}

	/* Open them, select the alternate setting, and run their completions on our thread */
	for (int k = 0; k < ib->intf_count; k++) {
		ibus_interface intf = ib->intf[k];
		bool wanted = false;
		for (int i = 0; i < profile->interface_count; i++)
			wanted |= by_index[i] == intf;
		if (!wanted)
			continue;
		IOReturn kr = (*intf)->USBInterfaceOpen(intf);
		if (kr == kIOReturnExclusiveAccess)
			kr = (*intf)->USBInterfaceOpenSeize(intf);
		if (kr != kIOReturnSuccess) {
			ibus_log("[IOUSBBus] interface %u: open failed: 0x%08x", ib->intf_number[k], kr);
			return false;
		}
		if (profile->alt_setting) {
			kr = (*intf)->SetAlternateInterface(intf, profile->alt_setting);
			if (kr != kIOReturnSuccess) {
				ibus_log("[IOUSBBus] interface %u: alternate setting %u failed: 0x%08x", ib->intf_number[k],
					 profile->alt_setting, kr);
				return false;
			}
			usleep(50000);
		}
		if ((*intf)->CreateInterfaceAsyncEventSource(intf, &ib->intf_source[k]) != kIOReturnSuccess) {
			ibus_log("[IOUSBBus] interface %u: no async event source", ib->intf_number[k]);
			return false;
		}
		CFRunLoopAddSource(ib->completion_loop, ib->intf_source[k], kCFRunLoopDefaultMode);
		CFRunLoopWakeUp(ib->completion_loop);
	}

	/* Every endpoint of the claimed interfaces to the engine, which picks its pipes. The handle
	 * is the interface's index in ib->intf (bits 8+) and the pipeRef (bits 0-7). */
	struct ozzy_usb_endpoint eps[32];
	unsigned int n = 0;
	for (int k = 0; k < ib->intf_count; k++) {
		ibus_interface intf = ib->intf[k];
		bool wanted = false;
		for (int i = 0; i < profile->interface_count; i++)
			wanted |= by_index[i] == intf;
		UInt8 count = 0;
		if (!wanted || (*intf)->GetNumEndpoints(intf, &count) != kIOReturnSuccess)
			continue;
		for (UInt8 ref = 1; ref <= count && n < 32; ref++) {
			UInt8 dir = 0, number = 0, type = 0, interval = 0;
			UInt16 max_packet = 0;
			if ((*intf)->GetPipeProperties(intf, ref, &dir, &number, &type, &max_packet, &interval) !=
			    kIOReturnSuccess)
				continue;
			eps[n].intf = ib->intf_number[k];
			eps[n].address = (uint8_t)((number & 0x0F) | (dir == kUSBIn ? 0x80 : 0));
			eps[n].attributes = type;   /* kUSBControl/Isoc/Bulk/Interrupt = bmAttributes & 3 */
			eps[n].interval = interval;
			eps[n].max_packet = max_packet;
			eps[n++].handle = (void *)(uintptr_t)((k << 8) | ref);
		}
	}
	const struct ozzy_usb_endpoint *bound[OZZY_PIPE_ROLES];
	ozzy_engine_bind_endpoints(ib->engine, eps, n, bound);
	for (int role = 0; role < OZZY_PIPE_ROLES; role++) {
		if (!bound[role])
			continue;
		uintptr_t h = (uintptr_t)bound[role]->handle;
		int slot = ibus_pipe_slot(bound[role]->address);
		ib->pipe_intf[slot] = ib->intf[h >> 8];
		ib->pipe_ref[slot] = (uint8_t)(h & 0xFF);
		ib->pipe_addr[slot] = bound[role]->address;
		ibus_log("[IOUSBBus] role %d -> endpoint 0x%02x (interface %u, %s, %u bytes)", role,
			 bound[role]->address, bound[role]->intf,
			 (bound[role]->attributes & 3) == 2 ? "bulk" : (bound[role]->attributes & 3) == 3 ? "interrupt" :
			 (bound[role]->attributes & 3) == 1 ? "isochronous" : "control",
			 ozzy_usb_endpoint_bytes(bound[role]));
	}
	return true;
}

bool ibus_get_string(struct iousb_bus *ib, uint8_t index, char *dst, size_t size)
{
	uint8_t buf[256];
	if (!size)
		return false;
	dst[0] = 0;
	if (!index)
		return false;
	IOUSBDevRequestTO r = {
		.bmRequestType = USBmakebmRequestType(kUSBIn, kUSBStandard, kUSBDevice),
		.bRequest = kUSBRqGetDescriptor,
		.wValue = (uint16_t)((kUSBStringDesc << 8) | index),
		.wIndex = 0x0409,
		.wLength = sizeof(buf) - 1,
		.pData = buf,
		.noDataTimeout = 1000,
		.completionTimeout = 1000,
	};
	if ((*ib->dev)->DeviceRequestTO(ib->dev, &r) != kIOReturnSuccess || r.wLenDone < 2)
		return false;
	size_t len = r.wLenDone < buf[0] ? r.wLenDone : buf[0], chars = (len - 2) / 2, i;
	for (i = 0; i < chars && i + 1 < size; i++) {
		uint16_t c = (uint16_t)(buf[2 + 2 * i] | (buf[3 + 2 * i] << 8));
		dst[i] = c < 0x80 ? (char)c : '?';
	}
	dst[i] = 0;
	return true;
}

enum ozzy_usb_speed ibus_speed(const struct iousb_bus *ib)
{
	switch (ib->speed) {
	case kUSBDeviceSpeedLow: return OZZY_USB_SPEED_LOW;
	case kUSBDeviceSpeedFull: return OZZY_USB_SPEED_FULL;
	case kUSBDeviceSpeedHigh: return OZZY_USB_SPEED_HIGH;
	default: return ib->speed > kUSBDeviceSpeedHigh ? OZZY_USB_SPEED_SUPER : OZZY_USB_SPEED_UNKNOWN;
	}
}

struct ibus_stats ibus_take_stats(struct iousb_bus *ib)
{
	uint64_t g_out = atomic_exchange(&ib->max_gap[0], 0), g_in = atomic_exchange(&ib->max_gap[1], 0);
	uint64_t per_us = ns_to_abs(1000);
	struct ibus_stats s = {
		atomic_exchange(&ib->out_done, 0),
		atomic_exchange(&ib->in_done, 0),
		atomic_exchange(&ib->errors, 0),
		(uint32_t)(g_out / (per_us ? per_us : 1)),
		(uint32_t)(g_in / (per_us ? per_us : 1)),
	};
	return s;
}
