/* SPDX-License-Identifier: MIT */
#ifndef IOUSB_BUS_H
#define IOUSB_BUS_H

/* The engine's bus (engine/core/ozzy_bus.h) over IOUSBLib, for the macOS dev harness: the
 * engine runs in a command-line program and talks to the device from user space, with no
 * kext and SIP untouched. Not shipped (the macOS driver is the kext); it is also the
 * measurement for whether a userspace backend can stream cleanly.
 *
 * Transfers are IOUSBLib async pipe calls. Every completion is delivered on one real-time
 * thread that runs the interfaces' run loop sources, so engine callbacks never run
 * concurrently with each other (as on the kext's work loop and the WinUSB harness's port).
 * No isochronous transfers: the Ploytec bulk/interrupt family doesn't stream them. */

#include <stdatomic.h>
#include <pthread.h>
#include <dispatch/dispatch.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/usb/IOUSBLib.h>
#include "../../engine/core/ozzy_engine.h"

enum {
	IBUS_MAX_INTERFACES = 8,
	IBUS_PIPE_SLOTS = 32,   /* endpoint number in bits 0-3, direction in bit 4 (as KextBus) */
};

typedef IOUSBDeviceInterface650 **ibus_device;
typedef IOUSBInterfaceInterface650 **ibus_interface;

/* Zero it, then ibus_open */
struct iousb_bus {
	struct ozzy_bus bus;    /* what the engine runs on (ozzy_engine_attach) */
	struct ozzy_engine *engine;

	io_service_t service;
	ibus_device dev;
	uint8_t speed;                                   /* kUSBDeviceSpeed* */
	uint8_t i_manufacturer, i_product, i_serial;     /* string descriptor indexes */
	ibus_interface intf[IBUS_MAX_INTERFACES];
	uint8_t intf_number[IBUS_MAX_INTERFACES];        /* bInterfaceNumber of each */
	CFRunLoopSourceRef intf_source[IBUS_MAX_INTERFACES];
	int intf_count;
	ibus_interface pipe_intf[IBUS_PIPE_SLOTS];       /* interface per endpoint */
	uint8_t pipe_ref[IBUS_PIPE_SLOTS];               /* IOUSBLib pipeRef per endpoint */
	uint8_t pipe_addr[IBUS_PIPE_SLOTS];

	pthread_t completion_thread;
	CFRunLoopRef completion_loop;
	dispatch_semaphore_t loop_ready;
	atomic_int pending;
	atomic_uint out_done, in_done, errors;
	uint64_t last_done[2];                 /* mach time of the last completion, [0] out, [1] in */
	atomic_uint_fast64_t max_gap[2];       /* longest gap since the last ibus_take_stats, mach ticks */

	pthread_t timer_thread;
	atomic_bool timer_on;
	bool timer_running;
	uint32_t timer_interval_us;
};

/* Completed transfers and failures per direction since the last call, and the longest gap
 * between two completions of a direction in that time (harness stats) */
struct ibus_stats {
	uint32_t out_done, in_done, errors;
	uint32_t max_gap_out_us, max_gap_in_us;
};

/* The first present device with this vendor/product ID, or 0 (release it with IOObjectRelease) */
io_service_t ibus_find(uint16_t vid, uint16_t pid, int index);

/* Open @service (takes a reference) and start the completion thread */
bool ibus_open(struct iousb_bus *ib, io_service_t service, struct ozzy_engine *engine);
/* Abort every pipe, wait for outstanding transfers, close everything. Safe to call twice. */
void ibus_close(struct iousb_bus *ib);

/* What OzzyKext::ConfigureHardware does: configuration 1, claim the profile's interfaces,
 * select the alternate setting, hand every endpoint to ozzy_engine_bind_endpoints */
bool ibus_configure(struct iousb_bus *ib);

/* String descriptor @index (US English) as ASCII into @dst; false if there is none */
bool ibus_get_string(struct iousb_bus *ib, uint8_t index, char *dst, size_t size);

/* The connection speed as enum ozzy_usb_speed */
enum ozzy_usb_speed ibus_speed(const struct iousb_bus *ib);

struct ibus_stats ibus_take_stats(struct iousb_bus *ib);

void ibus_log(const char *fmt, ...) OZZY_PRINTF_FORMAT(1, 2);

#endif
