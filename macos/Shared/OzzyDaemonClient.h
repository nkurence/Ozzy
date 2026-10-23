#ifndef OzzyDaemonClient_h
#define OzzyDaemonClient_h

// The XPC protocol between ozzyd (macos/Backends/OzzyDaemon, the userspace backend) and its
// clients, and helpers for the clients: the daemon's counterpart of OzzyKextClient.h. C, so
// OzzyHAL, OzzyMIDI and the C test tools can all use it.
//
// Three requests, mirroring what the plug-ins do with the kext:
//   list            -> "devices": an array of dictionaries, one per device the daemon drives
//                      ("id", "location", "session", "audio", "vid", "pid", "name")
//   attach(id)      -> "shm": the device's shared memory (struct ozzy_shared_memory, the kext's
//                      layout), an xpc_shmem to map with xpc_shmem_map; "size"
//   call(id, method, a0, a1) -> "ret": an IOReturn. The kext's methods (ozzy_shared_data.h):
//                      OZZY_METHOD_SET_RATE (a0: Hz), OZZY_METHOD_APPLY_SETTING (a0: setting id,
//                      a1: value), each a restart of the engine, done when the reply comes;
//                      OZZY_METHOD_MIDI_OUT_PENDING one-way (no reply needed).
// Every reply carries "ret". A device's "session" is its shared memory's session_id, as with
// the kext: a new one means a new mapping.

#include <stdbool.h>
#include <stdint.h>
#include <sys/mman.h>
#include <xpc/xpc.h>

#define OZZYD_SERVICE           "com.ozzy.daemon"

#define OZZYD_KEY_OP            "op"
#define OZZYD_KEY_RET           "ret"
#define OZZYD_KEY_DEVICES       "devices"
#define OZZYD_KEY_ID            "id"
#define OZZYD_KEY_LOCATION      "location"
#define OZZYD_KEY_SESSION       "session"
#define OZZYD_KEY_AUDIO         "audio"
#define OZZYD_KEY_VID           "vid"
#define OZZYD_KEY_PID           "pid"
#define OZZYD_KEY_NAME          "name"
#define OZZYD_KEY_SHM           "shm"
#define OZZYD_KEY_SIZE          "size"
#define OZZYD_KEY_METHOD        "method"
#define OZZYD_KEY_A0            "a0"
#define OZZYD_KEY_A1            "a1"

enum {
	OZZYD_OP_LIST = 1,
	OZZYD_OP_ATTACH = 2,
	OZZYD_OP_CALL = 3,
};

// Test only, and only when ozzyd runs with -T: the device's completion thread sleeps a0 ms
// before its next completion, once (a host that stops serving the device). One-way.
#define OZZYD_METHOD_TEST_STALL 100

// A connection to the daemon, resumed, its events on @queue. @onError gets the XPC error
// (XPC_ERROR_CONNECTION_INTERRUPTED: the daemon went away, mappings stay but go stale;
// XPC_ERROR_CONNECTION_INVALID: there is no such service, or the connection is done).
static inline xpc_connection_t OzzyDaemonConnect(dispatch_queue_t queue, void (^onError)(xpc_object_t error)) {
	xpc_connection_t c = xpc_connection_create_mach_service(OZZYD_SERVICE, queue, 0);
	if (!c) return NULL;
	xpc_connection_set_event_handler(c, ^(xpc_object_t event) {
		if (xpc_get_type(event) == XPC_TYPE_ERROR && onError) onError(event);
	});
	xpc_connection_resume(c);
	return c;
}

// The reply to @message (released by the caller), or NULL if the daemon didn't answer
static inline xpc_object_t OzzyDaemonRequest(xpc_connection_t c, xpc_object_t message) {
	xpc_object_t reply = xpc_connection_send_message_with_reply_sync(c, message);
	if (reply && xpc_get_type(reply) != XPC_TYPE_DICTIONARY) {
		xpc_release(reply);
		return NULL;
	}
	return reply;
}

// The devices the daemon drives (an XPC array of dictionaries, released by the caller), or NULL
static inline xpc_object_t OzzyDaemonList(xpc_connection_t c) {
	xpc_object_t m = xpc_dictionary_create(NULL, NULL, 0);
	xpc_dictionary_set_uint64(m, OZZYD_KEY_OP, OZZYD_OP_LIST);
	xpc_object_t reply = OzzyDaemonRequest(c, m);
	xpc_release(m);
	if (!reply) return NULL;
	xpc_object_t devices = xpc_dictionary_get_value(reply, OZZYD_KEY_DEVICES);
	if (devices && xpc_get_type(devices) == XPC_TYPE_ARRAY) xpc_retain(devices);
	else devices = NULL;
	xpc_release(reply);
	return devices;
}

// Device @id's shared memory, mapped into this process (munmap(address, *size) when done), or NULL
static inline void *OzzyDaemonAttach(xpc_connection_t c, uint64_t id, size_t *size) {
	xpc_object_t m = xpc_dictionary_create(NULL, NULL, 0);
	xpc_dictionary_set_uint64(m, OZZYD_KEY_OP, OZZYD_OP_ATTACH);
	xpc_dictionary_set_uint64(m, OZZYD_KEY_ID, id);
	xpc_object_t reply = OzzyDaemonRequest(c, m);
	xpc_release(m);
	if (!reply) return NULL;
	void *address = NULL;
	xpc_object_t shm = xpc_dictionary_get_value(reply, OZZYD_KEY_SHM);
	if (shm && xpc_get_type(shm) == XPC_TYPE_SHMEM) {
		size_t n = xpc_shmem_map(shm, &address);
		if (n) *size = n;
		else address = NULL;
	}
	xpc_release(reply);
	return address;
}

// A method on device @id; its IOReturn (kIOReturnNotResponding, 0xe00002ed, if no reply came)
static inline int64_t OzzyDaemonCall(xpc_connection_t c, uint64_t id, uint64_t method, uint64_t a0, uint64_t a1) {
	xpc_object_t m = xpc_dictionary_create(NULL, NULL, 0);
	xpc_dictionary_set_uint64(m, OZZYD_KEY_OP, OZZYD_OP_CALL);
	xpc_dictionary_set_uint64(m, OZZYD_KEY_ID, id);
	xpc_dictionary_set_uint64(m, OZZYD_KEY_METHOD, method);
	xpc_dictionary_set_uint64(m, OZZYD_KEY_A0, a0);
	xpc_dictionary_set_uint64(m, OZZYD_KEY_A1, a1);
	xpc_object_t reply = OzzyDaemonRequest(c, m);
	xpc_release(m);
	if (!reply) return (int64_t)(int32_t)0xe00002ed;
	int64_t ret = xpc_dictionary_get_int64(reply, OZZYD_KEY_RET);
	xpc_release(reply);
	return ret;
}

// The same, one-way: no reply, doesn't block (OZZY_METHOD_MIDI_OUT_PENDING, OZZYD_METHOD_TEST_STALL)
static inline void OzzyDaemonSend(xpc_connection_t c, uint64_t id, uint64_t method, uint64_t a0, uint64_t a1) {
	xpc_object_t m = xpc_dictionary_create(NULL, NULL, 0);
	xpc_dictionary_set_uint64(m, OZZYD_KEY_OP, OZZYD_OP_CALL);
	xpc_dictionary_set_uint64(m, OZZYD_KEY_ID, id);
	xpc_dictionary_set_uint64(m, OZZYD_KEY_METHOD, method);
	xpc_dictionary_set_uint64(m, OZZYD_KEY_A0, a0);
	xpc_dictionary_set_uint64(m, OZZYD_KEY_A1, a1);
	xpc_connection_send_message(c, m);
	xpc_release(m);
}

#endif
