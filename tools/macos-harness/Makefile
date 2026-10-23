# ozzy-harness for macOS: an Ozzy engine over IOUSBLib, from user space (dev tool, not shipped),
# and ozzyd-client, which plays the same HAL part against ozzyd (macos/Backends/OzzyDaemon).
#   make            build ./ozzy-harness and ./ozzyd-client
#   make clean
# Needs the Xcode command line tools. No kext, no SIP change, no signing identity: the tools are
# ad-hoc signed by the linker, and IOUSBLib needs no entitlement outside a sandbox.

OZZY    := ../..
DAEMON  := $(OZZY)/macos/Backends/OzzyDaemon
CC      ?= cc
CFLAGS  ?= -std=gnu11 -O2 -g -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
# ozzyd-client too: the C view of the shared memory (the engine side's), the same layout as
# OzzyHAL's C++ one
CPPFLAGS += -DOZZY_USER_ENGINE -I$(OZZY)
# PCM transfers in flight up to 8 each way (-Q in, -O out), for measuring what a userspace host
# needs (common/ozzy_settings.h); the defaults stay 4
CPPFLAGS += -DOZZY_NUM_PCM_IN_QUEUES=8 -DOZZY_NUM_PCM_OUT_QUEUES=8
GIT_VERSION := $(shell git -C $(OZZY) describe --always --dirty 2>/dev/null)
ifneq ($(GIT_VERSION),)
CPPFLAGS += -DOZZY_GIT_VERSION='"$(GIT_VERSION)"'
endif
LDLIBS  += -framework IOKit -framework CoreFoundation -lm

CODEC   := $(OZZY)/common/devices/ploytec/ploytec_codec.c
ENGINE  := $(OZZY)/engine/core/ozzy_engine.c $(OZZY)/engine/core/ozzy_isoc_output.c \
           $(OZZY)/engine/devices/ploytec/ploytec_engine.c $(OZZY)/engine/devices/ploytec/ploytec_io.c \
           $(OZZY)/engine/devices/ploytec/ploytec_isoc_engine.c $(OZZY)/engine/devices/usbmidi/usb_midi_engine.c \
           $(OZZY)/common/devices/ploytec/ploytec_handshake.c $(CODEC) \
           $(OZZY)/common/usb_midi.c
HEADERS := $(wildcard $(OZZY)/engine/*/*.h $(OZZY)/engine/devices/*/*.h $(OZZY)/common/*.h \
           $(OZZY)/common/devices/*/*.h) $(DAEMON)/iousb_bus.h hal_role.h

all: ozzy-harness ozzyd-client

ozzy-harness: main.c hal_role.c $(DAEMON)/iousb_bus.c $(ENGINE) $(HEADERS)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ main.c hal_role.c $(DAEMON)/iousb_bus.c $(ENGINE) $(LDLIBS)

ozzyd-client: ozzyd-client.c hal_role.c $(CODEC) $(HEADERS) $(OZZY)/macos/Shared/OzzyDaemonClient.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ ozzyd-client.c hal_role.c $(CODEC) -lm

clean:
	rm -f ozzy-harness ozzyd-client
	rm -rf ozzy-harness.dSYM ozzyd-client.dSYM

.PHONY: all clean
