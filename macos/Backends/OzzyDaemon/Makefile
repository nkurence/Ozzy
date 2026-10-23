# ozzyd for macOS: Ozzy's userspace backend (ozzyd.c), an engine per device over IOUSBLib.
#   make                    build ./ozzyd
#   ./ozzyd-agent.sh load   run it as a LaunchAgent (no admin; unload stops it), for a test client
#                           (tools/macos-harness/ozzyd-client)
#   make clean
# Needs the Xcode command line tools; ad-hoc signed by the linker.

OZZY    := ../../..
CC      ?= cc
CFLAGS  ?= -std=gnu11 -O2 -g -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
CPPFLAGS += -DOZZY_USER_ENGINE -I$(OZZY)
# PCM transfers in flight up to 8 each way, as the macOS harness: the WZ4 at 96 kHz needs 8 x 200
# frames out to ride out a 14 ms stall (measured); the defaults stay 4 (common/ozzy_settings.h)
CPPFLAGS += -DOZZY_NUM_PCM_IN_QUEUES=8 -DOZZY_NUM_PCM_OUT_QUEUES=8
GIT_VERSION := $(shell git -C $(OZZY) describe --always --dirty 2>/dev/null)
ifneq ($(GIT_VERSION),)
CPPFLAGS += -DOZZY_GIT_VERSION='"$(GIT_VERSION)"'
endif
LDLIBS  += -framework IOKit -framework CoreFoundation -lm

ENGINE  := $(OZZY)/engine/core/ozzy_engine.c $(OZZY)/engine/core/ozzy_isoc_output.c \
           $(OZZY)/engine/devices/ploytec/ploytec_engine.c $(OZZY)/engine/devices/ploytec/ploytec_io.c \
           $(OZZY)/engine/devices/ploytec/ploytec_isoc_engine.c $(OZZY)/engine/devices/usbmidi/usb_midi_engine.c \
           $(OZZY)/common/devices/ploytec/ploytec_handshake.c $(OZZY)/common/devices/ploytec/ploytec_codec.c \
           $(OZZY)/common/usb_midi.c
HEADERS := $(wildcard $(OZZY)/engine/*/*.h $(OZZY)/engine/devices/*/*.h $(OZZY)/common/*.h \
           $(OZZY)/common/devices/*/*.h) iousb_bus.h $(OZZY)/macos/Shared/OzzyDaemonClient.h

ozzyd: ozzyd.c iousb_bus.c $(ENGINE) $(HEADERS)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ ozzyd.c iousb_bus.c $(ENGINE) $(LDLIBS)

clean:
	rm -f ozzyd
	rm -rf ozzyd.dSYM build

.PHONY: clean
