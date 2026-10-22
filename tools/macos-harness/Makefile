# ozzy-harness for macOS: an Ozzy engine over IOUSBLib, from user space (dev tool, not shipped).
#   make            build ./ozzy-harness
#   make clean
# Needs the Xcode command line tools. No kext, no SIP change, no signing identity: the harness
# is ad-hoc signed by the linker, and IOUSBLib needs no entitlement outside a sandbox.

OZZY    := ../..
CC      ?= cc
CFLAGS  ?= -std=gnu11 -O2 -g -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
CPPFLAGS += -DOZZY_USER_ENGINE -I$(OZZY)
# PCM IN transfers in flight up to 8 (-Q): margin for a userspace host (common/ozzy_settings.h)
CPPFLAGS += -DOZZY_NUM_PCM_IN_QUEUES=8
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
           $(OZZY)/common/devices/*/*.h) iousb_bus.h

ozzy-harness: main.c iousb_bus.c $(ENGINE) $(HEADERS)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ main.c iousb_bus.c $(ENGINE) $(LDLIBS)

clean:
	rm -f ozzy-harness
	rm -rf ozzy-harness.dSYM

.PHONY: clean
