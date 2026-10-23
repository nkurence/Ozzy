# ozzyd for macOS: Ozzy's userspace backend (ozzyd.c), an engine per device over IOUSBLib.
#   make                    build ./ozzyd
#   make hal                build build/OzzyHAL.driver, the HAL plug-in, without Xcode (the Xcode
#                           project builds the same sources), with the daemon's queue ceilings
#   ./ozzyd-agent.sh load   run it as a LaunchAgent (no admin; unload stops it), for a test client
#                           (tools/macos-harness/ozzyd-client)
#   sudo ./install-daemon.sh   install ozzyd as a LaunchDaemon and the HAL plug-in (uninstall: the
#                           same with "uninstall")
#   make clean
# Needs the Xcode command line tools; ad-hoc signed.

OZZY    := ../../..
CC      ?= cc
CXX     ?= c++
CFLAGS  ?= -std=gnu11 -O2 -g -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
# PCM transfers in flight up to 8 each way, as the macOS harness: the WZ4 at 96 kHz needs 8 x 200
# frames out to ride out a 14 ms stall (measured); the defaults stay 4 (common/ozzy_settings.h).
# The HAL takes the settings' value counts from the same macros, so it is built with them too.
QUEUES  := -DOZZY_NUM_PCM_IN_QUEUES=8 -DOZZY_NUM_PCM_OUT_QUEUES=8
GIT_VERSION := $(shell git -C $(OZZY) describe --always --dirty 2>/dev/null)
ifneq ($(GIT_VERSION),)
VERSION := -DOZZY_GIT_VERSION='"$(GIT_VERSION)"'
endif
CPPFLAGS += -DOZZY_USER_ENGINE -I$(OZZY) $(QUEUES) $(VERSION)
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

# The HAL plug-in (the client side of the shared memory: no OZZY_USER_ENGINE)
HAL      := $(OZZY)/macos/OzzyHAL
HALFLAGS := -O2 -g -Wall -I$(OZZY) $(QUEUES) $(VERSION)
HALDEPS  := $(HAL)/OzzyHAL.mm $(HAL)/OzzyHAL.h $(HAL)/PloytecCodec.cpp $(HAL)/PloytecCodec.h $(HAL)/Info.plist \
            $(OZZY)/common/devices/ploytec/ploytec_codec.c $(OZZY)/macos/Shared/OzzyDaemonClient.h \
            $(OZZY)/macos/Shared/OzzyKextClient.h $(OZZY)/macos/Shared/OzzyLog.h $(HEADERS)

build/OzzyHAL.driver: $(HALDEPS)
	rm -rf $@ build/hal-obj
	mkdir -p $@/Contents/MacOS build/hal-obj
	sed 's/$$(PRODUCT_BUNDLE_IDENTIFIER)/OzzyHAL/' $(HAL)/Info.plist > $@/Contents/Info.plist
	$(CC) -c -std=gnu11 $(HALFLAGS) -o build/hal-obj/ploytec_codec.o $(OZZY)/common/devices/ploytec/ploytec_codec.c
	$(CXX) -c -std=gnu++11 $(HALFLAGS) -o build/hal-obj/PloytecCodec.o $(HAL)/PloytecCodec.cpp
	$(CXX) -c -std=gnu++11 $(HALFLAGS) -o build/hal-obj/OzzyHAL.o $(HAL)/OzzyHAL.mm
	$(CXX) -bundle -o $@/Contents/MacOS/OzzyHAL build/hal-obj/*.o \
		-framework CoreAudio -framework CoreFoundation -framework IOKit
	codesign --force --sign - $@

hal: build/OzzyHAL.driver

clean:
	rm -f ozzyd
	rm -rf ozzyd.dSYM build

.PHONY: hal clean
