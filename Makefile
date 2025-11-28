# If RACK_DIR is not defined when calling the Makefile, default to two directories above
RACK_DIR ?= ../../rack-sdk

# FLAGS will be passed to both the C and C++ compiler
FLAGS += -pipe
CFLAGS +=
CXXFLAGS += -Idep -Idep/NeuralAmpModelerCore -Idep/NeuralAmpModelerCore/NAM -Idep/NeuralAmpModelerCore/Dependencies -Idep/NeuralAmpModelerCore/Dependencies/nlohmann
CXXFLAGS += -Idep/eigen3 -I$(RACK_DIR)/dep/include -I$(RACK_DIR)/dep/include/eigen3
CXXFLAGS += -Idep/vital/src/synthesis -Idep/vital/src/synthesis/framework -Idep/vital/src/synthesis/effects -Idep/vital/src/synthesis/filters -Idep/vital/src/synthesis/utilities -Idep/vital/src/common
CXXFLAGS += -Idep/vital/headless/JuceLibraryCode
CXXFLAGS += -Idep/link/include -Idep/link/modules/asio-standalone/asio/include
CXXFLAGS += -Idep/rtmidi


# Careful about linking to shared libraries, since you can't assume much about the user's environment and library search path.
# Static libraries are fine, but they should be added to this plugin's build system.
LDFLAGS +=

# Add .cpp files to the build
SOURCES += $(wildcard src/*.cpp)
SOURCES += $(wildcard src/dsp/*.cpp)
SOURCES += $(wildcard dep/NeuralAmpModelerCore/NAM/*.cpp)
# Vital DSP sources
SOURCES += dep/vital/src/synthesis/effects/compressor.cpp
SOURCES += dep/vital/src/synthesis/effects/distortion.cpp
SOURCES += dep/vital/src/synthesis/framework/operators.cpp
SOURCES += dep/vital/src/synthesis/framework/processor.cpp
SOURCES += dep/vital/src/synthesis/framework/processor_router.cpp
SOURCES += dep/vital/src/synthesis/framework/utils.cpp
SOURCES += dep/vital/src/synthesis/framework/value.cpp
SOURCES += dep/vital/src/synthesis/filters/linkwitz_riley_filter.cpp
SOURCES += dep/vital/src/synthesis/utilities/smooth_value.cpp
# RtMidi sources
SOURCES += dep/rtmidi/RtMidi.cpp

# Add files to the ZIP package when running `make dist`
# The compiled plugin and "plugin.json" are automatically added.
DISTRIBUTABLES += res
DISTRIBUTABLES += $(wildcard LICENSE*)
DISTRIBUTABLES += $(wildcard presets)

# Include the Rack plugin Makefile framework
include $(RACK_DIR)/plugin.mk

# Platform-specific MIDI API flags and Link platform detection
# IMPORTANT: Must come AFTER plugin.mk include (which defines ARCH_WIN, ARCH_MAC, ARCH_LIN)
ifdef ARCH_WIN
	# Windows: Use WinMM MIDI API
	CXXFLAGS += -D__WINDOWS_MM__
	LDFLAGS += -lwinmm
	# Link platform detection
	CXXFLAGS += -DLINK_PLATFORM_WINDOWS
	LDFLAGS += -lws2_32 -liphlpapi
endif
ifdef ARCH_MAC
	# macOS: Use CoreMIDI
	CXXFLAGS += -D__MACOSX_CORE__
	LDFLAGS += -framework CoreMIDI -framework CoreAudio -framework CoreFoundation
	# Link platform detection
	CXXFLAGS += -DLINK_PLATFORM_MACOSX
endif
ifdef ARCH_LIN
	# Linux: Use ALSA
	CXXFLAGS += -D__LINUX_ALSA__
	LDFLAGS += -lasound
	# Link platform detection
	CXXFLAGS += -DLINK_PLATFORM_LINUX
endif

# Upgrade to C++20
CXXFLAGS := $(filter-out -std=c++11,$(CXXFLAGS))
CXXFLAGS := $(filter-out -std=c++17,$(CXXFLAGS))
CXXFLAGS += -std=c++20

# Override macOS deployment target for C++17 filesystem support
# NAM library requires std::filesystem which is only available on macOS 10.15+
ifdef ARCH_MAC
	FLAGS := $(patsubst -mmacosx-version-min=10.9,-mmacosx-version-min=10.15,$(FLAGS))
	CFLAGS := $(patsubst -mmacosx-version-min=10.9,-mmacosx-version-min=10.15,$(CFLAGS))
	CXXFLAGS := $(patsubst -mmacosx-version-min=10.9,-mmacosx-version-min=10.15,$(CXXFLAGS))
	LDFLAGS := $(patsubst -mmacosx-version-min=10.9,-mmacosx-version-min=10.15,$(LDFLAGS))
endif

# Link filesystem library (needed for C++17 std::filesystem on Windows/Linux)
# macOS has it built into libc++ on 10.15+, doesn't need separate lib
# DISABLED: With C++20 and modern GCC, filesystem is integrated into libstdc++
# The separate -lstdc++fs causes c0000139 errors in VCV Rack environment
# ifndef ARCH_MAC
# 	LDFLAGS += -lstdc++fs
# endif
# Note: plugin.mk already sets -static-libstdc++ for Windows
# We rely on Rack's bundled runtime DLLs (libgcc_s_seh-1.dll, libwinpthread-1.dll)
# which are loaded from the Rack.exe directory when the plugin loads

# Override cleandep to prevent removal of git-tracked dependencies
# The rack-plugin-toolchain's cleandep runs "rm -rfv dep" which would delete:
# - dep/vital (git submodule)
# - dep/NeuralAmpModelerCore (vendored dependency)
# - dep/eigen3 (vendored dependency)
# Since all our deps are in git, we skip cleandep entirely
.PHONY: cleandep
cleandep:
	@echo "Skipping cleandep: all dependencies are tracked in git (dep/vital, dep/NeuralAmpModelerCore, dep/eigen3)"

# Ensure submodules are initialized when dep target is called
# In the VCV Rack Plugin Toolchain, dependencies are already included
# so we only run git submodule commands if .git directory exists
.PHONY: dep
dep:
	@if [ -d .git ]; then \
		git config --global --add safe.directory '*' || true; \
		git submodule update --init --recursive; \
		echo "Git submodules initialized"; \
	else \
		echo "No .git directory found, skipping submodule initialization (dependencies already present)"; \
	fi