# If RACK_DIR is not defined when calling the Makefile, default to two directories above
RACK_DIR ?= ../../rack-sdk

# FLAGS will be passed to both the C and C++ compiler
FLAGS +=
CFLAGS +=
CXXFLAGS += -Isrc/compiledDepencies -Isrc/compiledDepencies/NeuralAmpModelerCore -Isrc/compiledDepencies/NeuralAmpModelerCore/NAM -Isrc/compiledDepencies/NeuralAmpModelerCore/Dependencies -Isrc/compiledDepencies/NeuralAmpModelerCore/Dependencies/nlohmann
CXXFLAGS += -Isrc/compiledDepencies/eigen3 -I$(RACK_DIR)/dep/include -I$(RACK_DIR)/dep/include/eigen3
CXXFLAGS += -Isrc/compiledDepencies/vital/src/synthesis -Isrc/compiledDepencies/vital/src/synthesis/framework -Isrc/compiledDepencies/vital/src/synthesis/effects -Isrc/compiledDepencies/vital/src/synthesis/filters -Isrc/compiledDepencies/vital/src/synthesis/utilities -Isrc/compiledDepencies/vital/src/common
CXXFLAGS += -Isrc/compiledDepencies/vital/headless/JuceLibraryCode


# Careful about linking to shared libraries, since you can't assume much about the user's environment and library search path.
# Static libraries are fine, but they should be added to this plugin's build system.
LDFLAGS +=

# Add .cpp files to the build
SOURCES += $(wildcard src/*.cpp)
SOURCES += $(wildcard src/dsp/*.cpp)
SOURCES += $(wildcard src/compiledDepencies/NeuralAmpModelerCore/NAM/*.cpp)
# Vital DSP sources
SOURCES += src/compiledDepencies/vital/src/synthesis/effects/compressor.cpp
SOURCES += src/compiledDepencies/vital/src/synthesis/effects/distortion.cpp
SOURCES += src/compiledDepencies/vital/src/synthesis/framework/operators.cpp
SOURCES += src/compiledDepencies/vital/src/synthesis/framework/processor.cpp
SOURCES += src/compiledDepencies/vital/src/synthesis/framework/processor_router.cpp
SOURCES += src/compiledDepencies/vital/src/synthesis/framework/utils.cpp
SOURCES += src/compiledDepencies/vital/src/synthesis/framework/value.cpp
SOURCES += src/compiledDepencies/vital/src/synthesis/filters/linkwitz_riley_filter.cpp
SOURCES += src/compiledDepencies/vital/src/synthesis/utilities/smooth_value.cpp

# Add files to the ZIP package when running `make dist`
# The compiled plugin and "plugin.json" are automatically added.
DISTRIBUTABLES += res
DISTRIBUTABLES += $(wildcard LICENSE*)
DISTRIBUTABLES += $(wildcard presets)

# Include the Rack plugin Makefile framework
include $(RACK_DIR)/plugin.mk
CXXFLAGS := $(filter-out -std=c++11,$(CXXFLAGS))
CXXFLAGS += -std=c++17

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
ifndef ARCH_MAC
	LDFLAGS += -lstdc++fs
endif
# Statically link pthread to avoid DLL dependencies on Windows only
# Must come AFTER plugin.mk to override any settings
# On Linux, pthread must be dynamically linked for shared objects
ifdef ARCH_WIN
	LDFLAGS := $(filter-out -lpthread,$(LDFLAGS))
	LDFLAGS += -Wl,-Bstatic -lpthread -Wl,-Bdynamic
endif

# Override cleandep to prevent removal of git-tracked dependencies
# The rack-plugin-toolchain's cleandep runs "rm -rfv dep" which would delete our dependencies
# All our deps are now in src/compiledDepencies and tracked in git:
# - src/compiledDepencies/vital (vendored dependency)
# - src/compiledDepencies/NeuralAmpModelerCore (vendored dependency)
# - src/compiledDepencies/eigen3 (vendored dependency)
# Since all our deps are in git, we skip cleandep entirely
.PHONY: cleandep
cleandep:
	@echo "Skipping cleandep: all dependencies are tracked in git under src/compiledDepencies/"