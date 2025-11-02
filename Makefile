# If RACK_DIR is not defined when calling the Makefile, default to two directories above
RACK_DIR ?= ../../rack-sdk

# FLAGS will be passed to both the C and C++ compiler
FLAGS +=
CFLAGS +=
CXXFLAGS += -Idep -Idep/NeuralAmpModelerCore -Idep/NeuralAmpModelerCore/NAM -Idep/NeuralAmpModelerCore/Dependencies -Idep/NeuralAmpModelerCore/Dependencies/nlohmann
CXXFLAGS += -Idep/eigen3 -I$(RACK_DIR)/dep/include -I$(RACK_DIR)/dep/include/eigen3
CXXFLAGS += -Idep/SabnockOTT/Source -Idep/SabnockOTT/Source/vital_dsp


# Careful about linking to shared libraries, since you can't assume much about the user's environment and library search path.
# Static libraries are fine, but they should be added to this plugin's build system.
LDFLAGS +=

# Add .cpp files to the build
SOURCES += $(wildcard src/*.cpp)
SOURCES += $(wildcard src/dsp/*.cpp)
SOURCES += $(wildcard dep/NeuralAmpModelerCore/NAM/*.cpp)
SOURCES += dep/SabnockOTT/Source/vital_dsp/compressor.cpp
SOURCES += dep/SabnockOTT/Source/vital_dsp/framework/operators.cpp
SOURCES += dep/SabnockOTT/Source/vital_dsp/framework/processor.cpp
SOURCES += dep/SabnockOTT/Source/vital_dsp/framework/utils.cpp
SOURCES += dep/SabnockOTT/Source/vital_dsp/framework/value.cpp
SOURCES += dep/SabnockOTT/Source/vital_dsp/linkwitz_riley_filter.cpp
SOURCES += dep/SabnockOTT/Source/vital_dsp/utilities/smooth_value.cpp

# Add files to the ZIP package when running `make dist`
# The compiled plugin and "plugin.json" are automatically added.
DISTRIBUTABLES += res
DISTRIBUTABLES += $(wildcard LICENSE*)
DISTRIBUTABLES += $(wildcard presets)

# Include the Rack plugin Makefile framework
include $(RACK_DIR)/plugin.mk
CXXFLAGS := $(filter-out -std=c++11,$(CXXFLAGS))
CXXFLAGS += -std=c++17

# Link filesystem library (needed for C++17 std::filesystem on MinGW)
LDFLAGS += -lstdc++fs
# Statically link pthread to avoid DLL dependencies on Windows
# Must come AFTER plugin.mk to override any settings
# Force static linking of pthread before dynamic linking resumes
LDFLAGS := $(filter-out -lpthread,$(LDFLAGS))
LDFLAGS += -Wl,-Bstatic -lpthread -Wl,-Bdynamic