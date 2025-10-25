# If RACK_DIR is not defined when calling the Makefile, default to two directories above
RACK_DIR ?= ../../rack-sdk

# FLAGS will be passed to both the C and C++ compiler
FLAGS +=
CFLAGS +=
CXXFLAGS += -INeuralAmpModelerCore -INeuralAmpModelerCore/NAM -INeuralAmpModelerCore/Dependencies -INeuralAmpModelerCore/Dependencies/nlohmann
CXXFLAGS += -I$(RACK_DIR)/dep/include -I$(RACK_DIR)/dep/include/eigen3

# Careful about linking to shared libraries, since you can't assume much about the user's environment and library search path.
# Static libraries are fine, but they should be added to this plugin's build system.
LDFLAGS +=

# Add .cpp files to the build
SOURCES += $(wildcard src/*.cpp)
SOURCES += $(wildcard src/dsp/*.cpp)
SOURCES += $(wildcard NeuralAmpModelerCore/NAM/*.cpp)

# Add files to the ZIP package when running `make dist`
# The compiled plugin and "plugin.json" are automatically added.
DISTRIBUTABLES += res
DISTRIBUTABLES += $(wildcard LICENSE*)
DISTRIBUTABLES += $(wildcard presets)

# Include the Rack plugin Makefile framework
include $(RACK_DIR)/plugin.mk
