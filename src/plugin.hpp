#pragma once
#include <rack.hpp>


using namespace rack;

// Declare the Plugin, defined in plugin.cpp
extern Plugin* pluginInstance;
extern Model* modelTuringMaschine;
extern Model* modelTuringVoltsExpander;
extern Model* modelTuringGateExpander;

struct TuringVoltsExpanderMessage {
	uint8_t bits = 0;
};

// Declare each Model, defined in each module source file
// extern Model* modelMyModule;
