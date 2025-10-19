#pragma once
#include <rack.hpp>


using namespace rack;

// Declare the Plugin, defined in plugin.cpp
extern Plugin* pluginInstance;
// Declare each Model, defined in each module source file
// extern Model* modelMyModule;
extern Model* modelTuringMaschine;
extern Model* modelTuringVoltsExpander;
extern Model* modelTuringGateExpander;
extern Model* modelTape;
extern Model* modelBasimilusIteritasAlia;
extern Model* modelAtaraxicIteritasAlia;
extern Model* modelAhriman;

struct TuringVoltsExpanderMessage {
	uint8_t bits = 0;
};


