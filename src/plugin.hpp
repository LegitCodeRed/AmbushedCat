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
extern Model* modelKabaddon;
extern Model* modelAndras;
extern Model* modelAhriman;
extern Model* modelLeviathan;
extern Model* modelSitri;
extern Model* modelLilith;
extern Model* modelLilithAdvance;
extern Model* modelNergalAmp;
extern Model* modelBuer;
extern Model* modelXezbeth4X;
extern Model* modelSabnockOTT;
extern Model* modelUsbSync;

struct TuringVoltsExpanderMessage {
	uint8_t bits = 0;
};


