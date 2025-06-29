#include "plugin.hpp"


Plugin* pluginInstance;

void init(Plugin* p) {
	pluginInstance = p;
 	p->addModel(modelTuringMaschine);
 	p->addModel(modelTuringGateExpander);
 	p->addModel(modelTuringVoltsExpander);
	p->addModel(modelTape);
	// Add modules here
	// p->addModel(modelMyModule);

	// Any other plugin initialization may go here.
	// As an alternative, consider lazy-loading assets and lookup tables when your module is created to reduce startup times of Rack.
}
