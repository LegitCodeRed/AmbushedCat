#include "plugin.hpp"


struct TuringVoltsExpander : Module {
	enum ParamIds {
		BIT_0_PARAM,
		BIT_1_PARAM,
		BIT_2_PARAM,
		BIT_3_PARAM,
		BIT_4_PARAM,
		NUM_PARAMS
	};
	enum OutputIds {
		VOLTS_OUTPUT,
		VOLTSINV_OUTPUT,
		NUM_OUTPUTS
	};

	// Message struct shared with main module
	TuringVoltsExpanderMessage message;

	TuringVoltsExpander() {
		config(NUM_PARAMS, 0, NUM_OUTPUTS, 0);
		for (int i = 0; i < 5; i++) {
			configParam(i, 0.f, 10.f, 0.f, "Bit " + std::to_string(i) + " voltage");
		}

		configOutput(VOLTS_OUTPUT, "CV Out");
		configOutput(VOLTSINV_OUTPUT, "CV Inverted Out");
	}

	void process(const ProcessArgs& args) override {
		uint8_t bits = 0;
		
		Module* mod = this;
		while (mod->leftExpander.module) {
			mod = mod->leftExpander.module;

			if (mod->model && mod->model->slug == "TuringMaschine") {
				if (getLeftExpander().consumerMessage) {
					auto* msg = static_cast<TuringVoltsExpanderMessage*>(getLeftExpander().consumerMessage);
					bits = msg->bits;
				}
				break;
			}
		}

		
		// Sum voltages where bits are active
		float outputVoltage = 0.f;
		for (int i = 0; i < 5; i++) {
			if (bits & (1 << i)) {
				outputVoltage += params[i].getValue() * 0.2f;  // emulate resistor scaling
			}
		}
		outputVoltage = clamp(outputVoltage, 0.f, 10.f);
		outputs[VOLTS_OUTPUT].setVoltage(outputVoltage);
	}
};

struct BackgroundImage : Widget {
	std::string imagePath = asset::plugin(pluginInstance, "res/TuringMaschine-3.png");

	void draw(const DrawArgs& args) override {
		std::shared_ptr<Image> image = APP->window->loadImage(imagePath);
		if (image) {
			int w = box.size.x;
			int h = box.size.y;

			NVGpaint paint = nvgImagePattern(args.vg, 0, 0, w, h, 0.0f, image->handle, 1.0f);
			nvgBeginPath(args.vg);
			nvgRect(args.vg, 0, 0, w, h);
			nvgFillPaint(args.vg, paint);
			nvgFill(args.vg);
		}
	}
};

struct TuringVoltsExpanderWidget : ModuleWidget {
	TuringVoltsExpanderWidget(TuringVoltsExpander* module) {
		setModule(module);
		setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/TuringVoltsExpander.svg")));

		auto bg = new BackgroundImage();
		bg->box.pos = Vec(0, 0);
		bg->box.size = box.size; // Match panel size (e.g., 128.5 x 380 or 115 x 485)
		addChild(bg);

		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		for (int i = 0; i < 5; i++) {
			addParam(createParamCentered<RoundSmallBlackKnob>(
				mm2px(Vec(15.0, 40.0 + i * 15.0)), module, i));
		}

		addOutput(createOutputCentered<DarkPJ301MPort>(
			mm2px(Vec(10.0, 115.0)), module, TuringVoltsExpander::VOLTS_OUTPUT));
		
		addOutput(createOutputCentered<DarkPJ301MPort>(
			mm2px(Vec(20.0, 115.0)), module, TuringVoltsExpander::VOLTSINV_OUTPUT));
	}
};


Model* modelTuringVoltsExpander = createModel<TuringVoltsExpander, TuringVoltsExpanderWidget>("TuringVoltsExpander");