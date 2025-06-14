#include "plugin.hpp"
#include <cmath>
#include <vector>

struct TuringGateExpander : Module {
       enum ParamId {
               PARAMS_LEN
       };
	enum InputId {
		INPUTS_LEN
	};
	enum OutputId {
		GATE_OUTPUT_COMBO_1, // 1 + 2
		GATE_OUTPUT_COMBO_2, // 2 + 4
		GATE_OUTPUT_COMBO_3, // 4 + 7
		GATE_OUTPUT_COMBO_4, // 1 + 2 + 4 + 7
		GATE_OUTPUTS,
		NUM_OUTPUTS = GATE_OUTPUTS + 12,
		OUTPUTS_LEN
	};
	enum LightId {
		GATE_LIGHTS, // start index for 8 bit LEDs
		COMBO_LIGHT_1 = GATE_LIGHTS + 8,
		COMBO_LIGHT_2,
		COMBO_LIGHT_3,
		COMBO_LIGHT_4,
		NUM_LIGHTS,
		LIGHTS_LEN
	};

       float value[2] = {};
       uint8_t prevBits = 0;
       int gateMode = 0; // 0 = Gate, 1 = Tap
       float tapTimers[12] = {};
       // Duration of LED flash when in Tap mode
       const float tapTime = 0.05f;

       TuringGateExpander() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		getLeftExpander().producerMessage = &value[0];
		getLeftExpander().consumerMessage = &value[1];
       }

	
	void ClearOutputs(){
		// If no valid expander data, clear everything
		for (int i = 0; i < 8; ++i) {
			outputs[GATE_OUTPUTS + i].setVoltage(0.f);
			lights[GATE_LIGHTS + i].setBrightness(0.f);
		}

		outputs[GATE_OUTPUT_COMBO_1].setVoltage(0.f);
		outputs[GATE_OUTPUT_COMBO_2].setVoltage(0.f);
		outputs[GATE_OUTPUT_COMBO_3].setVoltage(0.f);
		outputs[GATE_OUTPUT_COMBO_4].setVoltage(0.f);

		lights[COMBO_LIGHT_1].setBrightness(0.f);
		lights[COMBO_LIGHT_2].setBrightness(0.f);
		lights[COMBO_LIGHT_3].setBrightness(0.f);
		lights[COMBO_LIGHT_4].setBrightness(0.f);
	}

       void process(const ProcessArgs& args) override {
                if (getLeftExpander().module && getLeftExpander().module->model && getLeftExpander().module->model->slug == "TuringMaschine") {
                        float* value = (float*) getLeftExpander().consumerMessage;
                        if (value) {
                                uint8_t bits = (uint8_t)value[0];

                                bool g1 = (bits >> 1) & 0x1;
                                bool g2 = (bits >> 2) & 0x1;
                                bool g4 = (bits >> 4) & 0x1;
                                bool g7 = (bits >> 7) & 0x1;

                                bool outA = g1 || g2;               // 1 + 2
                                bool outB = g2 || g4;               // 2 + 4
                                bool outC = g4 || g7;               // 4 + 7
                                bool outD = g1 || g2 || g4 || g7;   // 1 + 2 + 4 + 7

                                if (gateMode == 0) {
                                        for (int i = 0; i < 8; i++) {
                                                bool bit = (bits >> i) & 0x1;
                                                outputs[GATE_OUTPUTS + i].setVoltage(bit ? 10.f : 0.f);
                                                lights[GATE_LIGHTS + i].setBrightness(bit ? 1.f : 0.f);
                                        }
                                        outputs[GATE_OUTPUT_COMBO_1].setVoltage(outA ? 10.f : 0.f);
                                        outputs[GATE_OUTPUT_COMBO_2].setVoltage(outB ? 10.f : 0.f);
                                        outputs[GATE_OUTPUT_COMBO_3].setVoltage(outC ? 10.f : 0.f);
                                        outputs[GATE_OUTPUT_COMBO_4].setVoltage(outD ? 10.f : 0.f);

                                        lights[COMBO_LIGHT_1].setBrightness(outA ? 1.f : 0.f);
                                        lights[COMBO_LIGHT_2].setBrightness(outB ? 1.f : 0.f);
                                        lights[COMBO_LIGHT_3].setBrightness(outC ? 1.f : 0.f);
                                        lights[COMBO_LIGHT_4].setBrightness(outD ? 1.f : 0.f);
                                }
                                else {
                                        for (int i = 0; i < 8; i++) {
                                                bool bit = (bits >> i) & 0x1;
                                                bool prev = (prevBits >> i) & 0x1;
                                                if (bit != prev)
                                                        tapTimers[4 + i] = tapTime;
                                                if (tapTimers[4 + i] > 0.f) {
                                                        tapTimers[4 + i] -= args.sampleTime;
                                                        outputs[GATE_OUTPUTS + i].setVoltage(10.f);
                                                        lights[GATE_LIGHTS + i].setBrightness(1.f);
                                                }
                                                else {
                                                        outputs[GATE_OUTPUTS + i].setVoltage(0.f);
                                                        lights[GATE_LIGHTS + i].setBrightness(0.f);
                                                }
                                        }

                                        bool prev1 = (prevBits >> 1) & 0x1;
                                        bool prev2 = (prevBits >> 2) & 0x1;
                                        bool prev4 = (prevBits >> 4) & 0x1;
                                        bool prev7 = (prevBits >> 7) & 0x1;

                                        bool prevA = prev1 || prev2;
                                        bool prevB = prev2 || prev4;
                                        bool prevC = prev4 || prev7;
                                        bool prevD = prev1 || prev2 || prev4 || prev7;

                                        bool combos[4] = {outA, outB, outC, outD};
                                        bool prevCombos[4] = {prevA, prevB, prevC, prevD};
                                        for (int i = 0; i < 4; ++i) {
                                                if (combos[i] != prevCombos[i])
                                                        tapTimers[i] = tapTime;
                                                if (tapTimers[i] > 0.f) {
                                                        tapTimers[i] -= args.sampleTime;
                                                        outputs[i].setVoltage(10.f);
                                                        lights[COMBO_LIGHT_1 + i].setBrightness(1.f);
                                                }
                                                else {
                                                        outputs[i].setVoltage(0.f);
                                                        lights[COMBO_LIGHT_1 + i].setBrightness(0.f);
                                                }
                                        }
                                }

                                prevBits = bits;
                        }
                        return;
                }
                ClearOutputs();
        }

        json_t* dataToJson() override {
                json_t* root = json_object();
                json_object_set_new(root, "gateMode", json_integer(gateMode));
                return root;
        }

        void dataFromJson(json_t* root) override {
                json_t* jm = json_object_get(root, "gateMode");
                if (jm)
                        gateMode = json_integer_value(jm);
        }
};

struct BackgroundImage : Widget {
	std::string imagePath = asset::plugin(pluginInstance, "res/TuringMaschine-2.png");

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


struct TuringGateExpanderWidget : ModuleWidget {
        TuringGateExpanderWidget(TuringGateExpander* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/TuringGateExpander.svg")));

                auto bg = new BackgroundImage();
		bg->box.pos = Vec(0, 0);
		bg->box.size = box.size; // Match panel size (e.g., 128.5 x 380 or 115 x 485)
		addChild(bg);

		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		for (int i = 0; i < 8; i++) {
                        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(5.0, 20.0 + i * 12.5)), module, TuringGateExpander::GATE_OUTPUTS + i));
                        addChild(createLightCentered<SmallLight<RedLight>>(
                                mm2px(Vec(2.5, 20.0 + i * 12.5 - 6)),
                                module,
                                TuringGateExpander::GATE_LIGHTS + i
                        ));
                }

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.0,  20.0 + 1 * 12.5 - 5)), module, TuringGateExpander::GATE_OUTPUT_COMBO_1));
		addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(15-2.5, 20.0 + 1 * 12.5 - 11)), module, TuringGateExpander::COMBO_LIGHT_1));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.0, 20.0 + 2 * 12.5 -5)), module, TuringGateExpander::GATE_OUTPUT_COMBO_2));
		addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(15-2.5, 20.0 + 2 * 12.5 - 11)), module, TuringGateExpander::COMBO_LIGHT_2));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.0, 20.0 + 6 * 12.5 - 5)), module, TuringGateExpander::GATE_OUTPUT_COMBO_3));
		addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(15-2.5, 20.0 + 6 * 12.5 - 11)), module, TuringGateExpander::COMBO_LIGHT_3));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.0, 20.0 + 7 * 12.5 - 5)), module, TuringGateExpander::GATE_OUTPUT_COMBO_4));
                addChild(createLightCentered<SmallLight<RedLight>>(mm2px(Vec(15-2.5, 20.0 + 7 * 12.5 - 11)), module, TuringGateExpander::COMBO_LIGHT_4));
        }

        void appendContextMenu(Menu* menu) override {
                TuringGateExpander* module = getModule<TuringGateExpander>();
                menu->addChild(createIndexPtrSubmenuItem("Gate Mode",
                        {"Gate", "Tap"},
                        &module->gateMode
                ));
        }
};


Model* modelTuringGateExpander = createModel<TuringGateExpander, TuringGateExpanderWidget>("TuringGateExpander");