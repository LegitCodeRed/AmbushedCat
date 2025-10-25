#include "plugin.hpp"

#include <osdialog.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>

#include "NeuralAmpModelerCore/NAM/dsp.h"
#include "NeuralAmpModelerCore/NAM/get_dsp.h"

namespace {
inline float dbToGain(float db) {
        return std::pow(10.0f, db / 20.0f);
}

struct ToneShaper {
        double lowState = 0.0;

        void reset() {
                lowState = 0.0;
        }

        float process(float input, float toneAmount, float sampleRate) {
                const double cutoff = 1000.0;
                double alpha = std::exp(-2.0 * M_PI * cutoff / std::max(sampleRate, 1.0f));
                lowState = (1.0 - alpha) * input + alpha * lowState;
                double high = input - lowState;

                double tilt = toneAmount * 10.0; // +/-10 dB tilt
                double lowGain = std::pow(10.0, (-tilt) / 20.0);
                double highGain = std::pow(10.0, (tilt) / 20.0);

                return (float)(lowState * lowGain + high * highGain);
        }
};
} // namespace

struct NergalAmp : Module {
        enum ParamIds {
                DRIVE_PARAM,
                TONE_PARAM,
                INPUT_PARAM,
                OUTPUT_PARAM,
                NUM_PARAMS
        };
        enum InputIds {
                SIGNAL_INPUT,
                NUM_INPUTS
        };
        enum OutputIds {
                SIGNAL_OUTPUT,
                NUM_OUTPUTS
        };
        enum LightIds {
                LOADED_LIGHT,
                NUM_LIGHTS
        };

        std::unique_ptr<nam::DSP> model;
        std::string modelPath;
        std::string lastDirectory;
        double modelSampleRate = 48000.0;
        double resamplePhase = 0.0;
        NAM_SAMPLE lastModelOutput = 0.0;
        float previousModelInput = 0.0f;
        bool firstFrame = true;
        ToneShaper tone;

        NergalAmp() {
                config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

                configParam(DRIVE_PARAM, 0.f, 24.f, 6.f, "Drive", " dB");
                configParam(TONE_PARAM, -1.f, 1.f, 0.f, "Tone", "", 0.f, 1.f, 0.f);
                configParam(INPUT_PARAM, -24.f, 24.f, 0.f, "Input Trim", " dB");
                configParam(OUTPUT_PARAM, -24.f, 24.f, 0.f, "Output Trim", " dB");

                configInput(SIGNAL_INPUT, "Signal");
                configOutput(SIGNAL_OUTPUT, "Amped signal");

                onSampleRateChange();
        }

        void onSampleRateChange() override {
                Module::onSampleRateChange();
                resamplePhase = 0.0;
                firstFrame = true;
                tone.reset();
                if (model) {
                        const double effectiveRate = modelSampleRate > 0.0 ? modelSampleRate : APP->engine->getSampleRate();
                        model->Reset(effectiveRate, 64);
                }
        }

        void process(const ProcessArgs& args) override {
                const float inVolts = inputs[SIGNAL_INPUT].isConnected() ? inputs[SIGNAL_INPUT].getVoltage() : 0.f;
                const float dry = inVolts * 0.1f; // Normalize +/-10V to +/-1 range

                const float driveGain = dbToGain(params[DRIVE_PARAM].getValue());
                const float inputGain = dbToGain(params[INPUT_PARAM].getValue());
                const float outputGain = dbToGain(params[OUTPUT_PARAM].getValue());
                const float toneAmount = params[TONE_PARAM].getValue();

                const float modelInput = dry * inputGain * driveGain;

                float processed = modelInput;

                if (model) {
                        const double hostRate = args.sampleRate;
                        double targetRate = modelSampleRate > 0.0 ? modelSampleRate : hostRate;
                        if (targetRate <= 0.0) {
                                targetRate = hostRate;
                        }

                        const double ratio = targetRate / std::max(hostRate, 1.0);
                        double phase = resamplePhase;
                        const double total = phase + ratio;
                        int steps = (int)std::floor(total);
                        resamplePhase = total - steps;

                        if (firstFrame) {
                                previousModelInput = modelInput;
                                firstFrame = false;
                        }

                        for (int s = 0; s < steps; ++s) {
                                double t = (double)(s + 1) - phase;
                                t /= ratio;
                                t = math::clamp(t, 0.0, 1.0);
                                float interp = math::crossfade(previousModelInput, modelInput, (float)t);

                                NAM_SAMPLE inputFrame = (NAM_SAMPLE)interp;
                                NAM_SAMPLE outputFrame = 0.0;
                                model->process(&inputFrame, &outputFrame, 1);
                                lastModelOutput = outputFrame;
                        }

                        previousModelInput = modelInput;
                        processed = (float)lastModelOutput;
                }

                float shaped = tone.process(processed, toneAmount, args.sampleRate);
                float out = shaped * outputGain;

                outputs[SIGNAL_OUTPUT].setVoltage(out * 10.f);

                lights[LOADED_LIGHT].setBrightness(model ? 1.f : 0.f);
        }

        void clearModel() {
                model.reset();
                modelPath.clear();
                modelSampleRate = 48000.0;
                lastModelOutput = 0.0;
                resamplePhase = 0.0;
                firstFrame = true;
                tone.reset();
        }

        void loadModel(const std::string& path) {
                if (path.empty()) {
                        clearModel();
                        return;
                }

                try {
                        auto loaded = nam::get_dsp(path);
                        model = std::move(loaded);
                        modelPath = path;
                        std::filesystem::path fsPath(path);
                        if (fsPath.has_parent_path()) {
                                lastDirectory = fsPath.parent_path().string();
                        }

                        modelSampleRate = model ? model->GetExpectedSampleRate() : 48000.0;
                        if (!(modelSampleRate > 0.0)) {
                                modelSampleRate = 48000.0;
                        }
                        model->Reset(modelSampleRate, 64);
                        lastModelOutput = 0.0;
                        resamplePhase = 0.0;
                        firstFrame = true;
                        tone.reset();
                }
                catch (const std::exception& e) {
                        WARN("NergalAmp failed to load model %s: %s", path.c_str(), e.what());
                        clearModel();
                }
        }

        json_t* dataToJson() override {
                json_t* root = json_object();
                if (!modelPath.empty()) {
                        json_object_set_new(root, "modelPath", json_string(modelPath.c_str()));
                }
                if (!lastDirectory.empty()) {
                        json_object_set_new(root, "lastDirectory", json_string(lastDirectory.c_str()));
                }
                return root;
        }

        void dataFromJson(json_t* root) override {
                json_t* pathJ = json_object_get(root, "modelPath");
                if (pathJ && json_is_string(pathJ)) {
                        loadModel(json_string_value(pathJ));
                }
                json_t* dirJ = json_object_get(root, "lastDirectory");
                if (dirJ && json_is_string(dirJ)) {
                        lastDirectory = json_string_value(dirJ);
                }
        }
};

struct NergalAmpWidget : ModuleWidget {
        NergalAmpWidget(NergalAmp* module) {
                setModule(module);
                setPanel(createPanel(asset::plugin(pluginInstance, "res/NergalAmp.svg")));

                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
                addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.0, 32.0)), module, NergalAmp::DRIVE_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.0, 62.0)), module, NergalAmp::TONE_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.0, 92.0)), module, NergalAmp::INPUT_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.0, 122.0)), module, NergalAmp::OUTPUT_PARAM));

                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(15.0, 152.0)), module, NergalAmp::SIGNAL_INPUT));
                addOutput(createOutputCentered<DarkPJ301MPort>(mm2px(Vec(15.0, 172.0)), module, NergalAmp::SIGNAL_OUTPUT));

                addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(15.0, 18.0)), module, NergalAmp::LOADED_LIGHT));
        }

        void appendContextMenu(Menu* menu) override {
                ModuleWidget::appendContextMenu(menu);

                menu->addChild(new MenuSeparator());

                auto* module = getModule<NergalAmp>();

                std::string labelText = "Model: (none)";
                if (module && !module->modelPath.empty()) {
                        labelText = "Model: " + std::filesystem::path(module->modelPath).filename().string();
                }
                MenuLabel* label = createMenuLabel(labelText);
                label->disabled = true;
                menu->addChild(label);

                struct LoadItem : MenuItem {
                        NergalAmp* module = nullptr;
                        void onAction(const event::Action& e) override {
                                if (!module)
                                        return;
                                osdialog_filters* filters = osdialog_filters_parse("NAM file:n:nam");
                                const char* initialPath = module->lastDirectory.empty() ? nullptr : module->lastDirectory.c_str();
                                char* path = osdialog_file(OSDIALOG_OPEN, initialPath, nullptr, filters);
                                osdialog_filters_free(filters);
                                if (path) {
                                        std::string chosen(path);
                                        module->loadModel(chosen);
                                        free(path);
                                }
                        }
                };

                auto* loadItem = new LoadItem();
                loadItem->text = "Load NAM model";
                loadItem->module = module;
                loadItem->disabled = (module == nullptr);
                menu->addChild(loadItem);

                struct ClearItem : MenuItem {
                        NergalAmp* module = nullptr;
                        void onAction(const event::Action& e) override {
                                if (module)
                                        module->clearModel();
                        }
                };
                auto* clearItem = new ClearItem();
                clearItem->text = "Clear model";
                clearItem->module = module;
                clearItem->disabled = !(module && module->model);
                menu->addChild(clearItem);
        }
};

Model* modelNergalAmp = createModel<NergalAmp, NergalAmpWidget>("NergalAmp");
