#include "plugin.hpp"
#include <osdialog.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

#include "NeuralAmpModelerCore/NAM/dsp.h"
#include "NeuralAmpModelerCore/NAM/get_dsp.h"

namespace {
inline float dbToGain(float db) {
        return std::pow(10.0f, db / 20.0f);
}

// Soft clipping/saturation function
inline float softClip(float x) {
        // Smooth tanh-based saturation
        if (x > 1.5f)
                return 1.0f;
        if (x < -1.5f)
                return -1.0f;
        return std::tanh(x);
}

struct ToneShaper {
        double lowState = 0.0;
        double highState = 0.0;

        void reset() {
                lowState = 0.0;
                highState = 0.0;
        }

        float process(float input, float toneAmount, float sampleRate) {
                // Two-stage tone control: bass and treble shelving
                // Tone knob: -1 = dark (cut highs, boost lows), 0 = neutral, +1 = bright (boost highs, cut lows)

                // Low shelf around 200Hz
                const double lowCutoff = 200.0;
                double lowAlpha = std::exp(-2.0 * M_PI * lowCutoff / std::max(sampleRate, 1.0f));
                lowState = (1.0 - lowAlpha) * input + lowAlpha * lowState;

                // High shelf around 2kHz
                const double highCutoff = 2000.0;
                double highAlpha = std::exp(-2.0 * M_PI * highCutoff / std::max(sampleRate, 1.0f));
                highState = (1.0 - highAlpha) * input + highAlpha * highState;
                double highs = input - highState;

                // Apply tone-dependent gains
                double bassGain = std::pow(10.0, (-toneAmount * 6.0) / 20.0); // -6 to +6 dB
                double trebleGain = std::pow(10.0, (toneAmount * 8.0) / 20.0); // -8 to +8 dB

                // Mix: lows + mids + highs
                double mids = highState - lowState;
                return (float)(lowState * bassGain + mids + highs * trebleGain);
        }
};
} // namespace

struct NergalAmp : Module {
        enum ParamIds {
                DRIVE_PARAM,
                TONE_PARAM,
                INPUT_PARAM,
                OUTPUT_PARAM,
                MIX_PARAM,
                NUM_PARAMS
        };
        enum InputIds {
                SIGNAL_INPUT,
                DRIVE_CV_INPUT,
                TONE_CV_INPUT,
                INPUT_CV_INPUT,
                OUTPUT_CV_INPUT,
                MIX_CV_INPUT,
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

        // Audio thread variables (protected by modelMutex)
        std::unique_ptr<nam::DSP> model;
        std::string modelPath;
        double modelSampleRate = 48000.0;
        double resamplePhase = 0.0;
        NAM_SAMPLE lastModelOutput = 0.0;
        float previousModelInput = 0.0f;
        bool firstFrame = true;
        ToneShaper tone;

        // Thread safety
        std::mutex modelMutex;

        // Deferred loading (set from UI thread, processed on audio thread)
        std::string pendingModelPath;
        bool loadPending = false;

        // Settings
        bool enableClipper = true;

        // UI state
        std::string lastDirectory;

        NergalAmp() {
                config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

                configParam(INPUT_PARAM, -24.f, 24.f, 0.f, "Input Trim", " dB");
                configParam(DRIVE_PARAM, 0.f, 2.f, 1.f, "Drive/Gain", "x");
                configParam(TONE_PARAM, -1.f, 1.f, 0.f, "Tone");
                configParam(MIX_PARAM, 0.f, 1.f, 1.f, "Dry/Wet Mix", "%", 0.f, 100.f);
                configParam(OUTPUT_PARAM, -24.f, 24.f, 0.f, "Output Level", " dB");

                configInput(SIGNAL_INPUT, "Audio");
                configInput(INPUT_CV_INPUT, "Input Trim CV");
                configInput(DRIVE_CV_INPUT, "Drive CV");
                configInput(TONE_CV_INPUT, "Tone CV");
                configInput(MIX_CV_INPUT, "Mix CV");
                configInput(OUTPUT_CV_INPUT, "Output Level CV");

                configOutput(SIGNAL_OUTPUT, "Audio");

                onSampleRateChange();
        }

        void onSampleRateChange() override {
                Module::onSampleRateChange();
                std::lock_guard<std::mutex> lock(modelMutex);
                resamplePhase = 0.0;
                firstFrame = true;
                tone.reset();
                if (model) {
                        const double effectiveRate = modelSampleRate > 0.0 ? modelSampleRate : APP->engine->getSampleRate();
                        model->Reset(effectiveRate, 64);
                }
        }

        void process(const ProcessArgs& args) override {
                // Check for pending model load (happens on audio thread to avoid blocking)
                if (loadPending) {
                        loadPending = false;
                        loadModelInternal(pendingModelPath);
                }

                // Get input signal and normalize from ±10V to ±1 range
                const float inVolts = inputs[SIGNAL_INPUT].isConnected() ? inputs[SIGNAL_INPUT].getVoltage() : 0.f;
                const float drySignal = inVolts * 0.1f;

                // Get parameter values with CV modulation
                float inputCV = inputs[INPUT_CV_INPUT].isConnected() ? inputs[INPUT_CV_INPUT].getVoltage() * 2.4f : 0.f; // ±12dB
                float inputGain = dbToGain(math::clamp(params[INPUT_PARAM].getValue() + inputCV, -24.f, 24.f));

                float driveCV = inputs[DRIVE_CV_INPUT].isConnected() ? inputs[DRIVE_CV_INPUT].getVoltage() * 0.2f : 0.f; // ±1x
                float driveAmount = math::clamp(params[DRIVE_PARAM].getValue() + driveCV, 0.f, 2.f);

                float toneCV = inputs[TONE_CV_INPUT].isConnected() ? inputs[TONE_CV_INPUT].getVoltage() * 0.1f : 0.f; // ±0.5
                float toneAmount = math::clamp(params[TONE_PARAM].getValue() + toneCV, -1.f, 1.f);

                float mixCV = inputs[MIX_CV_INPUT].isConnected() ? inputs[MIX_CV_INPUT].getVoltage() * 0.1f : 0.f; // ±0.5
                float mixAmount = math::clamp(params[MIX_PARAM].getValue() + mixCV, 0.f, 1.f);

                float outputCV = inputs[OUTPUT_CV_INPUT].isConnected() ? inputs[OUTPUT_CV_INPUT].getVoltage() * 2.4f : 0.f; // ±12dB
                float outputGain = dbToGain(math::clamp(params[OUTPUT_PARAM].getValue() + outputCV, -24.f, 24.f));

                // Stage 1: Apply input trim
                const float trimmed = drySignal * inputGain;

                // Stage 2: Process through NAM model (with mutex protection)
                float modelOutput = trimmed;
                bool hasModel = false;

                {
                        std::lock_guard<std::mutex> lock(modelMutex);
                        hasModel = (model != nullptr);

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
                                        previousModelInput = trimmed;
                                        firstFrame = false;
                                }

                                for (int s = 0; s < steps; ++s) {
                                        double t = (double)(s + 1) - phase;
                                        t /= ratio;
                                        float tClamped = math::clamp((float)t, 0.f, 1.f);
                                        float interp = math::crossfade(previousModelInput, trimmed, tClamped);

                                        NAM_SAMPLE inputFrame = (NAM_SAMPLE)interp;
                                        NAM_SAMPLE outputFrame = 0.0;
                                        model->process(&inputFrame, &outputFrame, 1);
                                        lastModelOutput = outputFrame;
                                }

                                previousModelInput = trimmed;
                                modelOutput = (float)lastModelOutput;
                        }
                }

                // Stage 3: Apply drive/saturation after the amp model
                float driven = modelOutput * driveAmount;
                if (enableClipper && driveAmount > 1.0f) {
                        // Apply soft clipping when drive is above unity (if enabled)
                        driven = softClip(driven);
                }

                // Stage 4: Tone shaping
                float shaped = tone.process(driven, toneAmount, args.sampleRate);

                // Stage 5: Dry/Wet mix
                float mixed = math::crossfade(drySignal, shaped, mixAmount);

                // Stage 6: Output level and convert back to ±10V
                float out = mixed * outputGain * 10.f;

                outputs[SIGNAL_OUTPUT].setVoltage(out);

                lights[LOADED_LIGHT].setBrightness(hasModel ? 1.f : 0.f);
        }

        void clearModel() {
                std::lock_guard<std::mutex> lock(modelMutex);
                model.reset();
                modelPath.clear();
                modelSampleRate = 48000.0;
                lastModelOutput = 0.0;
                resamplePhase = 0.0;
                firstFrame = true;
                tone.reset();
        }

        // Internal loading - called from audio thread with path already validated
        void loadModelInternal(const std::string& path) {
                if (path.empty()) {
                        clearModel();
                        return;
                }

                try {
                        std::filesystem::path fsPath(path);

                        // Load the model (this is the slow part)
                        auto loaded = nam::get_dsp(fsPath);

                        if (!loaded) {
                                WARN("NergalAmp: Failed to load model %s - get_dsp returned null", path.c_str());
                                clearModel();
                                return;
                        }

                        double sampleRate = loaded->GetExpectedSampleRate();
                        if (!(sampleRate > 0.0)) {
                                sampleRate = APP->engine->getSampleRate();
                                if (!(sampleRate > 0.0)) {
                                        sampleRate = 48000.0;
                                }
                        }
                        loaded->Reset(sampleRate, 64);

                        // Quick atomic swap with mutex
                        {
                                std::lock_guard<std::mutex> lock(modelMutex);
                                model = std::move(loaded);
                                modelPath = path;
                                modelSampleRate = sampleRate;
                                lastModelOutput = 0.0;
                                resamplePhase = 0.0;
                                firstFrame = true;
                                tone.reset();
                        }

                        // Update UI state (outside mutex)
                        if (fsPath.has_parent_path()) {
                                lastDirectory = fsPath.parent_path().string();
                        }

                        INFO("NergalAmp: Successfully loaded model %s at %.0f Hz", path.c_str(), sampleRate);
                }
                catch (const std::exception& e) {
                        WARN("NergalAmp failed to load model %s: %s", path.c_str(), e.what());
                        clearModel();
                }
        }

        // Public API - called from UI thread, defers to audio thread
        void loadModel(const std::string& path) {
                pendingModelPath = path;
                loadPending = true;
        }

        json_t* dataToJson() override {
                json_t* root = json_object();

                // Copy strings with mutex protection
                std::string path, dir;
                {
                        std::lock_guard<std::mutex> lock(modelMutex);
                        path = modelPath;
                }
                dir = lastDirectory; // lastDirectory doesn't need mutex (UI only)

                if (!path.empty()) {
                        json_object_set_new(root, "modelPath", json_string(path.c_str()));
                }
                if (!dir.empty()) {
                        json_object_set_new(root, "lastDirectory", json_string(dir.c_str()));
                }
                json_object_set_new(root, "enableClipper", json_boolean(enableClipper));
                return root;
        }

        void dataFromJson(json_t* root) override {
                json_t* pathJ = json_object_get(root, "modelPath");
                if (pathJ && json_is_string(pathJ)) {
                        // loadModel is already deferred, safe to call from any thread
                        loadModel(json_string_value(pathJ));
                }
                json_t* dirJ = json_object_get(root, "lastDirectory");
                if (dirJ && json_is_string(dirJ)) {
                        lastDirectory = json_string_value(dirJ);
                }
                json_t* clipperJ = json_object_get(root, "enableClipper");
                if (clipperJ) {
                        enableClipper = json_boolean_value(clipperJ);
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

                // Model loaded indicator
                addChild(createLightCentered<MediumLight<GreenLight>>(mm2px(Vec(15.24, 15.0)), module, NergalAmp::LOADED_LIGHT));

                // Parameters - left column (knobs)
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(10.16, 30.0)), module, NergalAmp::INPUT_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(10.16, 50.0)), module, NergalAmp::DRIVE_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(10.16, 70.0)), module, NergalAmp::TONE_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(10.16, 90.0)), module, NergalAmp::MIX_PARAM));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(10.16, 110.0)), module, NergalAmp::OUTPUT_PARAM));

                // CV Inputs - right column
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.32, 30.0)), module, NergalAmp::INPUT_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.32, 50.0)), module, NergalAmp::DRIVE_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.32, 70.0)), module, NergalAmp::TONE_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.32, 90.0)), module, NergalAmp::MIX_CV_INPUT));
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20.32, 110.0)), module, NergalAmp::OUTPUT_CV_INPUT));

                // Audio I/O at bottom
                addInput(createInputCentered<PJ301MPort>(mm2px(Vec(7.62, 120.0)), module, NergalAmp::SIGNAL_INPUT));
                addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(22.86, 120.0)), module, NergalAmp::SIGNAL_OUTPUT));
        }

        void appendContextMenu(Menu* menu) override {
                ModuleWidget::appendContextMenu(menu);

                menu->addChild(new MenuSeparator());

                auto* module = getModule<NergalAmp>();

                std::string labelText = "Model: (none)";
                if (module) {
                        // Thread-safe copy of model path
                        std::string path;
                        {
                                std::lock_guard<std::mutex> lock(module->modelMutex);
                                path = module->modelPath;
                        }
                        if (!path.empty()) {
                                labelText = "Model: " + std::filesystem::path(path).filename().string();
                        }
                }
                MenuLabel* label = createMenuLabel(labelText);
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

                // Thread-safe check if model exists
                bool hasModel = false;
                if (module) {
                        std::lock_guard<std::mutex> lock(module->modelMutex);
                        hasModel = (module->model != nullptr);
                }
                clearItem->disabled = !hasModel;
                menu->addChild(clearItem);

                // Clipper toggle
                menu->addChild(new MenuSeparator());

                struct ClipperItem : MenuItem {
                        NergalAmp* module = nullptr;
                        void onAction(const event::Action& e) override {
                                if (module) {
                                        module->enableClipper = !module->enableClipper;
                                }
                        }
                };
                auto* clipperItem = new ClipperItem();
                clipperItem->text = "Enable soft clipper";
                clipperItem->module = module;
                clipperItem->rightText = CHECKMARK(module && module->enableClipper);
                clipperItem->disabled = (module == nullptr);
                menu->addChild(clipperItem);
        }
};

Model* modelNergalAmp = createModel<NergalAmp, NergalAmpWidget>("NergalAmp");
