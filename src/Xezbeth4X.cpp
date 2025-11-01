#include "plugin.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <string>

namespace {

using namespace rack::componentlibrary;
using rack::createInputCentered;
using rack::createLightCentered;
using rack::createOutputCentered;
using rack::createParamCentered;
using rack::createWidget;

static inline float dbToGain(float db) {
        if (db <= -80.f)
                return 0.f;
        return std::pow(10.f, db / 20.f);
}

static inline float amplitudeToDb(float amp) {
        if (amp <= 1e-9f)
                return -120.f;
        return 20.f * std::log10(amp);
}

struct MeterState {
        float rms = 0.f;
        float truePeak = 0.f;
        float peakHold = 0.f;
        float peakHoldTimer = 0.f;
        float clipTimer = 0.f;
        double prevL = 0.0;
        double prevR = 0.0;
};

struct Xezbeth4X : rack::engine::Module {
        enum ParamIds {
                CHANNEL_TRIM_PARAM,
                CHANNEL_TRIM_PARAM_LAST = CHANNEL_TRIM_PARAM + 3,
                CHANNEL_PAN_PARAM,
                CHANNEL_PAN_PARAM_LAST = CHANNEL_PAN_PARAM + 3,
                CHANNEL_MUTE_PARAM,
                CHANNEL_MUTE_PARAM_LAST = CHANNEL_MUTE_PARAM + 3,
                CHANNEL_SOLO_PARAM,
                CHANNEL_SOLO_PARAM_LAST = CHANNEL_SOLO_PARAM + 3,
                CHANNEL_PFL_PARAM,
                CHANNEL_PFL_PARAM_LAST = CHANNEL_PFL_PARAM + 3,
                MASTER_TRIM_PARAM,
                MONO_PARAM,
                DIM_PARAM,
                CLIPSAFE_PARAM,
                NUM_PARAMS
        };

        enum InputIds {
                CHANNEL_INPUT_L,
                CHANNEL_INPUT_L_LAST = CHANNEL_INPUT_L + 3,
                CHANNEL_INPUT_R,
                CHANNEL_INPUT_R_LAST = CHANNEL_INPUT_R + 3,
                NUM_INPUTS
        };

        enum OutputIds {
                MASTER_OUTPUT_L,
                MASTER_OUTPUT_R,
                PFL_OUTPUT_L,
                PFL_OUTPUT_R,
                POST_OUTPUT_L,
                POST_OUTPUT_R,
                NUM_OUTPUTS
        };

        enum LightIds {
                CHANNEL_POST_LIGHT,
                CHANNEL_POST_LIGHT_LAST = CHANNEL_POST_LIGHT + 3,
                CHANNEL_CLIP_LIGHT,
                CHANNEL_CLIP_LIGHT_LAST = CHANNEL_CLIP_LIGHT + 3,
                MASTER_CLIP_LIGHT,
                PFL_ACTIVE_LIGHT,
                NUM_LIGHTS
        };

        enum SummingStyle {
                SUMMING_CLEAN = 0,
                SUMMING_NEVE = 1
        };

        enum HarmonicDriveMode {
                DRIVE_OFF = 0,
                DRIVE_SUBTLE,
                DRIVE_MEDIUM,
                DRIVE_HOT
        };

        enum OvertoneFocus {
                OVERTONE_EVEN = 0,
                OVERTONE_BALANCED,
                OVERTONE_ODD
        };

        enum OversamplingQuality {
                OS_1X = 0,
                OS_2X,
                OS_4X,
                OS_8X
        };

        enum PanLaw {
                PAN_MINUS3 = 0,
                PAN_MINUS4_5,
                PAN_MINUS6
        };

        enum MeterResponse {
                RESPONSE_FAST = 0,
                RESPONSE_MEDIUM,
                RESPONSE_SLOW
        };

        enum HeadroomModel {
                HEADROOM_STANDARD = 0,
                HEADROOM_EXTENDED
        };

        std::array<MeterState, 4> channelMeters{};
        MeterState masterMeter{};
        MeterState pflMeter{};

        float lowShelfStateL = 0.f;
        float lowShelfStateR = 0.f;
        float highShelfStateL = 0.f;
        float highShelfStateR = 0.f;
        float lowShelfAlpha = 0.f;
        float highShelfAlpha = 0.f;

        double drivePrevL = 0.0;
        double drivePrevR = 0.0;
        float driveEnvL = 0.f;
        float driveEnvR = 0.f;

        int summingStyle = SUMMING_CLEAN;
        int harmonicDrive = DRIVE_OFF;
        int overtoneFocus = OVERTONE_BALANCED;
        int oversamplingQuality = OS_1X;
        int panLawSetting = PAN_MINUS3;
        bool meterPeakHold = true;
        int meterResponse = RESPONSE_MEDIUM;
        int headroomMode = HEADROOM_STANDARD;
        bool clipSafeEnabled = true;

        Xezbeth4X() {
                config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

                for (int i = 0; i < 4; ++i) {
                        configParam(CHANNEL_TRIM_PARAM + i, -60.f, 12.f, 0.f, "Channel " + std::to_string(i + 1) + " trim", " dB");
                        configParam(CHANNEL_PAN_PARAM + i, -1.f, 1.f, 0.f, "Channel " + std::to_string(i + 1) + " pan");
                        configButton(CHANNEL_MUTE_PARAM + i, "Channel " + std::to_string(i + 1) + " mute");
                        configButton(CHANNEL_SOLO_PARAM + i, "Channel " + std::to_string(i + 1) + " solo");
                        configButton(CHANNEL_PFL_PARAM + i, "Channel " + std::to_string(i + 1) + " PFL");

                        configInput(CHANNEL_INPUT_L + i, "Channel " + std::to_string(i + 1) + " left");
                        configInput(CHANNEL_INPUT_R + i, "Channel " + std::to_string(i + 1) + " right");
                }

                configParam(MASTER_TRIM_PARAM, -60.f, 12.f, 0.f, "Master trim", " dB");
                configButton(MONO_PARAM, "Monitor mono");
                configButton(DIM_PARAM, "Monitor dim");
                configButton(CLIPSAFE_PARAM, "Clip-Safe");

                configOutput(MASTER_OUTPUT_L, "Master left");
                configOutput(MASTER_OUTPUT_R, "Master right");
                configOutput(PFL_OUTPUT_L, "PFL left");
                configOutput(PFL_OUTPUT_R, "PFL right");
                configOutput(POST_OUTPUT_L, "Post record left");
                configOutput(POST_OUTPUT_R, "Post record right");

                onSampleRateChange();
        }

        void onSampleRateChange() override {
                float sr = APP->engine->getSampleRate();
                const float lowCut = 80.f;
                const float highCut = 7500.f;
                lowShelfAlpha = std::exp(-2.f * M_PI * lowCut / sr);
                highShelfAlpha = std::exp(-2.f * M_PI * highCut / sr);
        }

        float getPanCenterGain() const {
                switch (panLawSetting) {
                        default:
                        case PAN_MINUS3:
                                return std::pow(10.f, -3.f / 20.f);
                        case PAN_MINUS4_5:
                                return std::pow(10.f, -4.5f / 20.f);
                        case PAN_MINUS6:
                                return std::pow(10.f, -6.f / 20.f);
                }
        }

        int getOversampleFactor() const {
                switch (oversamplingQuality) {
                        default:
                        case OS_1X:
                                return 1;
                        case OS_2X:
                                return 2;
                        case OS_4X:
                                return 4;
                        case OS_8X:
                                return 8;
                }
        }

        float getHeadroom() const {
                switch (headroomMode) {
                        default:
                        case HEADROOM_STANDARD:
                                return std::pow(10.f, 24.f / 20.f);
                        case HEADROOM_EXTENDED:
                                return std::pow(10.f, 30.f / 20.f);
                }
        }

        void updateMeter(MeterState& meter, float left, float right, float sampleTime) {
                float energy = 0.5f * (left * left + right * right);
                float inst = std::sqrt(std::max(energy, 0.f));

                float tau = 0.075f;
                switch (meterResponse) {
                        case RESPONSE_FAST:
                                tau = 0.03f;
                                break;
                        case RESPONSE_MEDIUM:
                                tau = 0.075f;
                                break;
                        case RESPONSE_SLOW:
                                tau = 0.3f;
                                break;
                }
                float alpha = std::exp(-sampleTime / tau);
                meter.rms = rack::math::crossfade(inst, meter.rms, alpha);

                const int oversample = 4;
                double curL = left;
                double curR = right;
                double prevL = meter.prevL;
                double prevR = meter.prevR;
                double maxMag = std::max(std::fabs(curL), std::fabs(curR));
                for (int i = 1; i < oversample; ++i) {
                        double t = static_cast<double>(i) / oversample;
                        double interpL = prevL + (curL - prevL) * t;
                        double interpR = prevR + (curR - prevR) * t;
                        double mag = std::max(std::fabs(interpL), std::fabs(interpR));
                        if (mag > maxMag)
                                maxMag = mag;
                }
                meter.prevL = curL;
                meter.prevR = curR;
                meter.truePeak = static_cast<float>(maxMag);

                if (meterPeakHold) {
                        if (maxMag >= meter.peakHold) {
                                meter.peakHold = static_cast<float>(maxMag);
                                meter.peakHoldTimer = 0.f;
                        } else {
                                meter.peakHoldTimer += sampleTime;
                                const float holdTime = 0.6f;
                                if (meter.peakHoldTimer > holdTime) {
                                        float decay = std::exp(-sampleTime / 0.2f);
                                        meter.peakHold *= decay;
                                        if (meter.peakHold < meter.truePeak)
                                                meter.peakHold = meter.truePeak;
                                }
                        }
                } else {
                        meter.peakHold = meter.truePeak;
                        meter.peakHoldTimer = 0.f;
                }

                if (maxMag >= 1.f) {
                        meter.clipTimer = 0.18f;
                } else {
                        meter.clipTimer = std::max(0.f, meter.clipTimer - sampleTime);
                }
        }

        std::pair<float, float> applyTone(float inL, float inR) {
                float oneMinusLow = 1.f - lowShelfAlpha;
                float oneMinusHigh = 1.f - highShelfAlpha;

                lowShelfStateL = lowShelfAlpha * lowShelfStateL + oneMinusLow * inL;
                lowShelfStateR = lowShelfAlpha * lowShelfStateR + oneMinusLow * inR;

                float lowGain = 1.12f;
                float highGain = 1.05f;

                float liftedL = inL + (lowShelfStateL - inL) * (lowGain - 1.f);
                float liftedR = inR + (lowShelfStateR - inR) * (lowGain - 1.f);

                float highL = liftedL - highShelfStateL;
                float highR = liftedR - highShelfStateR;
                highShelfStateL = highShelfAlpha * highShelfStateL + oneMinusHigh * liftedL;
                highShelfStateR = highShelfAlpha * highShelfStateR + oneMinusHigh * liftedR;

                liftedL += highL * (highGain - 1.f);
                liftedR += highR * (highGain - 1.f);
                return {liftedL, liftedR};
        }

        float processDriveSample(float input, double& prev, float& env, int oversample, float driveBase,
                                 float evenWeight, float oddWeight, float evenBias, float headroom) {
                if (driveBase <= 0.f) {
                        prev = input / headroom;
                        return input;
                }

                double normalized = static_cast<double>(input) / headroom;
                normalized = static_cast<double>(rack::math::clamp(static_cast<float>(normalized), -4.f, 4.f));

                double last = prev;
                double acc = 0.0;

                for (int i = 0; i < oversample; ++i) {
                        double t = static_cast<double>(i + 1) / oversample;
                        double interp = last + (normalized - last) * t;
                        float absInterp = static_cast<float>(std::fabs(interp));
                        env += 0.04f * (absInterp - env);
                        env = rack::math::clamp(env, 0.f, 2.f);
                        float dynamicDrive = driveBase * (1.f + 0.6f * env);

                        double odd = interp - (interp * interp * interp) * (1.0 / 3.0);
                        double even = std::tanh((interp + evenBias) * dynamicDrive);
                        even -= std::tanh(evenBias * dynamicDrive);

                        double mix = oddWeight * odd + evenWeight * even;
                        double limited = mix / (1.0 + std::fabs(mix) * 0.25);
                        acc += limited;
                }

                prev = normalized;
                double averaged = acc / oversample;
                averaged = static_cast<double>(rack::math::clamp(static_cast<float>(averaged), -4.f, 4.f));
                return static_cast<float>(averaged * headroom);
        }

        std::pair<float, float> applyPan(float inL, float inR, float pan, float centerGain) {
                const float width = 0.5f;
                auto computeGains = [centerGain](float value) {
                        value = rack::math::clamp(value, -1.f, 1.f);
                        float angle = (value + 1.f) * (float)M_PI * 0.25f;
                        float left = std::cos(angle);
                        float right = std::sin(angle);
                        float scale = centerGain / std::sqrt(0.5f);
                        left *= scale;
                        right *= scale;
                        return std::pair<float, float>(left, right);
                };

                auto gainsL = computeGains(pan - width);
                auto gainsR = computeGains(pan + width);

                float outL = inL * gainsL.first + inR * gainsR.first;
                float outR = inL * gainsL.second + inR * gainsR.second;
                return {outL, outR};
        }

        void process(const ProcessArgs& args) override {
                const float sampleTime = args.sampleTime;
                const float centerGain = getPanCenterGain();
                const int oversample = getOversampleFactor();
                const float headroom = getHeadroom();

                bool anySolo = false;
                std::array<bool, 4> soloStates{};
                for (int i = 0; i < 4; ++i) {
                        bool solo = params[CHANNEL_SOLO_PARAM + i].getValue() > 0.5f;
                        soloStates[i] = solo;
                        anySolo = anySolo || solo;
                }

                bool anyPFL = false;
                double pflL = 0.0;
                double pflR = 0.0;

                double sumL = 0.0;
                double sumR = 0.0;

                for (int i = 0; i < 4; ++i) {
                        float trim = params[CHANNEL_TRIM_PARAM + i].getValue();
                        float trimGain = dbToGain(trim);
                        float pan = params[CHANNEL_PAN_PARAM + i].getValue();
                        bool mute = params[CHANNEL_MUTE_PARAM + i].getValue() > 0.5f;
                        bool solo = soloStates[i];
                        bool pfl = params[CHANNEL_PFL_PARAM + i].getValue() > 0.5f;

                        float inL = 0.f;
                        float inR = 0.f;
                        if (inputs[CHANNEL_INPUT_L + i].isConnected())
                                inL = inputs[CHANNEL_INPUT_L + i].getVoltage();
                        if (inputs[CHANNEL_INPUT_R + i].isConnected())
                                inR = inputs[CHANNEL_INPUT_R + i].getVoltage();
                        else
                                inR = inL;

                        float preL = inL * trimGain;
                        float preR = inR * trimGain;

                        if (pfl) {
                                pflL += preL;
                                pflR += preR;
                                anyPFL = true;
                        }

                        bool active = !mute;
                        if (anySolo)
                                active = solo;

                        float postL = 0.f;
                        float postR = 0.f;
                        if (active) {
                                auto panOut = applyPan(preL, preR, pan, centerGain);
                                postL = panOut.first;
                                postR = panOut.second;
                                sumL += postL;
                                sumR += postR;
                        }

                        updateMeter(channelMeters[i], postL, postR, sampleTime);
                        bool postActive = active && (std::fabs(postL) > 1e-4f || std::fabs(postR) > 1e-4f);
                        lights[CHANNEL_POST_LIGHT + i].setBrightness(postActive ? 1.f : 0.f);
                        lights[CHANNEL_CLIP_LIGHT + i].setBrightness(channelMeters[i].clipTimer > 0.f ? 1.f : 0.f);
                }

                auto toned = std::pair<float, float>(static_cast<float>(sumL), static_cast<float>(sumR));
                if (summingStyle == SUMMING_NEVE) {
                        toned = applyTone(toned.first, toned.second);
                }

                float driveAmount = 0.f;
                switch (harmonicDrive) {
                        default:
                        case DRIVE_OFF:
                                driveAmount = 0.f;
                                break;
                        case DRIVE_SUBTLE:
                                driveAmount = 0.45f;
                                break;
                        case DRIVE_MEDIUM:
                                driveAmount = 0.9f;
                                break;
                        case DRIVE_HOT:
                                driveAmount = 1.35f;
                                break;
                }

                float evenWeight = 0.5f;
                float oddWeight = 0.5f;
                float evenBias = 0.18f;
                switch (overtoneFocus) {
                        case OVERTONE_EVEN:
                                evenWeight = 0.65f;
                                oddWeight = 0.35f;
                                evenBias = 0.24f;
                                break;
                        case OVERTONE_BALANCED:
                                evenWeight = 0.5f;
                                oddWeight = 0.5f;
                                evenBias = 0.18f;
                                break;
                        case OVERTONE_ODD:
                                evenWeight = 0.35f;
                                oddWeight = 0.65f;
                                evenBias = 0.12f;
                                break;
                }

                float busL = toned.first;
                float busR = toned.second;

                if (summingStyle == SUMMING_NEVE && driveAmount > 0.f) {
                        busL = processDriveSample(busL, drivePrevL, driveEnvL, oversample, driveAmount, evenWeight, oddWeight, evenBias, headroom);
                        busR = processDriveSample(busR, drivePrevR, driveEnvR, oversample, driveAmount, evenWeight, oddWeight, evenBias, headroom);
                } else {
                        drivePrevL = static_cast<double>(busL) / headroom;
                        drivePrevR = static_cast<double>(busR) / headroom;
                        driveEnvL *= 0.999f;
                        driveEnvR *= 0.999f;
                }

                if (summingStyle == SUMMING_NEVE) {
                        auto limiter = [headroom](float x) {
                                float limit = headroom * 1.02f;
                                if (x > limit)
                                        return limit + (x - limit) / (1.f + (x - limit) * 0.4f);
                                if (x < -limit)
                                        return -limit + (x + limit) / (1.f - (x + limit) * 0.4f);
                                return x;
                        };
                        busL = limiter(busL);
                        busR = limiter(busR);
                }

                float masterTrim = dbToGain(params[MASTER_TRIM_PARAM].getValue());
                busL *= masterTrim;
                busR *= masterTrim;

                float postRecordL = busL;
                float postRecordR = busR;

                bool mono = params[MONO_PARAM].getValue() > 0.5f;
                bool dim = params[DIM_PARAM].getValue() > 0.5f;
                bool clipSafe = clipSafeEnabled && (params[CLIPSAFE_PARAM].getValue() > 0.5f);

                if (mono) {
                        float monoValue = 0.5f * (busL + busR);
                        busL = busR = monoValue;
                }

                if (dim) {
                        float dimGain = std::pow(10.f, -20.f / 20.f);
                        busL *= dimGain;
                        busR *= dimGain;
                }

                if (clipSafe) {
                        auto softClip = [](float x) {
                                const float threshold = 1.2f;
                                if (std::fabs(x) <= threshold)
                                        return x;
                                float sign = x >= 0.f ? 1.f : -1.f;
                                float over = std::fabs(x) - threshold;
                                float comp = over / (1.f + over * 3.f);
                                return sign * (threshold + comp);
                        };
                        busL = softClip(busL);
                        busR = softClip(busR);
                }

                updateMeter(masterMeter, busL, busR, sampleTime);
                lights[MASTER_CLIP_LIGHT].setBrightness(masterMeter.clipTimer > 0.f ? 1.f : 0.f);

                bool routePFLToMaster = anyPFL && !outputs[PFL_OUTPUT_L].isConnected() && !outputs[PFL_OUTPUT_R].isConnected();

                float pflOutL = static_cast<float>(pflL);
                float pflOutR = static_cast<float>(pflR);
                updateMeter(pflMeter, pflOutL, pflOutR, sampleTime);
                lights[PFL_ACTIVE_LIGHT].setBrightness(anyPFL ? 1.f : 0.f);

                if (anyPFL && routePFLToMaster) {
                        outputs[MASTER_OUTPUT_L].setVoltage(pflOutL);
                        outputs[MASTER_OUTPUT_R].setVoltage(pflOutR);
                } else {
                        outputs[MASTER_OUTPUT_L].setVoltage(busL);
                        outputs[MASTER_OUTPUT_R].setVoltage(busR);
                }

                outputs[PFL_OUTPUT_L].setVoltage(pflOutL);
                outputs[PFL_OUTPUT_R].setVoltage(pflOutR);

                outputs[POST_OUTPUT_L].setVoltage(postRecordL);
                outputs[POST_OUTPUT_R].setVoltage(postRecordR);
        }

        json_t* dataToJson() override {
                json_t* root = json_object();
                json_object_set_new(root, "summingStyle", json_integer(summingStyle));
                json_object_set_new(root, "harmonicDrive", json_integer(harmonicDrive));
                json_object_set_new(root, "overtoneFocus", json_integer(overtoneFocus));
                json_object_set_new(root, "oversamplingQuality", json_integer(oversamplingQuality));
                json_object_set_new(root, "panLaw", json_integer(panLawSetting));
                json_object_set_new(root, "meterPeakHold", json_boolean(meterPeakHold));
                json_object_set_new(root, "meterResponse", json_integer(meterResponse));
                json_object_set_new(root, "headroomMode", json_integer(headroomMode));
                json_object_set_new(root, "clipSafeEnabled", json_boolean(clipSafeEnabled));
                return root;
        }

        void dataFromJson(json_t* root) override {
                if (!root)
                        return;
                if (json_t* v = json_object_get(root, "summingStyle"))
                        summingStyle = rack::math::clamp((int)json_integer_value(v), 0, 1);
                if (json_t* v = json_object_get(root, "harmonicDrive"))
                        harmonicDrive = rack::math::clamp((int)json_integer_value(v), 0, 3);
                if (json_t* v = json_object_get(root, "overtoneFocus"))
                        overtoneFocus = rack::math::clamp((int)json_integer_value(v), 0, 2);
                if (json_t* v = json_object_get(root, "oversamplingQuality"))
                        oversamplingQuality = rack::math::clamp((int)json_integer_value(v), 0, 3);
                if (json_t* v = json_object_get(root, "panLaw"))
                        panLawSetting = rack::math::clamp((int)json_integer_value(v), 0, 2);
                if (json_t* v = json_object_get(root, "meterPeakHold"))
                        meterPeakHold = json_boolean_value(v);
                if (json_t* v = json_object_get(root, "meterResponse"))
                        meterResponse = rack::math::clamp((int)json_integer_value(v), 0, 2);
                if (json_t* v = json_object_get(root, "headroomMode"))
                        headroomMode = rack::math::clamp((int)json_integer_value(v), 0, 1);
                if (json_t* v = json_object_get(root, "clipSafeEnabled"))
                        clipSafeEnabled = json_boolean_value(v);
        }
};

struct StereoMeterWidget : rack::widget::Widget {
        Xezbeth4X* module = nullptr;
        int channel = -1;
        bool drawPeakHold = true;

        StereoMeterWidget() {
                box.size = rack::mm2px(rack::Vec(8.f, 70.f));
        }

        float meterNorm(float db) const {
                        float norm = (db + 60.f) / 60.f;
                        return rack::math::clamp(norm, 0.f, 1.f);
        }

        void draw(const DrawArgs& args) override {
                nvgSave(args.vg);
                nvgBeginPath(args.vg);
                nvgRoundedRect(args.vg, 0.f, 0.f, box.size.x, box.size.y, 2.f);
                nvgFillColor(args.vg, nvgRGB(24, 24, 26));
                nvgFill(args.vg);

                const MeterState* meter = nullptr;
                MeterState dummy;
                if (module) {
                        if (channel >= 0 && channel < 4)
                                meter = &module->channelMeters[channel];
                        else if (channel == 4)
                                meter = &module->masterMeter;
                        else if (channel == 5)
                                meter = &module->pflMeter;
                }
                if (!meter)
                        meter = &dummy;

                float rmsDb = amplitudeToDb(meter->rms);
                float peakDb = amplitudeToDb(meter->truePeak);
                float holdDb = amplitudeToDb(meter->peakHold);

                float rmsNorm = meterNorm(rmsDb);
                float peakNorm = meterNorm(peakDb);
                float holdNorm = meterNorm(holdDb);

                float height = box.size.y - 6.f;
                float width = box.size.x - 6.f;
                float baseX = 3.f;
                float baseY = box.size.y - 3.f;

                float rmsHeight = height * rmsNorm;
                nvgBeginPath(args.vg);
                nvgRect(args.vg, baseX, baseY - rmsHeight, width, rmsHeight);
                nvgFillColor(args.vg, nvgRGB(64, 180, 92));
                nvgFill(args.vg);

                float peakY = baseY - height * peakNorm;
                nvgBeginPath(args.vg);
                nvgMoveTo(args.vg, baseX, peakY);
                nvgLineTo(args.vg, baseX + width, peakY);
                nvgStrokeWidth(args.vg, 1.2f);
                nvgStrokeColor(args.vg, nvgRGB(240, 240, 240));
                nvgStroke(args.vg);

                if (drawPeakHold) {
                        float holdY = baseY - height * holdNorm;
                        nvgBeginPath(args.vg);
                        nvgMoveTo(args.vg, baseX, holdY);
                        nvgLineTo(args.vg, baseX + width, holdY);
                        nvgStrokeWidth(args.vg, 1.f);
                        nvgStrokeColor(args.vg, nvgRGB(240, 128, 64));
                        nvgStroke(args.vg);
                }

                if (meter->clipTimer > 0.f) {
                        nvgBeginPath(args.vg);
                        nvgRoundedRect(args.vg, baseX, 3.f, width, 6.f, 2.f);
                        nvgFillColor(args.vg, nvgRGB(220, 32, 32));
                        nvgFill(args.vg);
                }

                nvgRestore(args.vg);
        }
};

struct Xezbeth4XWidget : rack::app::ModuleWidget {
        explicit Xezbeth4XWidget(Xezbeth4X* module) {
                setModule(module);
                setPanel(createPanel(asset::plugin(pluginInstance, "res/Xezbeth4X.svg")));

                addChild(createWidget<ScrewBlack>(rack::Vec(rack::RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(rack::Vec(box.size.x - 2 * rack::RACK_GRID_WIDTH, 0)));
                addChild(createWidget<ScrewBlack>(rack::Vec(rack::RACK_GRID_WIDTH, rack::RACK_GRID_HEIGHT - rack::RACK_GRID_WIDTH)));
                addChild(createWidget<ScrewBlack>(rack::Vec(box.size.x - 2 * rack::RACK_GRID_WIDTH, rack::RACK_GRID_HEIGHT - rack::RACK_GRID_WIDTH)));

                const float columnSpacing = 32.f;
                const float columnStart = 18.f;
                const float trimY = 25.f;
                const float panY = 55.f;
                const float buttonStartY = 82.f;
                const float buttonStep = 12.f;
                const float inputTopY = 110.f;
                const float inputBottomY = 122.f;
                const float meterHeight = 78.f;

                for (int i = 0; i < 4; ++i) {
                        float columnX = columnStart + columnSpacing * i;
                        rack::Vec trimPos = rack::mm2px(rack::Vec(columnX, trimY));
                        rack::Vec panPos = rack::mm2px(rack::Vec(columnX, panY));
                        rack::Vec mutePos = rack::mm2px(rack::Vec(columnX - 6.f, buttonStartY));
                        rack::Vec soloPos = rack::mm2px(rack::Vec(columnX, buttonStartY + buttonStep));
                        rack::Vec pflPos = rack::mm2px(rack::Vec(columnX + 6.f, buttonStartY + 2 * buttonStep));
                        rack::Vec inLPos = rack::mm2px(rack::Vec(columnX - 4.f, inputTopY));
                        rack::Vec inRPos = rack::mm2px(rack::Vec(columnX + 4.f, inputBottomY));
                        rack::Vec meterPos = rack::mm2px(rack::Vec(columnX + 11.f, trimY - 4.f));

                        addParam(createParamCentered<RoundLargeBlackKnob>(trimPos, module, Xezbeth4X::CHANNEL_TRIM_PARAM + i));
                        addParam(createParamCentered<RoundLargeBlackKnob>(panPos, module, Xezbeth4X::CHANNEL_PAN_PARAM + i));

                        addParam(createParamCentered<LEDButton>(mutePos, module, Xezbeth4X::CHANNEL_MUTE_PARAM + i));
                        addParam(createParamCentered<LEDButton>(soloPos, module, Xezbeth4X::CHANNEL_SOLO_PARAM + i));
                        addParam(createParamCentered<LEDButton>(pflPos, module, Xezbeth4X::CHANNEL_PFL_PARAM + i));

                        addInput(createInputCentered<PJ301MPort>(inLPos, module, Xezbeth4X::CHANNEL_INPUT_L + i));
                        addInput(createInputCentered<PJ301MPort>(inRPos, module, Xezbeth4X::CHANNEL_INPUT_R + i));

                        auto* meter = new StereoMeterWidget();
                        meter->module = module;
                        meter->channel = i;
                        meter->box.pos = meterPos;
                        meter->box.size = rack::mm2px(rack::Vec(8.f, meterHeight));
                        addChild(meter);

                        addChild(createLightCentered<MediumLight<GreenLight>>(rack::mm2px(rack::Vec(columnX - 10.f, buttonStartY + buttonStep)), module, Xezbeth4X::CHANNEL_POST_LIGHT + i));
                        addChild(createLightCentered<SmallLight<RedLight>>(rack::mm2px(rack::Vec(columnX - 10.f, buttonStartY + buttonStep + 6.f)), module, Xezbeth4X::CHANNEL_CLIP_LIGHT + i));
                }

                const float masterX = 140.f;
                rack::Vec masterTrimPos = rack::mm2px(rack::Vec(masterX, 30.f));
                addParam(createParamCentered<RoundLargeBlackKnob>(masterTrimPos, module, Xezbeth4X::MASTER_TRIM_PARAM));

                addParam(createParamCentered<LEDButton>(rack::mm2px(rack::Vec(masterX - 10.f, 72.f)), module, Xezbeth4X::MONO_PARAM));
                addParam(createParamCentered<LEDButton>(rack::mm2px(rack::Vec(masterX, 86.f)), module, Xezbeth4X::DIM_PARAM));
                addParam(createParamCentered<LEDButton>(rack::mm2px(rack::Vec(masterX + 10.f, 100.f)), module, Xezbeth4X::CLIPSAFE_PARAM));

                auto* masterMeter = new StereoMeterWidget();
                masterMeter->module = module;
                masterMeter->channel = 4;
                masterMeter->box.pos = rack::mm2px(rack::Vec(masterX + 18.f, 20.f));
                masterMeter->box.size = rack::mm2px(rack::Vec(10.f, 90.f));
                addChild(masterMeter);

                auto* pflMeter = new StereoMeterWidget();
                pflMeter->module = module;
                pflMeter->channel = 5;
                pflMeter->box.pos = rack::mm2px(rack::Vec(masterX + 32.f, 20.f));
                pflMeter->box.size = rack::mm2px(rack::Vec(6.f, 90.f));
                pflMeter->drawPeakHold = false;
                addChild(pflMeter);

                addChild(createLightCentered<MediumLight<RedLight>>(rack::mm2px(rack::Vec(masterX + 18.f, 16.f)), module, Xezbeth4X::MASTER_CLIP_LIGHT));
                addChild(createLightCentered<SmallLight<YellowLight>>(rack::mm2px(rack::Vec(masterX + 32.f, 16.f)), module, Xezbeth4X::PFL_ACTIVE_LIGHT));

                const float outputTopY = 110.f;
                const float outputStepY = 14.f;
                const float outputLeftX = masterX - 6.f;
                const float outputRightX = masterX + 8.f;

                addOutput(createOutputCentered<DarkPJ301MPort>(rack::mm2px(rack::Vec(outputLeftX, outputTopY)), module, Xezbeth4X::MASTER_OUTPUT_L));
                addOutput(createOutputCentered<DarkPJ301MPort>(rack::mm2px(rack::Vec(outputRightX, outputTopY)), module, Xezbeth4X::MASTER_OUTPUT_R));

                addOutput(createOutputCentered<DarkPJ301MPort>(rack::mm2px(rack::Vec(outputLeftX, outputTopY + outputStepY)), module, Xezbeth4X::PFL_OUTPUT_L));
                addOutput(createOutputCentered<DarkPJ301MPort>(rack::mm2px(rack::Vec(outputRightX, outputTopY + outputStepY)), module, Xezbeth4X::PFL_OUTPUT_R));

                addOutput(createOutputCentered<DarkPJ301MPort>(rack::mm2px(rack::Vec(outputLeftX, outputTopY + 2 * outputStepY)), module, Xezbeth4X::POST_OUTPUT_L));
                addOutput(createOutputCentered<DarkPJ301MPort>(rack::mm2px(rack::Vec(outputRightX, outputTopY + 2 * outputStepY)), module, Xezbeth4X::POST_OUTPUT_R));
        }

        void appendContextMenu(rack::ui::Menu* menu) override {
                ModuleWidget::appendContextMenu(menu);
                auto* module = dynamic_cast<Xezbeth4X*>(this->module);
                if (!module)
                        return;

                menu->addChild(new rack::ui::MenuSeparator());

                menu->addChild(createIndexPtrSubmenuItem("Summing Style", {"Clean", "Neve-Style"}, &module->summingStyle));
                menu->addChild(createIndexPtrSubmenuItem("Harmonic Drive", {"Off", "Subtle", "Medium", "Hot"}, &module->harmonicDrive));
                menu->addChild(createIndexPtrSubmenuItem("Overtone Focus", {"Even-lean", "Balanced", "Odd-lean"}, &module->overtoneFocus));
                menu->addChild(createIndexPtrSubmenuItem("Oversampling", {"1×", "2×", "4×", "8×"}, &module->oversamplingQuality));
                menu->addChild(createIndexPtrSubmenuItem("Pan Law", {"−3 dB", "−4.5 dB", "−6 dB"}, &module->panLawSetting));
                menu->addChild(createIndexPtrSubmenuItem("Meter Ballistics", {"Fast", "Medium", "Slow"}, &module->meterResponse));
                menu->addChild(createIndexPtrSubmenuItem("Headroom Model", {"Standard (+24 dB)", "Extended (+30 dB)"}, &module->headroomMode));

                menu->addChild(createCheckMenuItem("Meter peak hold", "", [module]() { return module->meterPeakHold; }, [module]() {
                        module->meterPeakHold = !module->meterPeakHold;
                }));

                menu->addChild(createCheckMenuItem("Clip-Safe on master", "", [module]() { return module->clipSafeEnabled; }, [module]() {
                        module->clipSafeEnabled = !module->clipSafeEnabled;
                }));
        }
};

} // namespace

Model* modelXezbeth4X = createModel<Xezbeth4X, Xezbeth4XWidget>("Xezbeth4X");

