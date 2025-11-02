#include "plugin.hpp"
#include "dsp/dsp.hpp"
#include <array>
#include <cmath>
#include <vector>

using namespace rack;

namespace {

static constexpr int NUM_BANDS = 3;
static constexpr int MAX_LOOKAHEAD_SAMPLES = 256; // 2ms at 128kHz

inline float dbToAmp(float db) {
        return std::pow(10.f, db / 20.f);
}

inline float ampToDb(float amp) {
        amp = std::max(amp, 1e-6f);
        return 20.f * std::log10(amp);
}

struct BiquadFilter {
        float b0 = 1.f;
        float b1 = 0.f;
        float b2 = 0.f;
        float a1 = 0.f;
        float a2 = 0.f;
        float z1 = 0.f;
        float z2 = 0.f;

        void reset() {
                z1 = z2 = 0.f;
        }

        float process(float in) {
                float out = b0 * in + z1;
                z1 = b1 * in + z2 - a1 * out;
                z2 = b2 * in - a2 * out;
                return out;
        }

        void setLowpass(float sampleRate, float freq) {
                freq = std::max(freq, 10.f);
                float omega = 2.f * M_PI * freq / sampleRate;
                float sinw = std::sin(omega);
                float cosw = std::cos(omega);
                const float Q = std::sqrt(0.5f);
                float alpha = sinw / (2.f * Q);

                float a0 = 1.f + alpha;
                b0 = (1.f - cosw) * 0.5f / a0;
                b1 = (1.f - cosw) / a0;
                b2 = b0;
                a1 = -2.f * cosw / a0;
                a2 = (1.f - alpha) / a0;
        }

        void setHighpass(float sampleRate, float freq) {
                freq = std::max(freq, 10.f);
                float omega = 2.f * M_PI * freq / sampleRate;
                float sinw = std::sin(omega);
                float cosw = std::cos(omega);
                const float Q = std::sqrt(0.5f);
                float alpha = sinw / (2.f * Q);

                float a0 = 1.f + alpha;
                b0 = (1.f + cosw) * 0.5f / a0;
                b1 = -(1.f + cosw) / a0;
                b2 = b0;
                a1 = -2.f * cosw / a0;
                a2 = (1.f - alpha) / a0;
        }
};

struct LinkwitzRiley24 {
        BiquadFilter lp1;
        BiquadFilter lp2;
        BiquadFilter hp1;
        BiquadFilter hp2;

        void setCutoff(float sampleRate, float freq) {
                lp1.setLowpass(sampleRate, freq);
                lp2.setLowpass(sampleRate, freq);
                hp1.setHighpass(sampleRate, freq);
                hp2.setHighpass(sampleRate, freq);
        }

        float processLow(float in) {
                float x = lp1.process(in);
                return lp2.process(x);
        }

        float processHigh(float in) {
                float x = hp1.process(in);
                return hp2.process(x);
        }

        void reset() {
                lp1.reset();
                lp2.reset();
                hp1.reset();
                hp2.reset();
        }
};

struct EnvelopeFollower {
        float env = 0.f;

        float process(float in, float attack, float release, float sampleRate) {
                float coeffAttack = std::exp(-1.f / std::max(attack * 0.001f * sampleRate, 1.f));
                float coeffRelease = std::exp(-1.f / std::max(release * 0.001f * sampleRate, 1.f));
                float x = std::fabs(in);
                if (x > env) {
                        env = coeffAttack * env + (1.f - coeffAttack) * x;
                } else {
                        env = coeffRelease * env + (1.f - coeffRelease) * x;
                }
                return env;
        }
};

struct LookaheadBuffer {
        std::vector<dsp::Frame<2>> buffer;
        int writeIndex = 0;
        int delaySamples = 0;

        void setDelay(int samples) {
                delaySamples = std::max(samples, 0);
                int needed = delaySamples + 8;
                if ((int)buffer.size() < needed) {
                        buffer.assign(needed, {});
                        writeIndex = 0;
                }
        }

        void reset() {
                std::fill(buffer.begin(), buffer.end(), dsp::Frame<2>());
                writeIndex = 0;
        }

        dsp::Frame<2> process(const dsp::Frame<2>& in) {
                if (buffer.empty()) {
                        return in;
                }
                buffer[writeIndex] = in;
                int readIndex = writeIndex - delaySamples;
                if (readIndex < 0)
                        readIndex += buffer.size();
                writeIndex++;
                if (writeIndex >= (int)buffer.size())
                        writeIndex = 0;
                return buffer[readIndex];
        }
};

struct BandProcessor {
        EnvelopeFollower detectors[2];
        EnvelopeFollower linkedDetector;
        float autoMakeupDb = 0.f;
        float lastGainDb[2] = {0.f, 0.f};
};

} // namespace

struct SabnockOTT : Module {
        enum ParamIds {
                PARAM_INPUT_GAIN,
                PARAM_OUTPUT_GAIN,
                PARAM_MIX,
                PARAM_DEPTH,
                PARAM_TIME,
                PARAM_UP_AMT,
                PARAM_DOWN_AMT,
                PARAM_KNEE,
                PARAM_LOOKAHEAD,
                PARAM_ST_LINK,
                PARAM_SC_HPF,
                PARAM_XOVER_LOW,
                PARAM_XOVER_HIGH,
                PARAM_BAND_AMT_L,
                PARAM_ATTACK_L,
                PARAM_RELEASE_L,
                PARAM_MAKEUP_L,
                PARAM_AUTO_MAKEUP_L,
                PARAM_BAND_AMT_M,
                PARAM_ATTACK_M,
                PARAM_RELEASE_M,
                PARAM_MAKEUP_M,
                PARAM_AUTO_MAKEUP_M,
                PARAM_BAND_AMT_H,
                PARAM_ATTACK_H,
                PARAM_RELEASE_H,
                PARAM_MAKEUP_H,
                PARAM_AUTO_MAKEUP_H,
                NUM_PARAMS
        };

        enum InputIds {
                INPUT_IN_L,
                INPUT_IN_R,
                INPUT_SC_L,
                INPUT_SC_R,
                INPUT_CV_DEPTH,
                INPUT_CV_MIX,
                INPUT_CV_TIME,
                INPUT_CV_UP,
                INPUT_CV_DOWN,
                INPUT_CV_XLO,
                INPUT_CV_XHI,
                NUM_INPUTS
        };

        enum OutputIds {
                OUTPUT_OUT_L,
                OUTPUT_OUT_R,
                OUTPUT_GR_SUM,
                OUTPUT_LOW_TAP,
                OUTPUT_MID_TAP,
                OUTPUT_HIGH_TAP,
                NUM_OUTPUTS
        };

        enum LightIds {
                LIGHT_GR_LOW_UP_0,
                LIGHT_GR_LOW_UP_1,
                LIGHT_GR_LOW_UP_2,
                LIGHT_GR_LOW_UP_3,
                LIGHT_GR_LOW_UP_4,
                LIGHT_GR_LOW_DOWN_0,
                LIGHT_GR_LOW_DOWN_1,
                LIGHT_GR_LOW_DOWN_2,
                LIGHT_GR_LOW_DOWN_3,
                LIGHT_GR_LOW_DOWN_4,
                LIGHT_GR_MID_UP_0,
                LIGHT_GR_MID_UP_1,
                LIGHT_GR_MID_UP_2,
                LIGHT_GR_MID_UP_3,
                LIGHT_GR_MID_UP_4,
                LIGHT_GR_MID_DOWN_0,
                LIGHT_GR_MID_DOWN_1,
                LIGHT_GR_MID_DOWN_2,
                LIGHT_GR_MID_DOWN_3,
                LIGHT_GR_MID_DOWN_4,
                LIGHT_GR_HIGH_UP_0,
                LIGHT_GR_HIGH_UP_1,
                LIGHT_GR_HIGH_UP_2,
                LIGHT_GR_HIGH_UP_3,
                LIGHT_GR_HIGH_UP_4,
                LIGHT_GR_HIGH_DOWN_0,
                LIGHT_GR_HIGH_DOWN_1,
                LIGHT_GR_HIGH_DOWN_2,
                LIGHT_GR_HIGH_DOWN_3,
                LIGHT_GR_HIGH_DOWN_4,
                LIGHT_MAIN_OUT_L_0,
                LIGHT_MAIN_OUT_L_1,
                LIGHT_MAIN_OUT_L_2,
                LIGHT_MAIN_OUT_L_3,
                LIGHT_MAIN_OUT_L_4,
                LIGHT_MAIN_OUT_R_0,
                LIGHT_MAIN_OUT_R_1,
                LIGHT_MAIN_OUT_R_2,
                LIGHT_MAIN_OUT_R_3,
                LIGHT_MAIN_OUT_R_4,
                LIGHT_SC_ACTIVE,
                NUM_LIGHTS
        };

        std::array<LinkwitzRiley24, 2> lowCrossover;
        std::array<LinkwitzRiley24, 2> highCrossover;
        std::array<LinkwitzRiley24, 2> scLowCrossover;
        std::array<LinkwitzRiley24, 2> scHighCrossover;
        std::array<BandProcessor, NUM_BANDS> bands;
        std::array<EnvelopeFollower, 2> outputMeters;
        BiquadFilter scHPF[2][2];
        LookaheadBuffer lookahead;
        float lastSampleRate = 0.f;
        float currentXoverLow = -1.f;
        float currentXoverHigh = -1.f;
        float currentScHpf = -1.f;

        SabnockOTT() {
                config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

                configParam(PARAM_INPUT_GAIN, -24.f, 24.f, 0.f, "Input gain", " dB");
                configParam(PARAM_OUTPUT_GAIN, -24.f, 24.f, 0.f, "Output gain", " dB");
                configParam(PARAM_MIX, 0.f, 1.f, 1.f, "Mix", "%", 0.f, 100.f);
                configParam(PARAM_DEPTH, 0.f, 2.f, 1.f, "Depth", "%", 0.f, 100.f);
                configParam(PARAM_TIME, 0.25f, 4.f, 1.f, "Time", "x");
                configParam(PARAM_UP_AMT, 0.f, 1.f, 0.7f, "Up amount", "%", 0.f, 100.f);
                configParam(PARAM_DOWN_AMT, 0.f, 1.f, 1.f, "Down amount", "%", 0.f, 100.f);
                configParam(PARAM_KNEE, 0.f, 1.f, 0.6f, "Knee");
                configSwitch(PARAM_LOOKAHEAD, 0.f, 3.f, 2.f, "Lookahead", {"Off", "0.5 ms", "1 ms", "2 ms"});
                configSwitch(PARAM_ST_LINK, 0.f, 1.f, 0.f, "Stereo link", {"Linked", "Unlinked"});
                configParam(PARAM_SC_HPF, 20.f, 300.f, 120.f, "Sidechain HPF", " Hz");
                configParam(PARAM_XOVER_LOW, 100.f, 300.f, 120.f, "Low crossover", " Hz");
                configParam(PARAM_XOVER_HIGH, 1000.f, 6000.f, 2400.f, "High crossover", " Hz");

                configParam(PARAM_BAND_AMT_L, 0.f, 1.5f, 1.2f, "Low band depth", "%", 0.f, 100.f);
                configParam(PARAM_ATTACK_L, 0.1f, 50.f, 1.5f, "Low attack", " ms");
                configParam(PARAM_RELEASE_L, 10.f, 1000.f, 120.f, "Low release", " ms");
                configParam(PARAM_MAKEUP_L, -12.f, 12.f, 0.f, "Low makeup", " dB");
                configSwitch(PARAM_AUTO_MAKEUP_L, 0.f, 1.f, 1.f, "Low auto makeup", {"Off", "On"});

                configParam(PARAM_BAND_AMT_M, 0.f, 1.5f, 1.f, "Mid band depth", "%", 0.f, 100.f);
                configParam(PARAM_ATTACK_M, 0.1f, 50.f, 1.5f, "Mid attack", " ms");
                configParam(PARAM_RELEASE_M, 10.f, 1000.f, 120.f, "Mid release", " ms");
                configParam(PARAM_MAKEUP_M, -12.f, 12.f, 0.f, "Mid makeup", " dB");
                configSwitch(PARAM_AUTO_MAKEUP_M, 0.f, 1.f, 1.f, "Mid auto makeup", {"Off", "On"});

                configParam(PARAM_BAND_AMT_H, 0.f, 1.5f, 1.1f, "High band depth", "%", 0.f, 100.f);
                configParam(PARAM_ATTACK_H, 0.1f, 50.f, 1.5f, "High attack", " ms");
                configParam(PARAM_RELEASE_H, 10.f, 1000.f, 120.f, "High release", " ms");
                configParam(PARAM_MAKEUP_H, -12.f, 12.f, 0.f, "High makeup", " dB");
                configSwitch(PARAM_AUTO_MAKEUP_H, 0.f, 1.f, 1.f, "High auto makeup", {"Off", "On"});

                configInput(INPUT_IN_L, "Left input");
                configInput(INPUT_IN_R, "Right input");
                configInput(INPUT_SC_L, "Sidechain left");
                configInput(INPUT_SC_R, "Sidechain right");
                configInput(INPUT_CV_DEPTH, "Depth CV");
                configInput(INPUT_CV_MIX, "Mix CV");
                configInput(INPUT_CV_TIME, "Time CV");
                configInput(INPUT_CV_UP, "Up amount CV");
                configInput(INPUT_CV_DOWN, "Down amount CV");
                configInput(INPUT_CV_XLO, "Low crossover CV");
                configInput(INPUT_CV_XHI, "High crossover CV");

                configOutput(OUTPUT_OUT_L, "Left output");
                configOutput(OUTPUT_OUT_R, "Right output");
                configOutput(OUTPUT_GR_SUM, "Gain reduction sum");
                configOutput(OUTPUT_LOW_TAP, "Low band tap");
                configOutput(OUTPUT_MID_TAP, "Mid band tap");
                configOutput(OUTPUT_HIGH_TAP, "High band tap");
        }

        void onSampleRateChange() override {
                Module::onSampleRateChange();
                lastSampleRate = APP->engine->getSampleRate();
                for (auto& l : lowCrossover)
                        l.reset();
                for (auto& h : highCrossover)
                        h.reset();
                for (auto& l : scLowCrossover)
                        l.reset();
                for (auto& h : scHighCrossover)
                        h.reset();
                for (auto& b : bands)
                        b.detector.env = 0.f;
                for (auto& meter : outputMeters)
                        meter.env = 0.f;
                lookahead.reset();
                currentXoverLow = -1.f;
                currentXoverHigh = -1.f;
                currentScHpf = -1.f;
        }

        void process(const ProcessArgs& args) override {
                float sampleRate = args.sampleRate;
                if (sampleRate != lastSampleRate) {
                        onSampleRateChange();
                        lastSampleRate = sampleRate;
                }

                float inputGain = dbToAmp(params[PARAM_INPUT_GAIN].getValue());
                float outputGain = dbToAmp(params[PARAM_OUTPUT_GAIN].getValue());

                float depthParam = params[PARAM_DEPTH].getValue();
                if (inputs[INPUT_CV_DEPTH].isConnected()) {
                        float cv = rack::math::clamp(inputs[INPUT_CV_DEPTH].getVoltage() * 0.5f + depthParam, 0.f, 2.f);
                        depthParam = cv;
                }
                float depthScale = depthParam;

                float mixParam = params[PARAM_MIX].getValue();
                if (inputs[INPUT_CV_MIX].isConnected()) {
                        float cv = rack::math::clamp(inputs[INPUT_CV_MIX].getVoltage() * 0.5f + mixParam, 0.f, 1.f);
                        mixParam = cv;
                }

                float timeScalar = params[PARAM_TIME].getValue();
                if (inputs[INPUT_CV_TIME].isConnected()) {
                        float cv = inputs[INPUT_CV_TIME].getVoltage();
                        timeScalar *= std::pow(2.f, rack::math::clamp(cv * 0.2f, -2.f, 2.f));
                        timeScalar = rack::math::clamp(timeScalar, 0.1f, 8.f);
                }

                float upAmt = params[PARAM_UP_AMT].getValue();
                if (inputs[INPUT_CV_UP].isConnected()) {
                        upAmt = rack::math::clamp(upAmt + inputs[INPUT_CV_UP].getVoltage() * 0.5f, 0.f, 1.f);
                }

                float downAmt = params[PARAM_DOWN_AMT].getValue();
                if (inputs[INPUT_CV_DOWN].isConnected()) {
                        downAmt = rack::math::clamp(downAmt + inputs[INPUT_CV_DOWN].getVoltage() * 0.5f, 0.f, 1.f);
                }

                float kneeParam = params[PARAM_KNEE].getValue();
                float kneeDb = rack::math::rescale(kneeParam, 0.f, 1.f, 2.f, 24.f);

                float xOverLow = params[PARAM_XOVER_LOW].getValue();
                if (inputs[INPUT_CV_XLO].isConnected()) {
                        xOverLow = rack::math::clamp(xOverLow + inputs[INPUT_CV_XLO].getVoltage() * 40.f, 100.f, 300.f);
                }

                float xOverHigh = params[PARAM_XOVER_HIGH].getValue();
                if (inputs[INPUT_CV_XHI].isConnected()) {
                        xOverHigh = rack::math::clamp(xOverHigh + inputs[INPUT_CV_XHI].getVoltage() * 500.f, 1000.f, 6000.f);
                }
                xOverHigh = std::max(xOverHigh, xOverLow * 1.5f);

                if (std::fabs(xOverLow - currentXoverLow) > 0.01f || sampleRate != lastSampleRate) {
                        for (int c = 0; c < 2; ++c) {
                                lowCrossover[c].setCutoff(sampleRate, xOverLow);
                                scLowCrossover[c].setCutoff(sampleRate, xOverLow);
                        }
                        currentXoverLow = xOverLow;
                }

                if (std::fabs(xOverHigh - currentXoverHigh) > 0.01f || sampleRate != lastSampleRate) {
                        for (int c = 0; c < 2; ++c) {
                                highCrossover[c].setCutoff(sampleRate, xOverHigh);
                                scHighCrossover[c].setCutoff(sampleRate, xOverHigh);
                        }
                        currentXoverHigh = xOverHigh;
                }

                float scHpfValue = params[PARAM_SC_HPF].getValue();
                if (std::fabs(scHpfValue - currentScHpf) > 0.1f || sampleRate != lastSampleRate) {
                        for (int c = 0; c < 2; ++c) {
                                for (int stage = 0; stage < 2; ++stage) {
                                        scHPF[c][stage].setHighpass(sampleRate, scHpfValue);
                                }
                        }
                        currentScHpf = scHpfValue;
                }

                int lookaheadMode = (int)std::round(params[PARAM_LOOKAHEAD].getValue());
                int lookaheadSamples = 0;
                switch (lookaheadMode) {
                case 1:
                        lookaheadSamples = (int)std::round(0.0005f * sampleRate);
                        break;
                case 2:
                        lookaheadSamples = (int)std::round(0.001f * sampleRate);
                        break;
                case 3:
                        lookaheadSamples = (int)std::round(0.002f * sampleRate);
                        break;
                default:
                        lookaheadSamples = 0;
                        break;
                }
                lookaheadSamples = std::min(lookaheadSamples, MAX_LOOKAHEAD_SAMPLES);
                lookahead.setDelay(lookaheadSamples);

                bool stereoLinked = (params[PARAM_ST_LINK].getValue() < 0.5f);

                dsp::Frame<2> inFrame;
                inFrame.samples[0] = inputs[INPUT_IN_L].getVoltage() * inputGain;
                inFrame.samples[1] = inputs[INPUT_IN_R].getVoltage() * inputGain;

                dsp::Frame<2> scFrame;
                if (inputs[INPUT_SC_L].isConnected() || inputs[INPUT_SC_R].isConnected()) {
                        scFrame.samples[0] = inputs[INPUT_SC_L].getNormalVoltage(inFrame.samples[0]);
                        scFrame.samples[1] = inputs[INPUT_SC_R].getNormalVoltage(inFrame.samples[1]);
                        lights[LIGHT_SC_ACTIVE].setBrightness(1.f);
                } else {
                        scFrame = inFrame;
                        lights[LIGHT_SC_ACTIVE].setBrightness(0.f);
                }

                for (int c = 0; c < 2; ++c) {
                        float x = scHPF[c][0].process(scFrame.samples[c]);
                        x = scHPF[c][1].process(x);
                        scFrame.samples[c] = x;
                }

                dsp::Frame<2> delayedFrame = lookahead.process(inFrame);

                float bandSamples[NUM_BANDS][2];
                float scBandSamples[NUM_BANDS][2];

                for (int c = 0; c < 2; ++c) {
                        float low = lowCrossover[c].processLow(delayedFrame.samples[c]);
                        float remainder = delayedFrame.samples[c] - low;
                        float high = highCrossover[c].processHigh(remainder);
                        float mid = remainder - high;
                        bandSamples[0][c] = low;
                        bandSamples[1][c] = mid;
                        bandSamples[2][c] = high;

                        float scLow = scLowCrossover[c].processLow(scFrame.samples[c]);
                        float scRemainder = scFrame.samples[c] - scLow;
                        float scHigh = scHighCrossover[c].processHigh(scRemainder);
                        float scMid = scRemainder - scHigh;
                        scBandSamples[0][c] = scLow;
                        scBandSamples[1][c] = scMid;
                        scBandSamples[2][c] = scHigh;
                }

                auto computeBaseGain = [&](float envDb, float thresholdDb, float kneeDb, float downRatio, float upRatio,
                                           float depthScale, float bandAmt, float& grDown, float& grUp) {
                        float pivot = thresholdDb;
                        float kneeHalf = kneeDb * 0.5f;
                        float downGainDb = 0.f;
                        float upGainDb = 0.f;

                        if (envDb > pivot - kneeHalf) {
                                if (envDb > pivot + kneeHalf) {
                                        float over = envDb - pivot;
                                        float compressed = pivot + over / std::max(downRatio, 1.f);
                                        downGainDb = (compressed - envDb);
                                } else if (kneeDb > 0.f) {
                                        float diff = envDb - (pivot - kneeHalf);
                                        float blend = diff / std::max(kneeDb, 1e-6f);
                                        float over = envDb - pivot;
                                        float compressed = pivot + over / std::max(downRatio, 1.f);
                                        float soft = envDb + (compressed - envDb) * blend;
                                        downGainDb = soft - envDb;
                                }
                        }

                        if (envDb < pivot + kneeHalf) {
                                if (envDb < pivot - kneeHalf) {
                                        float under = pivot - envDb;
                                        float expandedDiff = under * std::min(upRatio, 1.f);
                                        float target = pivot - expandedDiff;
                                        upGainDb = target - envDb;
                                } else if (kneeDb > 0.f) {
                                        float diff = (pivot + kneeHalf) - envDb;
                                        float blend = diff / std::max(kneeDb, 1e-6f);
                                        float under = pivot - envDb;
                                        float expandedDiff = under * std::min(upRatio, 1.f);
                                        float target = pivot - expandedDiff;
                                        float soft = envDb + (target - envDb) * blend;
                                        upGainDb = soft - envDb;
                                }
                        }

                        downGainDb *= depthScale * bandAmt;
                        upGainDb *= depthScale * bandAmt;

                        grDown = downGainDb;
                        grUp = upGainDb;
                        return downGainDb + upGainDb;
                };

                float thresholdDb = -18.f;
                float globalGrDown = 0.f;
                float globalGrUp = 0.f;
                dsp::Frame<2> processedBands[NUM_BANDS];

                for (int band = 0; band < NUM_BANDS; ++band) {
                        float bandAmt = params[PARAM_BAND_AMT_L + band * 5].getValue();
                        float attack = params[PARAM_ATTACK_L + band * 5].getValue() * timeScalar;
                        float release = params[PARAM_RELEASE_L + band * 5].getValue() * timeScalar;
                        float makeupDb = params[PARAM_MAKEUP_L + band * 5].getValue();
                        bool autoMakeup = params[PARAM_AUTO_MAKEUP_L + band * 5].getValue() > 0.5f;

                        float downRatio = 1.f + depthScale * downAmt * 9.f;
                        float upRatio = 1.f / (1.f + depthScale * upAmt * 4.f);

                        float envDb[2];
                        if (stereoLinked) {
                                float detect = 0.5f * (scBandSamples[band][0] + scBandSamples[band][1]);
                                float env = bands[band].linkedDetector.process(detect, attack, release, sampleRate);
                                float db = ampToDb(env);
                                envDb[0] = envDb[1] = db;
                        } else {
                                for (int c = 0; c < 2; ++c) {
                                        float env = bands[band].detectors[c].process(scBandSamples[band][c], attack, release, sampleRate);
                                        envDb[c] = ampToDb(env);
                                }
                        }

                        float baseGainDb[2];
                        float grDownBand[2];
                        float grUpBand[2];
                        for (int c = 0; c < 2; ++c) {
                                baseGainDb[c] = computeBaseGain(envDb[c], thresholdDb, kneeDb, downRatio, upRatio, depthScale, bandAmt, grDownBand[c], grUpBand[c]);
                        }

                        float gainOffset = makeupDb;
                        if (autoMakeup) {
                                float avgDown = 0.5f * (grDownBand[0] + grDownBand[1]);
                                float avgUp = 0.5f * (grUpBand[0] + grUpBand[1]);
                                float target = -(0.6f * avgDown) - 0.25f * avgUp;
                                bands[band].autoMakeupDb += 0.02f * (target - bands[band].autoMakeupDb);
                                gainOffset = bands[band].autoMakeupDb;
                        } else {
                                bands[band].autoMakeupDb = 0.f;
                        }

                        for (int c = 0; c < 2; ++c) {
                                float totalGainDb = rack::math::clamp(baseGainDb[c] + gainOffset, -48.f, 48.f);
                                bands[band].lastGainDb[c] = totalGainDb;
                                float gain = dbToAmp(totalGainDb);
                                processedBands[band].samples[c] = bandSamples[band][c] * gain;
                                globalGrDown += grDownBand[c];
                                globalGrUp += grUpBand[c];
                        }
                }

                dsp::Frame<2> wet;
                for (int c = 0; c < 2; ++c) {
                        wet.samples[c] = processedBands[0].samples[c] + processedBands[1].samples[c] + processedBands[2].samples[c];
                }

                float dryWet = mixParam;
                dsp::Frame<2> out;
                out.samples[0] = rack::math::crossfade(inFrame.samples[0], wet.samples[0], dryWet) * outputGain;
                out.samples[1] = rack::math::crossfade(inFrame.samples[1], wet.samples[1], dryWet) * outputGain;

                outputs[OUTPUT_OUT_L].setVoltage(out.samples[0]);
                outputs[OUTPUT_OUT_R].setVoltage(out.samples[1]);

                outputs[OUTPUT_LOW_TAP].setVoltage(processedBands[0].samples[0]);
                outputs[OUTPUT_MID_TAP].setVoltage(processedBands[1].samples[0]);
                outputs[OUTPUT_HIGH_TAP].setVoltage(processedBands[2].samples[0]);

                float grSumDb = (globalGrDown + globalGrUp) * 0.5f;
                grSumDb = rack::math::clamp(grSumDb, -15.f, 15.f);
                outputs[OUTPUT_GR_SUM].setVoltage(rack::math::rescale(grSumDb, -15.f, 15.f, -5.f, 5.f));

                int bandUpBase[NUM_BANDS] = {
                        LIGHT_GR_LOW_UP_0,
                        LIGHT_GR_MID_UP_0,
                        LIGHT_GR_HIGH_UP_0
                };
                int bandDownBase[NUM_BANDS] = {
                        LIGHT_GR_LOW_DOWN_0,
                        LIGHT_GR_MID_DOWN_0,
                        LIGHT_GR_HIGH_DOWN_0
                };

                for (int band = 0; band < NUM_BANDS; ++band) {
                        float avgGainDb = 0.5f * (bands[band].lastGainDb[0] + bands[band].lastGainDb[1]);
                        float grDownDb = std::min(0.f, avgGainDb);
                        float grUpDb = std::max(0.f, avgGainDb);
                        float downMagnitude = std::min(std::fabs(grDownDb) / 12.f, 1.f);
                        float upMagnitude = std::min(std::fabs(grUpDb) / 12.f, 1.f);
                        for (int i = 0; i < 5; ++i) {
                                float threshold = (float)(i + 1) / 5.f;
                                lights[bandUpBase[band] + i].setBrightness(upMagnitude > threshold ? 1.f : upMagnitude > threshold - 0.2f ? 0.6f : 0.f);
                                lights[bandDownBase[band] + i].setBrightness(downMagnitude > threshold ? 1.f : downMagnitude > threshold - 0.2f ? 0.6f : 0.f);
                        }
                }

                for (int c = 0; c < 2; ++c) {
                        float env = outputMeters[c].process(out.samples[c], 5.f, 100.f, sampleRate);
                        float envDb = ampToDb(env);
                        float magnitude = rack::math::rescale(envDb, -30.f, 6.f, 0.f, 1.f);
                        for (int i = 0; i < 5; ++i) {
                                float threshold = (float)(i + 1) / 5.f;
                                int lightIndex = (c == 0 ? LIGHT_MAIN_OUT_L_0 : LIGHT_MAIN_OUT_R_0) + i;
                                lights[lightIndex].setBrightness(magnitude > threshold ? 1.f : magnitude > threshold - 0.2f ? 0.6f : 0.f);
                        }
                }
        }
};

struct SabnockOTTWidget : ModuleWidget {
        SabnockOTTWidget(SabnockOTT* module) {
                setModule(module);
                setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/SabnockOTT.svg")));

                using namespace rack::componentlibrary;

                auto mm = [](float x, float y) { return rack::mm2px(rack::Vec(x, y)); };

                addChild(createWidget<ScrewBlack>(mm(2.0f, 2.0f)));
                addChild(createWidget<ScrewBlack>(mm(2.0f, 124.5f)));
                addChild(createWidget<ScrewBlack>(mm(79.0f, 2.0f)));
                addChild(createWidget<ScrewBlack>(mm(79.0f, 124.5f)));

                addParam(createParamCentered<RoundHugeBlackKnob>(mm(15.0f, 18.0f), module, SabnockOTT::PARAM_INPUT_GAIN));
                addParam(createParamCentered<RoundHugeBlackKnob>(mm(66.0f, 18.0f), module, SabnockOTT::PARAM_OUTPUT_GAIN));

                addParam(createParamCentered<RoundLargeBlackKnob>(mm(15.0f, 41.0f), module, SabnockOTT::PARAM_MIX));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm(40.5f, 41.0f), module, SabnockOTT::PARAM_DEPTH));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm(66.0f, 41.0f), module, SabnockOTT::PARAM_TIME));

                addParam(createParamCentered<RoundLargeBlackKnob>(mm(15.0f, 64.0f), module, SabnockOTT::PARAM_UP_AMT));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm(40.5f, 64.0f), module, SabnockOTT::PARAM_DOWN_AMT));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm(66.0f, 64.0f), module, SabnockOTT::PARAM_KNEE));

                addParam(createParamCentered<RoundSmallBlackKnob>(mm(13.0f, 86.0f), module, SabnockOTT::PARAM_LOOKAHEAD));
                addParam(createParamCentered<CKSS>(mm(23.5f, 86.0f), module, SabnockOTT::PARAM_ST_LINK));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm(40.5f, 86.0f), module, SabnockOTT::PARAM_SC_HPF));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm(59.0f, 86.0f), module, SabnockOTT::PARAM_XOVER_LOW));
                addParam(createParamCentered<RoundLargeBlackKnob>(mm(72.5f, 86.0f), module, SabnockOTT::PARAM_XOVER_HIGH));

                const float bandX = 59.0f;
                const float bandSpacingY = 23.0f;
                for (int band = 0; band < NUM_BANDS; ++band) {
                        float y = 104.0f + bandSpacingY * band;
                        addParam(createParamCentered<RoundSmallBlackKnob>(mm(bandX, y), module, SabnockOTT::PARAM_BAND_AMT_L + band * 5));
                        addParam(createParamCentered<RoundSmallBlackKnob>(mm(bandX + 12.0f, y), module, SabnockOTT::PARAM_ATTACK_L + band * 5));
                        addParam(createParamCentered<RoundSmallBlackKnob>(mm(bandX + 24.0f, y), module, SabnockOTT::PARAM_RELEASE_L + band * 5));
                        addParam(createParamCentered<RoundSmallBlackKnob>(mm(bandX + 36.0f, y), module, SabnockOTT::PARAM_MAKEUP_L + band * 5));
                        addParam(createParamCentered<CKSS>(mm(bandX + 46.0f, y - 6.0f), module, SabnockOTT::PARAM_AUTO_MAKEUP_L + band * 5));
                }

                const float meterBaseX = 40.5f;
                const float meterTopY = 104.0f;
                const float meterSpacing = 4.2f;
                for (int i = 0; i < 5; ++i) {
                        addChild(createLightCentered<SmallLight<GreenLight>>(mm(11.0f, meterTopY + i * meterSpacing), module, SabnockOTT::LIGHT_MAIN_OUT_L_0 + i));
                        addChild(createLightCentered<SmallLight<GreenLight>>(mm(16.0f, meterTopY + i * meterSpacing), module, SabnockOTT::LIGHT_MAIN_OUT_R_0 + i));
                }

                for (int band = 0; band < NUM_BANDS; ++band) {
                        float x = meterBaseX + (band - 1) * 6.0f;
                        for (int i = 0; i < 5; ++i) {
                                float upY = meterTopY + (4 - i) * meterSpacing;
                                float downY = meterTopY + (5 + i) * meterSpacing;
                                int upId = (band == 0 ? SabnockOTT::LIGHT_GR_LOW_UP_0 : band == 1 ? SabnockOTT::LIGHT_GR_MID_UP_0 : SabnockOTT::LIGHT_GR_HIGH_UP_0) + i;
                                int downId = (band == 0 ? SabnockOTT::LIGHT_GR_LOW_DOWN_0 : band == 1 ? SabnockOTT::LIGHT_GR_MID_DOWN_0 : SabnockOTT::LIGHT_GR_HIGH_DOWN_0) + i;
                                addChild(createLightCentered<SmallLight<BlueLight>>(mm(x, upY), module, downId));
                                addChild(createLightCentered<SmallLight<CyanLight>>(mm(x, downY), module, upId));
                        }
                }

                addChild(createLightCentered<MediumLight<YellowLight>>(mm(23.5f, 96.0f), module, SabnockOTT::LIGHT_SC_ACTIVE));

                float jackY = 123.0f;
                float jackSpacing = 4.5f;
                float startX = 6.0f;
                int jackIndex = 0;
                auto jackX = [&](int index) { return startX + jackSpacing * index; };

                addInput(createInputCentered<PJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::INPUT_IN_L));
                addInput(createInputCentered<PJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::INPUT_IN_R));
                addInput(createInputCentered<PJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::INPUT_SC_L));
                addInput(createInputCentered<PJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::INPUT_SC_R));
                addInput(createInputCentered<PJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::INPUT_CV_DEPTH));
                addInput(createInputCentered<PJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::INPUT_CV_MIX));
                addInput(createInputCentered<PJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::INPUT_CV_TIME));
                addInput(createInputCentered<PJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::INPUT_CV_UP));
                addInput(createInputCentered<PJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::INPUT_CV_DOWN));
                addInput(createInputCentered<PJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::INPUT_CV_XLO));
                addInput(createInputCentered<PJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::INPUT_CV_XHI));

                addOutput(createOutputCentered<DarkPJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::OUTPUT_OUT_L));
                addOutput(createOutputCentered<DarkPJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::OUTPUT_OUT_R));
                addOutput(createOutputCentered<DarkPJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::OUTPUT_GR_SUM));
                addOutput(createOutputCentered<DarkPJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::OUTPUT_LOW_TAP));
                addOutput(createOutputCentered<DarkPJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::OUTPUT_MID_TAP));
                addOutput(createOutputCentered<DarkPJ301MPort>(mm(jackX(jackIndex++), jackY), module, SabnockOTT::OUTPUT_HIGH_TAP));
        }
};

Model* modelSabnockOTT = createModel<SabnockOTT, SabnockOTTWidget>("SabnockOTT");
