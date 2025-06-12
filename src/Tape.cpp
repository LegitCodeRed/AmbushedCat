#include "plugin.hpp"
#include "dsp/dsp.hpp"
#include "dsp/p42.hpp"
#include <dsp/resampler.hpp>
#include <cmath>
#define MAX_DELAY_SAMPLES 512
#define TAPE_DELAY_BUFFER_SIZE 2048
#define BASE_DELAY_SAMPLES 64

static const float modeBias[3] = {1.4f, 1.2f, 1.0f};
static const float modeDrive[3] = {1.2f, 1.0f, 0.8f};
static const float modeTone[3] = {0.8f, 0.9f, 1.0f};
static const float modeHiss[3] = {1.2f, 0.8f, 0.5f};
static const float modeStatic[3] = {1.1f, 1.0f, 0.7f};
static const float modeWF[3] = {1.2f, 1.0f, 0.8f};
static const float modeBump[3] = {1.2f, 1.0f, 0.7f};
// Amount of "glue" compression for each tape mode
static const float modeGlue[3] = {1.0f, 0.8f, 0.6f};

// Tape style noise scaling: index 0 = Vintage (no noise floor),
// index 1 = Classic (moderate noise floor),
// index 2 = Modern (very quiet noise floor),
// index 3 = Soulless (effectively silent)
static const float styleNoiseScale[4] = {1.f, 0.5f, 0.05f, 0.0f};

// Tape speed scaling. Index 0 = 7.5 IPS, 1 = 15 IPS, 2 = 30 IPS
static const float speedCutoffScale[3] = {0.6f, 0.8f, 1.0f};
static const float speedModScale[3]    = {2.0f, 1.0f, 0.5f};
static const float speedNoiseScale[3]  = {1.5f, 1.0f, 0.8f};

struct EqCurve {
    float lowFreq;
    float lowGainDb;
    float highFreq;
    float highGainDb;
};

static const EqCurve eqCurves[] = {
    {80.f, 4.f,  8000.f, -2.f}, // Bass
    {120.f, -3.f, 10000.f, 6.f}, // Highs
    {80.f, 3.f, 12000.f, 3.f}   // Mix
};

struct TapeAging {
	float eqWarmState = 0.f;
	float eqDrift = 1.f;

	float printBuffer[MAX_DELAY_SAMPLES * 2] = {};
	int printIndex = 0;

	void tickDrift() {
		eqWarmState += 0.00001f;
		eqDrift = 1.f + 0.05f * std::sin(eqWarmState);
	}

	void storePrint(float sample) {
		printBuffer[printIndex] = sample;
		printIndex = (printIndex + 1) % (MAX_DELAY_SAMPLES * 2);
	}

        float getPrintEcho() const {
                // The original implementation returned a small delayed sample
                // to emulate print-through on tape. This subtle echo was
                // reported to produce audible artifacts, especially when the
                // module is used on a final mix. Disable the effect by returning
                // silence.
                return 0.f;
        }
};

class TapeDelayBuffer {
public:
        float buffer[PORT_MAX_CHANNELS][TAPE_DELAY_BUFFER_SIZE] = {};
        int writeIndex = 0;
        float modSmooth[PORT_MAX_CHANNELS] = {};
        float prevOut[PORT_MAX_CHANNELS] = {};
        float prevDelay[PORT_MAX_CHANNELS] = {};

	float hermiteInterpolate(float a, float b, float c, float d, float t) {
		float t2 = t * t;
		float t3 = t2 * t;
		return 0.5f * (
			(2.f * b) +
			(-a + c) * t +
			(2.f * a - 5.f * b + 4.f * c - d) * t2 +
			(-a + 3.f * b - 3.f * c + d) * t3
		);
	}

	float readModulated(float input, float delaySamples, int channel, float sampleRate) {
		float totalDelay = BASE_DELAY_SAMPLES + delaySamples;
		// Ensure enough buffer history for Hermite (needs at least 3 past samples)
		totalDelay = rack::math::clamp(totalDelay, 4.f, (float)(TAPE_DELAY_BUFFER_SIZE - 4));

		// Smoothing to prevent rapid pointer jumps (already good)
		float smoothing = 0.002f * 44100.f / sampleRate;
		modSmooth[channel] += smoothing * (totalDelay - modSmooth[channel]);

		// === WRITE INPUT ===
		buffer[channel][writeIndex] = input;

		// === FLOAT READ INDEX ===
		float floatIndex = (float)writeIndex - modSmooth[channel];
		while (floatIndex < 0.f)
			floatIndex += (float)TAPE_DELAY_BUFFER_SIZE;
		floatIndex = fmodf(floatIndex, (float)TAPE_DELAY_BUFFER_SIZE);

		int index0 = (int)floorf(floatIndex);
		float frac = floatIndex - (float)index0;

		// Get wrapped indices for Hermite interpolation
		int indexM1 = (index0 - 1 + TAPE_DELAY_BUFFER_SIZE) % TAPE_DELAY_BUFFER_SIZE;
		int index1  = (index0 + 1) % TAPE_DELAY_BUFFER_SIZE;
		int index2  = (index0 + 2) % TAPE_DELAY_BUFFER_SIZE;

		float a = buffer[channel][indexM1];
		float b = buffer[channel][index0];
		float c = buffer[channel][index1];
		float d = buffer[channel][index2];

		float out = hermiteInterpolate(a, b, c, d, frac);

		// === ADVANCE WRITE POINTER ===
		writeIndex = (writeIndex + 1) % TAPE_DELAY_BUFFER_SIZE;
		return out;
	}
};

class WowFlutterModulator {
public:
    // WOW state
    float wowPhase = 0.f;
    float wowFreqMod = 0.f;
    float wowAmp = 1.f;
    float wowFreqTarget = 0.f;
    float wowAmpTarget = 1.f;
    int wowTimer = 0;
    float wowLFOFiltered = 0.f;

    // FLUTTER state (restored expressive style)
    float flutterPhase = 0.f;
    float flutterFreqMod = 0.f;
    float flutterFreqTarget = 0.f;
    float flutterAmp = 1.f;
    float flutterAmpTarget = 1.f;
    int flutterTimer = 0;
    float flutterLFO = 0.f;

    // Output smoothing
    float smoothed = 0.f;
    float smoothed2 = 0.f;

    float compute(float sampleRate, float wowAmount, float flutterAmount) {
        // === WOW (slow LFO) ===
        if (--wowTimer <= 0) {
            wowFreqTarget = (2.f * random::uniform() - 1.f) * 0.03f;  // ±0.03 Hz
            wowAmpTarget = 1.f + 0.1f * (2.f * random::uniform() - 1.f); // ±10%
            wowTimer = static_cast<int>(0.1f * sampleRate);
        }

        wowFreqMod += 0.001f * (wowFreqTarget - wowFreqMod);
        float wowSpeed = 0.35f + wowFreqMod;
        wowPhase += wowSpeed / sampleRate;
        if (wowPhase > 1.f) wowPhase -= 1.f;

        wowAmp += 0.001f * (wowAmpTarget - wowAmp);
        float wowLFO = wowAmp * std::sin(2.f * M_PI * wowPhase);
        wowLFOFiltered += 0.01f * (wowLFO - wowLFOFiltered);

        // === FLUTTER (faster, expressive LFO) ===
        if (--flutterTimer <= 0) {
            flutterFreqTarget = (2.f * random::uniform() - 1.f) * 0.5f;  // ±0.5 Hz
            flutterAmpTarget = 1.f + 0.2f * (2.f * random::uniform() - 1.f); // ±20%
            flutterTimer = static_cast<int>(0.02f * sampleRate);  // 50 updates/sec
        }

        flutterFreqMod += 0.02f * (flutterFreqTarget - flutterFreqMod);
        flutterAmp     += 0.02f * (flutterAmpTarget - flutterAmp);

        float flutterSpeed = 6.0f + flutterFreqMod;
        flutterPhase += flutterSpeed / sampleRate;
        if (flutterPhase > 1.f) flutterPhase -= 1.f;

        float rawFlutter = flutterAmp * std::sin(2.f * M_PI * flutterPhase);
        flutterLFO += 0.02f * (rawFlutter - flutterLFO);  // Soft smoothing

        // === Combine and clamp ===
        float mod = wowAmount * wowLFOFiltered + flutterAmount * flutterLFO;
        // Allow deeper modulation range for more audible wow and flutter
        mod = rack::math::clamp(mod, -0.30f, 0.30f);

        // Two-stage smoothing for delay modulation safety
        float smoothingFactor = rack::math::clamp(0.001f * 44100.f / sampleRate, 0.001f, 0.01f);
        smoothed += smoothingFactor * (mod - smoothed);
        smoothed2 += smoothingFactor * (smoothed - smoothed2);

        return smoothed2;
    }

    float getWowPhase() const { return wowPhase; }
    float getFlutterPhase() const { return flutterPhase; }
};


class TapeGlue {
public:
        float env = 0.f;
        float hpState = 0.f;        // sidechain high-pass state

        float process(float x, float amount, int algo) {
			// High-pass sidechain so bass passes more freely
			float hp = x - hpState;
			hpState += 0.01f * hp;

			float rect = std::fabs(hp);
			env += 0.01f * (rect - env);
			// raise compression threshold so small signals stay uncompressed
			float compEnv = std::max(0.f, env - 0.4f);

			float gain = 1.f;
			switch (algo) {
					default:
					case 0:
							// experimental soft knee
							gain = 1.f / (1.f + amount * compEnv * compEnv);
							break;
					case 1:
							// dynamic ratio based on level
							gain = 1.f / (1.f + amount * compEnv * (1.f + 0.5f * compEnv));
							break;
					case 2:
							// subtle SSL-style bus compression with tape glue
							gain = 1.f / (1.f + 0.5f * amount * compEnv);
							break;
			}

			float compressed = x * gain;
			// Mild saturation for extra warmth when glue is engaged
			float warmed = std::tanh(compressed * (1.f + 0.5f * amount));
			return 0.6f * warmed + 0.4f * compressed;
        }
};


struct Tape : Module {
        enum ParamId {
                INPUT_PARAM,
                DRIVE_PARAM,
                TONE_PARAM,
                LEVEL_PARAM,
                BIAS_PARAM,
                WOW_PARAM,
                FLUTTER_PARAM,
                HISS_PARAM,
                NOISE_PARAM,
                SWEETSPOT_PARAM,
                TRANSFORM_PARAM,
                PARAMS_LEN
        };
       enum InputId {
               LEFT_INPUT,
               RIGHT_INPUT,
               INPUTS_LEN
       };
        enum OutputId {
                LEFT_OUTPUT,
                RIGHT_OUTPUT,
                OUTPUTS_LEN
        };
	enum LightId {
		LIGHTS_LEN
	};

       static const int OS_FACTOR = 2;

       struct ChannelState {
               float biasState = 0.f;
               float toneState = 0.f;
               float deEmphasisState = 0.f;
               float lowpassState = 0.f;
               float brightnessState = 0.f;
               float prevSaturated = 0.f;

               float hissHPState = 0.f;
               float hissBPState = 0.f;

               float tapeNoiseHP = 0.f;
               float tapeNoiseBP = 0.f;

               TapeAging aging;
               TapeDelayBuffer delay;
               TapeGlue glue;

               float hissLP = 0.f;
               float staticLP = 0.f;

               float mixHPState = 0.f;

               Biquad eqLow;
               Biquad eqHigh;
               bool eqInit = false;

               float modSmoothed1 = 0.f;
               float modSmoothed2 = 0.f;

               dsp::Upsampler<OS_FACTOR, 8> driveUpsampler;
               dsp::Decimator<OS_FACTOR, 8> driveDecimator;
               P42Circuit transformer;
       };

       ChannelState channels[2];

       WowFlutterModulator wowFlutter;

       int tapeMode = 0; // 0: I, 1: II, 2: IV
       int tapeStyle = 2; // 0: Vintage, 1: Classic, 2: Modern (default)
       int driveMode = 0; // 0: Single, 1: Bus, 2: Mix
       int tapeSpeed = 1; // 0: 7.5 IPS, 1: 15 IPS (default), 2: 30 IPS

       int eqCurve = 0; // 0: Bass, 1: Highs, 2: Mix

	static float saturateSingle(float x, float drive) {
		// Gentler "tape" saturation using a soft knee curve
		// Distortion engages gradually above ~3 dB
		if (drive <= 0.f)
						return x;
		float driveAdj = std::max(0.f, drive - 1.f);
		float k = driveAdj * 2.f;
		float shaped = (1.f + k) * x / (1.f + k * std::fabs(x));
		const float blend = 0.7f;  // keep some dry signal
		float saturated = blend * shaped + (1.f - blend) * x;

		const float start = 1.41254f;  // 3 dB
		const float end = 1.58489f;    // 4 dB
		float mix = rack::math::clamp((std::fabs(x) - start) / (end - start), 0.f, 1.f);
		return saturated * mix + x * (1.f - mix);
	}

	static float saturateBus(float x, float drive) {
		// Two-stage saturation for bus processing
		if (drive <= 0.f)
						return x;
		float stage1 = saturateSingle(x, drive);
		float stage2 = 0.6f * std::tanh(stage1 * drive * 0.8f) + 0.4f * stage1;
		return 0.5f * stage2 + 0.5f * x;
	}

        static float saturateMix(float x, float drive, float& hpState, float sampleRate) {
                // Multi-stage saturation tailored for full mixes.
                // Apply a gentle high-pass to preserve low frequencies and
                // let the higher frequencies drive the nonlinearity harder.
		if (drive <= 0.f)
						return x;

		// Simple first order high-pass filter around 120 Hz
		float hpAlpha = std::exp(-2.f * M_PI * 120.f / sampleRate);
		hpState = hpAlpha * hpState + (1.f - hpAlpha) * x;
		float low  = hpState;
		float high = x - low;

		float stage1 = std::tanh(high * drive * 0.8f);
		float stage2 = std::tanh(stage1 * drive * 0.6f);
		float stage3 = saturateSingle(stage2, drive * 0.6f);

		// Emphasise the processed highs a bit and mix back with lows
		float highSat = 0.4f * stage3 + 0.6f * high;
                return low + 1.05f * highSat;
        }

       float processChannel(ChannelState& st, float in, const ProcessArgs& args) {
               float inputGain = params[INPUT_PARAM].getValue();
               float drive = params[DRIVE_PARAM].getValue();

               float userBias = params[BIAS_PARAM].getValue();
               float biasAmount = modeBias[tapeMode] * userBias;

               float biasMod = 0.9f + 0.1f * std::sin(2.f * M_PI * wowFlutter.getFlutterPhase() * 2.0f);
               float biasFiltered = st.biasState + 0.2f * (in - st.biasState);
               st.biasState = biasFiltered;
               float preFiltered = in + biasFiltered * biasAmount * biasMod;

               float driven = preFiltered * inputGain;

               // Drive knob controls additional tape saturation
               float driveScaled = drive * modeDrive[tapeMode];
               float satDrive = (driveScaled <= 0.f) ? 1.f : driveScaled;

               float upBuf[OS_FACTOR];
               float satBuf[OS_FACTOR];
               st.driveUpsampler.process(driven, upBuf);
               for (int i = 0; i < OS_FACTOR; i++) {
                       switch (driveMode) {
                               default:
                               case 0:
                                       satBuf[i] = saturateSingle(upBuf[i], satDrive);
                                       break;
                               case 1:
                                       satBuf[i] = saturateBus(upBuf[i], satDrive);
                                       break;
                               case 2:
                                       satBuf[i] = saturateMix(upBuf[i], satDrive, st.mixHPState, args.sampleRate * OS_FACTOR);
                                       break;
                       }
               }

               float saturated = st.driveDecimator.process(satBuf);

               float warmTail = 0.02f * st.prevSaturated;
               st.prevSaturated = saturated;
               float saturatedWithTail = saturated + warmTail;

               // Glue compression responds to the input level
               float glueAmount = std::max(0.f, inputGain - 1.f) * modeGlue[tapeMode];
               float glued = st.glue.process(saturatedWithTail, glueAmount, driveMode);

               float wowAmount = params[WOW_PARAM].getValue() * modeWF[tapeMode];
               float flutterAmount = params[FLUTTER_PARAM].getValue() * modeWF[tapeMode];
               float rawMod = wowFlutter.compute(args.sampleRate, wowAmount, flutterAmount);

               st.modSmoothed1 += 0.001f * (rawMod - st.modSmoothed1);
               st.modSmoothed2 += 0.001f * (st.modSmoothed1 - st.modSmoothed2);

               float modDepth = 0.02f * speedModScale[tapeSpeed];
               float delaySamples = st.modSmoothed2 * modDepth * args.sampleRate;

               float delayed = st.delay.readModulated(glued, delaySamples, 0, args.sampleRate);

               float tone = params[TONE_PARAM].getValue() * modeTone[tapeMode];
               tone = clamp(tone, 0.f, 1.f);
               float cutoff = 200.f + 20000.f * tone;
               cutoff *= speedCutoffScale[tapeSpeed];
               float alpha = std::exp(-2.f * M_PI * cutoff / args.sampleRate);
               alpha = clamp(alpha, 0.0001f, 0.9999f);
               st.toneState = alpha * st.toneState + (1.f - alpha) * delayed;

               float deEmphasized = st.toneState * st.aging.eqDrift + 0.04f * (st.deEmphasisState - st.toneState);
               st.deEmphasisState = st.toneState;

               float bumpSensitivity = 0.4f * modeBump[tapeMode];
               float bumpThreshold = 1.2f;

               float bumpIntensity = std::max(0.f, (inputGain * drive - bumpThreshold) * bumpSensitivity);
               st.lowpassState += 0.05f * (deEmphasized - st.lowpassState);

               float highpassEstimate = deEmphasized - st.lowpassState;
               if (std::abs(highpassEstimate) < 0.1f)
                       bumpIntensity *= 0.5f;

               bumpIntensity = rack::math::clamp(bumpIntensity, 0.f, 1.f);

               float lowBump = deEmphasized + (st.lowpassState - deEmphasized) * bumpIntensity;

               float bassRestore = lowBump + 0.1f * (lowBump - st.lowpassState);

               float toneTrim = 1.0f - 0.02f * std::tanh(driven * 0.3f);
               float signal = bassRestore * toneTrim;

               float highComponent = signal - st.brightnessState;
               float biasTilt = userBias - 1.f;
               float highBoost = fmaxf(biasTilt, 0.f) * 3.6f;
               float lowBoost  = fmaxf(-biasTilt, 0.f) * 1.6f;

               float finalBrightness = signal + (0.1f + highBoost) * highComponent;
               finalBrightness += lowBoost * (st.lowpassState - signal);
               st.brightnessState = signal;

               if (!st.eqInit) {
                       st.eqLow.reset();
                       st.eqHigh.reset();
                       st.eqInit = true;
               }
               float sweetDrive = params[SWEETSPOT_PARAM].getValue();
               float lowGain = eqCurves[eqCurve].lowGainDb * sweetDrive;
               float highGain = eqCurves[eqCurve].highGainDb * sweetDrive;
               st.eqLow.setLowShelf(args.sampleRate, eqCurves[eqCurve].lowFreq, lowGain);
               st.eqHigh.setHighShelf(args.sampleRate, eqCurves[eqCurve].highFreq, highGain);
               float eqProcessed = st.eqHigh.process(st.eqLow.process(finalBrightness));

               float hissAmount = params[HISS_PARAM].getValue();
               float white = 2.f * random::uniform() - 1.f;

               float hp = white - st.hissHPState;
               st.hissHPState = white;

               float bp = hp - st.hissBPState * 0.9f;
               st.hissBPState = bp;

               float hissShaped = bp * 1.5f;
               float hissScale = modeHiss[tapeMode];
               float hissSignal = hissShaped * hissAmount * hissScale * 2.f * styleNoiseScale[tapeStyle] * speedNoiseScale[tapeSpeed];

               st.hissLP += 0.05f * (hissSignal - st.hissLP);
               hissSignal = st.hissLP;
               if (std::abs(eqProcessed) < 0.01f)
                       hissSignal *= 0.25f;

               float hissToneTrim = 1.0f - 0.6f * (1.0f - tone);
               hissSignal *= hissToneTrim;

               float noiseAmount = params[NOISE_PARAM].getValue();
               float tapeWhite = 2.f * random::uniform() - 1.f;
               float noiseHP = tapeWhite - st.tapeNoiseHP;
               st.tapeNoiseHP = tapeWhite;
               float noiseBP = noiseHP - st.tapeNoiseBP * 0.85f;
               st.tapeNoiseBP = noiseBP;

               float wowNoiseMod = 1.f + 0.05f * std::sin(2.f * M_PI * wowFlutter.getWowPhase() * 1.5f);
               float tapeStatic = noiseBP * 0.8f * wowNoiseMod * noiseAmount * modeStatic[tapeMode] * 2.f * styleNoiseScale[tapeStyle] * speedNoiseScale[tapeSpeed];

               st.staticLP += 0.03f * (tapeStatic - st.staticLP);
               tapeStatic = st.staticLP;
               tapeStatic *= 0.9f;

               float level = params[LEVEL_PARAM].getValue();
               st.aging.storePrint(glued);
               float printEcho = st.aging.getPrintEcho();
               float xformDrive = params[TRANSFORM_PARAM].getValue();
               float transformed = st.transformer.process(eqProcessed, 1.f + xformDrive, args.sampleRate);
               return transformed * level + hissSignal + tapeStatic + printEcho;
       }
	

	Tape() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		// === CORE GAIN STAGING ===
		configParam(INPUT_PARAM, 0.0f, 3.0f, 1.0f, "Input Level");	// 0 = mute, 1 = nominal, 3 = slam
		configParam(DRIVE_PARAM, 0.0f, 4.0f, 1.3f, "Tape Saturation Drive");	// 1.0–2.5 is typical tape
		configParam(LEVEL_PARAM, 0.0f, 2.5f, 1.2f, "Output Level");	// recovery/headroom
		configParam(BIAS_PARAM, 0.5f, 2.5f, 1.9f, "Bias Calibrate");

		// === TONE SHAPING ===
		configParam(TONE_PARAM, 0.0f, 1.0f, 0.85f, "Tone");                  // 0 = dull (Type I), 1 = open (metal)

		// === MODULATION ===
		configParam(WOW_PARAM, 0.0f, 1.5f, 0.01f, "Wow");                    // 0.1–0.3 is typical vintage
		configParam(FLUTTER_PARAM, 0.0f, 2.5f, 0.004f, "Flutter");          // 0.002–0.008 is realistic

		// === NOISES ===
		configParam(HISS_PARAM, 0.0f, 6.0f, 0.12f, "Hiss Amount");           // extended range
		configParam(NOISE_PARAM, 0.0f, 6.0f, 0.2f, "Tape Static");           // extended range
		configParam(SWEETSPOT_PARAM, -1.f, 1.f, 0.3f, "Sweetspot Drive");
		configParam(TRANSFORM_PARAM, 0.f, 3.f, 0.f, "Transformer Drive");
        }

        void process(const ProcessArgs& args) override {
                constexpr float VOLT_SCALE = 0.2f;

                float inL = inputs[LEFT_INPUT].getVoltage() * VOLT_SCALE;
                float inR = inputs[RIGHT_INPUT].isConnected() ? inputs[RIGHT_INPUT].getVoltage() * VOLT_SCALE : inL;

                float left = processChannel(channels[0], inL, args);
                float right = processChannel(channels[1], inR, args);

                bool rightConn = outputs[RIGHT_OUTPUT].isConnected();
                if (!rightConn) {
                        left = 0.5f * (left + right);
                }

                outputs[LEFT_OUTPUT].setVoltage(left / VOLT_SCALE);
                outputs[RIGHT_OUTPUT].setVoltage(right / VOLT_SCALE);
        }

        json_t* dataToJson() override {
                json_t* root = json_object();
                json_object_set_new(root, "tapeMode", json_integer(tapeMode));
                json_object_set_new(root, "tapeStyle", json_integer(tapeStyle));
                json_object_set_new(root, "driveMode", json_integer(driveMode));
                json_object_set_new(root, "tapeSpeed", json_integer(tapeSpeed));
                json_object_set_new(root, "eqCurve", json_integer(eqCurve));

                return root;
        }

	void dataFromJson(json_t* root) override {
		// Restore shiftReg.bits
		json_t* modeJ = json_object_get(root, "tapeMode");
		if (modeJ) {
				tapeMode = json_integer_value(modeJ);
		}
                json_t* styleJ = json_object_get(root, "tapeStyle");
                if (styleJ) {
                                tapeStyle = json_integer_value(styleJ);
                }
                json_t* driveJ = json_object_get(root, "driveMode");
                if (driveJ) {
                                driveMode = json_integer_value(driveJ);
                }
                json_t* speedJ = json_object_get(root, "tapeSpeed");
                if (speedJ) {
                                tapeSpeed = json_integer_value(speedJ);
                }
                json_t* eqJ = json_object_get(root, "eqCurve");
                if (eqJ) {
                                eqCurve = json_integer_value(eqJ);
                }
        }
};


struct TapeWidget : ModuleWidget {
	TapeWidget(Tape* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/Tape.svg")));

		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ThemedScrew>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ThemedScrew>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

               addInput(createInputCentered<PJ301MPort>(mm2px(Vec(10, 20)), module, Tape::LEFT_INPUT));
               addInput(createInputCentered<PJ301MPort>(mm2px(Vec(20, 20)), module, Tape::RIGHT_INPUT));
                addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10, 100)), module, Tape::LEFT_OUTPUT));
                addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(20, 100)), module, Tape::RIGHT_OUTPUT));

		
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(50, 40)), module, Tape::INPUT_PARAM));
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10, 40)), module, Tape::DRIVE_PARAM));
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10, 60)), module, Tape::TONE_PARAM));
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10, 80)), module, Tape::LEVEL_PARAM));
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(50, 60)), module, Tape::BIAS_PARAM));
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(50, 80)), module, Tape::SWEETSPOT_PARAM));
                addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(50, 100)), module, Tape::TRANSFORM_PARAM));

		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30, 40)), module, Tape::FLUTTER_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30, 60)), module, Tape::WOW_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30, 80)), module, Tape::HISS_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(30, 100)), module, Tape::NOISE_PARAM));
	}

        void appendContextMenu(Menu* menu) override {
                Tape* module = getModule<Tape>();
                menu->addChild(createIndexPtrSubmenuItem("Tape Mode",
                        {"I", "II", "IV"},
                        &module->tapeMode
                ));
                menu->addChild(createIndexPtrSubmenuItem("Tape Style",
                        {"Vintage", "Classic", "Modern", "Soulless"},
                        &module->tapeStyle
                ));
                menu->addChild(createIndexPtrSubmenuItem("Drive and Glue Mode",
                        {"Single", "Bus", "Mix"},
                        &module->driveMode
                ));
                menu->addChild(createIndexPtrSubmenuItem("Tape Speed",
                        {"7.5 IPS", "15 IPS", "30 IPS"},
                        &module->tapeSpeed
                ));
                menu->addChild(createIndexPtrSubmenuItem("EQ Curve",
                        {"Bass", "Highs", "Mix"},
                        &module->eqCurve
                ));
        }
};


Model* modelTape = createModel<Tape, TapeWidget>("Tape");