#include "plugin.hpp"
#include <cmath>
#include <algorithm>

using namespace rack;

namespace {

// High-quality biquad filter with proper coefficient calculation
struct Biquad {
	float b0 = 1.f, b1 = 0.f, b2 = 0.f;
	float a1 = 0.f, a2 = 0.f;
	float z1 = 0.f, z2 = 0.f;

	void reset() {
		z1 = z2 = 0.f;
	}

	float process(float in) {
		float out = b0 * in + z1;
		z1 = b1 * in - a1 * out + z2;
		z2 = b2 * in - a2 * out;
		return out;
	}

	void setLowpass(float sampleRate, float freq) {
		freq = clamp(freq, 20.f, sampleRate * 0.49f);
		float w0 = 2.f * M_PI * freq / sampleRate;
		float cosw0 = std::cos(w0);
		float sinw0 = std::sin(w0);
		float alpha = sinw0 * M_SQRT1_2; // Q = 0.707 (Butterworth)

		float a0 = 1.f + alpha;
		b0 = (1.f - cosw0) * 0.5f / a0;
		b1 = (1.f - cosw0) / a0;
		b2 = b0;
		a1 = -2.f * cosw0 / a0;
		a2 = (1.f - alpha) / a0;
	}

	void setHighpass(float sampleRate, float freq) {
		freq = clamp(freq, 20.f, sampleRate * 0.49f);
		float w0 = 2.f * M_PI * freq / sampleRate;
		float cosw0 = std::cos(w0);
		float sinw0 = std::sin(w0);
		float alpha = sinw0 * M_SQRT1_2;

		float a0 = 1.f + alpha;
		b0 = (1.f + cosw0) * 0.5f / a0;
		b1 = -(1.f + cosw0) / a0;
		b2 = b0;
		a1 = -2.f * cosw0 / a0;
		a2 = (1.f - alpha) / a0;
	}
};

// Linkwitz-Riley 4th order crossover (phase-coherent)
struct LR4Crossover {
	Biquad lpA, lpB, hpA, hpB;

	void setCutoff(float sampleRate, float freq) {
		lpA.setLowpass(sampleRate, freq);
		lpB.setLowpass(sampleRate, freq);
		hpA.setHighpass(sampleRate, freq);
		hpB.setHighpass(sampleRate, freq);
	}

	float low(float in) {
		return lpB.process(lpA.process(in));
	}

	float high(float in) {
		return hpB.process(hpA.process(in));
	}

	void reset() {
		lpA.reset(); lpB.reset();
		hpA.reset(); hpB.reset();
	}
};

// RMS-style envelope follower for smooth, musical compression
struct EnvelopeFollower {
	float env = 0.f;

	float process(float input, float attack, float release) {
		// Use absolute value for peak detection
		float rectified = std::abs(input);

		// Attack/release with proper coefficient
		float coeff = (rectified > env) ? attack : release;
		env += (rectified - env) * coeff;

		return env;
	}

	void reset() {
		env = 0.f;
	}
};

// High-quality OTT-style dual compressor
struct DualCompressor {
	EnvelopeFollower envFollower;
	float gainSmooth = 1.f;

	// OTT-style compression with separate upward/downward
	float process(float input, float upAmount, float downAmount, float attack, float release) {
		// Get envelope (linear, normalized to ~1.0 for typical signal)
		float env = envFollower.process(input / 5.f, attack, release); // Normalize typical 5V audio

		// Convert to dB for compression calculation
		float envDb = (env > 1e-6f) ? 20.f * std::log10f(env) : -120.f;

		// OTT uses 0dB threshold for the envelope
		const float threshold = 0.f;
		float gainDb = 0.f;

		// UPWARD compression: Bring up quiet parts (below threshold)
		if (upAmount > 0.f) {
			float below = std::max(0.f, threshold - envDb);
			// At full upAmount, use ratio of 100:1 (very aggressive)
			float ratio = 1.f + upAmount * 99.f;
			gainDb += below * (1.f - 1.f / ratio);
		}

		// DOWNWARD compression: Reduce loud parts (above threshold)
		if (downAmount > 0.f) {
			float above = std::max(0.f, envDb - threshold);
			// At full downAmount, use ratio of 100:1 (very aggressive)
			float ratio = 1.f + downAmount * 99.f;
			gainDb -= above * (1.f - 1.f / ratio);
		}

		// Convert to linear gain
		float targetGain = std::pow(10.f, gainDb / 20.f);

		// Very fast smoothing for OTT character (essentially no smoothing)
		float smoothCoeff = 0.5f; // Instant response
		gainSmooth += (targetGain - gainSmooth) * smoothCoeff;

		return gainSmooth;
	}

	void reset() {
		envFollower.reset();
		gainSmooth = 1.f;
	}
};

} // namespace

struct SabnockOTT : Module {
	enum ParamId {
		PARAM_DEPTH,      // Master compression amount
		PARAM_TIME,       // Master attack/release time
		PARAM_INPUT,      // Input gain
		PARAM_OUTPUT,     // Output gain
		PARAM_LOW_UP,     // Low band upward compression
		PARAM_LOW_DOWN,   // Low band downward compression
		PARAM_MID_UP,     // Mid band upward compression
		PARAM_MID_DOWN,   // Mid band downward compression
		PARAM_HIGH_UP,    // High band upward compression
		PARAM_HIGH_DOWN,  // High band downward compression
		NUM_PARAMS
	};
	enum InputId {
		INPUT_L,
		INPUT_R,
		NUM_INPUTS
	};
	enum OutputId {
		OUTPUT_L,
		OUTPUT_R,
		NUM_OUTPUTS
	};
	enum LightId {
		NUM_LIGHTS
	};

	// DSP components per channel
	struct Channel {
		LR4Crossover xover1; // Split low from mid+high
		LR4Crossover xover2; // Split mid from high
		DualCompressor compLow;
		DualCompressor compMid;
		DualCompressor compHigh;

		void reset() {
			xover1.reset();
			xover2.reset();
			compLow.reset();
			compMid.reset();
			compHigh.reset();
		}
	};

	Channel channels[2];
	float crossover1Freq = 250.f;  // Low/Mid split
	float crossover2Freq = 2500.f; // Mid/High split

	SabnockOTT() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

		// Classic OTT controls
		configParam(PARAM_DEPTH, 0.f, 1.f, 0.5f, "Depth", "%", 0.f, 100.f);
		configParam(PARAM_TIME, 0.f, 1.f, 0.5f, "Time", "%", 0.f, 100.f);
		configParam(PARAM_INPUT, -12.f, 12.f, 0.f, "Input Gain", " dB");
		configParam(PARAM_OUTPUT, -12.f, 12.f, 0.f, "Output Gain", " dB");

		// Per-band controls
		configParam(PARAM_LOW_UP, 0.f, 1.f, 0.5f, "Low Up", "%", 0.f, 100.f);
		configParam(PARAM_LOW_DOWN, 0.f, 1.f, 0.5f, "Low Down", "%", 0.f, 100.f);
		configParam(PARAM_MID_UP, 0.f, 1.f, 0.5f, "Mid Up", "%", 0.f, 100.f);
		configParam(PARAM_MID_DOWN, 0.f, 1.f, 0.5f, "Mid Down", "%", 0.f, 100.f);
		configParam(PARAM_HIGH_UP, 0.f, 1.f, 0.5f, "High Up", "%", 0.f, 100.f);
		configParam(PARAM_HIGH_DOWN, 0.f, 1.f, 0.5f, "High Down", "%", 0.f, 100.f);

		configInput(INPUT_L, "Left");
		configInput(INPUT_R, "Right");
		configOutput(OUTPUT_L, "Left");
		configOutput(OUTPUT_R, "Right");
	}

	void onReset() override {
		for (int c = 0; c < 2; c++) {
			channels[c].reset();
		}
	}

	void onSampleRateChange() override {
		float sr = APP->engine->getSampleRate();
		for (int c = 0; c < 2; c++) {
			channels[c].xover1.setCutoff(sr, crossover1Freq);
			channels[c].xover2.setCutoff(sr, crossover2Freq);
		}
	}

	void process(const ProcessArgs& args) override {
		float sampleRate = args.sampleRate;

		// Get parameters
		float depth = params[PARAM_DEPTH].getValue();
		float timeParam = params[PARAM_TIME].getValue();
		float inputGain = std::pow(10.f, params[PARAM_INPUT].getValue() / 20.f);
		float outputGain = std::pow(10.f, params[PARAM_OUTPUT].getValue() / 20.f);

		// Calculate time constants (OTT-style: fast attack, slower release)
		// Attack: 0.1ms to 10ms (very fast by default)
		float attackMs = 0.1f + timeParam * timeParam * 9.9f;
		// Release: 10ms to 1000ms (much slower)
		float releaseMs = 10.f + timeParam * timeParam * 990.f;

		// Convert to coefficient (exponential decay)
		float attack = 1.f - std::exp(-1000.f / (attackMs * sampleRate));
		float release = 1.f - std::exp(-1000.f / (releaseMs * sampleRate));

		// Per-band amounts (scaled by master depth)
		float lowUp = params[PARAM_LOW_UP].getValue() * depth;
		float lowDown = params[PARAM_LOW_DOWN].getValue() * depth;
		float midUp = params[PARAM_MID_UP].getValue() * depth;
		float midDown = params[PARAM_MID_DOWN].getValue() * depth;
		float highUp = params[PARAM_HIGH_UP].getValue() * depth;
		float highDown = params[PARAM_HIGH_DOWN].getValue() * depth;

		// Update crossover frequencies (fixed for now)
		if (channels[0].xover1.lpA.b0 == 0.f) {
			onSampleRateChange();
		}

		// Process each channel
		for (int c = 0; c < 2; c++) {
			int inputId = (c == 0) ? INPUT_L : INPUT_R;
			int outputId = (c == 0) ? OUTPUT_L : OUTPUT_R;

			// Get input (with mono normalization)
			float in = 0.f;
			if (inputs[inputId].isConnected()) {
				in = inputs[inputId].getVoltage() * inputGain;
			} else if (c == 1 && inputs[INPUT_L].isConnected()) {
				in = inputs[INPUT_L].getVoltage() * inputGain;
			}

			// Split into 3 bands using cascaded crossovers
			float lowMidHigh = in;
			float low = channels[c].xover1.low(lowMidHigh);

			float midHigh = channels[c].xover1.high(in);
			float mid = channels[c].xover2.low(midHigh);
			float high = channels[c].xover2.high(midHigh);

			// Compress each band
			float gainLow = channels[c].compLow.process(low, lowUp, lowDown, attack, release);
			float gainMid = channels[c].compMid.process(mid, midUp, midDown, attack, release);
			float gainHigh = channels[c].compHigh.process(high, highUp, highDown, attack, release);

			// Apply gains
			low *= gainLow;
			mid *= gainMid;
			high *= gainHigh;

			// Sum bands and apply output gain
			float out = (low + mid + high) * outputGain;

			outputs[outputId].setVoltage(out);
		}
	}
};

struct SabnockOTTWidget : ModuleWidget {
	SabnockOTTWidget(SabnockOTT* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/SabnockOTT.svg")));

		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		// Classic OTT: 4 big knobs at top
		addParam(createParamCentered<RoundHugeBlackKnob>(mm2px(Vec(20.f, 25.f)), module, SabnockOTT::PARAM_DEPTH));
		addParam(createParamCentered<RoundHugeBlackKnob>(mm2px(Vec(50.f, 25.f)), module, SabnockOTT::PARAM_TIME));
		addParam(createParamCentered<RoundHugeBlackKnob>(mm2px(Vec(20.f, 50.f)), module, SabnockOTT::PARAM_INPUT));
		addParam(createParamCentered<RoundHugeBlackKnob>(mm2px(Vec(50.f, 50.f)), module, SabnockOTT::PARAM_OUTPUT));

		// Band controls: UP and DOWN for each band
		float bandY = 75.f;
		float bandSpacing = 16.f;

		// LOW BAND
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(20.f, bandY)), module, SabnockOTT::PARAM_LOW_UP));
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(50.f, bandY)), module, SabnockOTT::PARAM_LOW_DOWN));

		// MID BAND
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(20.f, bandY + bandSpacing)), module, SabnockOTT::PARAM_MID_UP));
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(50.f, bandY + bandSpacing)), module, SabnockOTT::PARAM_MID_DOWN));

		// HIGH BAND
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(20.f, bandY + 2 * bandSpacing)), module, SabnockOTT::PARAM_HIGH_UP));
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(50.f, bandY + 2 * bandSpacing)), module, SabnockOTT::PARAM_HIGH_DOWN));

		// I/O at bottom
		float ioY = 117.f;
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(15.f, ioY)), module, SabnockOTT::INPUT_L));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(25.f, ioY)), module, SabnockOTT::INPUT_R));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(45.f, ioY)), module, SabnockOTT::OUTPUT_L));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(55.f, ioY)), module, SabnockOTT::OUTPUT_R));
	}
};

Model* modelSabnockOTT = createModel<SabnockOTT, SabnockOTTWidget>("SabnockOTT");
