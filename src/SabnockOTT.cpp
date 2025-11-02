#include "plugin.hpp"
#include "vital_dsp/compressor.h"
#include "vital_dsp/framework/value.h"
#include "vital_dsp/utilities/smooth_value.h"
#include <cmath>
#include <algorithm>
#include <array>

using namespace rack;

struct SabnockOTT : Module {
	enum ParamId {
		PARAM_ATTACK,          // Attack time
		PARAM_RELEASE,         // Release time
		PARAM_MIX,             // Dry/Wet mix (was depth)
		PARAM_LOW_GAIN,        // Low band output gain
		PARAM_MID_GAIN,        // Mid band output gain
		PARAM_HIGH_GAIN,       // High band output gain
		PARAM_INPUT,           // Input gain
		PARAM_OUTPUT,          // Output gain
		// Low band controls
		PARAM_LOW_UP_THRESH,   // Low band upward threshold
		PARAM_LOW_UP_RATIO,    // Low band upward ratio
		PARAM_LOW_DOWN_THRESH, // Low band downward threshold
		PARAM_LOW_DOWN_RATIO,  // Low band downward ratio
		// Mid band controls
		PARAM_MID_UP_THRESH,   // Mid band upward threshold
		PARAM_MID_UP_RATIO,    // Mid band upward ratio
		PARAM_MID_DOWN_THRESH, // Mid band downward threshold
		PARAM_MID_DOWN_RATIO,  // Mid band downward ratio
		// High band controls
		PARAM_HIGH_UP_THRESH,  // High band upward threshold
		PARAM_HIGH_UP_RATIO,   // High band upward ratio
		PARAM_HIGH_DOWN_THRESH,// High band downward threshold
		PARAM_HIGH_DOWN_RATIO, // High band downward ratio
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

	// Vital DSP components
	std::unique_ptr<vital::MultibandCompressor> compressor;
	std::unique_ptr<vital::Output> sig_in;
	std::array<vital::Value*, 21> vals = {};

	float in_gain = 1.0f;
	float out_gain = 1.0f;

	float crossover1Freq = 120.f;  // Low/Mid split
	float crossover2Freq = 2500.f; // Mid/High split

	SabnockOTT() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

		// Main controls (top row)
		configParam(PARAM_ATTACK, 0.f, 1.f, 0.5f, "Attack", "%", 0.f, 100.f);
		configParam(PARAM_RELEASE, 0.f, 1.f, 0.5f, "Release", "%", 0.f, 100.f);
		configParam(PARAM_MIX, 0.f, 1.f, 1.0f, "Mix", "%", 0.f, 100.f);

		// Band gain controls (makeup gain per band)
		configParam(PARAM_LOW_GAIN, -30.f, 30.f, 16.3f, "Low Band Gain", " dB");
		configParam(PARAM_MID_GAIN, -30.f, 30.f, 11.7f, "Mid Band Gain", " dB");
		configParam(PARAM_HIGH_GAIN, -30.f, 30.f, 16.3f, "High Band Gain", " dB");

		// Input/Output gain
		configParam(PARAM_INPUT, -60.f, 30.f, 0.f, "Input Gain", " dB");
		configParam(PARAM_OUTPUT, -60.f, 30.f, 0.f, "Output Gain", " dB");

		// LOW BAND (Top = Downward/Upper, Bottom = Upward/Lower)
		// Top knob: Downward (compression) - Upper threshold
		configParam(PARAM_LOW_DOWN_THRESH, -79.f, -1.f, -28.5f, "Low Downward Threshold", " dB");
		configParam(PARAM_LOW_DOWN_RATIO, 0.f, 1.f, 0.9f, "Low Downward Ratio", "%", 0.f, 100.f);
		// Bottom knob: Upward (expansion) - Lower threshold
		configParam(PARAM_LOW_UP_THRESH, -79.f, -1.f, -35.0f, "Low Upward Threshold", " dB");
		configParam(PARAM_LOW_UP_RATIO, 0.f, 1.f, 0.8f, "Low Upward Ratio", "%", 0.f, 100.f);

		// MID BAND (Top = Downward/Upper, Bottom = Upward/Lower)
		// Top knob: Downward (compression) - Upper threshold
		configParam(PARAM_MID_DOWN_THRESH, -79.f, -1.f, -25.0f, "Mid Downward Threshold", " dB");
		configParam(PARAM_MID_DOWN_RATIO, 0.f, 1.f, 0.857f, "Mid Downward Ratio", "%", 0.f, 100.f);
		// Bottom knob: Upward (expansion) - Lower threshold
		configParam(PARAM_MID_UP_THRESH, -79.f, -1.f, -36.0f, "Mid Upward Threshold", " dB");
		configParam(PARAM_MID_UP_RATIO, 0.f, 1.f, 0.8f, "Mid Upward Ratio", "%", 0.f, 100.f);

		// HIGH BAND (Top = Downward/Upper, Bottom = Upward/Lower)
		// Top knob: Downward (compression) - Upper threshold
		configParam(PARAM_HIGH_DOWN_THRESH, -79.f, -1.f, -30.0f, "High Downward Threshold", " dB");
		configParam(PARAM_HIGH_DOWN_RATIO, 0.f, 1.f, 1.0f, "High Downward Ratio", "%", 0.f, 100.f);
		// Bottom knob: Upward (expansion) - Lower threshold
		configParam(PARAM_HIGH_UP_THRESH, -79.f, -1.f, -35.0f, "High Upward Threshold", " dB");
		configParam(PARAM_HIGH_UP_RATIO, 0.f, 1.f, 0.8f, "High Upward Ratio", "%", 0.f, 100.f);

		configInput(INPUT_L, "Left");
		configInput(INPUT_R, "Right");
		configOutput(OUTPUT_L, "Left");
		configOutput(OUTPUT_R, "Right");

		// Initialize vital DSP
		compressor = std::make_unique<vital::MultibandCompressor>();
		sig_in = std::make_unique<vital::Output>();
		compressor->plug(sig_in.get(), vital::MultibandCompressor::kAudio);

		initVals();
		for (int i = 0; i < vals.size(); i++) {
			compressor->plug(vals[i], i + 1);
		}
	}

	~SabnockOTT() {
		for (int i = 0; i < vals.size(); i++) {
			delete vals[i];
		}
	}

	void initVals() {
		for (int i = 0; i < vals.size(); i++) {
			vals[i] = new vital::SmoothValue(0);
		}
		vals[vital::MultibandCompressor::kEnabledBands - 1]->set(vital::MultibandCompressor::kMultiband);
		updateParams();
	}

	void updateParams() {
		// Get parameters from Rack
		float attackParam = params[PARAM_ATTACK].getValue();
		float releaseParam = params[PARAM_RELEASE].getValue();
		float mix = params[PARAM_MIX].getValue();
		in_gain = params[PARAM_INPUT].getValue();
		out_gain = params[PARAM_OUTPUT].getValue();

		// Calculate attack/release times (scale from 0-1 with exponential curve)
		// Using exponential scaling for more musical response
		float att_time = std::pow(attackParam, 2.0f);   // 0-1 range
		float rel_time = std::pow(releaseParam, 2.0f);  // 0-1 range

		// Get band gains from controls
		float lgain = params[PARAM_LOW_GAIN].getValue();
		float mgain = params[PARAM_MID_GAIN].getValue();
		float hgain = params[PARAM_HIGH_GAIN].getValue();

		// Get thresholds from controls
		// Lower threshold = upward compression (expansion) - bottom knob
		// Upper threshold = downward compression - top knob
		float ll_thres = params[PARAM_LOW_UP_THRESH].getValue();      // Lower (upward/expansion)
		float lu_thres = params[PARAM_LOW_DOWN_THRESH].getValue();    // Upper (downward/compression)
		float bl_thres = params[PARAM_MID_UP_THRESH].getValue();      // Lower (upward/expansion)
		float bu_thres = params[PARAM_MID_DOWN_THRESH].getValue();    // Upper (downward/compression)
		float hl_thres = params[PARAM_HIGH_UP_THRESH].getValue();     // Lower (upward/expansion)
		float hu_thres = params[PARAM_HIGH_DOWN_THRESH].getValue();   // Upper (downward/compression)

		// Enforce threshold constraint: upper threshold must be >= lower threshold
		// If they overlap, push the lower one down
		if (lu_thres < ll_thres) {
			ll_thres = lu_thres;
			params[PARAM_LOW_UP_THRESH].setValue(ll_thres);
		}
		if (bu_thres < bl_thres) {
			bl_thres = bu_thres;
			params[PARAM_MID_UP_THRESH].setValue(bl_thres);
		}
		if (hu_thres < hl_thres) {
			hl_thres = hu_thres;
			params[PARAM_HIGH_UP_THRESH].setValue(hl_thres);
		}

		// Get ratios directly from controls
		float ll_ratio = params[PARAM_LOW_UP_RATIO].getValue();
		float lu_ratio = params[PARAM_LOW_DOWN_RATIO].getValue();
		float bl_ratio = params[PARAM_MID_UP_RATIO].getValue();
		float bu_ratio = params[PARAM_MID_DOWN_RATIO].getValue();
		float hl_ratio = params[PARAM_HIGH_UP_RATIO].getValue();
		float hu_ratio = params[PARAM_HIGH_DOWN_RATIO].getValue();

		// Set thresholds
		vals[vital::MultibandCompressor::kLowLowerThreshold - 1]->set(ll_thres);
		vals[vital::MultibandCompressor::kLowUpperThreshold - 1]->set(lu_thres);
		vals[vital::MultibandCompressor::kBandLowerThreshold - 1]->set(bl_thres);
		vals[vital::MultibandCompressor::kBandUpperThreshold - 1]->set(bu_thres);
		vals[vital::MultibandCompressor::kHighLowerThreshold - 1]->set(hl_thres);
		vals[vital::MultibandCompressor::kHighUpperThreshold - 1]->set(hu_thres);

		// Set ratios
		// Note: Lower = Upward compression (expansion), Upper = Downward compression
		vals[vital::MultibandCompressor::kLowLowerRatio - 1]->set(ll_ratio);
		vals[vital::MultibandCompressor::kLowUpperRatio - 1]->set(lu_ratio);
		vals[vital::MultibandCompressor::kBandLowerRatio - 1]->set(bl_ratio);
		vals[vital::MultibandCompressor::kBandUpperRatio - 1]->set(bu_ratio);
		vals[vital::MultibandCompressor::kHighLowerRatio - 1]->set(hl_ratio);
		vals[vital::MultibandCompressor::kHighUpperRatio - 1]->set(hu_ratio);

		// Set attack/release
		vals[vital::MultibandCompressor::kAttack - 1]->set(att_time);
		vals[vital::MultibandCompressor::kRelease - 1]->set(rel_time);

		// Set output gains (from controls)
		vals[vital::MultibandCompressor::kLowOutputGain - 1]->set(lgain);
		vals[vital::MultibandCompressor::kBandOutputGain - 1]->set(mgain);
		vals[vital::MultibandCompressor::kHighOutputGain - 1]->set(hgain);

		// Set crossover frequencies
		vals[vital::MultibandCompressor::kLMFrequency - 1]->set(crossover1Freq);
		vals[vital::MultibandCompressor::kMHFrequency - 1]->set(crossover2Freq);

		// Set mix (dry/wet)
		vals[vital::MultibandCompressor::kMix - 1]->set(mix);
	}

	void onReset() override {
		compressor->reset(vital::constants::kFullMask);
	}

	void onSampleRateChange() override {
		float sr = APP->engine->getSampleRate();
		compressor->setSampleRate(sr);
	}

	void readAudio(vital::poly_float* comp_buf, float* leftBuf, float* rightBuf, int samples) {
		vital::mono_float* comp_output = (vital::mono_float*)comp_buf;
		float mag = vital::utils::dbToMagnitude(in_gain);

		for (int i = 0; i < samples; ++i) {
			comp_output[vital::poly_float::kSize * i + 0] = leftBuf[i] * mag;
			comp_output[vital::poly_float::kSize * i + 1] = rightBuf[i] * mag;
		}
	}

	void writeAudio(vital::poly_float* comp_buf, float* leftBuf, float* rightBuf, int samples) {
		const vital::mono_float* comp_output = (const vital::mono_float*)comp_buf;
		float mag = vital::utils::dbToMagnitude(out_gain);

		for (int i = 0; i < samples; ++i) {
			leftBuf[i] = comp_output[vital::poly_float::kSize * i + 0] * mag;
			rightBuf[i] = comp_output[vital::poly_float::kSize * i + 1] * mag;
		}
	}

	void process(const ProcessArgs& args) override {
		// Update parameters
		updateParams();

		// Get input
		float inL = inputs[INPUT_L].getVoltage();
		float inR = inputs[INPUT_R].isConnected() ? inputs[INPUT_R].getVoltage() : inL;

		// Convert from 5V to normalized float (assuming +/- 5V range)
		float leftBuf = inL / 5.0f;
		float rightBuf = inR / 5.0f;

		// Process values (smooth parameters)
		for (auto& val : vals) {
			val->process(1);
		}

		// Read audio into vital buffer
		readAudio(sig_in->buffer, &leftBuf, &rightBuf, 1);

		// Process compression
		compressor->process(1);

		// Write audio from vital buffer
		writeAudio(compressor->output(vital::MultibandCompressor::kAudioOut)->buffer,
		           &leftBuf, &rightBuf, 1);

		// Convert back to 5V range and output
		outputs[OUTPUT_L].setVoltage(leftBuf * 5.0f);
		outputs[OUTPUT_R].setVoltage(rightBuf * 5.0f);
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

		// Top row: Attack, Release, Mix (3 big knobs)
		addParam(createParamCentered<RoundHugeBlackKnob>(mm2px(Vec(15.f, 20.f)), module, SabnockOTT::PARAM_ATTACK));
		addParam(createParamCentered<RoundHugeBlackKnob>(mm2px(Vec(35.f, 20.f)), module, SabnockOTT::PARAM_RELEASE));
		addParam(createParamCentered<RoundHugeBlackKnob>(mm2px(Vec(55.f, 20.f)), module, SabnockOTT::PARAM_MIX));

		// Second row: Band Gains (Low, Mid, High - 3 medium knobs)
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(15.f, 40.f)), module, SabnockOTT::PARAM_LOW_GAIN));
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(35.f, 40.f)), module, SabnockOTT::PARAM_MID_GAIN));
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(55.f, 40.f)), module, SabnockOTT::PARAM_HIGH_GAIN));

		// Third row: Input and Output gain
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(25.f, 58.f)), module, SabnockOTT::PARAM_INPUT));
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(45.f, 58.f)), module, SabnockOTT::PARAM_OUTPUT));

		// Band compression controls: Each column has TOP and BOTTOM knobs
		// TOP knob (large) = Downward compression threshold (upper threshold)
		// Small knob below TOP = Downward compression ratio
		// BOTTOM knob (large) = Upward expansion threshold (lower threshold)
		// Small knob below BOTTOM = Upward expansion ratio

		float bandStartY = 70.f;
		float bandSpacing = 23.f;
		float leftX = 20.f;
		float rightX = 50.f;
		float knobPairOffset = 8.f;  // Distance between large knob and its small ratio knob

		// LOW BAND (Left column)
		float lowY = bandStartY;
		// Top pair: Downward (compression) - upper threshold
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(leftX, lowY)), module, SabnockOTT::PARAM_LOW_DOWN_THRESH));
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(leftX, lowY + knobPairOffset)), module, SabnockOTT::PARAM_LOW_DOWN_RATIO));
		// Bottom pair: Upward (expansion) - lower threshold
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(leftX, lowY + bandSpacing)), module, SabnockOTT::PARAM_LOW_UP_THRESH));
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(leftX, lowY + bandSpacing + knobPairOffset)), module, SabnockOTT::PARAM_LOW_UP_RATIO));

		// MID BAND (Middle column)
		float midY = bandStartY;
		// Top pair: Downward (compression) - upper threshold
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(35.f, midY)), module, SabnockOTT::PARAM_MID_DOWN_THRESH));
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(35.f, midY + knobPairOffset)), module, SabnockOTT::PARAM_MID_DOWN_RATIO));
		// Bottom pair: Upward (expansion) - lower threshold
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(35.f, midY + bandSpacing)), module, SabnockOTT::PARAM_MID_UP_THRESH));
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(35.f, midY + bandSpacing + knobPairOffset)), module, SabnockOTT::PARAM_MID_UP_RATIO));

		// HIGH BAND (Right column)
		float highY = bandStartY;
		// Top pair: Downward (compression) - upper threshold
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(rightX, highY)), module, SabnockOTT::PARAM_HIGH_DOWN_THRESH));
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(rightX, highY + knobPairOffset)), module, SabnockOTT::PARAM_HIGH_DOWN_RATIO));
		// Bottom pair: Upward (expansion) - lower threshold
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(rightX, highY + bandSpacing)), module, SabnockOTT::PARAM_HIGH_UP_THRESH));
		addParam(createParamCentered<RoundSmallBlackKnob>(mm2px(Vec(rightX, highY + bandSpacing + knobPairOffset)), module, SabnockOTT::PARAM_HIGH_UP_RATIO));

		// I/O at bottom
		float ioY = 117.f;
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(15.f, ioY)), module, SabnockOTT::INPUT_L));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(25.f, ioY)), module, SabnockOTT::INPUT_R));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(45.f, ioY)), module, SabnockOTT::OUTPUT_L));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(55.f, ioY)), module, SabnockOTT::OUTPUT_R));
	}
};

Model* modelSabnockOTT = createModel<SabnockOTT, SabnockOTTWidget>("SabnockOTT");
