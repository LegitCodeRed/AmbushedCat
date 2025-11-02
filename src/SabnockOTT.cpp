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
		PARAM_ATTACK,     // Attack time
		PARAM_RELEASE,    // Release time
		PARAM_MIX,        // Dry/Wet mix (was depth)
		PARAM_LOW_GAIN,   // Low band output gain
		PARAM_MID_GAIN,   // Mid band output gain
		PARAM_HIGH_GAIN,  // High band output gain
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

		// Per-band compression controls (0-100%, default 100%)
		configParam(PARAM_LOW_UP, 0.f, 1.f, 1.0f, "Low Upward", "%", 0.f, 100.f);
		configParam(PARAM_LOW_DOWN, 0.f, 1.f, 1.0f, "Low Downward", "%", 0.f, 100.f);
		configParam(PARAM_MID_UP, 0.f, 1.f, 1.0f, "Mid Upward", "%", 0.f, 100.f);
		configParam(PARAM_MID_DOWN, 0.f, 1.f, 1.0f, "Mid Downward", "%", 0.f, 100.f);
		configParam(PARAM_HIGH_UP, 0.f, 1.f, 1.0f, "High Upward", "%", 0.f, 100.f);
		configParam(PARAM_HIGH_DOWN, 0.f, 1.f, 1.0f, "High Downward", "%", 0.f, 100.f);

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

		// Default thresholds (from original vitOTT)
		float ll_thres = -35.0f;  // Low band lower (upward) threshold
		float lu_thres = -28.0f;  // Low band upper (downward) threshold
		float bl_thres = -36.0f;  // Mid band lower (upward) threshold
		float bu_thres = -25.0f;  // Mid band upper (downward) threshold
		float hl_thres = -35.0f;  // High band lower (upward) threshold
		float hu_thres = -30.0f;  // High band upper (downward) threshold

		// Default ratios (base compression amounts)
		float ll_ratio = 0.8f;   // Low band upward (expansion) ratio
		float lu_ratio = 0.9f;   // Low band downward (compression) ratio
		float bl_ratio = 0.8f;   // Mid band upward (expansion) ratio
		float bu_ratio = 0.857f; // Mid band downward (compression) ratio
		float hl_ratio = 0.8f;   // High band upward (expansion) ratio
		float hu_ratio = 1.0f;   // High band downward (compression) ratio

		// Get band gains from controls
		float lgain = params[PARAM_LOW_GAIN].getValue();
		float mgain = params[PARAM_MID_GAIN].getValue();
		float hgain = params[PARAM_HIGH_GAIN].getValue();

		// Get per-band compression controls (0-1 range, default 1.0 = 100%)
		float lowUp = params[PARAM_LOW_UP].getValue();      // Low band upward (expansion)
		float lowDown = params[PARAM_LOW_DOWN].getValue();  // Low band downward (compression)
		float midUp = params[PARAM_MID_UP].getValue();      // Mid band upward (expansion)
		float midDown = params[PARAM_MID_DOWN].getValue();  // Mid band downward (compression)
		float highUp = params[PARAM_HIGH_UP].getValue();    // High band upward (expansion)
		float highDown = params[PARAM_HIGH_DOWN].getValue();// High band downward (compression)

		// Set thresholds (fixed values from OTT)
		vals[vital::MultibandCompressor::kLowLowerThreshold - 1]->set(ll_thres);
		vals[vital::MultibandCompressor::kLowUpperThreshold - 1]->set(lu_thres);
		vals[vital::MultibandCompressor::kBandLowerThreshold - 1]->set(bl_thres);
		vals[vital::MultibandCompressor::kBandUpperThreshold - 1]->set(bu_thres);
		vals[vital::MultibandCompressor::kHighLowerThreshold - 1]->set(hl_thres);
		vals[vital::MultibandCompressor::kHighUpperThreshold - 1]->set(hu_thres);

		// Set ratios (scaled by per-band controls)
		// Note: Lower = Upward compression (expansion), Upper = Downward compression
		vals[vital::MultibandCompressor::kLowLowerRatio - 1]->set(ll_ratio * lowUp);
		vals[vital::MultibandCompressor::kLowUpperRatio - 1]->set(lu_ratio * lowDown);
		vals[vital::MultibandCompressor::kBandLowerRatio - 1]->set(bl_ratio * midUp);
		vals[vital::MultibandCompressor::kBandUpperRatio - 1]->set(bu_ratio * midDown);
		vals[vital::MultibandCompressor::kHighLowerRatio - 1]->set(hl_ratio * highUp);
		vals[vital::MultibandCompressor::kHighUpperRatio - 1]->set(hu_ratio * highDown);

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

		// Band compression controls: LEFT=Upward (expansion), RIGHT=Downward (compression)
		float bandY = 75.f;
		float bandSpacing = 16.f;

		// LOW BAND (left=upward expansion, right=downward compression)
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(20.f, bandY)), module, SabnockOTT::PARAM_LOW_UP));
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(50.f, bandY)), module, SabnockOTT::PARAM_LOW_DOWN));

		// MID BAND (left=upward expansion, right=downward compression)
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(20.f, bandY + bandSpacing)), module, SabnockOTT::PARAM_MID_UP));
		addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(50.f, bandY + bandSpacing)), module, SabnockOTT::PARAM_MID_DOWN));

		// HIGH BAND (left=upward expansion, right=downward compression)
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
