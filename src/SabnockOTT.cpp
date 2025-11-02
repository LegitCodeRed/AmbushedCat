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

		// Classic OTT controls
		configParam(PARAM_DEPTH, 0.f, 1.f, 1.0f, "Depth", "%", 0.f, 100.f);
		configParam(PARAM_TIME, 0.f, 1.f, 0.5f, "Time", "%", 0.f, 100.f);
		configParam(PARAM_INPUT, -60.f, 30.f, 0.f, "Input Gain", " dB");
		configParam(PARAM_OUTPUT, -60.f, 30.f, 0.f, "Output Gain", " dB");

		// Per-band controls
		configParam(PARAM_LOW_UP, 0.f, 2.f, 1.0f, "Low Up");
		configParam(PARAM_LOW_DOWN, 0.f, 2.f, 1.0f, "Low Down");
		configParam(PARAM_MID_UP, 0.f, 2.f, 1.0f, "Mid Up");
		configParam(PARAM_MID_DOWN, 0.f, 2.f, 1.0f, "Mid Down");
		configParam(PARAM_HIGH_UP, 0.f, 2.f, 1.0f, "High Up");
		configParam(PARAM_HIGH_DOWN, 0.f, 2.f, 1.0f, "High Down");

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
		float depth = params[PARAM_DEPTH].getValue();
		float timeParam = params[PARAM_TIME].getValue();
		in_gain = params[PARAM_INPUT].getValue();
		out_gain = params[PARAM_OUTPUT].getValue();

		float upward = params[PARAM_LOW_UP].getValue();
		float downward = params[PARAM_LOW_DOWN].getValue();

		// Calculate attack/release times
		float att_time = timeParam;
		float rel_time = timeParam;

		// Default thresholds (from original vitOTT)
		float ll_thres = -35.0f;
		float lu_thres = -28.0f;
		float bl_thres = -36.0f;
		float bu_thres = -25.0f;
		float hl_thres = -35.0f;
		float hu_thres = -30.0f;

		// Default ratios
		float ll_ratio = 0.8f;
		float lu_ratio = 0.9f;
		float bl_ratio = 0.8f;
		float bu_ratio = 0.857f;
		float hl_ratio = 0.8f;
		float hu_ratio = 1.0f;

		// Gains from vitOTT defaults
		float lgain = 16.3f;
		float mgain = 11.7f;
		float hgain = 16.3f;

		// Get per-band controls
		float lowUp = params[PARAM_LOW_UP].getValue();
		float lowDown = params[PARAM_LOW_DOWN].getValue();
		float midUp = params[PARAM_MID_UP].getValue();
		float midDown = params[PARAM_MID_DOWN].getValue();
		float highUp = params[PARAM_HIGH_UP].getValue();
		float highDown = params[PARAM_HIGH_DOWN].getValue();

		// Set thresholds
		vals[vital::MultibandCompressor::kLowLowerThreshold - 1]->set(ll_thres);
		vals[vital::MultibandCompressor::kLowUpperThreshold - 1]->set(lu_thres);
		vals[vital::MultibandCompressor::kBandLowerThreshold - 1]->set(bl_thres);
		vals[vital::MultibandCompressor::kBandUpperThreshold - 1]->set(bu_thres);
		vals[vital::MultibandCompressor::kHighLowerThreshold - 1]->set(hl_thres);
		vals[vital::MultibandCompressor::kHighUpperThreshold - 1]->set(hu_thres);

		// Set ratios (scaled by depth and per-band controls)
		vals[vital::MultibandCompressor::kLowLowerRatio - 1]->set(ll_ratio * depth * lowUp);
		vals[vital::MultibandCompressor::kLowUpperRatio - 1]->set(lu_ratio * depth * lowDown);
		vals[vital::MultibandCompressor::kBandLowerRatio - 1]->set(bl_ratio * depth * midUp);
		vals[vital::MultibandCompressor::kBandUpperRatio - 1]->set(bu_ratio * depth * midDown);
		vals[vital::MultibandCompressor::kHighLowerRatio - 1]->set(hl_ratio * depth * highUp);
		vals[vital::MultibandCompressor::kHighUpperRatio - 1]->set(hu_ratio * depth * highDown);

		// Set attack/release
		vals[vital::MultibandCompressor::kAttack - 1]->set(att_time);
		vals[vital::MultibandCompressor::kRelease - 1]->set(rel_time);

		// Set output gains
		vals[vital::MultibandCompressor::kLowOutputGain - 1]->set(lgain);
		vals[vital::MultibandCompressor::kBandOutputGain - 1]->set(mgain);
		vals[vital::MultibandCompressor::kHighOutputGain - 1]->set(hgain);

		// Set crossover frequencies
		vals[vital::MultibandCompressor::kLMFrequency - 1]->set(crossover1Freq);
		vals[vital::MultibandCompressor::kMHFrequency - 1]->set(crossover2Freq);

		// Set mix to 100%
		vals[vital::MultibandCompressor::kMix - 1]->set(1.0f);
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
