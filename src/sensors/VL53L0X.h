/*  EasyLocomotion
 *
 *  Copyright (C) 2018  foxis (Andrius Mikonis <andrius.mikonis@gmail.com>)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  License copied from https://github.com/pololu/vl53l0x-arduino
 * =========================
 *
 * Copyright (c) 2017 Pololu Corporation.  For more information, see
 *
 * https://www.pololu.com/
 * https://forum.pololu.com/
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * =================================================================
 *
 * Most of the functionality of this library is based on the VL53L0X
 * API provided by ST (STSW-IMG005), and some of the explanatory
 * comments are quoted or paraphrased from the API source code, API
 * user manual (UM2039), and the VL53L0X datasheet.
 *
 * The following applies to source code reproduced or derived from
 * the API:
 *
 * -----------------------------------------------------------------
 *
 * Copyright © 2016, STMicroelectronics International N.V.  All
 * rights reserved.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of STMicroelectronics nor the
 * names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior
 * written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 * NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 * IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * -----------------------------------------------------------------
 */

#if !defined(VL53L0X_H)
#define VL53L0X_H

#include <Wire.h>
#include "../TwoWireDevice.h"
#include "../math_utils.h"
#include "rangesensorbase.h"
#include "VL53L0X_tuning.h"

namespace Locomotion {

#define VL53L0X_ADDRESS 0b0101001

//
// VL53L0X is a I2C TOF laser ranging chip
// Adapted code from https://github.com/pololu/vl53l0x-arduino
//
class VL53L0X : public TwoWireDevice, public RangeSensorBase {
	// register addresses from API vl53l0x_device.h (ordered as listed there)
	 enum regAddr
	 {
		 SYSRANGE_START                              = 0x00,

		 SYSTEM_THRESH_HIGH                          = 0x0C,
		 SYSTEM_THRESH_LOW                           = 0x0E,

		 SYSTEM_SEQUENCE_CONFIG                      = 0x01,
		 SYSTEM_RANGE_CONFIG                         = 0x09,
		 SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

		 SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

		 GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

		 SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

		 RESULT_INTERRUPT_STATUS                     = 0x13,
		 RESULT_RANGE_STATUS                         = 0x14,

		 RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
		 RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
		 RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
		 RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
		 RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

		 ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

		 I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

		 MSRC_CONFIG_CONTROL                         = 0x60,

		 PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
		 PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
		 PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
		 PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

		 FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
		 FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
		 FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
		 FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

		 PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
		 PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

		 PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
		 PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
		 PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

		 SYSTEM_HISTOGRAM_BIN                        = 0x81,
		 HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
		 HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

		 FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
		 FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
		 FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
		 CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

		 MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

		 SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
		 IDENTIFICATION_MODEL_ID                     = 0xC0,
		 IDENTIFICATION_REVISION_ID                  = 0xC2,

		 OSC_CALIBRATE_VAL                           = 0xF8,

		 GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
		 GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
		 GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
		 GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
		 GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
		 GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
		 GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

		 GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
		 DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
		 DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
		 POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

		 VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

		 ALGO_PHASECAL_LIM                           = 0x30,
		 ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
	 };

	 struct SequenceStepEnables
   {
     boolean tcc, msrc, dss, pre_range, final_range;
   };

   struct SequenceStepTimeouts
   {
     uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

     uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
     uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
   };
	 uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
	 uint32_t measurement_timing_budget_us;
	 unsigned long io_timeout;

	 bool continuous_measurement;
	 bool measurement_initiated;

public:
	enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };

public:
	VL53L0X(TwoWire * wire)
		: VL53L0X(wire, VL53L0X_ADDRESS, NULL, NULL) { }
	VL53L0X(TwoWire * wire, MeasurementCallback_t callback, CallbackData_t * callback_data)
			: VL53L0X(wire, VL53L0X_ADDRESS, callback, callback_data) { }
  VL53L0X(TwoWire * wire, byte address, MeasurementCallback_t callback, CallbackData_t * callback_data)
		: TwoWireDevice(wire, address),
		  RangeSensorBase(callback, callback_data) {
			io_timeout = 0;
			measurement_initiated = false;
			continuous_measurement = false;
  }

	// interface implementation
  virtual void begin(bool init) {
		TwoWireDevice::begin(init);
		if (init)
			reset();
  }

	virtual bool reset() {
		measurement_initiated = false;
		continuous_measurement = false;
		return init(true);
  }

	void setIOTimeout(unsigned long io_timeout_us) { io_timeout = io_timeout_us; }

	virtual real_t getMaxDistance() { return 2000; }
	virtual real_t getMinDistance() { return 0; }
	virtual real_t getMinYaw() { return -.5 * 25.0 * 3.1415 / 180.0; }
	virtual real_t getMaxYaw() { return .5 * 25.0 * 3.1415 / 180.0; }
	virtual real_t getMinPitch() { return -.5 * 25.0 * 3.1415 / 180.0; }
	virtual real_t getMaxPitch() { return .5 * 25.0 * 3.1415 / 180.0; }

	virtual bool startSingleSampling() {
		return startRangeSingle();
	}

	virtual bool startContinuousSampling() {
		startContinuous();
		return true;
	}
	virtual bool stopContinuousSampling() {
		stopContinuous();
		return true;
	}

	virtual bool readMeasurement(Measurement_t * result, uint16_t max_readings, uint16_t offset = 0) {
		if (!continuous_measurement && !measurement_initiated) {
			startSingleSampling();
		}

		return readRangeMillimeters(result, offset);
	}

	// VL53L0X specific interface
	bool setSignalRateLimit(float limit_Mcps);
	float getSignalRateLimit(void);

	bool setMeasurementTimingBudget(uint32_t budget_us);
	uint32_t getMeasurementTimingBudget(void);

	bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);
	uint8_t getVcselPulsePeriod(vcselPeriodType type);

	bool calibrate();

private:
	bool init(bool io_2v8);

	void startContinuous(uint32_t period_ms = 0);
	void stopContinuous(void);

	bool readRangeMillimeters(Measurement_t * result, uint16_t offset);
	bool startRangeSingle(void);

	bool performSingleRefCalibration(uint8_t vhv_init_byte);
	bool getSpadInfo(uint8_t * count, bool * type_is_aperture);
	void getSequenceStepEnables(SequenceStepEnables * enables);
	void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);
	uint16_t decodeTimeout(uint16_t reg_val);
	uint16_t encodeTimeout(uint16_t timeout_mclks);
	uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
	uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
};

// Implementation
// =============================================================================

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)


bool VL53L0X::calibrate() {
	write8(SYSTEM_SEQUENCE_CONFIG, 0x01);
	if (!performSingleRefCalibration(0x40)) { return false; }
	write8(SYSTEM_SEQUENCE_CONFIG, 0x02);
	if (!performSingleRefCalibration(0x00)) { return false; }
	write8(SYSTEM_SEQUENCE_CONFIG, 0xE8);
	return true;
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool VL53L0X::init(bool io_2v8)
{
  // VL53L0X_DataInit() begin

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  if (io_2v8)
  {
    write8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
      read8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
  }

  // "Set I2C standard mode"
  write8(0x88, 0x00);

  write8(0x80, 0x01);
  write8(0xFF, 0x01);
  write8(0x00, 0x00);
  stop_variable = read8(0x91);
  write8(0x00, 0x01);
  write8(0xFF, 0x00);
  write8(0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  write8(MSRC_CONFIG_CONTROL, read8(MSRC_CONFIG_CONTROL) | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  setSignalRateLimit(0.25);

  write8(SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end

  // VL53L0X_StaticInit() begin

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];
  read(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  write8(0xFF, 0x01);
  write8(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  write8(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  write8(0xFF, 0x00);
  write8(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

  write(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

	write_bulk_P(VL53L0X_tuning_data, sizeof(VL53L0X_tuning_data) / sizeof(VL53L0X_tuning_data[0]));

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin

  write8(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  write8(GPIO_HV_MUX_ACTIVE_HIGH, read8(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  write8(SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end

  measurement_timing_budget_us = getMeasurementTimingBudget();

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

  write8(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  setMeasurementTimingBudget(measurement_timing_budget_us);

  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  return calibrate();
  // VL53L0X_PerformRefCalibration() end
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool VL53L0X::setSignalRateLimit(float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  write16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7), MSBFirst);
  return true;
}

// Get the return signal rate limit check value in MCPS
float VL53L0X::getSignalRateLimit(void)
{
  return (float)read16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, MSBFirst) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool VL53L0X::setMeasurementTimingBudget(uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    write16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks), MSBFirst);

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t VL53L0X::getMeasurementTimingBudget(void)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
bool VL53L0X::setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
        write8(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
        write8(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
        write8(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
        write8(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return false;
    }
    write8(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    write8(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    write16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_pre_range_timeout_mclks), MSBFirst);

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    write8(MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        write8(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        write8(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        write8(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        write8(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        write8(0xFF, 0x01);
        write8(ALGO_PHASECAL_LIM, 0x30);
        write8(0xFF, 0x00);
        break;

      case 10:
        write8(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        write8(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        write8(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        write8(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        write8(0xFF, 0x01);
        write8(ALGO_PHASECAL_LIM, 0x20);
        write8(0xFF, 0x00);
        break;

      case 12:
        write8(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        write8(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        write8(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        write8(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        write8(0xFF, 0x01);
        write8(ALGO_PHASECAL_LIM, 0x20);
        write8(0xFF, 0x00);
        break;

      case 14:
        write8(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        write8(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        write8(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        write8(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        write8(0xFF, 0x01);
        write8(ALGO_PHASECAL_LIM, 0x20);
        write8(0xFF, 0x00);
        break;

      default:
        // invalid period
        return false;
    }

    // apply new VCSEL period
    write8(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    write16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_final_range_timeout_mclks), MSBFirst);

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return false;
  }

  // "Finally, the timing budget must be re-applied"

  setMeasurementTimingBudget(measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8_t sequence_config = read8(SYSTEM_SEQUENCE_CONFIG);
  write8(SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(0x0);
  write8(SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t VL53L0X::getVcselPulsePeriod(vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(read8(PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(read8(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void VL53L0X::startContinuous(uint32_t period_ms)
{
  write8(0x80, 0x01);
  write8(0xFF, 0x01);
  write8(0x00, 0x00);
  write8(0x91, stop_variable);
  write8(0x00, 0x01);
  write8(0xFF, 0x00);
  write8(0x80, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

    uint16_t osc_calibrate_val = read16(OSC_CALIBRATE_VAL, MSBFirst);

    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }

    write32(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms, MSBFirst);

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

    write8(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }
  else
  {
    // continuous back-to-back mode
    write8(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }

	measurement_initiated = true;
	continuous_measurement = true;
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void VL53L0X::stopContinuous(void)
{
  write8(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

  write8(0xFF, 0x01);
  write8(0x00, 0x00);
  write8(0x91, 0x00);
  write8(0x00, 0x01);
  write8(0xFF, 0x00);

	measurement_initiated = false;
	continuous_measurement = false;
}

// Returns a range reading in millimeters when continuous mode is active
// or after call to startRangeSingle
bool VL53L0X::readRangeMillimeters(Measurement_t * result, uint16_t offset)
{
	if (!wait_while(RESULT_INTERRUPT_STATUS, 0x07, 0x00, io_timeout)) {
		result->num_readings = 1;
		result->status = 0;
		result->timeout = true;
		result->readings[offset].distance = INFINITY;
		return false;
	}

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  uint16_t range = read16(RESULT_RANGE_STATUS + 10, MSBFirst);

  write8(SYSTEM_INTERRUPT_CLEAR, 0x01);

	result->num_readings = 1;
	result->status = 0;
	result->timeout = false;
	result->readings[offset].distance = range;

	if (!continuous_measurement) {
		measurement_initiated = false;
	}

  return true;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
bool VL53L0X::startRangeSingle(void)
{
  write8(0x80, 0x01);
  write8(0xFF, 0x01);
  write8(0x00, 0x00);
  write8(0x91, stop_variable);
  write8(0x00, 0x01);
  write8(0xFF, 0x00);
  write8(0x80, 0x00);
  write8(SYSRANGE_START, 0x01);

	measurement_initiated = true;
	continuous_measurement = false;

  // "Wait until start bit has been cleared"
	return wait_while(SYSRANGE_START, 0x01, 0x01, io_timeout);
}


// Private Methods /////////////////////////////////////////////////////////////

// based on VL53L0X_perform_single_ref_calibration()
bool VL53L0X::performSingleRefCalibration(uint8_t vhv_init_byte)
{
	write8(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP
	wait_while(RESULT_INTERRUPT_STATUS, 0x07, 0x00, io_timeout);
	write8(SYSTEM_INTERRUPT_CLEAR, 0x01);
	write8(SYSRANGE_START, 0x00);

	return true;
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool VL53L0X::getSpadInfo(uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  write8(0x80, 0x01);
  write8(0xFF, 0x01);
  write8(0x00, 0x00);

  write8(0xFF, 0x06);
  write8(0x83, read8(0x83) | 0x04);
  write8(0xFF, 0x07);
  write8(0x81, 0x01);

  write8(0x80, 0x01);

  write8(0x94, 0x6b);
  write8(0x83, 0x00);
	if (!wait_while(0x83, 0xFF, 0x00, io_timeout))
		return false;
  write8(0x83, 0x01);
  tmp = read8(0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  write8(0x81, 0x00);
  write8(0xFF, 0x06);
  write8(0x83, read8(0x83)  & ~0x04);
  write8(0xFF, 0x01);
  write8(0x00, 0x01);

  write8(0xFF, 0x00);
  write8(0x80, 0x00);

  return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void VL53L0X::getSequenceStepEnables(SequenceStepEnables * enables)
{
  uint8_t sequence_config = read8(SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void VL53L0X::getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = read8(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(read16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, MSBFirst));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    decodeTimeout(read16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, MSBFirst));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t VL53L0X::decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t VL53L0X::encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t VL53L0X::timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t VL53L0X::timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

}

#endif
