/**
 * @file MPU60X0.h
 * @author Basit Ayantunde (rlamarrr@gmail.com)
 * @brief
 * @version 0.1
 * @date 2019-08-23
 *
 *
 * @copyright Copyright (c) 2019 Basit Ayantunde
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
#ifndef DRIVERS_MPU60X0_MPU60X0_H_
#define DRIVERS_MPU60X0_MPU60X0_H_

#include <chrono>  // NOLINT
#include <cinttypes>
#include <cstdio>
#include <tuple>
#include <utility>

extern "C" {
#include "main.h"  // NOLINT

#ifdef STM32F7
#include "stm32f7xx_hal.h"       // NOLINT
#include "stm32f7xx_hal_i2c.h"   // NOLINT
#include "stm32f7xx_hal_uart.h"  // NOLINT
#else
#ifdef STM32F4
#include "stm32f4xx_hal.h"       // NOLINT
#include "stm32f4xx_hal_i2c.h"   // NOLINT
#include "stm32f4xx_hal_uart.h"  // NOLINT
#else
#ifdef STM32F3
#include "stm32f3xx_hal.h"       // NOLINT
#include "stm32f3xx_hal_i2c.h"   // NOLINT
#include "stm32f3xx_hal_uart.h"  // NOLINT
#endif
}

#include "bit_definitions.h"  // NOLINT
#include "data_types.h"       // NOLINT
#include "registers.h"        // NOLINT

namespace mpu60X0 {

enum struct Status : uint8_t {
	OK = HAL_StatusTypeDef::HAL_OK,
	Error = HAL_StatusTypeDef::HAL_ERROR,
	Busy = HAL_StatusTypeDef::HAL_BUSY,
	Timeout = HAL_StatusTypeDef::HAL_TIMEOUT,
};

// Blocking master mode API that maximizes utilization of Burst byte Reads
struct MPU60X0 {
	using duration_type = std::chrono::duration<uint32_t, std::milli>;

private:
	uint8_t device_address_;
	I2C_HandleTypeDef* handle_;

public:
	MPU60X0(uint8_t address, I2C_HandleTypeDef* handle) :
			handle_ { handle } {
		device_address_ = address << 1U;
	}

	MPU60X0(const MPU60X0&) {
	}
	MPU60X0(MPU60X0&&) {
	}
	MPU60X0& operator=(const MPU60X0&) = default;
	MPU60X0& operator=(MPU60X0&&) = default;
	~MPU60X0() {
		device_address_ = 0U;
		handle_ = nullptr;
	}

	inline Status WriteByte_(register_type reg, uint8_t data,
			duration_type timeout) {
		uint8_t buffer[2] = { reg, data };
		return static_cast<Status>(HAL_I2C_Master_Transmit(handle_,
				device_address_, buffer, sizeof(buffer), timeout.count()));
	}

	/**
	 * @brief
	 *
	 * @template-parameter ByteRep:
	 * @parameter reg:
	 * @parameter b_rep:
	 * @parameter timeout:
	 * @returns Status:
	 */
	template<typename ByteRep>
	inline Status WriteByteAs_(register_type reg, ByteRep b_rep,
			duration_type timeout) {
		static_assert(sizeof(ByteRep) == 1U, "Type must be byte sized");
		uint8_t value = reinterpret_cast<uint8_t&>(b_rep);
		printf("[WriteByteAs_] Sending value: %u\n", value);
		return WriteByte_(reg, value, timeout);
	}

	/**
	 * @brief
	 *
	 * @parameter reg:
	 * @parameter data:
	 * @parameter size:
	 * @parameter timeout:
	 * @returns Status:
	 */
	inline Status BurstWrite_(register_type reg, const uint8_t* data,
			uint16_t size, duration_type timeout) {
		return static_cast<Status>(HAL_I2C_Master_Transmit(handle_,
				device_address_, const_cast<uint8_t*>(data), size,
				timeout.count()));
	}

	inline std::pair<Status, uint8_t> ReadByte_(register_type reg,
			duration_type timeout) {
		uint8_t data { };
		Status status = static_cast<Status>(HAL_I2C_Master_Transmit(handle_,
				device_address_, &reg, 1, timeout.count()));
		if (status != Status::OK)
			return std::make_pair(status, data);
		status = static_cast<Status>(HAL_I2C_Master_Receive(handle_,
				device_address_, &data, 1, timeout.count()));
		return std::make_pair(status, data);
	}

	template<typename ByteRep>
	inline std::pair<Status, ByteRep> ReadByteAs_(register_type reg,
			duration_type timeout) {

		static_assert(sizeof(ByteRep) == 1U, "Type must be byte sized");
		std::pair<Status, uint8_t> data = ReadByte_(reg, timeout);
		ByteRep res { };
		*(reinterpret_cast<uint8_t*>(&res)) = data.second;

		return std::make_pair(data.first, res);
	}

	inline Status BurstRead_(register_type reg, uint8_t* data, uint16_t size,
			duration_type timeout) {
		Status status = static_cast<Status>(HAL_I2C_Master_Transmit(handle_,
				device_address_, &reg, 1, timeout.count()));
		if (status != Status::OK)
			return status;
		status = static_cast<Status>(HAL_I2C_Master_Receive(handle_,
				device_address_, data, size, timeout.count()));
		return status;
	}

	inline bool Ready(uint32_t trials, duration_type timeout) {
		return HAL_I2C_IsDeviceReady(handle_, device_address_, trials,
				timeout.count()) == HAL_OK;
	}

	// that the device be configured to use one of the gyroscopes (or an external
	// clock source) as the clock reference for improved stability
	inline Status Initialize() {
		PowerManagement1_Config cfg { };
		cfg.clock_source = ClockSource::PllGyro_X;
		cfg.reset_registers = true;
		duration_type timeout = duration_type { 250 };
		return WritePowerManagement1_Config(cfg, timeout);
	}

	/**
	 *
	 *
	 **/

	inline Status WriteSelfTestX(uint8_t data, duration_type timeout) {
		return WriteByte_(register_map::SelfTestX, data, timeout);
	}

	inline std::pair<Status, uint8_t> ReadSelfTestX(duration_type timeout) {
		return ReadByte_(register_map::SelfTestX, timeout);
	}

	inline Status WriteSelfTestY(uint8_t data, duration_type timeout) {
		return WriteByte_(register_map::SelfTestY, data, timeout);
	}

	inline std::pair<Status, uint8_t> ReadSelfTestY(duration_type timeout) {
		return ReadByte_(register_map::SelfTestY, timeout);
	}

	inline Status WriteSelfTestZ(uint8_t data, duration_type timeout) {
		return WriteByte_(register_map::SelfTestZ, data, timeout);
	}

	inline std::pair<Status, uint8_t> ReadSelfTestZ(duration_type timeout) {
		return ReadByte_(register_map::SelfTestZ, timeout);
	}

	inline Status WriteSelfTestA(uint8_t data, duration_type timeout) {
		return WriteByte_(register_map::SelfTestA, data, timeout);
	}

	inline std::pair<Status, uint8_t> ReadSelfTestA(duration_type timeout) {
		return ReadByte_(register_map::SelfTestA, timeout);
	}

	inline std::pair<Status, uint8_t> GetGyroscopeX_SelfTestValue(
			duration_type timeout) {
		std::pair<Status, uint8_t> res = ReadSelfTestX(timeout);
		res.second &= 00011111U;
		return res;
	}

	inline std::pair<Status, uint8_t> GetGyroscopeY_SelfTestValue(
			duration_type timeout) {
		std::pair<Status, uint8_t> res = ReadSelfTestY(timeout);
		res.second &= 00011111U;
		return res;
	}

	inline std::pair<Status, uint8_t> GetGyroscopeZ_SelfTestValue(
			duration_type timeout) {
		std::pair<Status, uint8_t> res = ReadSelfTestZ(timeout);
		res.second &= 00011111U;
		return res;
	}

	// 5 bit precision
	inline std::pair<Status, uint8_t> GetAccelerometerX_SelfTestValue(
			duration_type timeout) {
		std::pair<Status, uint8_t> res_head = ReadSelfTestX(timeout);
		if (res_head.first != Status::OK)
			return res_head;
		std::pair<Status, uint8_t> res_tail = ReadSelfTestA(timeout);
		res_tail.second = ((res_head.second & 0b11100000U) >> 3U)
				| ((res_tail.second & 0b00110000U) >> 4U);
		return res_tail;
	}

	inline std::pair<Status, uint8_t> GetAccelerometerY_SelfTestValue(
			duration_type timeout) {
		std::pair<Status, uint8_t> res_head = ReadSelfTestY(timeout);
		if (res_head.first != Status::OK)
			return res_head;
		std::pair<Status, uint8_t> res_tail = ReadSelfTestA(timeout);
		res_tail.second = ((res_head.second & 0b11100000U) >> 3U)
				| ((res_tail.second & 0b00001100U) >> 2U);
		return res_tail;
	}

	inline std::pair<Status, uint8_t> GetAccelerometerZ_SelfTestValue(
			duration_type timeout) {
		std::pair<Status, uint8_t> res_head = ReadSelfTestZ(timeout);
		if (res_head.first != Status::OK)
			return res_head;
		std::pair<Status, uint8_t> res_tail = ReadSelfTestA(timeout);
		res_tail.second = ((res_head.second & 0b11100000U) >> 3U)
				| (res_tail.second & 0b00000011U);
		return res_tail;
	}

	inline Status SetGyroscopeX_SelfTestValue(uint8_t data,
			duration_type timeout) {
		std::pair<Status, uint8_t> res = ReadSelfTestX(timeout);

		if (res.first != Status::OK)
			return res.first;

		res.second = (res.second & 0b11100000U) | (0b00011111U & data);

		return WriteSelfTestX(res.second, timeout);
	}

	inline Status SetGyroscopeY_SelfTestValue(uint8_t data,
			duration_type timeout) {
		std::pair<Status, uint8_t> res = ReadSelfTestY(timeout);

		if (res.first != Status::OK)
			return res.first;

		res.second = (res.second & 0b11100000U) | (0b00011111U & data);

		return WriteSelfTestY(res.second, timeout);
	}

	inline Status SetGyroscopeZ_SelfTestValue(uint8_t data,
			duration_type timeout) {
		std::pair<Status, uint8_t> res = ReadSelfTestZ(timeout);

		if (res.first != Status::OK)
			return res.first;

		res.second = (res.second & 0b11100000U) | (0b00011111U & data);

		return WriteSelfTestZ(res.second, timeout);
	}

	/**
	 * @brief
	 *
	 * @parameter data: 5 bit value
	 * @parameter timeout:
	 * @returns Status:
	 */
	inline Status SetAccelerometerX_SelfTestValue(uint8_t data,
			duration_type timeout) {
		std::pair<Status, uint8_t> res_head = ReadSelfTestX(timeout);

		if (res_head.first != Status::OK)
			return res_head.first;

		res_head.second = ((data << 3U) & 0b11100000U)
				| (res_head.second & 0b00011111U);
		res_head.first = WriteSelfTestX(res_head.second, timeout);

		if (res_head.first != Status::OK)
			return res_head.first;

		res_head = ReadSelfTestA(timeout);

		if (res_head.first != Status::OK)
			return res_head.first;

		res_head.second = ((data << 4) & 0b00110000U)
				| (res_head.second & 0b11001111U);

		return WriteSelfTestA(res_head.second, timeout);
	}

	inline Status SetAccelerometerY_SelfTestValue(uint8_t data,
			duration_type timeout) {
		std::pair<Status, uint8_t> res_head = ReadSelfTestY(timeout);

		if (res_head.first != Status::OK)
			return res_head.first;

		res_head.second = ((data << 3) & 0b11100000U)
				| (res_head.second & 0b00011111U);
		res_head.first = WriteSelfTestY(res_head.second, timeout);

		if (res_head.first != Status::OK)
			return res_head.first;

		res_head = ReadSelfTestA(timeout);

		if (res_head.first != Status::OK)
			return res_head.first;

		res_head.second = ((data << 2) & 0b00001100U)
				| (res_head.second & 0b11110011U);

		return WriteSelfTestA(res_head.second, timeout);
	}

	inline Status SetAccelerometerZ_SelfTestValue(uint8_t data,
			duration_type timeout) {
		std::pair<Status, uint8_t> res_head = ReadSelfTestZ(timeout);

		if (res_head.first != Status::OK)
			return res_head.first;

		res_head.second = ((data << 3) & 0b11100000U)
				| (res_head.second & 0b00011111U);
		res_head.first = WriteSelfTestZ(res_head.second, timeout);

		if (res_head.first != Status::OK)
			return res_head.first;

		res_head = ReadSelfTestA(timeout);

		if (res_head.first != Status::OK)
			return res_head.first;

		res_head.second = (data & 0b00000011U)
				| (res_head.second & 0b11111100U);

		return WriteSelfTestA(res_head.second, timeout);
	}

	/**
	 *
	 *
	 **/

	inline Status WriteSampleRateDivider(uint8_t data, duration_type timeout) {
		return WriteByte_(register_map::SampleRateDivider, data, timeout);
	}

	inline std::pair<Status, uint8_t> ReadSampleRateDivider(
			duration_type timeout) {
		return ReadByte_(register_map::SampleRateDivider, timeout);
	}
	/**
	 *
	 *
	 **/

	inline Status WriteConfig(Config config, duration_type timeout) {
		return WriteByteAs_<Config>(register_map::Config, config, timeout);
	}

	inline std::pair<Status, Config> ReadConfig(duration_type timeout) {
		return ReadByteAs_<Config>(register_map::Config, timeout);
	}

	inline Status SetExternalFrameSync(FrameSync fsync, duration_type timeout) {
		//
		std::pair<Status, Config> conf = ReadConfig(timeout);

		if (conf.first != Status::OK)
			return conf.first;

		conf.second.ext_frame_sync =
				static_cast<FrameSync>(static_cast<uint8_t>(fsync) & 0b111U);

		return WriteConfig(conf.second, timeout);
	}

	inline Status SetDlpfConfig(DlpfConfig dlpf_cfg, duration_type timeout) {
		std::pair<Status, Config> conf = ReadConfig(timeout);

		if (conf.first != Status::OK)
			return conf.first;

		conf.second.dlpf_config =
				static_cast<DlpfConfig>(static_cast<uint8_t>(dlpf_cfg) & 0b111U);

		return WriteConfig(conf.second, timeout);
	}

	inline std::pair<Status, FrameSync> GetExternalFrameSync(
			duration_type timeout) {
		std::pair<Status, Config> conf = ReadConfig(timeout);

		return std::make_pair(conf.first,
				FrameSync { conf.second.ext_frame_sync });
	}

	inline std::pair<Status, DlpfConfig> GetDlpfConfig(duration_type timeout) {
		std::pair<Status, Config> conf = ReadConfig(timeout);

		return std::make_pair(conf.first,
				DlpfConfig { conf.second.dlpf_config });
	}

	/**
	 *
	 *
	 **/

	inline Status WriteGyroscopeConfig(GyroscopeConfig gyro_cfg,
			duration_type timeout) {
		return WriteByteAs_(register_map::GyroscopeConfig, gyro_cfg, timeout);
	}

	inline std::pair<Status, GyroscopeConfig> ReadGyroscopeConfig(
			duration_type timeout) {
		return ReadByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig,
				timeout);
	}

	inline Status SetGyroscopeX_SelfTestState(bool activate,
			duration_type timeout) {
		std::pair<Status, GyroscopeConfig> res = ReadByteAs_<GyroscopeConfig>(
				register_map::GyroscopeConfig, timeout);

		if (res.first != Status::OK)
			return res.first;

		res.second.x_self_test = activate;

		return WriteByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig,
				res.second, timeout);
	}

	inline Status SetGyroscopeY_SelfTestState(bool activate,
			duration_type timeout) {
		std::pair<Status, GyroscopeConfig> res = ReadByteAs_<GyroscopeConfig>(
				register_map::GyroscopeConfig, timeout);

		if (res.first != Status::OK)
			return res.first;

		res.second.y_self_test = activate;

		return WriteByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig,
				res.second, timeout);
	}

	inline Status SetGyroscopeZ_SelfTestState(bool activate,
			duration_type timeout) {
		std::pair<Status, GyroscopeConfig> res = ReadByteAs_<GyroscopeConfig>(
				register_map::GyroscopeConfig, timeout);

		if (res.first != Status::OK)
			return res.first;

		res.second.z_self_test = activate;

		return WriteByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig,
				res.second, timeout);
	}

	// 3 bit, x, y, z
	inline Status SetGyroscopeSelfTestStates(uint8_t tri_state,
			duration_type timeout) {
		std::pair<Status, GyroscopeConfig> res = ReadByteAs_<GyroscopeConfig>(
				register_map::GyroscopeConfig, timeout);

		if (res.first != Status::OK)
			return res.first;

		res.second.x_self_test = static_cast<bool>(0b0000100 & tri_state);
		res.second.y_self_test = static_cast<bool>(0b0000010 & tri_state);
		res.second.z_self_test = static_cast<bool>(0b0000001 & tri_state);

		return WriteByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig,
				res.second, timeout);
	}

	inline Status SetGyroscopeFullScaleRange(GyroscopeFullScale full_scale,
			duration_type timeout) {
		std::pair<Status, GyroscopeConfig> res = ReadByteAs_<GyroscopeConfig>(
				register_map::GyroscopeConfig, timeout);

		if (res.first != Status::OK)
			return res.first;

		res.second.full_scale = full_scale;

		return WriteByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig,
				res.second, timeout);
	}

	inline std::pair<Status, bool> GetGyroscopeX_SelfTestState(
			duration_type timeout) {
		std::pair<Status, GyroscopeConfig> res = ReadByteAs_<GyroscopeConfig>(
				register_map::GyroscopeConfig, timeout);

		return std::make_pair(res.first, bool { res.second.x_self_test });
	}

	inline std::pair<Status, bool> GetGyroscopeY_SelfTestState(
			duration_type timeout) {
		std::pair<Status, GyroscopeConfig> res = ReadByteAs_<GyroscopeConfig>(
				register_map::GyroscopeConfig, timeout);

		return std::make_pair(res.first, bool { res.second.y_self_test });
	}

	inline std::pair<Status, bool> GetGyroscopeZ_SelfTestState(
			duration_type timeout) {
		std::pair<Status, GyroscopeConfig> res = ReadByteAs_<GyroscopeConfig>(
				register_map::GyroscopeConfig, timeout);

		return std::make_pair(res.first, bool { res.second.z_self_test });
	}

	// first bit, second bit, third bit
	inline std::pair<Status, uint8_t> GetGyroscopeSelfTestStates(
			duration_type timeout) {
		uint8_t data { };

		std::pair<Status, GyroscopeConfig> res = ReadGyroscopeConfig(timeout);

		if (res.first != Status::OK)
			return std::make_pair(res.first, data);

		data = (static_cast<uint8_t>(res.second.x_self_test) << 2U)
				| (static_cast<uint8_t>(res.second.y_self_test) << 1U)
				| static_cast<uint8_t>(res.second.z_self_test);

		return std::make_pair(res.first, data);
	}

	inline std::pair<Status, GyroscopeFullScale> GetGyroscopeFullScaleRange(
			duration_type timeout) {
		std::pair<Status, GyroscopeConfig> res = ReadByteAs_<GyroscopeConfig>(
				register_map::GyroscopeConfig, timeout);

		return std::make_pair(res.first,
				GyroscopeFullScale { res.second.full_scale });
	}

	/**
	 *
	 *
	 **/

	inline Status WriteAccelerometerConfig(AccelerometerConfig acc_cfg,
			duration_type timeout) {
		return WriteByteAs_<AccelerometerConfig>(
				register_map::AccelerometerConfig, acc_cfg, timeout);
	}

	inline std::pair<Status, AccelerometerConfig> ReadAccelerometerConfig(
			duration_type timeout) {
		return ReadByteAs_<AccelerometerConfig>(
				register_map::AccelerometerConfig, timeout);
	}

	inline Status SetAccelerometerX_SelfTestState(bool activate,
			duration_type timeout) {
		std::pair<Status, AccelerometerConfig> acc_cfg =
				ReadAccelerometerConfig(timeout);

		if (acc_cfg.first != Status::OK)
			return acc_cfg.first;

		acc_cfg.second.x_self_test = activate;

		return WriteByteAs_<AccelerometerConfig>(
				register_map::AccelerometerConfig, acc_cfg.second, timeout);
	}

	inline Status SetAccelerometerY_SelfTestState(bool activate,
			duration_type timeout) {
		std::pair<Status, AccelerometerConfig> acc_cfg =
				ReadAccelerometerConfig(timeout);

		if (acc_cfg.first != Status::OK)
			return acc_cfg.first;

		acc_cfg.second.y_self_test = activate;

		return WriteByteAs_<AccelerometerConfig>(
				register_map::AccelerometerConfig, acc_cfg.second, timeout);
	}

	inline Status SetAccelerometerZ_SelfTestState(bool activate,
			duration_type timeout) {
		std::pair<Status, AccelerometerConfig> acc_cfg =
				ReadAccelerometerConfig(timeout);

		if (acc_cfg.first != Status::OK)
			return acc_cfg.first;

		acc_cfg.second.z_self_test = activate;

		return WriteByteAs_<AccelerometerConfig>(
				register_map::AccelerometerConfig, acc_cfg.second, timeout);
	}

	inline Status SetAccelerometerSelfTestStates(uint8_t tri_state,
			duration_type timeout) {
		std::pair<Status, AccelerometerConfig> res = ReadByteAs_<
				AccelerometerConfig>(register_map::AccelerometerConfig,
				timeout);

		if (res.first != Status::OK)
			return res.first;

		res.second.x_self_test = static_cast<bool>(0b0000100 & tri_state);
		res.second.y_self_test = static_cast<bool>(0b0000010 & tri_state);
		res.second.z_self_test = static_cast<bool>(0b0000001 & tri_state);

		return WriteByteAs_<AccelerometerConfig>(
				register_map::AccelerometerConfig, res.second, timeout);
	}

	inline Status SetAccelerometerFullScaleRange(
			AccelerometerFullScale full_scale, duration_type timeout) {
		std::pair<Status, AccelerometerConfig> res = ReadByteAs_<
				AccelerometerConfig>(register_map::AccelerometerConfig,
				timeout);

		if (res.first != Status::OK)
			return res.first;

		res.second.full_scale = full_scale;

		return WriteByteAs_<AccelerometerConfig>(
				register_map::AccelerometerConfig, res.second, timeout);
	}

	inline std::pair<Status, bool> GetAccelerometerX_SelfTestState(
			duration_type timeout) {
		std::pair<Status, AccelerometerConfig> res = ReadByteAs_<
				AccelerometerConfig>(register_map::GyroscopeConfig, timeout);

		return std::make_pair(res.first, bool { res.second.x_self_test });
	}

	inline std::pair<Status, bool> GetAccelerometerY_SelfTestState(
			duration_type timeout) {
		std::pair<Status, AccelerometerConfig> res = ReadByteAs_<
				AccelerometerConfig>(register_map::GyroscopeConfig, timeout);

		return std::make_pair(res.first, bool { res.second.y_self_test });
	}

	inline std::pair<Status, bool> GetAccelerometerZ_SelfTestState(
			duration_type timeout) {
		std::pair<Status, AccelerometerConfig> res = ReadByteAs_<
				AccelerometerConfig>(register_map::GyroscopeConfig, timeout);

		return std::make_pair(res.first, bool { res.second.z_self_test });
	}

	inline std::pair<Status, uint8_t> GetAccelerometerSelfTestStates(
			duration_type timeout) {
		uint8_t data { };

		std::pair<Status, AccelerometerConfig> res = ReadAccelerometerConfig(
				timeout);

		if (res.first != Status::OK)
			return std::make_pair(res.first, data);

		data = (static_cast<uint8_t>(res.second.x_self_test) << 2U)
				| (static_cast<uint8_t>(res.second.y_self_test) << 1U)
				| static_cast<uint8_t>(res.second.z_self_test);

		return std::make_pair(res.first, data);
	}

	inline std::pair<Status, AccelerometerFullScale> GetAccelerometerFullScaleRange(
			duration_type timeout) {
		std::pair<Status, AccelerometerConfig> res = ReadAccelerometerConfig(
				timeout);
		return std::make_pair(res.first,
				AccelerometerFullScale { res.second.full_scale });
	}

	/**
	 *
	 *
	 **/

	inline Status WriteFreeFallAccelerationThreshold(uint8_t acc_thresh,
			duration_type timeout) {
		return WriteByte_(register_map::FreeFallAccelerationThreshold,
				acc_thresh, timeout);
	}

	inline std::pair<Status, uint8_t> ReadFreeFallAccelerationThreshold(
			duration_type timeout) {
		return ReadByte_(register_map::FreeFallAccelerationThreshold, timeout);
	}

	/**
	 *
	 *
	 **/

	inline Status WriteFreeFallDuration(uint8_t ff_dur, duration_type timeout) {
		return WriteByte_(register_map::FreeFallDuration, ff_dur, timeout);
	}

	inline std::pair<Status, uint8_t> ReadFreeFallDuration(
			duration_type timeout) {
		return ReadByte_(register_map::FreeFallDuration, timeout);
	}

	/**
	 *
	 *
	 **/

	inline Status WriteMotionDetectionThreshold(uint8_t mot_det_thresh,
			duration_type timeout) {
		return WriteByte_(register_map::MotionDetectionThreshold,
				mot_det_thresh, timeout);
	}

	inline std::pair<Status, uint8_t> ReadMotionDetectionThreshold(
			duration_type timeout) {
		return ReadByte_(register_map::MotionDetectionThreshold, timeout);
	}

	/**
	 *
	 *
	 **/

	inline Status WriteMotionDetectionDuration(uint8_t mot_det_dur,
			duration_type timeout) {
		return WriteByte_(register_map::MotionDetectionDuration, mot_det_dur,
				timeout);
	}

	inline std::pair<Status, uint8_t> ReadMotionDetectionDuration(
			duration_type timeout) {
		return ReadByte_(register_map::MotionDetectionDuration, timeout);
	}

	/**
	 *
	 *
	 **/

	inline Status WriteZeroMotionDetectionThreshold(uint8_t mot_det_thresh,
			duration_type timeout) {
		return WriteByte_(register_map::MotionDetectionThreshold,
				mot_det_thresh, timeout);
	}

	inline std::pair<Status, uint8_t> ReadZeroMotionDetectionThreshold(
			duration_type timeout) {
		return ReadByte_(register_map::MotionDetectionThreshold, timeout);
	}

	/**
	 *
	 *
	 **/

	inline Status WriteZeroMotionDetectionDuration(uint8_t mot_det_dur,
			duration_type timeout) {
		return WriteByte_(register_map::MotionDetectionDuration, mot_det_dur,
				timeout);
	}

	inline std::pair<Status, uint8_t> ReadZeroMotionDetectionDuration(
			duration_type timeout) {
		return ReadByte_(register_map::MotionDetectionDuration, timeout);
	}

	/**
	 *
	 *
	 **/

	inline Status WriteFifoEnableConfig(FifoEnableConfig fifo_en_cfg,
			duration_type timeout) {
		return WriteByteAs_<FifoEnableConfig>(register_map::FifoEnable,
				fifo_en_cfg, timeout);
	}

	inline std::pair<Status, FifoEnableConfig> ReadFifoEnableConfig(
			duration_type timeout) {
		return ReadByteAs_<FifoEnableConfig>(register_map::FifoEnable, timeout);
	}

	inline Status SetTemperatureFifoEnable(bool enable, duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		if (res.first != Status::OK)
			return res.first;

		res.second.enable_temperature_fifo = enable;

		return WriteFifoEnableConfig(res.second, timeout);
	}

	inline Status SetGyroscopeX_FifoEnable(bool enable, duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		if (res.first != Status::OK)
			return res.first;

		res.second.enable_gyroscope_x_fifo = enable;

		return WriteFifoEnableConfig(res.second, timeout);
	}

	inline Status SetGyroscopeY_FifoEnable(bool enable, duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		if (res.first != Status::OK)
			return res.first;

		res.second.enable_gyroscope_y_fifo = enable;

		return WriteFifoEnableConfig(res.second, timeout);
	}

	inline Status SetGyroscopeZ_FifoEnable(bool enable, duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		if (res.first != Status::OK)
			return res.first;

		res.second.enable_gyroscope_z_fifo = enable;

		return WriteFifoEnableConfig(res.second, timeout);
	}

	inline Status SetAccelerometerFifoEnable(bool enable,
			duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		if (res.first != Status::OK)
			return res.first;

		res.second.enable_acceleration_fifo = enable;

		return WriteFifoEnableConfig(res.second, timeout);
	}

	inline Status SetSlave2_FifoEnable(bool enable, duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		if (res.first != Status::OK)
			return res.first;

		res.second.enable_slave2_fifo = enable;

		return WriteFifoEnableConfig(res.second, timeout);
	}

	inline Status SetSlave1_FifoEnable(bool enable, duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		if (res.first != Status::OK)
			return res.first;

		res.second.enable_slave1_fifo = enable;

		return WriteFifoEnableConfig(res.second, timeout);
	}

	inline Status SetSlave0_FifoEnable(bool enable, duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		if (res.first != Status::OK)
			return res.first;

		res.second.enable_slave0_fifo = enable;

		return WriteFifoEnableConfig(res.second, timeout);
	}

	inline std::pair<Status, bool> GetTemperatureFifoEnable(
			duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		return std::make_pair(res.first,
				bool { res.second.enable_temperature_fifo });
	}

	inline std::pair<Status, bool> GetGyroscopeX_FifoEnable(
			duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		return std::make_pair(res.first,
				bool { res.second.enable_gyroscope_x_fifo });
	}

	inline std::pair<Status, bool> GetGyroscopeY_FifoEnable(
			duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		return std::make_pair(res.first,
				bool { res.second.enable_gyroscope_y_fifo });
	}

	inline std::pair<Status, bool> GetGyroscopeZ_FifoEnable(
			duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		return std::make_pair(res.first,
				bool { res.second.enable_gyroscope_z_fifo });
	}

	inline std::pair<Status, bool> GetAccelerometerFifoEnable(
			duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		return std::make_pair(res.first,
				bool { res.second.enable_acceleration_fifo });
	}

	inline std::pair<Status, bool> GetSlave2_FifoEnable(duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		return std::make_pair(res.first, bool { res.second.enable_slave2_fifo });
	}

	inline std::pair<Status, bool> GetSlave1_FifoEnable(duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		return std::make_pair(res.first, bool { res.second.enable_slave1_fifo });
	}

	inline std::pair<Status, bool> GetSlave0_FifoEnable(duration_type timeout) {
		std::pair<Status, FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
		return std::make_pair(res.first, bool { res.second.enable_slave0_fifo });
	}

	/**
	 *
	 *
	 **/

	inline Status WriteI2cMasterCtrlConfig(I2cMasterCtrlConfig mast_cfg,
			duration_type timeout) {
		return WriteByteAs_<I2cMasterCtrlConfig>(register_map::I2cMasterCtrl,
				mast_cfg, timeout);
	}

	inline std::pair<Status, I2cMasterCtrlConfig> ReadI2cMasterCtrlConfig(
			duration_type timeout) {
		return ReadByteAs_<I2cMasterCtrlConfig>(register_map::I2cMasterCtrl,
				timeout);
	}

	inline Status SetMultiMasterEnable(bool enable, duration_type timeout) {
		std::pair<Status, I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(
				timeout);

		res.second.enable_multi_master = enable;

		return WriteByteAs_<I2cMasterCtrlConfig>(register_map::I2cMasterCtrl,
				res.second, timeout);
	}

	// DRI - Data ready Interrupt
	inline Status SetDRI_AwaitExtSensorData(bool enable,
			duration_type timeout) {
		std::pair<Status, I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(
				timeout);

		res.second.dri_await_ext_sensor_data = enable;

		return WriteByteAs_<I2cMasterCtrlConfig>(register_map::I2cMasterCtrl,
				res.second, timeout);
	}

	inline Status SetSlave3_FifoEnable(bool enable, duration_type timeout) {
		std::pair<Status, I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(
				timeout);

		res.second.enable_slave3_fifo = enable;

		return WriteByteAs_<I2cMasterCtrlConfig>(register_map::I2cMasterCtrl,
				res.second, timeout);
	}

	inline Status SetI2cMasterReadWriteTransition(
			I2cMasterReadWriteTransition rw_trans, duration_type timeout) {
		std::pair<Status, I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(
				timeout);

		res.second.rw_transition =
				static_cast<I2cMasterReadWriteTransition>(static_cast<uint8_t>(rw_trans)
						& 0b1U);

		return WriteByteAs_<I2cMasterCtrlConfig>(register_map::I2cMasterCtrl,
				res.second, timeout);
	}
	inline Status SetI2cMasterClockSpeed(I2cMasterClockSpeed clock_speed,
			duration_type timeout) {
		std::pair<Status, I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(
				timeout);

		res.second.clock_speed =
				static_cast<I2cMasterClockSpeed>(static_cast<uint8_t>(clock_speed)
						& 0b1111U);

		return WriteByteAs_<I2cMasterCtrlConfig>(register_map::I2cMasterCtrl,
				res.second, timeout);
	}

	inline std::pair<Status, bool> GetMultiMasterEnable(duration_type timeout) {
		std::pair<Status, I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(
				timeout);

		return std::make_pair(res.first,
				bool { res.second.enable_multi_master });
	}

	inline std::pair<Status, bool> GetDRI_AwaitExtSensorData(
			duration_type timeout) {
		std::pair<Status, I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(
				timeout);

		return std::make_pair(res.first,
				bool { res.second.dri_await_ext_sensor_data });
	}

	inline std::pair<Status, bool> GetSlave3_FifoEnable(duration_type timeout) {
		std::pair<Status, I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(
				timeout);

		return std::make_pair(res.first, bool { res.second.enable_slave3_fifo });
	}

	inline std::pair<Status, I2cMasterReadWriteTransition> GetI2cMasterReadWriteTransition(
			duration_type timeout) {
		std::pair<Status, I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(
				timeout);

		return std::make_pair(res.first,
				I2cMasterReadWriteTransition { res.second.rw_transition });
	}

	inline std::pair<Status, I2cMasterClockSpeed> GetI2cMasterClockSpeed(
			duration_type timeout) {
		std::pair<Status, I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(
				timeout);

		return std::make_pair(res.first,
				I2cMasterClockSpeed { res.second.clock_speed });
	}

	/**
	 *
	 *
	 **/

	/**
	 *
	 *
	 **/

	/**
	 *
	 *
	 **/

	/**
	 *
	 *
	 **/

	/**
	 *
	 *
	 **/

	inline Status WriteInterruptPinConfig(InterruptPinConfig config,
			duration_type timeout) {
		return WriteByteAs_<InterruptPinConfig>(
				register_map::InterruptPinConfig, config, timeout);
	}

	inline std::pair<Status, InterruptPinConfig> ReadInterruptPinConfig(
			duration_type timeout) {
		return ReadByteAs_<InterruptPinConfig>(register_map::InterruptPinConfig,
				timeout);
	}

	/**
	 *
	 *
	 **/

	inline Status WriteInterruptEnableConfig(InterruptEnableConfig int_en_cfg,
			duration_type timeout) {
		return WriteByteAs_<InterruptEnableConfig>(
				register_map::InterruptEnable, int_en_cfg, timeout);
	}

	inline std::pair<Status, InterruptEnableConfig> ReadInterruptEnableConfig(
			duration_type timeout) {
		return ReadByteAs_<InterruptEnableConfig>(register_map::InterruptEnable,
				timeout);
	}

	inline Status SetFreeFallDetectionInterruptEnable(bool enable,
			duration_type timeout) {
		std::pair<Status, InterruptEnableConfig> res =
				ReadInterruptEnableConfig(timeout);

		if (res.first != Status::OK)
			return res.first;
		res.second.enable_freefall_detection_interrupt = enable;
		return WriteInterruptEnableConfig(res.second, timeout);
	}

	inline Status SetMotionDetectionInterruptEnable(bool enable,
			duration_type timeout) {
		std::pair<Status, InterruptEnableConfig> res =
				ReadInterruptEnableConfig(timeout);

		if (res.first != Status::OK)
			return res.first;
		res.second.enable_motion_detection_interrupt = enable;

		return WriteInterruptEnableConfig(res.second, timeout);
	}

	inline Status SetZeroMotionDetectionInterruptEnable(bool enable,
			duration_type timeout) {
		std::pair<Status, InterruptEnableConfig> res =
				ReadInterruptEnableConfig(timeout);

		if (res.first != Status::OK)
			return res.first;
		res.second.enable_zero_motion_detection_interrupt = enable;
		return WriteInterruptEnableConfig(res.second, timeout);
	}

	inline Status SetFifoOverflowInterruptEnable(bool enable,
			duration_type timeout) {
		std::pair<Status, InterruptEnableConfig> res =
				ReadInterruptEnableConfig(timeout);

		if (res.first != Status::OK)
			return res.first;
		res.second.fifo_overflow_interrupt_enable = enable;

		return WriteInterruptEnableConfig(res.second, timeout);
	}

	inline Status SetI2cMasterInterruptEnable(bool enable,
			duration_type timeout) {
		std::pair<Status, InterruptEnableConfig> res =
				ReadInterruptEnableConfig(timeout);

		if (res.first != Status::OK)
			return res.first;
		res.second.i2c_master_interrupt_enable = enable;

		return WriteInterruptEnableConfig(res.second, timeout);
	}

	inline Status SetDataReadyInterruptEnable(bool enable,
			duration_type timeout) {
		std::pair<Status, InterruptEnableConfig> res =
				ReadInterruptEnableConfig(timeout);

		if (res.first != Status::OK)
			return res.first;
		res.second.data_ready_interrupt_enable = enable;

		return WriteInterruptEnableConfig(res.second, timeout);
	}

	inline std::pair<Status, bool> GetFreeFallDetectionInterruptEnable(
			duration_type timeout) {
		std::pair<Status, InterruptEnableConfig> res =
				ReadInterruptEnableConfig(timeout);
		return std::make_pair(res.first,
				bool { res.second.enable_freefall_detection_interrupt });
	}

	inline std::pair<Status, bool> GetMotionDetectionInterruptEnable(
			duration_type timeout) {
		std::pair<Status, InterruptEnableConfig> res =
				ReadInterruptEnableConfig(timeout);
		return std::make_pair(res.first,
				bool { res.second.enable_motion_detection_interrupt });
	}

	inline std::pair<Status, bool> GetZeroMotionDetectionInterruptEnable(
			duration_type timeout) {
		std::pair<Status, InterruptEnableConfig> res =
				ReadInterruptEnableConfig(timeout);
		return std::make_pair(res.first,
				bool { res.second.enable_zero_motion_detection_interrupt });
	}

	inline std::pair<Status, bool> GetFifoOverflowInterruptEnable(
			duration_type timeout) {
		std::pair<Status, InterruptEnableConfig> res =
				ReadInterruptEnableConfig(timeout);
		return std::make_pair(res.first,
				bool { res.second.fifo_overflow_interrupt_enable });
	}

	inline std::pair<Status, bool> GetI2cMasterInterruptEnable(
			duration_type timeout) {
		std::pair<Status, InterruptEnableConfig> res =
				ReadInterruptEnableConfig(timeout);
		return std::make_pair(res.first,
				bool { res.second.i2c_master_interrupt_enable });
	}

	inline std::pair<Status, bool> GetDataReadyInterruptEnable(
			duration_type timeout) {
		std::pair<Status, InterruptEnableConfig> res =
				ReadInterruptEnableConfig(timeout);
		return std::make_pair(res.first,
				bool { res.second.data_ready_interrupt_enable });
	}

	/**
	 *
	 *
	 **/

	inline std::pair<Status, InterruptStatus> ReadInterruptStatus(
			duration_type timeout) {
		return ReadByteAs_<InterruptStatus>(register_map::InterruptStatus,
				timeout);
	}

	inline std::pair<Status, bool> GetFreeFallDetectionInterruptState(
			duration_type timeout) {
		std::pair<Status, InterruptStatus> res = ReadInterruptStatus(timeout);
		return std::make_pair(res.first, bool { res.second.free_fall_detected });
	}
	inline std::pair<Status, bool> GetMotionInterruptState(
			duration_type timeout) {
		std::pair<Status, InterruptStatus> res = ReadInterruptStatus(timeout);
		return std::make_pair(res.first, bool { res.second.motion_detected });
	}
	inline std::pair<Status, bool> GetZeroMotionInterruptState(
			duration_type timeout) {
		std::pair<Status, InterruptStatus> res = ReadInterruptStatus(timeout);
		return std::make_pair(res.first,
				bool { res.second.zero_motion_detected });
	}
	inline std::pair<Status, bool> GetFifoOverflowInterruptState(
			duration_type timeout) {
		std::pair<Status, InterruptStatus> res = ReadInterruptStatus(timeout);
		return std::make_pair(res.first, bool { res.second.fifo_overflow });
	}
	inline std::pair<Status, bool> GetI2cMasterInterruptState(
			duration_type timeout) {
		std::pair<Status, InterruptStatus> res = ReadInterruptStatus(timeout);
		return std::make_pair(res.first,
				bool { res.second.i2c_master_interrupt });
	}
	inline std::pair<Status, bool> GetDataReadyInterruptState(
			duration_type timeout) {
		std::pair<Status, InterruptStatus> res = ReadInterruptStatus(timeout);
		return std::make_pair(res.first, bool { res.second.data_ready });
	}

	/**
	 *
	 *
	 **/

	inline std::pair<Status, int16_t> ReadAccelerometerX(
			duration_type timeout) {
		int16_t data { };
		Status status = BurstRead_(register_map::AccelerometerX_OutH,
				reinterpret_cast<uint8_t*>(&data), sizeof(data), timeout);

		return std::make_pair(status, data);
	}
	inline std::pair<Status, int16_t> ReadAccelerometerY(
			duration_type timeout) {
		int16_t data { };
		Status status = BurstRead_(register_map::AccelerometerY_OutH,
				reinterpret_cast<uint8_t*>(&data), sizeof(data), timeout);

		return std::make_pair(status, data);
	}
	inline std::pair<Status, int16_t> ReadAccelerometerZ(
			duration_type timeout) {
		int16_t data { };
		Status status = BurstRead_(register_map::AccelerometerZ_OutH,
				reinterpret_cast<uint8_t*>(&data), sizeof(data), timeout);

		return std::make_pair(status, data);
	}
	inline std::pair<Status, TriAxialData> ReadAccelerometer(
			duration_type timeout) {
		TriAxialData data { };

		Status status = BurstRead_(register_map::AccelerometerX_OutH,
				reinterpret_cast<uint8_t*>(&data), sizeof(data), timeout);

		return std::make_pair(status, data);
	}

	inline std::pair<Status, BiAxialData> ReadAccelerometerXY(
			duration_type timeout) {
		BiAxialData data { };

		Status status = BurstRead_(register_map::AccelerometerX_OutH,
				reinterpret_cast<uint8_t*>(&data), sizeof(data), timeout);

		return std::make_pair(status, data);
	}
	inline std::pair<Status, BiAxialData> ReadAccelerometerYZ(
			duration_type timeout) {
		BiAxialData data { };

		Status status = BurstRead_(register_map::AccelerometerY_OutH,
				reinterpret_cast<uint8_t*>(&data), sizeof(data), timeout);

		return std::make_pair(status, data);
	}

	/**
	 *
	 *
	 **/

	inline std::pair<Status, int16_t> ReadTemperature(duration_type timeout) {
		int16_t data { };

		Status status = BurstRead_(register_map::TemperatureOutH,
				reinterpret_cast<uint8_t*>(&data), sizeof(data), timeout);

		return std::make_pair(status, data);
	}

	/**
	 *
	 *
	 **/

	inline std::pair<Status, int16_t> ReadGyroscopeX(duration_type timeout) {
		int16_t data { };
		Status status = BurstRead_(register_map::GyroscopeX_OutH,
				reinterpret_cast<uint8_t*>(&data), sizeof(data), timeout);

		return std::make_pair(status, data);
	}
	inline std::pair<Status, int16_t> ReadGyroscopeY(duration_type timeout) {
		int16_t data { };
		Status status = BurstRead_(register_map::GyroscopeY_OutH,
				reinterpret_cast<uint8_t*>(&data), sizeof(data), timeout);

		return std::make_pair(status, data);
	}
	inline std::pair<Status, int16_t> ReadGyroscopeZ(duration_type timeout) {
		int16_t data { };
		Status status = BurstRead_(register_map::GyroscopeZ_OutH,
				reinterpret_cast<uint8_t*>(&data), sizeof(data), timeout);

		return std::make_pair(status, data);
	}
	inline std::pair<Status, TriAxialData> ReadGyroscope(
			duration_type timeout) {
		TriAxialData data { };
		Status status = BurstRead_(register_map::GyroscopeX_OutH,
				reinterpret_cast<uint8_t*>(&data), sizeof(data), timeout);

		return std::make_pair(status, data);
	}

	inline std::pair<Status, BiAxialData> ReadGyroscopeXY(
			duration_type timeout) {
		BiAxialData data { };
		Status status = BurstRead_(register_map::GyroscopeX_OutH,
				reinterpret_cast<uint8_t*>(&data), sizeof(data), timeout);

		return std::make_pair(status, data);
	}
	inline std::pair<Status, BiAxialData> ReadGyroscopeYZ(
			duration_type timeout) {
		BiAxialData data { };
		Status status = BurstRead_(register_map::GyroscopeY_OutH,
				reinterpret_cast<uint8_t*>(&data), sizeof(data), timeout);

		return std::make_pair(status, data);
	}

	/**
	 *
	 *
	 **/

	inline std::pair<Status, uint8_t> ReadExternalSensorData(
			uint8_t sensor_data_index, duration_type timeout) {
		// TODO(lamarrr): find a better implementation with bound checking
		return ReadByte_(register_map::ExternalSensorData0 + sensor_data_index,
				timeout);
	}

	/**
	 *
	 *
	 **/

	inline std::pair<Status, MotionDetectionStatus> ReadMotionDetectionStatus(
			duration_type timeout) {
		return ReadByteAs_<MotionDetectionStatus>(
				register_map::MotionDetectionStatus, timeout);
	}
	inline std::pair<Status, bool> GetNegativeX_MotionDetected(
			duration_type timeout) {
		std::pair<Status, MotionDetectionStatus> res = ReadByteAs_<
				MotionDetectionStatus>(register_map::MotionDetectionStatus,
				timeout);

		return std::make_pair(res.first, bool { res.second.x_negative });
	}
	inline std::pair<Status, bool> GetPositiveX_MotionDetected(
			duration_type timeout) {
		std::pair<Status, MotionDetectionStatus> res = ReadByteAs_<
				MotionDetectionStatus>(register_map::MotionDetectionStatus,
				timeout);

		return std::make_pair(res.first, bool { res.second.x_positive });
	}
	inline std::pair<Status, bool> GetNegativeY_MotionDetected(
			duration_type timeout) {
		std::pair<Status, MotionDetectionStatus> res = ReadByteAs_<
				MotionDetectionStatus>(register_map::MotionDetectionStatus,
				timeout);

		return std::make_pair(res.first, bool { res.second.y_negative });
	}
	inline std::pair<Status, bool> GetPositiveY_MotionDetected(
			duration_type timeout) {
		std::pair<Status, MotionDetectionStatus> res = ReadByteAs_<
				MotionDetectionStatus>(register_map::MotionDetectionStatus,
				timeout);

		return std::make_pair(res.first, bool { res.second.y_positive });
	}
	inline std::pair<Status, bool> GetNegativeZ_MotionDetected(
			duration_type timeout) {
		std::pair<Status, MotionDetectionStatus> res = ReadByteAs_<
				MotionDetectionStatus>(register_map::MotionDetectionStatus,
				timeout);

		return std::make_pair(res.first, bool { res.second.z_negative });
	}
	inline std::pair<Status, bool> GetPositiveZ_MotionDetected(
			duration_type timeout) {
		std::pair<Status, MotionDetectionStatus> res = ReadByteAs_<
				MotionDetectionStatus>(register_map::MotionDetectionStatus,
				timeout);

		return std::make_pair(res.first, bool { res.second.z_positive });
	}

	inline std::pair<Status, bool> GetZeroMotionDetected(
			duration_type timeout) {
		std::pair<Status, MotionDetectionStatus> res = ReadByteAs_<
				MotionDetectionStatus>(register_map::MotionDetectionStatus,
				timeout);
		return std::make_pair(res.first, bool { res.second.zero_motion });
	}

	/**
	 *
	 *
	 **/

	inline Status WriteI2cSlave0_DataOut(uint8_t data, duration_type timeout) {
		return WriteByte_(register_map::I2cSlave0_DataOut, data, timeout);
	}
	inline std::pair<Status, uint8_t> ReadI2cSlave0_DataOut(
			duration_type timeout) {
		return ReadByte_(register_map::I2cSlave0_DataOut, timeout);
	}

	inline Status WriteI2cSlave1_DataOut(uint8_t data, duration_type timeout) {
		return WriteByte_(register_map::I2cSlave1_DataOut, data, timeout);
	}
	inline std::pair<Status, uint8_t> ReadI2cSlave1_DataOut(
			duration_type timeout) {
		return ReadByte_(register_map::I2cSlave1_DataOut, timeout);
	}

	inline Status WriteI2cSlave2_DataOut(uint8_t data, duration_type timeout) {
		return WriteByte_(register_map::I2cSlave2_DataOut, data, timeout);
	}
	inline std::pair<Status, uint8_t> ReadI2cSlave2_DataOut(
			duration_type timeout) {
		return ReadByte_(register_map::I2cSlave2_DataOut, timeout);
	}

	inline Status WriteI2cSlave3_DataOut(uint8_t data, duration_type timeout) {
		return WriteByte_(register_map::I2cSlave3_DataOut, data, timeout);
	}
	inline std::pair<Status, uint8_t> ReadI2cSlave3_DataOut(
			duration_type timeout) {
		return ReadByte_(register_map::I2cSlave3_DataOut, timeout);
	}

	/**
	 *
	 *
	 **/

	inline Status WriteI2cMasterDelayCtrlConfig(
			I2cMasterDelayCtrlConfig ctrl_cfg, duration_type timeout) {
		return WriteByteAs_<I2cMasterDelayCtrlConfig>(
				register_map::I2cMasterDelayControl, ctrl_cfg, timeout);
	}
	inline std::pair<Status, I2cMasterDelayCtrlConfig> ReadI2cMasterDelayCtrlConfig(
			duration_type timeout) {
		return ReadByteAs_<I2cMasterDelayCtrlConfig>(
				register_map::I2cMasterDelayControl, timeout);
	}

	/**
	 *
	 *
	 **/

	inline Status WriteSignalPathResetConfig(SignalPathResetConfig spr_cfg,
			duration_type timeout) {
		return WriteByteAs_<SignalPathResetConfig>(
				register_map::SignalPathReset, spr_cfg, timeout);
	}
	inline std::pair<Status, SignalPathResetConfig> ReadSignalPathResetConfig(
			duration_type timeout) {
		return ReadByteAs_<SignalPathResetConfig>(register_map::SignalPathReset,
				timeout);
	}

	inline Status SetGyroscopeSignalPathReset(bool reset,
			duration_type timeout) {
		std::pair<Status, SignalPathResetConfig> cfg =
				ReadSignalPathResetConfig(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;

		cfg.second.gyroscope_reset = reset;
		return WriteSignalPathResetConfig(cfg.second, timeout);
	}
	inline Status SetAccelerometerSignalPathReset(bool reset,
			duration_type timeout) {
		std::pair<Status, SignalPathResetConfig> cfg =
				ReadSignalPathResetConfig(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;

		cfg.second.accelerometer_reset = reset;
		return WriteSignalPathResetConfig(cfg.second, timeout);
	}
	inline Status SetTemperatureSignalPathReset(bool reset,
			duration_type timeout) {
		std::pair<Status, SignalPathResetConfig> cfg =
				ReadSignalPathResetConfig(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;

		cfg.second.temperature_reset = reset;
		return WriteSignalPathResetConfig(cfg.second, timeout);
	}

	inline std::pair<Status, bool> GetGyroscopeSignalPathReset(
			duration_type timeout) {
		std::pair<Status, SignalPathResetConfig> cfg =
				ReadSignalPathResetConfig(timeout);

		return std::make_pair(cfg.first, bool { cfg.second.gyroscope_reset });
	}
	inline std::pair<Status, bool> GetAccelerometerSignalPathReset(
			duration_type timeout) {
		std::pair<Status, SignalPathResetConfig> cfg =
				ReadSignalPathResetConfig(timeout);

		return std::make_pair(cfg.first,
				bool { cfg.second.accelerometer_reset });
	}
	inline std::pair<Status, bool> GetTemperatureSignalPathReset(
			duration_type timeout) {
		std::pair<Status, SignalPathResetConfig> cfg =
				ReadSignalPathResetConfig(timeout);

		return std::make_pair(cfg.first, bool { cfg.second.temperature_reset });
	}

	/**
	 *
	 *
	 **/

	inline Status WriteMotionDetectionCtrlConfig(MotionDetectionCtrlConfig cfg,
			duration_type timeout) {
		return WriteByteAs_<MotionDetectionCtrlConfig>(
				register_map::MotionDetectionControl, cfg, timeout);
	}
	inline std::pair<Status, MotionDetectionCtrlConfig> GetMotionDetectionCtrlConfig(
			MotionDetectionCtrlConfig, duration_type timeout) {
		return ReadByteAs_<MotionDetectionCtrlConfig>(
				register_map::MotionDetectionControl, timeout);
	}

	/**
	 *
	 *
	 **/

	inline Status WriteUserCtrlConfig(UserCtrlConfig cfg,
			duration_type timeout) {
		return WriteByteAs_<UserCtrlConfig>(register_map::UserControl, cfg,
				timeout);
	}
	inline std::pair<Status, UserCtrlConfig> ReadUserCtrlConfig(
			duration_type timeout) {
		return ReadByteAs_<UserCtrlConfig>(register_map::UserControl, timeout);
	}

	inline Status SetFifoEnable(bool enable, duration_type timeout) {
		std::pair<Status, UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;
		cfg.second.fifo_enable = enable;

		return WriteUserCtrlConfig(cfg.second, timeout);
	}
	inline Status SetI2cMasterEnable(bool enable, duration_type timeout) {
		std::pair<Status, UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;
		cfg.second.i2c_master_mode_enable = enable;

		return WriteUserCtrlConfig(cfg.second, timeout);
	}
	inline Status SetUseSpiInterface(bool enable, duration_type timeout) {
		std::pair<Status, UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;
		cfg.second.enable_spi_interface = enable;

		return WriteUserCtrlConfig(cfg.second, timeout);
	}
	inline Status SetFifoReset(bool enable, duration_type timeout) {
		std::pair<Status, UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;
		cfg.second.reset_fifo_buffer = enable;

		return WriteUserCtrlConfig(cfg.second, timeout);
	}
	inline Status SetI2cMasterReset(bool enable, duration_type timeout) {
		std::pair<Status, UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;
		cfg.second.reset_i2c_master = enable;

		return WriteUserCtrlConfig(cfg.second, timeout);
	}
	inline Status SetSignalPathReset(bool enable, duration_type timeout) {
		std::pair<Status, UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;
		cfg.second.clear_sensor_register_and_path = enable;

		return WriteUserCtrlConfig(cfg.second, timeout);
	}

	inline std::pair<Status, bool> GetFifoEnable(duration_type timeout) {
		std::pair<Status, UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
		return std::make_pair(cfg.first, bool { cfg.second.fifo_enable });
	}
	inline std::pair<Status, bool> GetI2cMasterEnable(duration_type timeout) {
		std::pair<Status, UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
		return std::make_pair(cfg.first,
				bool { cfg.second.i2c_master_mode_enable });
	}
	inline std::pair<Status, bool> GetUseSpiInterface(duration_type timeout) {
		std::pair<Status, UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
		return std::make_pair(cfg.first,
				bool { cfg.second.enable_spi_interface });
	}
	inline std::pair<Status, bool> GetFifoReset(duration_type timeout) {
		std::pair<Status, UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
		return std::make_pair(cfg.first, bool { cfg.second.reset_fifo_buffer });
	}
	inline std::pair<Status, bool> GetI2cMasterReset(duration_type timeout) {
		std::pair<Status, UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
		return std::make_pair(cfg.first, bool { cfg.second.reset_i2c_master });
	}
	inline std::pair<Status, bool> GetSignalPathReset(duration_type timeout) {
		std::pair<Status, UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
		return std::make_pair(cfg.first,
				bool { cfg.second.clear_sensor_register_and_path });
	}

	/**
	 *
	 *
	 **/

	inline Status WritePowerManagement1_Config(PowerManagement1_Config cfg,
			duration_type timeout) {
		return WriteByteAs_<PowerManagement1_Config>(
				register_map::PowerManagement1, cfg, timeout);
	}
	inline std::pair<Status, PowerManagement1_Config> ReadPowerManagement1_Config(
			duration_type timeout) {
		return ReadByteAs_<PowerManagement1_Config>(
				register_map::PowerManagement1, timeout);
	}

	inline Status SetDeviceRegisterReset(bool reset, duration_type timeout) {
		std::pair<Status, PowerManagement1_Config> cfg =
				ReadPowerManagement1_Config(timeout);

		if (cfg.first != Status::OK)
			return cfg.first;

		cfg.second.reset_registers = reset;

		return WritePowerManagement1_Config(cfg.second, timeout);
	}
	inline Status SetSleep(bool sleep, duration_type timeout) {
		std::pair<Status, PowerManagement1_Config> cfg =
				ReadPowerManagement1_Config(timeout);

		if (cfg.first != Status::OK)
			return cfg.first;

		cfg.second.sleep_mode = sleep;

		return WritePowerManagement1_Config(cfg.second, timeout);
	}
	inline Status SetSleepCycle(bool cycle, duration_type timeout) {
		std::pair<Status, PowerManagement1_Config> cfg =
				ReadPowerManagement1_Config(timeout);

		if (cfg.first != Status::OK)
			return cfg.first;

		cfg.second.cycle_sleep = cycle;

		return WritePowerManagement1_Config(cfg.second, timeout);
	}
	inline Status SetTemperatureDisable(bool disable, duration_type timeout) {
		std::pair<Status, PowerManagement1_Config> cfg =
				ReadPowerManagement1_Config(timeout);

		if (cfg.first != Status::OK)
			return cfg.first;

		cfg.second.disable_temperature = disable;

		return WritePowerManagement1_Config(cfg.second, timeout);
	}
	inline Status SetClockSource(ClockSource clk, duration_type timeout) {
		std::pair<Status, PowerManagement1_Config> cfg =
				ReadPowerManagement1_Config(timeout);

		if (cfg.first != Status::OK)
			return cfg.first;

		cfg.second.clock_source =
				static_cast<ClockSource>(static_cast<uint8_t>(clk) & 0b111U);

		return WritePowerManagement1_Config(cfg.second, timeout);
	}

	inline std::pair<Status, bool> GetDeviceRegisterReset(
			duration_type timeout) {
		std::pair<Status, PowerManagement1_Config> cfg =
				ReadPowerManagement1_Config(timeout);
		return std::make_pair(cfg.first, bool { cfg.second.reset_registers });
	}
	inline std::pair<Status, bool> GetSleep(duration_type timeout) {
		std::pair<Status, PowerManagement1_Config> cfg =
				ReadPowerManagement1_Config(timeout);
		return std::make_pair(cfg.first, bool { cfg.second.sleep_mode });
	}
	inline std::pair<Status, bool> GetSleepCycle(duration_type timeout) {
		std::pair<Status, PowerManagement1_Config> cfg =
				ReadPowerManagement1_Config(timeout);
		return std::make_pair(cfg.first, bool { cfg.second.cycle_sleep });
	}
	inline std::pair<Status, bool> GetTemperatureDisable(
			duration_type timeout) {
		std::pair<Status, PowerManagement1_Config> cfg =
				ReadPowerManagement1_Config(timeout);
		return std::make_pair(cfg.first,
				bool { cfg.second.disable_temperature });
	}
	inline std::pair<Status, ClockSource> GetClockSource(
			duration_type timeout) {
		std::pair<Status, PowerManagement1_Config> cfg =
				ReadPowerManagement1_Config(timeout);
		return std::make_pair(cfg.first,
				ClockSource { cfg.second.clock_source });
	}

	/**
	 *
	 *
	 **/

	inline Status WritePowerManagement2_Config(PowerManagement2_Config cfg,
			duration_type timeout) {
		return WriteByteAs_<PowerManagement2_Config>(
				register_map::PowerManagement2, cfg, timeout);
	}
	inline std::pair<Status, PowerManagement2_Config> ReadPowerManagement2_Config(
			duration_type timeout) {
		return ReadByteAs_<PowerManagement2_Config>(
				register_map::PowerManagement2, timeout);
	}

	inline Status SetWakeupFrequency(WakeupFrequency wfreq,
			duration_type timeout) {
		std::pair<Status, PowerManagement2_Config> cfg =
				ReadPowerManagement2_Config(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;
		cfg.second.wakeup_frequency =
				static_cast<WakeupFrequency>(static_cast<uint8_t>(wfreq) & 0b11U);
		return WriteByteAs_<PowerManagement2_Config>(
				register_map::PowerManagement2, cfg.second, timeout);
	}
	inline Status SetAccelerometerX_Standby(bool standby,
			duration_type timeout) {
		std::pair<Status, PowerManagement2_Config> cfg =
				ReadPowerManagement2_Config(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;
		cfg.second.accelerometer_x_standby = standby;
		return WriteByteAs_<PowerManagement2_Config>(
				register_map::PowerManagement2, cfg.second, timeout);
	}
	inline Status SetAccelerometerY_Standby(bool standby,
			duration_type timeout) {
		std::pair<Status, PowerManagement2_Config> cfg =
				ReadPowerManagement2_Config(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;
		cfg.second.accelerometer_y_standby = standby;
		return WriteByteAs_<PowerManagement2_Config>(
				register_map::PowerManagement2, cfg.second, timeout);
	}
	inline Status SetAccelerometerZ_Standby(bool standby,
			duration_type timeout) {
		std::pair<Status, PowerManagement2_Config> cfg =
				ReadPowerManagement2_Config(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;
		cfg.second.accelerometer_z_standby = standby;
		return WriteByteAs_<PowerManagement2_Config>(
				register_map::PowerManagement2, cfg.second, timeout);
	}
	inline Status SetGyroscopeX_Standby(bool standby, duration_type timeout) {
		std::pair<Status, PowerManagement2_Config> cfg =
				ReadPowerManagement2_Config(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;
		cfg.second.gyroscope_x_standby = standby;
		return WriteByteAs_<PowerManagement2_Config>(
				register_map::PowerManagement2, cfg.second, timeout);
	}
	inline Status SetGyroscopeY_Standby(bool standby, duration_type timeout) {
		std::pair<Status, PowerManagement2_Config> cfg =
				ReadPowerManagement2_Config(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;
		cfg.second.gyroscope_y_standby = standby;
		return WriteByteAs_<PowerManagement2_Config>(
				register_map::PowerManagement2, cfg.second, timeout);
	}
	inline Status SetGyroscopeZ_Standby(bool standby, duration_type timeout) {
		std::pair<Status, PowerManagement2_Config> cfg =
				ReadPowerManagement2_Config(timeout);
		if (cfg.first != Status::OK)
			return cfg.first;
		cfg.second.gyroscope_z_standby = standby;
		return WriteByteAs_<PowerManagement2_Config>(
				register_map::PowerManagement2, cfg.second, timeout);
	}

	inline std::pair<Status, WakeupFrequency> GetWakeupFrequency(
			duration_type timeout) {
		std::pair<Status, PowerManagement2_Config> cfg =
				ReadPowerManagement2_Config(timeout);

		return std::make_pair(cfg.first,
				WakeupFrequency { cfg.second.wakeup_frequency });
	}
	inline std::pair<Status, bool> GetAccelerometerX_Standby(
			duration_type timeout) {
		std::pair<Status, PowerManagement2_Config> cfg =
				ReadPowerManagement2_Config(timeout);

		return std::make_pair(cfg.first,
				bool { cfg.second.accelerometer_x_standby });
	}
	inline std::pair<Status, bool> GetAccelerometerY_Standby(
			duration_type timeout) {
		std::pair<Status, PowerManagement2_Config> cfg =
				ReadPowerManagement2_Config(timeout);

		return std::make_pair(cfg.first,
				bool { cfg.second.accelerometer_y_standby });
	}
	inline std::pair<Status, bool> GetAccelerometerZ_Standby(
			duration_type timeout) {
		std::pair<Status, PowerManagement2_Config> cfg =
				ReadPowerManagement2_Config(timeout);

		return std::make_pair(cfg.first,
				bool { cfg.second.accelerometer_z_standby });
	}
	inline std::pair<Status, bool> GetGyroscopeX_Standby(
			duration_type timeout) {
		std::pair<Status, PowerManagement2_Config> cfg =
				ReadPowerManagement2_Config(timeout);

		return std::make_pair(cfg.first,
				bool { cfg.second.gyroscope_x_standby });
	}
	inline std::pair<Status, bool> GetGyroscopeY_Standby(
			duration_type timeout) {
		std::pair<Status, PowerManagement2_Config> cfg =
				ReadPowerManagement2_Config(timeout);

		return std::make_pair(cfg.first,
				bool { cfg.second.gyroscope_y_standby });
	}
	inline std::pair<Status, bool> GetGyroscopeZ_Standby(
			duration_type timeout) {
		std::pair<Status, PowerManagement2_Config> cfg =
				ReadPowerManagement2_Config(timeout);

		return std::make_pair(cfg.first,
				bool { cfg.second.gyroscope_z_standby });
	}

	/**
	 *
	 *
	 **/

	inline std::pair<Status, uint16_t> ReadFifoCount(duration_type timeout) {
		uint16_t fifo_cnt { };

		Status status = BurstRead_(register_map::FifoCountH,
				reinterpret_cast<uint8_t*>(&fifo_cnt), sizeof(fifo_cnt),
				timeout);
		return std::make_pair(status, fifo_cnt);
	}

	/**
	 *
	 *
	 **/

	inline Status WriteFifoData(uint8_t data, duration_type timeout) {
		return WriteByte_(register_map::FifoReadWrite, data, timeout);
	}
	inline std::pair<Status, uint8_t> ReadFifoData(duration_type timeout) {
		return ReadByte_(register_map::FifoReadWrite, timeout);
	}

	/**
	 *
	 *
	 **/

	inline std::pair<Status, uint8_t> ReadWhoAmI(duration_type timeout) {
		return ReadByte_(register_map::FifoReadWrite, timeout);
	}

	/**
	 *
	 *
	 **/
};
}  // namespace mpu60X0
#endif                        // DRIVERS_MPU60X0_MPU60X0_H_