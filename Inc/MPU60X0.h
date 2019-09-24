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
// loads dependencies and device definitions for the peripherals
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
#endif
#endif
}

#include "bit_definitions.h"  // NOLINT
#include "data_types.h"       // NOLINT
#include "registers.h"        // NOLINT

namespace mpu60X0 {

// TODO(lamarrr): encapsulate this to a source file to prevent user access
#define RETURN_F_IF_ERROR(v) \
  if (v.first != Status::OK) return v.first
#define RETURN_IF_ERROR(v) \
  if (v.first != Status::OK) return v

enum struct Status : uint8_t {
  OK = HAL_StatusTypeDef::HAL_OK,
  Error = HAL_StatusTypeDef::HAL_ERROR,
  Busy = HAL_StatusTypeDef::HAL_BUSY,
  Timeout = HAL_StatusTypeDef::HAL_TIMEOUT,
};

template <typename ValueType>
using Result = std::pair<Status, ValueType>;

// Blocking master mode API that maximizes utilization of Burst byte Reads
struct MPU60X0 {
  using duration_type = std::chrono::duration<uint32_t, std::milli>;

 private:
  uint8_t device_address_;
  I2C_HandleTypeDef* handle_;

 public:
  MPU60X0(uint8_t address, I2C_HandleTypeDef* handle) : handle_{handle} {
    device_address_ = address << 1U;
  }

  MPU60X0(const MPU60X0&) noexcept {}
  MPU60X0(MPU60X0&&) noexcept {}
  MPU60X0& operator=(const MPU60X0&) noexcept = default;
  MPU60X0& operator=(MPU60X0&&) noexcept = default;
  ~MPU60X0() noexcept {
    device_address_ = 0U;
    handle_ = nullptr;
  }

  Status WriteByte_(register_type reg, uint8_t data,
                    duration_type timeout) noexcept {
    uint8_t buffer[2] = {reg, data};
    return static_cast<Status>(HAL_I2C_Master_Transmit(
        handle_, device_address_, buffer, sizeof(buffer), timeout.count()));
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
  template <typename ByteRep>
  Status WriteByteAs_(register_type reg, ByteRep b_rep,
                      duration_type timeout) noexcept {
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
  Status BurstWrite_(register_type reg, const uint8_t* data, uint16_t size,
                     duration_type timeout) noexcept {
    return static_cast<Status>(HAL_I2C_Master_Transmit(
        handle_, device_address_, const_cast<uint8_t*>(data), size,
        timeout.count()));
  }

  Result<uint8_t> ReadByte_(register_type reg, duration_type timeout) noexcept {
    uint8_t data{};
    Status status = static_cast<Status>(HAL_I2C_Master_Transmit(
        handle_, device_address_, &reg, 1, timeout.count()));

    if (status != Status::OK) return std::make_pair(status, data);

    status = static_cast<Status>(HAL_I2C_Master_Receive(
        handle_, device_address_, &data, 1, timeout.count()));
    return std::make_pair(status, data);
  }

  template <typename ByteRep>
  Result<ByteRep> ReadByteAs_(register_type reg,
                              duration_type timeout) noexcept {
    static_assert(sizeof(ByteRep) == 1U, "Type must be byte sized");
    Result<uint8_t> data = ReadByte_(reg, timeout);
    ByteRep res{};
    *(reinterpret_cast<uint8_t*>(&res)) = data.second;

    return std::make_pair(data.first, res);
  }

  Status BurstRead_(register_type reg, uint8_t* data, uint16_t size,
                    duration_type timeout) noexcept {
    Status status = static_cast<Status>(HAL_I2C_Master_Transmit(
        handle_, device_address_, &reg, 1, timeout.count()));
    if (status != Status::OK) return status;
    status = static_cast<Status>(HAL_I2C_Master_Receive(
        handle_, device_address_, data, size, timeout.count()));
    return status;
  }

  bool Ready(uint32_t trials, duration_type timeout) noexcept {
    return HAL_I2C_IsDeviceReady(handle_, device_address_, trials,
                                 timeout.count()) == HAL_OK;
  }

  // that the device be configured to use one of the gyroscopes (or an external
  // clock source) as the clock reference for improved stability
  Status Initialize() {
    PowerManagement1_Config cfg{};
    cfg.clock_source = ClockSource::PllGyro_X;
    cfg.reset_registers = true;
    duration_type timeout = duration_type{250};
    return WritePowerManagement1_Config(cfg, timeout);
  }

  /**
   *
   *
   **/

  Status WriteSelfTestX(uint8_t data, duration_type timeout) noexcept {
    return WriteByte_(register_map::SelfTestX, data, timeout);
  }

  Result<uint8_t> ReadSelfTestX(duration_type timeout) noexcept {
    return ReadByte_(register_map::SelfTestX, timeout);
  }

  Status WriteSelfTestY(uint8_t data, duration_type timeout) noexcept {
    return WriteByte_(register_map::SelfTestY, data, timeout);
  }

  Result<uint8_t> ReadSelfTestY(duration_type timeout) noexcept {
    return ReadByte_(register_map::SelfTestY, timeout);
  }

  Status WriteSelfTestZ(uint8_t data, duration_type timeout) noexcept {
    return WriteByte_(register_map::SelfTestZ, data, timeout);
  }

  Result<uint8_t> ReadSelfTestZ(duration_type timeout) noexcept {
    return ReadByte_(register_map::SelfTestZ, timeout);
  }

  Status WriteSelfTestA(uint8_t data, duration_type timeout) noexcept {
    return WriteByte_(register_map::SelfTestA, data, timeout);
  }

  Result<uint8_t> ReadSelfTestA(duration_type timeout) noexcept {
    return ReadByte_(register_map::SelfTestA, timeout);
  }

  Result<uint8_t> GetGyroscopeX_SelfTestValue(duration_type timeout) noexcept {
    Result<uint8_t> res = ReadSelfTestX(timeout);
    res.second &= 00011111U;
    return res;
  }

  Result<uint8_t> GetGyroscopeY_SelfTestValue(duration_type timeout) noexcept {
    Result<uint8_t> res = ReadSelfTestY(timeout);
    res.second &= 00011111U;
    return res;
  }

  Result<uint8_t> GetGyroscopeZ_SelfTestValue(duration_type timeout) noexcept {
    Result<uint8_t> res = ReadSelfTestZ(timeout);
    res.second &= 00011111U;
    return res;
  }

  // 5 bit precision
  Result<uint8_t> GetAccelerometerX_SelfTestValue(
      duration_type timeout) noexcept {
    Result<uint8_t> res_head = ReadSelfTestX(timeout);

    RETURN_IF_ERROR(res_head);

    Result<uint8_t> res_tail = ReadSelfTestA(timeout);
    res_tail.second = ((res_head.second & 0b11100000U) >> 3U) |
                      ((res_tail.second & 0b00110000U) >> 4U);
    return res_tail;
  }

  Result<uint8_t> GetAccelerometerY_SelfTestValue(
      duration_type timeout) noexcept {
    Result<uint8_t> res_head = ReadSelfTestY(timeout);

    RETURN_IF_ERROR(res_head);

    Result<uint8_t> res_tail = ReadSelfTestA(timeout);
    res_tail.second = ((res_head.second & 0b11100000U) >> 3U) |
                      ((res_tail.second & 0b00001100U) >> 2U);
    return res_tail;
  }

  Result<uint8_t> GetAccelerometerZ_SelfTestValue(
      duration_type timeout) noexcept {
    Result<uint8_t> res_head = ReadSelfTestZ(timeout);

    RETURN_IF_ERROR(res_head);

    Result<uint8_t> res_tail = ReadSelfTestA(timeout);
    res_tail.second = ((res_head.second & 0b11100000U) >> 3U) |
                      (res_tail.second & 0b00000011U);
    return res_tail;
  }

  Status SetGyroscopeX_SelfTestValue(uint8_t data,
                                     duration_type timeout) noexcept {
    Result<uint8_t> res = ReadSelfTestX(timeout);

    RETURN_F_IF_ERROR(res);

    res.second = (res.second & 0b11100000U) | (0b00011111U & data);

    return WriteSelfTestX(res.second, timeout);
  }

  Status SetGyroscopeY_SelfTestValue(uint8_t data,
                                     duration_type timeout) noexcept {
    Result<uint8_t> res = ReadSelfTestY(timeout);

    RETURN_F_IF_ERROR(res);

    res.second = (res.second & 0b11100000U) | (0b00011111U & data);

    return WriteSelfTestY(res.second, timeout);
  }

  Status SetGyroscopeZ_SelfTestValue(uint8_t data,
                                     duration_type timeout) noexcept {
    Result<uint8_t> res = ReadSelfTestZ(timeout);

    RETURN_F_IF_ERROR(res);

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
  Status SetAccelerometerX_SelfTestValue(uint8_t data,
                                         duration_type timeout) noexcept {
    Result<uint8_t> res_head = ReadSelfTestX(timeout);

    RETURN_F_IF_ERROR(res_head);

    res_head.second =
        ((data << 3U) & 0b11100000U) | (res_head.second & 0b00011111U);
    res_head.first = WriteSelfTestX(res_head.second, timeout);

    RETURN_F_IF_ERROR(res_head);

    res_head = ReadSelfTestA(timeout);

    RETURN_F_IF_ERROR(res_head);

    res_head.second =
        ((data << 4) & 0b00110000U) | (res_head.second & 0b11001111U);

    return WriteSelfTestA(res_head.second, timeout);
  }

  Status SetAccelerometerY_SelfTestValue(uint8_t data,
                                         duration_type timeout) noexcept {
    Result<uint8_t> res_head = ReadSelfTestY(timeout);

    RETURN_F_IF_ERROR(res_head);

    res_head.second =
        ((data << 3) & 0b11100000U) | (res_head.second & 0b00011111U);
    res_head.first = WriteSelfTestY(res_head.second, timeout);

    RETURN_F_IF_ERROR(res_head);

    res_head = ReadSelfTestA(timeout);

    RETURN_F_IF_ERROR(res_head);

    res_head.second =
        ((data << 2) & 0b00001100U) | (res_head.second & 0b11110011U);

    return WriteSelfTestA(res_head.second, timeout);
  }

  Status SetAccelerometerZ_SelfTestValue(uint8_t data,
                                         duration_type timeout) noexcept {
    Result<uint8_t> res_head = ReadSelfTestZ(timeout);

    RETURN_F_IF_ERROR(res_head);

    res_head.second =
        ((data << 3) & 0b11100000U) | (res_head.second & 0b00011111U);
    res_head.first = WriteSelfTestZ(res_head.second, timeout);

    RETURN_F_IF_ERROR(res_head);

    res_head = ReadSelfTestA(timeout);

    RETURN_F_IF_ERROR(res_head);

    res_head.second = (data & 0b00000011U) | (res_head.second & 0b11111100U);

    return WriteSelfTestA(res_head.second, timeout);
  }

  /**
   *
   *
   **/

  Status WriteSampleRateDivider(uint8_t data, duration_type timeout) noexcept {
    return WriteByte_(register_map::SampleRateDivider, data, timeout);
  }

  Result<uint8_t> ReadSampleRateDivider(duration_type timeout) noexcept {
    return ReadByte_(register_map::SampleRateDivider, timeout);
  }
  /**
   *
   *
   **/

  Status WriteConfig(Config config, duration_type timeout) noexcept {
    return WriteByteAs_<Config>(register_map::Config, config, timeout);
  }

  Result<Config> ReadConfig(duration_type timeout) noexcept {
    return ReadByteAs_<Config>(register_map::Config, timeout);
  }

  Status SetExternalFrameSync(FrameSync fsync, duration_type timeout) noexcept {
    //
    Result<Config> conf = ReadConfig(timeout);

    RETURN_F_IF_ERROR(conf);

    conf.second.ext_frame_sync =
        static_cast<FrameSync>(static_cast<uint8_t>(fsync) & 0b111U);

    return WriteConfig(conf.second, timeout);
  }

  Status SetDlpfConfig(DlpfConfig dlpf_cfg, duration_type timeout) noexcept {
    Result<Config> conf = ReadConfig(timeout);

    RETURN_F_IF_ERROR(conf);

    conf.second.dlpf_config =
        static_cast<DlpfConfig>(static_cast<uint8_t>(dlpf_cfg) & 0b111U);

    return WriteConfig(conf.second, timeout);
  }

  Result<FrameSync> GetExternalFrameSync(duration_type timeout) noexcept {
    Result<Config> conf = ReadConfig(timeout);

    return std::make_pair(conf.first, FrameSync{conf.second.ext_frame_sync});
  }

  Result<DlpfConfig> GetDlpfConfig(duration_type timeout) noexcept {
    Result<Config> conf = ReadConfig(timeout);

    return std::make_pair(conf.first, DlpfConfig{conf.second.dlpf_config});
  }

  /**
   *
   *
   **/

  Status WriteGyroscopeConfig(GyroscopeConfig gyro_cfg,
                              duration_type timeout) noexcept {
    return WriteByteAs_(register_map::GyroscopeConfig, gyro_cfg, timeout);
  }

  Result<GyroscopeConfig> ReadGyroscopeConfig(duration_type timeout) noexcept {
    return ReadByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig, timeout);
  }

  Status SetGyroscopeX_SelfTestState(bool activate,
                                     duration_type timeout) noexcept {
    Result<GyroscopeConfig> res =
        ReadByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig, timeout);

    RETURN_F_IF_ERROR(res);

    res.second.x_self_test = activate;

    return WriteByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig,
                                         res.second, timeout);
  }

  Status SetGyroscopeY_SelfTestState(bool activate,
                                     duration_type timeout) noexcept {
    Result<GyroscopeConfig> res =
        ReadByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig, timeout);

    RETURN_F_IF_ERROR(res);

    res.second.y_self_test = activate;

    return WriteByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig,
                                         res.second, timeout);
  }

  Status SetGyroscopeZ_SelfTestState(bool activate,
                                     duration_type timeout) noexcept {
    Result<GyroscopeConfig> res =
        ReadByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig, timeout);

    RETURN_F_IF_ERROR(res);

    res.second.z_self_test = activate;

    return WriteByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig,
                                         res.second, timeout);
  }

  // 3 bit, x, y, z
  Status SetGyroscopeSelfTestStates(uint8_t tri_state,
                                    duration_type timeout) noexcept {
    Result<GyroscopeConfig> res =
        ReadByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig, timeout);

    RETURN_F_IF_ERROR(res);

    res.second.x_self_test = static_cast<bool>(0b0000100 & tri_state);
    res.second.y_self_test = static_cast<bool>(0b0000010 & tri_state);
    res.second.z_self_test = static_cast<bool>(0b0000001 & tri_state);

    return WriteByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig,
                                         res.second, timeout);
  }

  Status SetGyroscopeFullScaleRange(GyroscopeFullScale full_scale,
                                    duration_type timeout) noexcept {
    Result<GyroscopeConfig> res =
        ReadByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig, timeout);

    RETURN_F_IF_ERROR(res);

    res.second.full_scale = full_scale;

    return WriteByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig,
                                         res.second, timeout);
  }

  Result<bool> GetGyroscopeX_SelfTestState(duration_type timeout) noexcept {
    Result<GyroscopeConfig> res =
        ReadByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig, timeout);

    return std::make_pair(res.first, bool{res.second.x_self_test});
  }

  Result<bool> GetGyroscopeY_SelfTestState(duration_type timeout) noexcept {
    Result<GyroscopeConfig> res =
        ReadByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig, timeout);

    return std::make_pair(res.first, bool{res.second.y_self_test});
  }

  Result<bool> GetGyroscopeZ_SelfTestState(duration_type timeout) noexcept {
    Result<GyroscopeConfig> res =
        ReadByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig, timeout);

    return std::make_pair(res.first, bool{res.second.z_self_test});
  }

  // first bit, second bit, third bit
  Result<uint8_t> GetGyroscopeSelfTestStates(duration_type timeout) noexcept {
    uint8_t data{};

    Result<GyroscopeConfig> res = ReadGyroscopeConfig(timeout);

    if (res.first != Status::OK) return std::make_pair(res.first, data);

    data = (static_cast<uint8_t>(res.second.x_self_test) << 2U) |
           (static_cast<uint8_t>(res.second.y_self_test) << 1U) |
           static_cast<uint8_t>(res.second.z_self_test);

    return std::make_pair(res.first, data);
  }

  Result<GyroscopeFullScale> GetGyroscopeFullScaleRange(
      duration_type timeout) noexcept {
    Result<GyroscopeConfig> res =
        ReadByteAs_<GyroscopeConfig>(register_map::GyroscopeConfig, timeout);

    return std::make_pair(res.first, GyroscopeFullScale{res.second.full_scale});
  }

  /**
   *
   *
   **/

  Status WriteAccelerometerConfig(AccelerometerConfig acc_cfg,
                                  duration_type timeout) noexcept {
    return WriteByteAs_<AccelerometerConfig>(register_map::AccelerometerConfig,
                                             acc_cfg, timeout);
  }

  Result<AccelerometerConfig> ReadAccelerometerConfig(
      duration_type timeout) noexcept {
    return ReadByteAs_<AccelerometerConfig>(register_map::AccelerometerConfig,
                                            timeout);
  }

  Status SetAccelerometerX_SelfTestState(bool activate,
                                         duration_type timeout) noexcept {
    Result<AccelerometerConfig> acc_cfg = ReadAccelerometerConfig(timeout);

    RETURN_F_IF_ERROR(acc_cfg);

    acc_cfg.second.x_self_test = activate;

    return WriteByteAs_<AccelerometerConfig>(register_map::AccelerometerConfig,
                                             acc_cfg.second, timeout);
  }

  Status SetAccelerometerY_SelfTestState(bool activate,
                                         duration_type timeout) noexcept {
    Result<AccelerometerConfig> acc_cfg = ReadAccelerometerConfig(timeout);

    RETURN_F_IF_ERROR(acc_cfg);

    acc_cfg.second.y_self_test = activate;

    return WriteByteAs_<AccelerometerConfig>(register_map::AccelerometerConfig,
                                             acc_cfg.second, timeout);
  }

  Status SetAccelerometerZ_SelfTestState(bool activate,
                                         duration_type timeout) noexcept {
    Result<AccelerometerConfig> acc_cfg = ReadAccelerometerConfig(timeout);

    RETURN_F_IF_ERROR(acc_cfg);

    acc_cfg.second.z_self_test = activate;

    return WriteByteAs_<AccelerometerConfig>(register_map::AccelerometerConfig,
                                             acc_cfg.second, timeout);
  }

  Status SetAccelerometerSelfTestStates(uint8_t tri_state,
                                        duration_type timeout) noexcept {
    Result<AccelerometerConfig> res = ReadByteAs_<AccelerometerConfig>(
        register_map::AccelerometerConfig, timeout);

    RETURN_F_IF_ERROR(res);

    res.second.x_self_test = static_cast<bool>(0b0000100 & tri_state);
    res.second.y_self_test = static_cast<bool>(0b0000010 & tri_state);
    res.second.z_self_test = static_cast<bool>(0b0000001 & tri_state);

    return WriteByteAs_<AccelerometerConfig>(register_map::AccelerometerConfig,
                                             res.second, timeout);
  }

  Status SetAccelerometerFullScaleRange(AccelerometerFullScale full_scale,
                                        duration_type timeout) noexcept {
    Result<AccelerometerConfig> res = ReadByteAs_<AccelerometerConfig>(
        register_map::AccelerometerConfig, timeout);

    RETURN_F_IF_ERROR(res);

    res.second.full_scale = full_scale;

    return WriteByteAs_<AccelerometerConfig>(register_map::AccelerometerConfig,
                                             res.second, timeout);
  }

  Result<bool> GetAccelerometerX_SelfTestState(duration_type timeout) noexcept {
    Result<AccelerometerConfig> res = ReadByteAs_<AccelerometerConfig>(
        register_map::GyroscopeConfig, timeout);

    return std::make_pair(res.first, bool{res.second.x_self_test});
  }

  Result<bool> GetAccelerometerY_SelfTestState(duration_type timeout) noexcept {
    Result<AccelerometerConfig> res = ReadByteAs_<AccelerometerConfig>(
        register_map::GyroscopeConfig, timeout);

    return std::make_pair(res.first, bool{res.second.y_self_test});
  }

  Result<bool> GetAccelerometerZ_SelfTestState(duration_type timeout) noexcept {
    Result<AccelerometerConfig> res = ReadByteAs_<AccelerometerConfig>(
        register_map::GyroscopeConfig, timeout);

    return std::make_pair(res.first, bool{res.second.z_self_test});
  }

  Result<uint8_t> GetAccelerometerSelfTestStates(
      duration_type timeout) noexcept {
    uint8_t data{};

    Result<AccelerometerConfig> res = ReadAccelerometerConfig(timeout);

    if (res.first != Status::OK) return std::make_pair(res.first, data);

    data = (static_cast<uint8_t>(res.second.x_self_test) << 2U) |
           (static_cast<uint8_t>(res.second.y_self_test) << 1U) |
           static_cast<uint8_t>(res.second.z_self_test);

    return std::make_pair(res.first, data);
  }

  Result<AccelerometerFullScale> GetAccelerometerFullScaleRange(
      duration_type timeout) noexcept {
    Result<AccelerometerConfig> res = ReadAccelerometerConfig(timeout);
    return std::make_pair(res.first,
                          AccelerometerFullScale{res.second.full_scale});
  }

  /**
   *
   *
   **/

  Status WriteFreeFallAccelerationThreshold(uint8_t acc_thresh,
                                            duration_type timeout) noexcept {
    return WriteByte_(register_map::FreeFallAccelerationThreshold, acc_thresh,
                      timeout);
  }

  Result<uint8_t> ReadFreeFallAccelerationThreshold(
      duration_type timeout) noexcept {
    return ReadByte_(register_map::FreeFallAccelerationThreshold, timeout);
  }

  /**
   *
   *
   **/

  Status WriteFreeFallDuration(uint8_t ff_dur, duration_type timeout) noexcept {
    return WriteByte_(register_map::FreeFallDuration, ff_dur, timeout);
  }

  Result<uint8_t> ReadFreeFallDuration(duration_type timeout) noexcept {
    return ReadByte_(register_map::FreeFallDuration, timeout);
  }

  /**
   *
   *
   **/

  Status WriteMotionDetectionThreshold(uint8_t mot_det_thresh,
                                       duration_type timeout) noexcept {
    return WriteByte_(register_map::MotionDetectionThreshold, mot_det_thresh,
                      timeout);
  }

  Result<uint8_t> ReadMotionDetectionThreshold(duration_type timeout) noexcept {
    return ReadByte_(register_map::MotionDetectionThreshold, timeout);
  }

  /**
   *
   *
   **/

  Status WriteMotionDetectionDuration(uint8_t mot_det_dur,
                                      duration_type timeout) noexcept {
    return WriteByte_(register_map::MotionDetectionDuration, mot_det_dur,
                      timeout);
  }

  Result<uint8_t> ReadMotionDetectionDuration(duration_type timeout) noexcept {
    return ReadByte_(register_map::MotionDetectionDuration, timeout);
  }

  /**
   *
   *
   **/

  Status WriteZeroMotionDetectionThreshold(uint8_t mot_det_thresh,
                                           duration_type timeout) noexcept {
    return WriteByte_(register_map::MotionDetectionThreshold, mot_det_thresh,
                      timeout);
  }

  Result<uint8_t> ReadZeroMotionDetectionThreshold(
      duration_type timeout) noexcept {
    return ReadByte_(register_map::MotionDetectionThreshold, timeout);
  }

  /**
   *
   *
   **/

  Status WriteZeroMotionDetectionDuration(uint8_t mot_det_dur,
                                          duration_type timeout) noexcept {
    return WriteByte_(register_map::MotionDetectionDuration, mot_det_dur,
                      timeout);
  }

  Result<uint8_t> ReadZeroMotionDetectionDuration(
      duration_type timeout) noexcept {
    return ReadByte_(register_map::MotionDetectionDuration, timeout);
  }

  /**
   *
   *
   **/

  Status WriteFifoEnableConfig(FifoEnableConfig fifo_en_cfg,
                               duration_type timeout) noexcept {
    return WriteByteAs_<FifoEnableConfig>(register_map::FifoEnable, fifo_en_cfg,
                                          timeout);
  }

  Result<FifoEnableConfig> ReadFifoEnableConfig(
      duration_type timeout) noexcept {
    return ReadByteAs_<FifoEnableConfig>(register_map::FifoEnable, timeout);
  }

  Status SetTemperatureFifoEnable(bool enable, duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);

    RETURN_F_IF_ERROR(res);

    res.second.enable_temperature_fifo = enable;

    return WriteFifoEnableConfig(res.second, timeout);
  }

  Status SetGyroscopeX_FifoEnable(bool enable, duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);

    RETURN_F_IF_ERROR(res);

    res.second.enable_gyroscope_x_fifo = enable;

    return WriteFifoEnableConfig(res.second, timeout);
  }

  Status SetGyroscopeY_FifoEnable(bool enable, duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);

    RETURN_F_IF_ERROR(res);

    res.second.enable_gyroscope_y_fifo = enable;

    return WriteFifoEnableConfig(res.second, timeout);
  }

  Status SetGyroscopeZ_FifoEnable(bool enable, duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);

    RETURN_F_IF_ERROR(res);

    res.second.enable_gyroscope_z_fifo = enable;

    return WriteFifoEnableConfig(res.second, timeout);
  }

  Status SetAccelerometerFifoEnable(bool enable,
                                    duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);

    RETURN_F_IF_ERROR(res);

    res.second.enable_acceleration_fifo = enable;

    return WriteFifoEnableConfig(res.second, timeout);
  }

  Status SetSlave2_FifoEnable(bool enable, duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);

    RETURN_F_IF_ERROR(res);

    res.second.enable_slave2_fifo = enable;

    return WriteFifoEnableConfig(res.second, timeout);
  }

  Status SetSlave1_FifoEnable(bool enable, duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);

    RETURN_F_IF_ERROR(res);

    res.second.enable_slave1_fifo = enable;

    return WriteFifoEnableConfig(res.second, timeout);
  }

  Status SetSlave0_FifoEnable(bool enable, duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);

    RETURN_F_IF_ERROR(res);

    res.second.enable_slave0_fifo = enable;

    return WriteFifoEnableConfig(res.second, timeout);
  }

  Result<bool> GetTemperatureFifoEnable(duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
    return std::make_pair(res.first, bool{res.second.enable_temperature_fifo});
  }

  Result<bool> GetGyroscopeX_FifoEnable(duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
    return std::make_pair(res.first, bool{res.second.enable_gyroscope_x_fifo});
  }

  Result<bool> GetGyroscopeY_FifoEnable(duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
    return std::make_pair(res.first, bool{res.second.enable_gyroscope_y_fifo});
  }

  Result<bool> GetGyroscopeZ_FifoEnable(duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
    return std::make_pair(res.first, bool{res.second.enable_gyroscope_z_fifo});
  }

  Result<bool> GetAccelerometerFifoEnable(duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
    return std::make_pair(res.first, bool{res.second.enable_acceleration_fifo});
  }

  Result<bool> GetSlave2_FifoEnable(duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
    return std::make_pair(res.first, bool{res.second.enable_slave2_fifo});
  }

  Result<bool> GetSlave1_FifoEnable(duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
    return std::make_pair(res.first, bool{res.second.enable_slave1_fifo});
  }

  Result<bool> GetSlave0_FifoEnable(duration_type timeout) noexcept {
    Result<FifoEnableConfig> res = ReadFifoEnableConfig(timeout);
    return std::make_pair(res.first, bool{res.second.enable_slave0_fifo});
  }

  /**
   *
   *
   **/

  Status WriteI2cMasterCtrlConfig(I2cMasterCtrlConfig mast_cfg,
                                  duration_type timeout) noexcept {
    return WriteByteAs_<I2cMasterCtrlConfig>(register_map::I2cMasterCtrl,
                                             mast_cfg, timeout);
  }

  Result<I2cMasterCtrlConfig> ReadI2cMasterCtrlConfig(
      duration_type timeout) noexcept {
    return ReadByteAs_<I2cMasterCtrlConfig>(register_map::I2cMasterCtrl,
                                            timeout);
  }

  Status SetMultiMasterEnable(bool enable, duration_type timeout) noexcept {
    Result<I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(timeout);

    res.second.enable_multi_master = enable;

    return WriteByteAs_<I2cMasterCtrlConfig>(register_map::I2cMasterCtrl,
                                             res.second, timeout);
  }

  // DRI - Data ready Interrupt
  Status SetDRI_AwaitExtSensorData(bool enable,
                                   duration_type timeout) noexcept {
    Result<I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(timeout);

    res.second.dri_await_ext_sensor_data = enable;

    return WriteByteAs_<I2cMasterCtrlConfig>(register_map::I2cMasterCtrl,
                                             res.second, timeout);
  }

  Status SetSlave3_FifoEnable(bool enable, duration_type timeout) noexcept {
    Result<I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(timeout);

    res.second.enable_slave3_fifo = enable;

    return WriteByteAs_<I2cMasterCtrlConfig>(register_map::I2cMasterCtrl,
                                             res.second, timeout);
  }

  Status SetI2cMasterReadWriteTransition(I2cMasterReadWriteTransition rw_trans,
                                         duration_type timeout) noexcept {
    Result<I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(timeout);

    res.second.rw_transition = static_cast<I2cMasterReadWriteTransition>(
        static_cast<uint8_t>(rw_trans) & 0b1U);

    return WriteByteAs_<I2cMasterCtrlConfig>(register_map::I2cMasterCtrl,
                                             res.second, timeout);
  }

  Status SetI2cMasterClockSpeed(I2cMasterClockSpeed clock_speed,
                                duration_type timeout) noexcept {
    Result<I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(timeout);

    res.second.clock_speed = static_cast<I2cMasterClockSpeed>(
        static_cast<uint8_t>(clock_speed) & 0b1111U);

    return WriteByteAs_<I2cMasterCtrlConfig>(register_map::I2cMasterCtrl,
                                             res.second, timeout);
  }

  Result<bool> GetMultiMasterEnable(duration_type timeout) noexcept {
    Result<I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(timeout);

    return std::make_pair(res.first, bool{res.second.enable_multi_master});
  }

  Result<bool> GetDRI_AwaitExtSensorData(duration_type timeout) noexcept {
    Result<I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(timeout);

    return std::make_pair(res.first,
                          bool{res.second.dri_await_ext_sensor_data});
  }

  Result<bool> GetSlave3_FifoEnable(duration_type timeout) noexcept {
    Result<I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(timeout);

    return std::make_pair(res.first, bool{res.second.enable_slave3_fifo});
  }

  Result<I2cMasterReadWriteTransition> GetI2cMasterReadWriteTransition(
      duration_type timeout) noexcept {
    Result<I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(timeout);

    return std::make_pair(
        res.first, I2cMasterReadWriteTransition{res.second.rw_transition});
  }

  Result<I2cMasterClockSpeed> GetI2cMasterClockSpeed(
      duration_type timeout) noexcept {
    Result<I2cMasterCtrlConfig> res = ReadI2cMasterCtrlConfig(timeout);

    return std::make_pair(res.first,
                          I2cMasterClockSpeed{res.second.clock_speed});
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

  Status WriteInterruptPinConfig(InterruptPinConfig config,
                                 duration_type timeout) noexcept {
    return WriteByteAs_<InterruptPinConfig>(register_map::InterruptPinConfig,
                                            config, timeout);
  }

  Result<InterruptPinConfig> ReadInterruptPinConfig(
      duration_type timeout) noexcept {
    return ReadByteAs_<InterruptPinConfig>(register_map::InterruptPinConfig,
                                           timeout);
  }

  /**
   *
   *
   **/

  Status WriteInterruptEnableConfig(InterruptEnableConfig int_en_cfg,
                                    duration_type timeout) noexcept {
    return WriteByteAs_<InterruptEnableConfig>(register_map::InterruptEnable,
                                               int_en_cfg, timeout);
  }

  Result<InterruptEnableConfig> ReadInterruptEnableConfig(
      duration_type timeout) noexcept {
    return ReadByteAs_<InterruptEnableConfig>(register_map::InterruptEnable,
                                              timeout);
  }

  Status SetFreeFallDetectionInterruptEnable(bool enable,
                                             duration_type timeout) noexcept {
    Result<InterruptEnableConfig> res = ReadInterruptEnableConfig(timeout);

    RETURN_F_IF_ERROR(res);

    res.second.enable_freefall_detection_interrupt = enable;
    return WriteInterruptEnableConfig(res.second, timeout);
  }

  Status SetMotionDetectionInterruptEnable(bool enable,
                                           duration_type timeout) noexcept {
    Result<InterruptEnableConfig> res = ReadInterruptEnableConfig(timeout);

    RETURN_F_IF_ERROR(res);

    res.second.enable_motion_detection_interrupt = enable;

    return WriteInterruptEnableConfig(res.second, timeout);
  }

  Status SetZeroMotionDetectionInterruptEnable(bool enable,
                                               duration_type timeout) noexcept {
    Result<InterruptEnableConfig> res = ReadInterruptEnableConfig(timeout);

    RETURN_F_IF_ERROR(res);

    res.second.enable_zero_motion_detection_interrupt = enable;
    return WriteInterruptEnableConfig(res.second, timeout);
  }

  Status SetFifoOverflowInterruptEnable(bool enable,
                                        duration_type timeout) noexcept {
    Result<InterruptEnableConfig> res = ReadInterruptEnableConfig(timeout);

    RETURN_F_IF_ERROR(res);

    res.second.fifo_overflow_interrupt_enable = enable;
    return WriteInterruptEnableConfig(res.second, timeout);
  }

  Status SetI2cMasterInterruptEnable(bool enable,
                                     duration_type timeout) noexcept {
    Result<InterruptEnableConfig> res = ReadInterruptEnableConfig(timeout);

    RETURN_F_IF_ERROR(res);

    res.second.i2c_master_interrupt_enable = enable;

    return WriteInterruptEnableConfig(res.second, timeout);
  }

  Status SetDataReadyInterruptEnable(bool enable,
                                     duration_type timeout) noexcept {
    Result<InterruptEnableConfig> res = ReadInterruptEnableConfig(timeout);

    RETURN_F_IF_ERROR(res);

    res.second.data_ready_interrupt_enable = enable;

    return WriteInterruptEnableConfig(res.second, timeout);
  }

  Result<bool> GetFreeFallDetectionInterruptEnable(
      duration_type timeout) noexcept {
    Result<InterruptEnableConfig> res = ReadInterruptEnableConfig(timeout);
    return std::make_pair(res.first,
                          bool{res.second.enable_freefall_detection_interrupt});
  }

  Result<bool> GetMotionDetectionInterruptEnable(
      duration_type timeout) noexcept {
    Result<InterruptEnableConfig> res = ReadInterruptEnableConfig(timeout);
    return std::make_pair(res.first,
                          bool{res.second.enable_motion_detection_interrupt});
  }

  Result<bool> GetZeroMotionDetectionInterruptEnable(
      duration_type timeout) noexcept {
    Result<InterruptEnableConfig> res = ReadInterruptEnableConfig(timeout);
    return std::make_pair(
        res.first, bool{res.second.enable_zero_motion_detection_interrupt});
  }

  Result<bool> GetFifoOverflowInterruptEnable(duration_type timeout) noexcept {
    Result<InterruptEnableConfig> res = ReadInterruptEnableConfig(timeout);
    return std::make_pair(res.first,
                          bool{res.second.fifo_overflow_interrupt_enable});
  }

  Result<bool> GetI2cMasterInterruptEnable(duration_type timeout) noexcept {
    Result<InterruptEnableConfig> res = ReadInterruptEnableConfig(timeout);
    return std::make_pair(res.first,
                          bool{res.second.i2c_master_interrupt_enable});
  }

  Result<bool> GetDataReadyInterruptEnable(duration_type timeout) noexcept {
    Result<InterruptEnableConfig> res = ReadInterruptEnableConfig(timeout);
    return std::make_pair(res.first,
                          bool{res.second.data_ready_interrupt_enable});
  }

  /**
   *
   *
   **/

  Result<InterruptStatus> ReadInterruptStatus(duration_type timeout) noexcept {
    return ReadByteAs_<InterruptStatus>(register_map::InterruptStatus, timeout);
  }

  Result<bool> GetFreeFallDetectionInterruptState(
      duration_type timeout) noexcept {
    Result<InterruptStatus> res = ReadInterruptStatus(timeout);
    return std::make_pair(res.first, bool{res.second.free_fall_detected});
  }

  Result<bool> GetMotionInterruptState(duration_type timeout) noexcept {
    Result<InterruptStatus> res = ReadInterruptStatus(timeout);
    return std::make_pair(res.first, bool{res.second.motion_detected});
  }

  Result<bool> GetZeroMotionInterruptState(duration_type timeout) noexcept {
    Result<InterruptStatus> res = ReadInterruptStatus(timeout);
    return std::make_pair(res.first, bool{res.second.zero_motion_detected});
  }

  Result<bool> GetFifoOverflowInterruptState(duration_type timeout) noexcept {
    Result<InterruptStatus> res = ReadInterruptStatus(timeout);
    return std::make_pair(res.first, bool{res.second.fifo_overflow});
  }

  Result<bool> GetI2cMasterInterruptState(duration_type timeout) noexcept {
    Result<InterruptStatus> res = ReadInterruptStatus(timeout);
    return std::make_pair(res.first, bool{res.second.i2c_master_interrupt});
  }

  Result<bool> GetDataReadyInterruptState(duration_type timeout) noexcept {
    Result<InterruptStatus> res = ReadInterruptStatus(timeout);
    return std::make_pair(res.first, bool{res.second.data_ready});
  }

  /**
   *
   *
   **/

  Result<int16_t> ReadAccelerometerX(duration_type timeout) noexcept {
    uint8_t data[2]{};
    Status status = BurstRead_(register_map::AccelerometerX_OutH, data,
                               sizeof(data), timeout);
    int16_t a_x =
        (static_cast<int16_t>(data[0]) << 8) | (static_cast<int16_t>(data[1]));
    return std::make_pair(status, a_x);
  }

  Result<int16_t> ReadAccelerometerY(duration_type timeout) noexcept {
    uint8_t data[2]{};

    Status status = BurstRead_(register_map::AccelerometerY_OutH, data,
                               sizeof(data), timeout);
    int16_t a_y =
        (static_cast<int16_t>(data[0]) << 8) | (static_cast<int16_t>(data[1]));
    return std::make_pair(status, a_y);
  }

  Result<int16_t> ReadAccelerometerZ(duration_type timeout) noexcept {
    uint8_t data[2]{};
    Status status = BurstRead_(register_map::AccelerometerZ_OutH, data,
                               sizeof(data), timeout);
    int16_t a_z =
        (static_cast<int16_t>(data[0]) << 8) | (static_cast<int16_t>(data[1]));

    return std::make_pair(status, a_z);
  }

  Result<TriAxialData> ReadAccelerometer(duration_type timeout) noexcept {
    uint8_t data[2 * 3];
    Status status = BurstRead_(register_map::AccelerometerX_OutH, data,
                               sizeof(data), timeout);
    TriAxialData a{.x = (static_cast<int16_t>(data[0]) << 8) |
                        (static_cast<int16_t>(data[1])),
                   .y = (static_cast<int16_t>(data[2]) << 8) |
                        (static_cast<int16_t>(data[3])),
                   .z = (static_cast<int16_t>(data[4]) << 8) |
                        (static_cast<int16_t>(data[5]))};
    return std::make_pair(status, a);
  }

  Result<BiAxialData> ReadAccelerometerXY(duration_type timeout) noexcept {
    uint8_t data[2 * 2]{};

    Status status = BurstRead_(register_map::AccelerometerX_OutH, data,
                               sizeof(data), timeout);
    TriAxialData a_xy{.a = (static_cast<int16_t>(data[0]) << 8) |
                           (static_cast<int16_t>(data[1])),
                      .b = (static_cast<int16_t>(data[1]) << 8) |
                           (static_cast<int16_t>(data[2]))};
    return std::make_pair(status, a_xy);
  }

  Result<BiAxialData> ReadAccelerometerYZ(duration_type timeout) noexcept {
    uint8_t data[2 * 2]{};

    Status status = BurstRead_(register_map::AccelerometerY_OutH, data,
                               sizeof(data), timeout);
    TriAxialData a_yz{.a = (static_cast<int16_t>(data[0]) << 8) |
                           (static_cast<int16_t>(data[1])),
                      .b = (static_cast<int16_t>(data[1]) << 8) |
                           (static_cast<int16_t>(data[2]))};

    return std::make_pair(status, a_yz);
  }

  /**
   *
   *
   **/

  Result<int16_t> ReadTemperature(duration_type timeout) noexcept {
    uint8_t data[2]{};

    Status status =
        BurstRead_(register_map::TemperatureOutH, data, sizeof(data), timeout);

    int16_t temp =
        (static_cast<int16_t>(data[0]) << 8) | (static_cast<int16_t>(data[1]));

    return std::make_pair(status, temp);
  }

  /**
   *
   *
   **/

  Result<int16_t> ReadGyroscopeX(duration_type timeout) noexcept {
    uint8_t data[2]{};
    Status status =
        BurstRead_(register_map::GyroscopeX_OutH, data, sizeof(data), timeout);
    int16_t gx =
        (static_cast<int16_t>(data[0]) << 8) | (static_cast<int16_t>(data[1]));
    return std::make_pair(status, gx);
  }

  Result<int16_t> ReadGyroscopeY(duration_type timeout) noexcept {
    uint8_t data[2]{};
    Status status =
        BurstRead_(register_map::GyroscopeY_OutH, data, sizeof(data), timeout);
    int16_t gy =
        (static_cast<int16_t>(data[0]) << 8) | (static_cast<int16_t>(data[1]));

    return std::make_pair(status, gy);
  }

  Result<int16_t> ReadGyroscopeZ(duration_type timeout) noexcept {
    uint8_t data[2]{};
    Status status =
        BurstRead_(register_map::GyroscopeZ_OutH, data, sizeof(data), timeout);
    int16_t gz =
        (static_cast<int16_t>(data[0]) << 8) | (static_cast<int16_t>(data[1]));

    return std::make_pair(status, gz);
  }

  Result<TriAxialData> ReadGyroscope(duration_type timeout) noexcept {
    uint8_t data[2 * 3]{};
    Status status =
        BurstRead_(register_map::GyroscopeX_OutH, data, sizeof(data), timeout);
    TriAxialData g{.x = (static_cast<int16_t>(data[0]) << 8) |
                        (static_cast<int16_t>(data[1])),
                   .y = (static_cast<int16_t>(data[2]) << 8) |
                        (static_cast<int16_t>(data[3])),
                   .z = (static_cast<int16_t>(data[4]) << 8) |
                        (static_cast<int16_t>(data[5]))};
    return std::make_pair(status, g);
  }

  Result<BiAxialData> ReadGyroscopeXY(duration_type timeout) noexcept {
    uint8_t data[2 * 2]{};
    Status status =
        BurstRead_(register_map::GyroscopeX_OutH, data, sizeof(data), timeout);
    BiAxialData xy{.a = (static_cast<int16_t>(data[0]) << 8) |
                        (static_cast<int16_t>(data[1])),
                   .b = (static_cast<int16_t>(data[1]) << 8) |
                        (static_cast<int16_t>(data[2]))};

    return std::make_pair(status, xy);
  }

  Result<BiAxialData> ReadGyroscopeYZ(duration_type timeout) noexcept {
    uint8_t data[2 * 2]{};
    Status status =
        BurstRead_(register_map::GyroscopeY_OutH, data, sizeof(data), timeout);
    BiAxialData yz{.a = (static_cast<int16_t>(data[0]) << 8) |
                        (static_cast<int16_t>(data[1])),
                   .b = (static_cast<int16_t>(data[1]) << 8) |
                        (static_cast<int16_t>(data[2]))};

    return std::make_pair(status, yz);
  }

  /**
   *
   *
   **/

  Result<uint8_t> ReadExternalSensorData(uint8_t sensor_data_index,
                                         duration_type timeout) noexcept {
    // TODO(lamarrr): find a better implementation with bound checking
    return ReadByte_(register_map::ExternalSensorData0 + sensor_data_index,
                     timeout);
  }

  /**
   *
   *
   **/

  Result<MotionDetectionStatus> ReadMotionDetectionStatus(
      duration_type timeout) noexcept {
    return ReadByteAs_<MotionDetectionStatus>(
        register_map::MotionDetectionStatus, timeout);
  }

  Result<bool> GetNegativeX_MotionDetected(duration_type timeout) noexcept {
    Result<MotionDetectionStatus> res = ReadByteAs_<MotionDetectionStatus>(
        register_map::MotionDetectionStatus, timeout);

    return std::make_pair(res.first, bool{res.second.x_negative});
  }
  Result<bool> GetPositiveX_MotionDetected(duration_type timeout) noexcept {
    Result<MotionDetectionStatus> res = ReadByteAs_<MotionDetectionStatus>(
        register_map::MotionDetectionStatus, timeout);

    return std::make_pair(res.first, bool{res.second.x_positive});
  }

  Result<bool> GetNegativeY_MotionDetected(duration_type timeout) noexcept {
    Result<MotionDetectionStatus> res = ReadByteAs_<MotionDetectionStatus>(
        register_map::MotionDetectionStatus, timeout);

    return std::make_pair(res.first, bool{res.second.y_negative});
  }

  Result<bool> GetPositiveY_MotionDetected(duration_type timeout) noexcept {
    Result<MotionDetectionStatus> res = ReadByteAs_<MotionDetectionStatus>(
        register_map::MotionDetectionStatus, timeout);

    return std::make_pair(res.first, bool{res.second.y_positive});
  }

  Result<bool> GetNegativeZ_MotionDetected(duration_type timeout) noexcept {
    Result<MotionDetectionStatus> res = ReadByteAs_<MotionDetectionStatus>(
        register_map::MotionDetectionStatus, timeout);

    return std::make_pair(res.first, bool{res.second.z_negative});
  }

  Result<bool> GetPositiveZ_MotionDetected(duration_type timeout) noexcept {
    Result<MotionDetectionStatus> res = ReadByteAs_<MotionDetectionStatus>(
        register_map::MotionDetectionStatus, timeout);

    return std::make_pair(res.first, bool{res.second.z_positive});
  }

  Result<bool> GetZeroMotionDetected(duration_type timeout) noexcept {
    Result<MotionDetectionStatus> res = ReadByteAs_<MotionDetectionStatus>(
        register_map::MotionDetectionStatus, timeout);
    return std::make_pair(res.first, bool{res.second.zero_motion});
  }

  /**
   *
   *
   **/

  Status WriteI2cSlave0_DataOut(uint8_t data, duration_type timeout) noexcept {
    return WriteByte_(register_map::I2cSlave0_DataOut, data, timeout);
  }

  Result<uint8_t> ReadI2cSlave0_DataOut(duration_type timeout) noexcept {
    return ReadByte_(register_map::I2cSlave0_DataOut, timeout);
  }

  Status WriteI2cSlave1_DataOut(uint8_t data, duration_type timeout) noexcept {
    return WriteByte_(register_map::I2cSlave1_DataOut, data, timeout);
  }

  Result<uint8_t> ReadI2cSlave1_DataOut(duration_type timeout) noexcept {
    return ReadByte_(register_map::I2cSlave1_DataOut, timeout);
  }

  Status WriteI2cSlave2_DataOut(uint8_t data, duration_type timeout) noexcept {
    return WriteByte_(register_map::I2cSlave2_DataOut, data, timeout);
  }

  Result<uint8_t> ReadI2cSlave2_DataOut(duration_type timeout) noexcept {
    return ReadByte_(register_map::I2cSlave2_DataOut, timeout);
  }

  Status WriteI2cSlave3_DataOut(uint8_t data, duration_type timeout) noexcept {
    return WriteByte_(register_map::I2cSlave3_DataOut, data, timeout);
  }

  Result<uint8_t> ReadI2cSlave3_DataOut(duration_type timeout) noexcept {
    return ReadByte_(register_map::I2cSlave3_DataOut, timeout);
  }

  /**
   *
   *
   **/

  Status WriteI2cMasterDelayCtrlConfig(I2cMasterDelayCtrlConfig ctrl_cfg,
                                       duration_type timeout) noexcept {
    return WriteByteAs_<I2cMasterDelayCtrlConfig>(
        register_map::I2cMasterDelayControl, ctrl_cfg, timeout);
  }

  Result<I2cMasterDelayCtrlConfig> ReadI2cMasterDelayCtrlConfig(
      duration_type timeout) noexcept {
    return ReadByteAs_<I2cMasterDelayCtrlConfig>(
        register_map::I2cMasterDelayControl, timeout);
  }

  /**
   *
   *
   **/

  Status WriteSignalPathResetConfig(SignalPathResetConfig spr_cfg,
                                    duration_type timeout) noexcept {
    return WriteByteAs_<SignalPathResetConfig>(register_map::SignalPathReset,
                                               spr_cfg, timeout);
  }

  Result<SignalPathResetConfig> ReadSignalPathResetConfig(
      duration_type timeout) noexcept {
    return ReadByteAs_<SignalPathResetConfig>(register_map::SignalPathReset,
                                              timeout);
  }

  Status SetGyroscopeSignalPathReset(bool reset,
                                     duration_type timeout) noexcept {
    Result<SignalPathResetConfig> cfg = ReadSignalPathResetConfig(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.gyroscope_reset = reset;
    return WriteSignalPathResetConfig(cfg.second, timeout);
  }

  Status SetAccelerometerSignalPathReset(bool reset,
                                         duration_type timeout) noexcept {
    Result<SignalPathResetConfig> cfg = ReadSignalPathResetConfig(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.accelerometer_reset = reset;
    return WriteSignalPathResetConfig(cfg.second, timeout);
  }

  Status SetTemperatureSignalPathReset(bool reset,
                                       duration_type timeout) noexcept {
    Result<SignalPathResetConfig> cfg = ReadSignalPathResetConfig(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.temperature_reset = reset;
    return WriteSignalPathResetConfig(cfg.second, timeout);
  }

  Result<bool> GetGyroscopeSignalPathReset(duration_type timeout) noexcept {
    Result<SignalPathResetConfig> cfg = ReadSignalPathResetConfig(timeout);

    return std::make_pair(cfg.first, bool{cfg.second.gyroscope_reset});
  }

  Result<bool> GetAccelerometerSignalPathReset(duration_type timeout) noexcept {
    Result<SignalPathResetConfig> cfg = ReadSignalPathResetConfig(timeout);

    return std::make_pair(cfg.first, bool{cfg.second.accelerometer_reset});
  }

  Result<bool> GetTemperatureSignalPathReset(duration_type timeout) noexcept {
    Result<SignalPathResetConfig> cfg = ReadSignalPathResetConfig(timeout);

    return std::make_pair(cfg.first, bool{cfg.second.temperature_reset});
  }

  /**
   *
   *
   **/

  Status WriteMotionDetectionCtrlConfig(MotionDetectionCtrlConfig cfg,
                                        duration_type timeout) noexcept {
    return WriteByteAs_<MotionDetectionCtrlConfig>(
        register_map::MotionDetectionControl, cfg, timeout);
  }

  Result<MotionDetectionCtrlConfig> GetMotionDetectionCtrlConfig(
      MotionDetectionCtrlConfig, duration_type timeout) noexcept {
    return ReadByteAs_<MotionDetectionCtrlConfig>(
        register_map::MotionDetectionControl, timeout);
  }

  /**
   *
   *
   **/

  Status WriteUserCtrlConfig(UserCtrlConfig cfg,
                             duration_type timeout) noexcept {
    return WriteByteAs_<UserCtrlConfig>(register_map::UserControl, cfg,
                                        timeout);
  }

  Result<UserCtrlConfig> ReadUserCtrlConfig(duration_type timeout) noexcept {
    return ReadByteAs_<UserCtrlConfig>(register_map::UserControl, timeout);
  }

  Status SetFifoEnable(bool enable, duration_type timeout) noexcept {
    Result<UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.fifo_enable = enable;

    return WriteUserCtrlConfig(cfg.second, timeout);
  }

  Status SetI2cMasterEnable(bool enable, duration_type timeout) noexcept {
    Result<UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.i2c_master_mode_enable = enable;

    return WriteUserCtrlConfig(cfg.second, timeout);
  }

  Status SetUseSpiInterface(bool enable, duration_type timeout) noexcept {
    Result<UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.enable_spi_interface = enable;

    return WriteUserCtrlConfig(cfg.second, timeout);
  }

  Status SetFifoReset(bool enable, duration_type timeout) noexcept {
    Result<UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.reset_fifo_buffer = enable;

    return WriteUserCtrlConfig(cfg.second, timeout);
  }

  Status SetI2cMasterReset(bool enable, duration_type timeout) noexcept {
    Result<UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.reset_i2c_master = enable;

    return WriteUserCtrlConfig(cfg.second, timeout);
  }

  Status SetSignalPathReset(bool enable, duration_type timeout) noexcept {
    Result<UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.clear_sensor_register_and_path = enable;

    return WriteUserCtrlConfig(cfg.second, timeout);
  }

  Result<bool> GetFifoEnable(duration_type timeout) noexcept {
    Result<UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
    return std::make_pair(cfg.first, bool{cfg.second.fifo_enable});
  }

  Result<bool> GetI2cMasterEnable(duration_type timeout) noexcept {
    Result<UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
    return std::make_pair(cfg.first, bool{cfg.second.i2c_master_mode_enable});
  }

  Result<bool> GetUseSpiInterface(duration_type timeout) noexcept {
    Result<UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
    return std::make_pair(cfg.first, bool{cfg.second.enable_spi_interface});
  }

  Result<bool> GetFifoReset(duration_type timeout) noexcept {
    Result<UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
    return std::make_pair(cfg.first, bool{cfg.second.reset_fifo_buffer});
  }

  Result<bool> GetI2cMasterReset(duration_type timeout) noexcept {
    Result<UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
    return std::make_pair(cfg.first, bool{cfg.second.reset_i2c_master});
  }

  Result<bool> GetSignalPathReset(duration_type timeout) noexcept {
    Result<UserCtrlConfig> cfg = ReadUserCtrlConfig(timeout);
    return std::make_pair(cfg.first,
                          bool{cfg.second.clear_sensor_register_and_path});
  }

  /**
   *
   *
   **/

  Status WritePowerManagement1_Config(PowerManagement1_Config cfg,
                                      duration_type timeout) noexcept {
    return WriteByteAs_<PowerManagement1_Config>(register_map::PowerManagement1,
                                                 cfg, timeout);
  }

  Result<PowerManagement1_Config> ReadPowerManagement1_Config(
      duration_type timeout) noexcept {
    return ReadByteAs_<PowerManagement1_Config>(register_map::PowerManagement1,
                                                timeout);
  }

  Status SetDeviceRegisterReset(bool reset, duration_type timeout) noexcept {
    Result<PowerManagement1_Config> cfg = ReadPowerManagement1_Config(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.reset_registers = reset;

    return WritePowerManagement1_Config(cfg.second, timeout);
  }

  Status SetSleep(bool sleep, duration_type timeout) noexcept {
    Result<PowerManagement1_Config> cfg = ReadPowerManagement1_Config(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.sleep_mode = sleep;

    return WritePowerManagement1_Config(cfg.second, timeout);
  }

  Status SetSleepCycle(bool cycle, duration_type timeout) noexcept {
    Result<PowerManagement1_Config> cfg = ReadPowerManagement1_Config(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.cycle_sleep = cycle;

    return WritePowerManagement1_Config(cfg.second, timeout);
  }

  Status SetTemperatureDisable(bool disable, duration_type timeout) noexcept {
    Result<PowerManagement1_Config> cfg = ReadPowerManagement1_Config(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.disable_temperature = disable;

    return WritePowerManagement1_Config(cfg.second, timeout);
  }

  Status SetClockSource(ClockSource clk, duration_type timeout) noexcept {
    Result<PowerManagement1_Config> cfg = ReadPowerManagement1_Config(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.clock_source =
        static_cast<ClockSource>(static_cast<uint8_t>(clk) & 0b111U);

    return WritePowerManagement1_Config(cfg.second, timeout);
  }

  Result<bool> GetDeviceRegisterReset(duration_type timeout) noexcept {
    Result<PowerManagement1_Config> cfg = ReadPowerManagement1_Config(timeout);
    return std::make_pair(cfg.first, bool{cfg.second.reset_registers});
  }

  Result<bool> GetSleep(duration_type timeout) noexcept {
    Result<PowerManagement1_Config> cfg = ReadPowerManagement1_Config(timeout);
    return std::make_pair(cfg.first, bool{cfg.second.sleep_mode});
  }

  Result<bool> GetSleepCycle(duration_type timeout) noexcept {
    Result<PowerManagement1_Config> cfg = ReadPowerManagement1_Config(timeout);
    return std::make_pair(cfg.first, bool{cfg.second.cycle_sleep});
  }

  Result<bool> GetTemperatureDisable(duration_type timeout) noexcept {
    Result<PowerManagement1_Config> cfg = ReadPowerManagement1_Config(timeout);
    return std::make_pair(cfg.first, bool{cfg.second.disable_temperature});
  }

  Result<ClockSource> GetClockSource(duration_type timeout) noexcept {
    Result<PowerManagement1_Config> cfg = ReadPowerManagement1_Config(timeout);
    return std::make_pair(cfg.first, ClockSource{cfg.second.clock_source});
  }

  /**
   *
   *
   **/

  Status WritePowerManagement2_Config(PowerManagement2_Config cfg,
                                      duration_type timeout) noexcept {
    return WriteByteAs_<PowerManagement2_Config>(register_map::PowerManagement2,
                                                 cfg, timeout);
  }

  Result<PowerManagement2_Config> ReadPowerManagement2_Config(
      duration_type timeout) noexcept {
    return ReadByteAs_<PowerManagement2_Config>(register_map::PowerManagement2,
                                                timeout);
  }

  Status SetWakeupFrequency(WakeupFrequency wfreq,
                            duration_type timeout) noexcept {
    Result<PowerManagement2_Config> cfg = ReadPowerManagement2_Config(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.wakeup_frequency =
        static_cast<WakeupFrequency>(static_cast<uint8_t>(wfreq) & 0b11U);
    return WriteByteAs_<PowerManagement2_Config>(register_map::PowerManagement2,
                                                 cfg.second, timeout);
  }

  Status SetAccelerometerX_Standby(bool standby,
                                   duration_type timeout) noexcept {
    Result<PowerManagement2_Config> cfg = ReadPowerManagement2_Config(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.accelerometer_x_standby = standby;
    return WriteByteAs_<PowerManagement2_Config>(register_map::PowerManagement2,
                                                 cfg.second, timeout);
  }

  Status SetAccelerometerY_Standby(bool standby,
                                   duration_type timeout) noexcept {
    Result<PowerManagement2_Config> cfg = ReadPowerManagement2_Config(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.accelerometer_y_standby = standby;
    return WriteByteAs_<PowerManagement2_Config>(register_map::PowerManagement2,
                                                 cfg.second, timeout);
  }

  Status SetAccelerometerZ_Standby(bool standby,
                                   duration_type timeout) noexcept {
    Result<PowerManagement2_Config> cfg = ReadPowerManagement2_Config(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.accelerometer_z_standby = standby;
    return WriteByteAs_<PowerManagement2_Config>(register_map::PowerManagement2,
                                                 cfg.second, timeout);
  }

  Status SetGyroscopeX_Standby(bool standby, duration_type timeout) noexcept {
    Result<PowerManagement2_Config> cfg = ReadPowerManagement2_Config(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.gyroscope_x_standby = standby;
    return WriteByteAs_<PowerManagement2_Config>(register_map::PowerManagement2,
                                                 cfg.second, timeout);
  }

  Status SetGyroscopeY_Standby(bool standby, duration_type timeout) noexcept {
    Result<PowerManagement2_Config> cfg = ReadPowerManagement2_Config(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.gyroscope_y_standby = standby;
    return WriteByteAs_<PowerManagement2_Config>(register_map::PowerManagement2,
                                                 cfg.second, timeout);
  }

  Status SetGyroscopeZ_Standby(bool standby, duration_type timeout) noexcept {
    Result<PowerManagement2_Config> cfg = ReadPowerManagement2_Config(timeout);

    RETURN_F_IF_ERROR(cfg);

    cfg.second.gyroscope_z_standby = standby;
    return WriteByteAs_<PowerManagement2_Config>(register_map::PowerManagement2,
                                                 cfg.second, timeout);
  }

  Result<WakeupFrequency> GetWakeupFrequency(duration_type timeout) noexcept {
    Result<PowerManagement2_Config> cfg = ReadPowerManagement2_Config(timeout);

    return std::make_pair(cfg.first,
                          WakeupFrequency{cfg.second.wakeup_frequency});
  }

  Result<bool> GetAccelerometerX_Standby(duration_type timeout) noexcept {
    Result<PowerManagement2_Config> cfg = ReadPowerManagement2_Config(timeout);

    return std::make_pair(cfg.first, bool{cfg.second.accelerometer_x_standby});
  }

  Result<bool> GetAccelerometerY_Standby(duration_type timeout) noexcept {
    Result<PowerManagement2_Config> cfg = ReadPowerManagement2_Config(timeout);

    return std::make_pair(cfg.first, bool{cfg.second.accelerometer_y_standby});
  }

  Result<bool> GetAccelerometerZ_Standby(duration_type timeout) noexcept {
    Result<PowerManagement2_Config> cfg = ReadPowerManagement2_Config(timeout);

    return std::make_pair(cfg.first, bool{cfg.second.accelerometer_z_standby});
  }

  Result<bool> GetGyroscopeX_Standby(duration_type timeout) noexcept {
    Result<PowerManagement2_Config> cfg = ReadPowerManagement2_Config(timeout);

    return std::make_pair(cfg.first, bool{cfg.second.gyroscope_x_standby});
  }

  Result<bool> GetGyroscopeY_Standby(duration_type timeout) noexcept {
    Result<PowerManagement2_Config> cfg = ReadPowerManagement2_Config(timeout);

    return std::make_pair(cfg.first, bool{cfg.second.gyroscope_y_standby});
  }

  Result<bool> GetGyroscopeZ_Standby(duration_type timeout) noexcept {
    Result<PowerManagement2_Config> cfg = ReadPowerManagement2_Config(timeout);

    return std::make_pair(cfg.first, bool{cfg.second.gyroscope_z_standby});
  }

  /**
   *
   *
   **/

  Result<uint16_t> ReadFifoCount(duration_type timeout) noexcept {
    uint8_t data[2]{};

    Status status =
        BurstRead_(register_map::FifoCountH, data, sizeof(data), timeout);

    uint16_t fifo_cnt = (static_cast<uint16_t>(data[0]) << 8) |
                        (static_cast<uint16_t>(data[1]));
    return std::make_pair(status, fifo_cnt);
  }

  /**
   *
   *
   **/

  Status WriteFifoData(uint8_t data, duration_type timeout) noexcept {
    return WriteByte_(register_map::FifoReadWrite, data, timeout);
  }

  Result<uint8_t> ReadFifoData(duration_type timeout) noexcept {
    return ReadByte_(register_map::FifoReadWrite, timeout);
  }

  /**
   *
   *
   **/

  Result<uint8_t> ReadWhoAmI(duration_type timeout) noexcept {
    return ReadByte_(register_map::FifoReadWrite, timeout);
  }

  /**
   *
   *
   **/
};
}  // namespace mpu60X0
#endif  // DRIVERS_MPU60X0_MPU60X0_H_
