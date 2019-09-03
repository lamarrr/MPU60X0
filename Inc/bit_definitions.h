/**
 * @file bit_definitions.h
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

#ifndef DRIVERS_MPU60X0_BIT_DEFINITIONS_H_
#define DRIVERS_MPU60X0_BIT_DEFINITIONS_H_

#include <cinttypes>

#define MPU_CDEBUG

#ifdef MPU_CDEBUG
#define MPU_SIZE_CASSERT(x) \
  static_assert(sizeof(x) == 1U, "sizeof " #x " is larger than a byte");
#else
#define MPU_SIZE_CASSERT(x)
#endif

namespace mpu60X0 {
// Good 'Ol bit packing
enum struct FrameSync : uint8_t {
  InputDisabled = 0U,
  TemperatureOutL = 1U,
  GyroscopeX_OutL = 2U,
  GyroscopeY_OutL = 3U,
  GyroscopeZ_OutL = 4U,
  AccelerometerX_OutL = 5U,
  AccelerometerY_OutL = 6U,
  AccelerometerZ_OutL = 7U
};

enum struct DlpfConfig : uint8_t {
  A_260Hz_G_256Hz = 0U,
  A_184Hz_G_188Hz = 1U,
  A_94Hz_G_98Hz = 2U,
  A_44Hz_G_42Hz = 3U,
  A_21Hz_G_20Hz = 4U,
  A_10Hz_G_10Hz = 5U,
  A_5Hz_G_5Hz = 6U,
  ___reserved__ = 7U
};

struct Config {
  DlpfConfig dlpf_config : 3U;
  FrameSync ext_frame_sync : 3U;
  uint8_t __reserved__ : 2U;
  Config() { *reinterpret_cast<uint8_t*>(this) = 0U; }
};

MPU_SIZE_CASSERT(Config);

enum struct GyroscopeFullScale : uint8_t {
  Dps250 = 0U,
  Dps500 = 1U,
  Dps1000 = 2U,
  Dps2000 = 3U,
};

struct GyroscopeConfig {
  uint8_t __reserved__ : 3U;
  GyroscopeFullScale full_scale : 2U;
  bool z_self_test : 1U;
  bool y_self_test : 1U;
  bool x_self_test : 1U;

  GyroscopeConfig() { *reinterpret_cast<uint8_t*>(this) = 0U; }
};

MPU_SIZE_CASSERT(GyroscopeConfig);

enum struct AccelerometerFullScale : uint8_t {
  k2G = 0U,
  k4G = 1U,
  k8G = 2U,
  k16G = 3U
};

struct AccelerometerConfig {
  uint8_t __reserved__ : 3U;
  AccelerometerFullScale full_scale : 2U;
  bool z_self_test : 1U;
  bool y_self_test : 1U;
  bool x_self_test : 1U;

  AccelerometerConfig() { *reinterpret_cast<uint8_t*>(this) = 0U; }
};

MPU_SIZE_CASSERT(AccelerometerConfig);

struct FifoEnableConfig {
  bool enable_slave0_fifo : 1U;
  bool enable_slave1_fifo : 1U;
  bool enable_slave2_fifo : 1U;
  bool enable_acceleration_fifo : 1U;
  bool enable_gyroscope_z_fifo : 1U;
  bool enable_gyroscope_y_fifo : 1U;
  bool enable_gyroscope_x_fifo : 1U;
  bool enable_temperature_fifo : 1U;

  FifoEnableConfig() { *reinterpret_cast<uint8_t*>(this) = 0U; }
};

MPU_SIZE_CASSERT(FifoEnableConfig);

enum struct I2cMasterClockSpeed : uint8_t {
  F348kHz = 0U,
  F333kHz = 1U,
  F320kHz = 2U,
  F308kHz = 3U,
  F296kHz = 4U,
  F286kHz = 5U,
  F276kHz = 6U,
  F267kHz = 7U,
  F258kHz = 8U,
  F500kHz = 9U,
  F471kHz = 10U,
  F444kHz = 11U,
  F421kHz = 12U,
  F400kHz = 13U,
  F381kHz = 14U,
  F364kHz = 15U,
};

enum struct I2cMasterReadWriteTransition : uint8_t {
  Restart = 0U,
  StartStop = 1U,
};

struct I2cMasterCtrlConfig {
  I2cMasterClockSpeed clock_speed : 4U;
  I2cMasterReadWriteTransition rw_transition : 1U;
  bool enable_slave3_fifo : 1U;
  bool dri_await_ext_sensor_data : 1U;
  bool enable_multi_master : 1U;

  I2cMasterCtrlConfig() { *reinterpret_cast<uint8_t*>(this) = 0U; }
};

MPU_SIZE_CASSERT(I2cMasterCtrlConfig);

struct InterruptPinConfig {
  uint8_t ___reserved___ : 1U;
  bool bypass_i2c_enable : 1U;
  bool frame_sync_enable : 1U;
  uint8_t frame_sync_logic_level : 1U;
  bool clear_on_read : 1U;
  bool latch_enable_until_interrupt_clear : 1U;
  bool open_drain : 1U;
  uint8_t logic_level : 1U;

  InterruptPinConfig() { *reinterpret_cast<uint8_t*>(this) = 0U; }
};

MPU_SIZE_CASSERT(InterruptPinConfig);

struct InterruptEnableConfig {
  bool data_ready_interrupt_enable : 1U;
  uint8_t __reserved__ : 2U;
  bool i2c_master_interrupt_enable : 1U;
  bool fifo_overflow_interrupt_enable : 1U;
  bool enable_zero_motion_detection_interrupt : 1U;
  bool enable_motion_detection_interrupt : 1U;
  bool enable_freefall_detection_interrupt : 1U;

  InterruptEnableConfig() { *reinterpret_cast<uint8_t*>(this) = 0U; }
};

MPU_SIZE_CASSERT(InterruptEnableConfig);

struct InterruptStatus {
  bool data_ready : 1U;
  uint8_t __reserved__ : 2U;
  bool i2c_master_interrupt : 1U;
  bool fifo_overflow : 1U;
  bool zero_motion_detected : 1U;
  bool motion_detected : 1U;
  bool free_fall_detected : 1U;

  InterruptStatus() { *reinterpret_cast<uint8_t*>(this) = 0U; }
};

MPU_SIZE_CASSERT(InterruptStatus);

enum struct ClockSource : uint8_t {
  Internal_8MHz = 0U,
  PllGyro_X = 1U,
  PllGyro_Y = 2U,
  PllGyro_Z = 3U,
  External_32_768kHz = 4U,
  External_19_2MHz = 5U,
  ClockSourceNone = 7U
};

struct I2cMasterDelayCtrlConfig {
  bool slave0_delayed_access : 1U;
  bool slave1_delayed_access : 1U;
  bool slave2_delayed_access : 1U;
  bool slave3_delayed_access : 1U;
  bool slave4_delayed_access : 1U;
  uint8_t __reserved__ : 2U;
  bool delayed_shadow : 1U;

  I2cMasterDelayCtrlConfig() { *reinterpret_cast<uint8_t*>(this) = 0U; }
};

MPU_SIZE_CASSERT(I2cMasterDelayCtrlConfig);

struct SignalPathResetConfig {
  bool temperature_reset : 1U;
  bool accelerometer_reset : 1U;
  bool gyroscope_reset : 1U;
  uint8_t __reserved__ : 5U;

  SignalPathResetConfig() { *reinterpret_cast<uint8_t*>(this) = 0U; }
};

MPU_SIZE_CASSERT(SignalPathResetConfig);

struct MotionDetectionCtrlConfig {
  uint8_t motion_counter_decrement_rate : 2U;
  uint8_t freefall_counter_decrement_rate : 2U;
  uint8_t accelerometer_poweron_delay : 2U;
  uint8_t __reserved__ : 2U;

  MotionDetectionCtrlConfig() { *reinterpret_cast<uint8_t*>(this) = 0U; }
};

MPU_SIZE_CASSERT(MotionDetectionCtrlConfig);

struct UserCtrlConfig {
  bool clear_sensor_register_and_path : 1U;
  bool reset_i2c_master : 1U;
  bool reset_fifo_buffer : 1U;
  uint8_t __reserved1__ : 1U;
  bool enable_spi_interface : 1U;
  bool i2c_master_mode_enable : 1U;
  bool fifo_enable : 1U;
  uint8_t __reserved2__ : 1U;

  UserCtrlConfig() { *reinterpret_cast<uint8_t*>(this) = 0U; }
};

MPU_SIZE_CASSERT(UserCtrlConfig);

struct PowerManagement1_Config {
  ClockSource clock_source : 3U;
  bool disable_temperature : 1U;
  uint8_t __reserved__ : 1U;
  bool cycle_sleep : 1U;
  bool sleep_mode : 1U;
  bool reset_registers : 1U;

  PowerManagement1_Config() { *reinterpret_cast<uint8_t*>(this) = 0U; }
};

MPU_SIZE_CASSERT(PowerManagement1_Config);

enum struct WakeupFrequency : uint8_t {
  F1_25Hz = 0U,
  F5Hz = 1U,
  F25Hz = 2U,
  F40Hz = 3U
};

struct PowerManagement2_Config {
  bool gyroscope_z_standby : 1U;
  bool gyroscope_y_standby : 1U;
  bool gyroscope_x_standby : 1U;
  bool accelerometer_z_standby : 1U;
  bool accelerometer_y_standby : 1U;
  bool accelerometer_x_standby : 1U;
  WakeupFrequency wakeup_frequency : 2U;

  PowerManagement2_Config() { *reinterpret_cast<uint8_t*>(this) = 0U; }
};

MPU_SIZE_CASSERT(PowerManagement1_Config);

struct MotionDetectionStatus {
  bool zero_motion : 1;
  uint8_t __reserved__ : 1;
  bool z_positive : 1;
  bool z_negative : 1;
  bool y_positive : 1;
  bool y_negative : 1;
  bool x_positive : 1;
  bool x_negative : 1;

  MotionDetectionStatus() { *reinterpret_cast<uint8_t*>(this) = 0U; }
};

MPU_SIZE_CASSERT(MotionDetectionStatus);

};      // namespace mpu60X0
#endif  // DRIVERS_MPU60X0_BIT_DEFINITIONS_H_
