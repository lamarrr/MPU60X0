/**
 * @file registers.h
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

#ifndef DRIVERS_MPU60X0_REGISTERS_H_
#define DRIVERS_MPU60X0_REGISTERS_H_

#include <cinttypes>

namespace mpu60X0 {

using register_type = uint8_t;

namespace register_map {
constexpr register_type SelfTestX = 13U;
constexpr register_type SelfTestY = 14U;
constexpr register_type SelfTestZ = 15U;
constexpr register_type SelfTestA = 16U;
constexpr register_type SampleRateDivider = 25U;
constexpr register_type Config = 26U;
constexpr register_type GyroscopeConfig = 27U;
constexpr register_type AccelerometerConfig = 28U;
constexpr register_type FreeFallAccelerationThreshold = 29U;
constexpr register_type FreeFallDuration = 30U;
constexpr register_type MotionDetectionThreshold = 31U;
constexpr register_type MotionDetectionDuration = 32U;
constexpr register_type ZeroMotionDetectionThreshold = 33U;
constexpr register_type ZeroMotionDetectionDuration = 34U;
constexpr register_type FifoEnable = 35U;
constexpr register_type I2cMasterCtrl = 36U;
constexpr register_type I2cSlave0_RW_Address = 37U;
constexpr register_type I2cSlave0_Register = 38U;
constexpr register_type I2cSlave0_Control = 39U;
constexpr register_type I2cSlave1_RW_Address = 40U;
constexpr register_type I2cSlave1_Register = 41U;
constexpr register_type I2cSlave1_Control = 42U;
constexpr register_type I2cSlave2_RW_Address = 43U;
constexpr register_type I2cSlave2_Register = 44U;
constexpr register_type I2cSlave2_Control = 45U;
constexpr register_type I2cSlave3_RW_Address = 46U;
constexpr register_type I2cSlave3_Register = 47U;
constexpr register_type I2cSlave3_Control = 48U;
constexpr register_type I2cSlave4_RW_Address = 49U;
constexpr register_type I2cSlave4_Register = 50U;
constexpr register_type I2cSlave4_D0 = 51U;
constexpr register_type I2cSlave4_Control = 52U;
constexpr register_type I2cSlave4_D1 = 53U;
constexpr register_type I2cMasterStatus = 54U;
constexpr register_type InterruptPinConfig = 55U;
constexpr register_type InterruptEnable = 56U;
constexpr register_type InterruptStatus = 58U;
constexpr register_type AccelerometerX_OutH = 59U;
constexpr register_type AccelerometerX_OutL = 60U;
constexpr register_type AccelerometerY_OutH = 61U;
constexpr register_type AccelerometerY_OutL = 62U;
constexpr register_type AccelerometerZ_OutH = 63U;
constexpr register_type AccelerometerZ_OutL = 64U;
constexpr register_type TemperatureOutH = 65U;
constexpr register_type TemperatureOutL = 66U;
constexpr register_type GyroscopeX_OutH = 67U;
constexpr register_type GyroscopeX_OutL = 68U;
constexpr register_type GyroscopeY_OutH = 69U;
constexpr register_type GyroscopeY_OutL = 70U;
constexpr register_type GyroscopeZ_OutH = 71U;
constexpr register_type GyroscopeZ_OutL = 72U;
constexpr register_type ExternalSensorData0 = 73U;
constexpr register_type ExternalSensorData1 = 74U;
constexpr register_type ExternalSensorData2 = 75U;
constexpr register_type ExternalSensorData3 = 76U;
constexpr register_type ExternalSensorData4 = 77U;
constexpr register_type ExternalSensorData5 = 78U;
constexpr register_type ExternalSensorData6 = 79U;
constexpr register_type ExternalSensorData7 = 80U;
constexpr register_type ExternalSensorData8 = 81U;
constexpr register_type ExternalSensorData9 = 82U;
constexpr register_type ExternalSensorData10 = 83U;
constexpr register_type ExternalSensorData11 = 84U;
constexpr register_type ExternalSensorData12 = 85U;
constexpr register_type ExternalSensorData13 = 86U;
constexpr register_type ExternalSensorData14 = 87U;
constexpr register_type ExternalSensorData15 = 88U;
constexpr register_type ExternalSensorData16 = 89U;
constexpr register_type ExternalSensorData17 = 90U;
constexpr register_type ExternalSensorData18 = 91U;
constexpr register_type ExternalSensorData19 = 92U;
constexpr register_type ExternalSensorData20 = 93U;
constexpr register_type ExternalSensorData21 = 94U;
constexpr register_type ExternalSensorData22 = 95U;
constexpr register_type ExternalSensorData23 = 96U;
constexpr register_type MotionDetectionStatus = 97U;
constexpr register_type I2cSlave0_DataOut = 99U;
constexpr register_type I2cSlave1_DataOut = 100U;
constexpr register_type I2cSlave2_DataOut = 101U;
constexpr register_type I2cSlave3_DataOut = 102U;
constexpr register_type I2cMasterDelayControl = 103U;
constexpr register_type SignalPathReset = 104U;
constexpr register_type MotionDetectionControl = 105U;
constexpr register_type UserControl = 106U;
constexpr register_type PowerManagement1 = 107U;
constexpr register_type PowerManagement2 = 108U;
constexpr register_type FifoCountH = 114U;
constexpr register_type FifoCountL = 115U;
constexpr register_type FifoReadWrite = 116U;
constexpr register_type WhoAmI = 117U;
};  // namespace register_map

};      // namespace mpu60X0
#endif  // DRIVERS_MPU60X0_REGISTERS_H_
