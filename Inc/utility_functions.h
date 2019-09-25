/**
 * @file utility_functions.h
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

#ifndef DRIVERS_MPU60X0_UTILITY_FUNCTIONS_H_
#define DRIVERS_MPU60X0_UTILITY_FUNCTIONS_H_

#include <cinttypes>
#include <limits>
#include <type_traits>
#include "bit_definitions.h"  // NOLINT
#include "data_types.h"       // NOLINT

namespace mpu60X0 {

// Utility Functions
constexpr float kBasisI16 =
    static_cast<float>(std::numeric_limits<int16_t>::max());
// smallest and closest to int16 in precision is float32, using higher is
// redundant
float TemperatureToDegreesCelsius(int16_t temp) {
  return (temp / 340.0F) + 36.53F;
}
float OrientationToMS1(int16_t value, GyroscopeFullScale scale) {
  switch (scale) {
    case GyroscopeFullScale::DegPs1000:
      return (value / kBasisI16) * 1000.0F;

    case GyroscopeFullScale::DegPs2000:
      return (value / kBasisI16) * 2000.0F;

    case GyroscopeFullScale::DegPs250:
      return (value / kBasisI16) * 250.0F;

    case GyroscopeFullScale::DegPs500:
      return (value / kBasisI16) * 500.0F;

    default:
      return std::numeric_limits<float>::signaling_NaN();
  }
}

float AccelerationToMs2(int16_t value, AccelerometerFullScale scale) {
  switch (scale) {
    case AccelerometerFullScale::k16G:
      return (value / kBasisI16) * 16.0F;

    case AccelerometerFullScale::k2G:
      return (value / kBasisI16) * 2.0F;

    case AccelerometerFullScale::k4G:
      return (value / kBasisI16) * 4.0F;

    case AccelerometerFullScale::k8G:
      return (value / kBasisI16) * 8.0F;

    default:
      return std::numeric_limits<float>::signaling_NaN();
  }
}
float GyroscopeX_FactoryTrimValue(uint8_t){};
float GyroscopeY_FactoryTrimValue(uint8_t){};
float GyroscopeZ_FactoryTrimValue(uint8_t){};

float AccelerometerX_FactoryTrimValue(uint8_t){};
float AccelerometerY_FactoryTrimValue(uint8_t){};
float AccelerometerZ_FactoryTrimValue(uint8_t){};

int16_t SelfTestResponse(int16_t, int16_t){};
float FactoryTrimChange(float, int16_t){};

};  // namespace mpu60X0
// namespace mpu60X0

#endif  // DRIVERS_MPU60X0_UTILITY_FUNCTIONS_H_
