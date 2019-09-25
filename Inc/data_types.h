/**
 * @file data_types.h
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

#ifndef DRIVERS_MPU60X0_DATA_TYPES_H_
#define DRIVERS_MPU60X0_DATA_TYPES_H_

#include <cinttypes>

namespace mpu60X0 {

struct TriAxialData {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct BiAxialData {
  int16_t a;
  int16_t b;
};

};  // namespace mpu60X0
// namespace mpu60X0
#endif  // DRIVERS_MPU60X0_DATA_TYPES_H_
