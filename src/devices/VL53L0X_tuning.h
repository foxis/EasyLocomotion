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

#include "../hal/readwritemixin.h"

#if !defined(VLX53L0X_TUNING_H)
#define VLX53L0X_TUNING_H

namespace Locomotion {

const ReadWriteMixin::calib_data8_t VL53L0X_tuning_data[] PROGMEM = {
	{0xFF, 0x01},
  {0x00, 0x00},

  {0xFF, 0x00},
  {0x09, 0x00},
  {0x10, 0x00},
  {0x11, 0x00},

  {0x24, 0x01},
  {0x25, 0xFF},
  {0x75, 0x00},

  {0xFF, 0x01},
  {0x4E, 0x2C},
  {0x48, 0x00},
  {0x30, 0x20},

  {0xFF, 0x00},
  {0x30, 0x09},
  {0x54, 0x00},
  {0x31, 0x04},
  {0x32, 0x03},
  {0x40, 0x83},
  {0x46, 0x25},
  {0x60, 0x00},
  {0x27, 0x00},
  {0x50, 0x06},
  {0x51, 0x00},
  {0x52, 0x96},
  {0x56, 0x08},
  {0x57, 0x30},
  {0x61, 0x00},
  {0x62, 0x00},
  {0x64, 0x00},
  {0x65, 0x00},
  {0x66, 0xA0},

  {0xFF, 0x01},
  {0x22, 0x32},
  {0x47, 0x14},
  {0x49, 0xFF},
  {0x4A, 0x00},

  {0xFF, 0x00},
  {0x7A, 0x0A},
  {0x7B, 0x00},
  {0x78, 0x21},

  {0xFF, 0x01},
  {0x23, 0x34},
  {0x42, 0x00},
  {0x44, 0xFF},
  {0x45, 0x26},
  {0x46, 0x05},
  {0x40, 0x40},
  {0x0E, 0x06},
  {0x20, 0x1A},
  {0x43, 0x40},

  {0xFF, 0x00},
  {0x34, 0x03},
  {0x35, 0x44},

  {0xFF, 0x01},
  {0x31, 0x04},
  {0x4B, 0x09},
  {0x4C, 0x05},
  {0x4D, 0x04},

  {0xFF, 0x00},
  {0x44, 0x00},
  {0x45, 0x20},
  {0x47, 0x08},
  {0x48, 0x28},
  {0x67, 0x00},
  {0x70, 0x04},
  {0x71, 0x01},
  {0x72, 0xFE},
  {0x76, 0x00},
  {0x77, 0x00},

  {0xFF, 0x01},
  {0x0D, 0x01},

  {0xFF, 0x00},
  {0x80, 0x01},
  {0x01, 0xF8},

  {0xFF, 0x01},
  {0x8E, 0x01},
  {0x00, 0x01},
  {0xFF, 0x00},
  {0x80, 0x00},
};

}

#endif
