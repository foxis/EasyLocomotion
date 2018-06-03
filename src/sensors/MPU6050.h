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
 */

#if !defined(MPU6050_H)
#define MPU6050_H

#include <Wire.h>
#define SENSORS_MPU6050_ATTACHED
#include <Sensors.h>
#include "../TwoWireDevice.h"
#include "../MadgwickAHRS/MadgwickAHRS.h"

class MPU6050 : public TwoWireDevice {
public:
	MPU6050()
}

#endif
