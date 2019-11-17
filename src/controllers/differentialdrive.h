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

#if !defined(DIFFERENTIAL_DRIVE_CONTROLLER_H)
#define DIFFERENTIAL_DRIVE_CONTROLLER_H

namespace Locomotion {

template<typename T>
class _DifferentialDriveController {
public:
	_DifferentialDriveController(T wheelBase) : _wheelBase(wheelBase) {}
protected:
	T wheelBase;

	virtual void setSpeed(T a, T b) = 0;

	/// power - differential drive train power
	/// w - rotational power (normalized wL)
	/// all control parameters range -1.0 ... +1.0
	void setDifferentialSpeed(T power, T w)
	{
		T a = (2.0 * power + w) / 2.0;
		T b = (2.0 * power - w) / 2.0;
		setSpeed(a, b);
	}
};

};

#endif
