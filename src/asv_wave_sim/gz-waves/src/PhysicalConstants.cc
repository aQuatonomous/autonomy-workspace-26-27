// Copyright (C) 2019  Rhys Mainwaring
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "gz/waves/PhysicalConstants.hh"

#include <cstdlib>

namespace gz
{
namespace waves
{

//////////////////////////////////////////////////
// PhysicalConstants

double PhysicalConstants::Gravity()
{
  // return -9.8;
  return -9.81;
}

double PhysicalConstants::G()
{
  return 6.67408E-11;
}

double PhysicalConstants::WaterDensity()
{
  // GZ_WAVES_WATER_DENSITY overrides the seawater default, e.g. 1000 for a
  // freshwater venue.
  static const double density = []()
  {
    const char *env = std::getenv("GZ_WAVES_WATER_DENSITY");
    if (env != nullptr)
    {
      char *end = nullptr;
      const double value = std::strtod(env, &end);
      if (end != env && value > 0.0)
        return value;
    }
    return 1025.0;
  }();
  return density;
}

double PhysicalConstants::WaterKinematicViscosity()
{
  return 1.0533E-6;
}

}  // namespace waves
}  // namespace gz
