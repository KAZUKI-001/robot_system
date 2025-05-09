// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef NEDO_MCL__PARTICLE_HPP_
#define NEDO_MCL__PARTICLE_HPP_

#include "nedo_mcl/LikelihoodFieldMap.hpp"
#include "nedo_mcl/Pose.hpp"

namespace nedo_mcl
{
class Particle
{
      public:
	Particle(double x, double y, double t, double w);
	Particle(const Particle & other) = default;

	Particle & operator=(const Particle & other)
	{
		if (this != &other) {
			this->p_ = other.p_;
			this->w_ = other.w_;
		}
		return *this;
	}

	double likelihood(LikelihoodFieldMap * map, Scan & scan);
	bool wallConflict(LikelihoodFieldMap * map, Scan & scan, double threshold, bool replace);
	Pose p_;
	double w_;

      private:
	bool isPenetrating(
	  double ox, double oy, double range, uint16_t direction, LikelihoodFieldMap * map,
	  double & hit_lx, double & hit_ly);

	bool checkWallConflict(
	  LikelihoodFieldMap * map, double ox, double oy, double range, uint16_t direction,
	  double threshold, bool replace);

	void sensorReset(
	  double ox, double oy, double range1, uint16_t direction1, double hit_lx1, double hit_ly1,
	  double range2, uint16_t direction2, double hit_lx2, double hit_ly2);
};

}  // namespace nedo_mcl

#endif	// NEDO_MCL__PARTICLE_HPP_
