/*
Trajectory
Copyright (C) 2019 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef PLANNING_TRAJECTORY_H
#define PLANNING_TRAJECTORY_H

#include <iostream>
#include <ctime>

#include <math.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <planning_representations/FlatState.h>

namespace planner {

class Trajectory
{
public:

  Trajectory() {};
  virtual ~Trajectory() {};

  virtual void setStartTime(const double t0) { t0_ = t0; };
  virtual void setEndTime(const double tf) { tf_ = tf; };

  virtual double tstart() const { return t0_; };
  virtual double tfinal() const { return tf_; };
  virtual double duration() const { return tfinal() - tstart(); };

  virtual std::vector<FlatState> samplePath(double dt, double tstart, double tfinal) const = 0;
  virtual std::vector<FlatState> samplePath(double dt) const { return samplePath(dt, tstart(), tfinal()); };

  virtual std::vector<FlatState> samplePathInLocalFrame(double dt, double tstart, double tfinal) const = 0;
  virtual std::vector<FlatState> samplePathInLocalFrame(double dt) const { return samplePath(dt, tstart(), tfinal()); };

  virtual FlatState getWorldPoseAtTime(double t) const = 0;
  virtual FlatState getLocalPoseAtTime(double t) const = 0;

  virtual FlatState getInitialWorldPose() const
    { return getWorldPoseAtTime(tstart()); };
  virtual FlatState getFinalWorldPose() const
    { return getWorldPoseAtTime(tfinal()); };
  virtual FlatState getInitialLocalPose() const
    { return getLocalPoseAtTime(tstart()); };
  virtual FlatState getFinalLocalPose() const
    { return getLocalPoseAtTime(tfinal()); };

private:
  double t0_;
  double tf_;


};

} // namespace planner

#endif
