/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Max-Planck-Institute for Intelligent Systems,
 *                     University of Southern California,
 *                     Karlsruhe Institute of Technology
 *    Jan Issac (jan.issac@gmail.com)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @date 05/15/2014
 * @author Jan Issac (jan.issac@gmail.com)
 * Max-Planck-Institute for Intelligent Systems, University of Southern California (USC),
 *   Karlsruhe Institute of Technology (KIT)
 */


#include "oni_vicon_common/type_conversion.hpp"
#include "oni_vicon_common/global_calibration.hpp"

using namespace oni_vicon;

GlobalCalibration::GlobalCalibration()
{
    setIdentity();

    camera_intrinsics_.f = 0.;
    camera_intrinsics_.cx = 0.;
    camera_intrinsics_.cx = 0.;
}

void GlobalCalibration::calibrate(sensor_msgs::CameraInfoConstPtr camera_info,
                                  const tf::Pose& vicon_reference_frame,
                                  const tf::Pose& depth_sensor_reference_frame)
{
    // set camera intrinsics
    toCameraIntrinsics(camera_info, camera_intrinsics_);

    // Assuming the local frame in both systems is exactley the same, the global calibration
    // boils down to transforming a reference pose from vicon to depth sensor. This task requires
    // a special calibration object which it's local frame is the same in both systems
    vicon_to_camera_transform_.mult(depth_sensor_reference_frame, vicon_reference_frame.inverse());
}

tf::Pose GlobalCalibration::transformViconToCamera(const tf::Pose& vicon_pose) const
{
    tf::Pose camera_pose;
    transformViconToCamera(vicon_pose, camera_pose);

    return camera_pose;
}

void GlobalCalibration::transformViconToCamera(const tf::Pose& vicon_pose,
                                                          tf::Pose& camera_pose) const
{
    camera_pose.mult(vicon_to_camera_transform_, vicon_pose);
}

void GlobalCalibration::setIdentity()
{
    vicon_to_camera_transform_.setIdentity();
}

const CameraIntrinsics& GlobalCalibration::cameraIntrinsics() const
{
    return camera_intrinsics_;
}

const tf::Transform& GlobalCalibration::viconToCameraTransform() const
{
    return vicon_to_camera_transform_;
}

void GlobalCalibration::cameraIntrinsics(const CameraIntrinsics& camera_intrinsics)
{
    camera_intrinsics_ = camera_intrinsics;
}

void GlobalCalibration::viconToCameraTransform(const tf::Transform& vicon_to_camera_transform)
{
    vicon_to_camera_transform_ = vicon_to_camera_transform;
}
