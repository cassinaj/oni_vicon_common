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
 * @date 05/14/2014
 * @author Jan Issac (jan.issac@gmail.com)
 * Max-Planck-Institute for Intelligent Systems, University of Southern California (USC),
 *   Karlsruhe Institute of Technology (KIT)
 */

#include "oni_vicon_common/transformer.hpp"

// c++/std
#include <iostream>
#include <fstream>

// boost
#include <boost/filesystem.hpp>

// ros
#if ROS_VERSION_MINIMUM(1, 3, 0)
#include <sensor_msgs/distortion_models.h>
#endif

using namespace oni_vicon;

Transformer::Transformer()
{
    global_calibration_.setIdentity();
    local_calibration_.setIdentity();
}

Transformer::Transformer(const geometry_msgs::Pose& global_calib_transform,
                         const geometry_msgs::Pose& local_calib_transform)
{
    // toTfTransform(global_calib_transform, global_transform_);
    // toTfTransform(local_calib_transform, local_transform_);
}

Transformer::~Transformer()
{
}

tf::Pose Transformer::viconPoseToCameraPose(const tf::Pose& vicon) const
{
    tf::Pose camera_pose = local_calibration_.transformViconLocalToCameraLocal(vicon);
    camera_pose = global_calibration_.transformViconToCamera(camera_pose);

    return camera_pose;
}

void Transformer::calibrateGlobally(sensor_msgs::CameraInfoConstPtr camera_info,
                                    const tf::Pose& vicon_reference_frame,
                                    const tf::Pose& depth_sensor_reference_frame)
{
    global_calibration_.calibrate(camera_info, vicon_reference_frame, depth_sensor_reference_frame);

    // reset local calibration
    local_calibration_.setIdentity();
}

void Transformer::calibrateLocally(const tf::Pose& vicon_reference_frame,
                                            const tf::Pose& depth_sensor_reference_frame,
                                            const std::string object,
                                            const std::string object_display)
{
    // This assumes a given global calibration
    local_calibration_.calibrate(global_calibration_.viconToCameraTransform(vicon_reference_frame),
                                 depth_sensor_reference_frame,
                                 object,
                                 object_display);
}

const CameraIntrinsics& Transformer::cameraIntrinsics() const
{
    return global_calibration_.cameraIntrinsics();
}


std::string Transformer::object() const
{
    return local_calibration_.object();
}

std::string Transformer::objectDisplay() const
{
    return local_calibration_.objectDisplay();
}
