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

#include "oni_vicon_common/local_calibration.hpp"

using namespace oni_vicon;

LocalCalibration::LocalCalibration()
{
    setIdentity();
}

void LocalCalibration::calibrate(const tf::Pose& vicon_reference_frame,
                                 const tf::Pose& depth_sensor_reference_frame,
                                 const std::string& object_mesh,
                                 const std::string& object_mesh_display)
{
    vicon_local_to_camera_local_.mult(depth_sensor_reference_frame.inverse(),
                                      vicon_reference_frame);

    vicon_local_to_camera_local_ = vicon_local_to_camera_local_.inverse();

    object_mesh_ = object_mesh;
    object_mesh_display_ = object_mesh_display;
}

const tf::Transform& LocalCalibration::viconLocalToCameraLocal() const
{
    return vicon_local_to_camera_local_;
}

void LocalCalibration::viconLocalToCameraLocal(
        const tf::Transform& vicon_local_to_camera_local) const
{
    vicon_local_to_camera_local_ = vicon_local_to_camera_local;
}

tf::Pose LocalCalibration::transformViconLocalToCameraLocal(const tf::Pose& vicon_local) const
{
    tf::Pose camera_local;
    transformViconLocalToCameraLocal(vicon_local, camera_local);

    return camera_local;
}

void LocalCalibration::transformViconLocalToCameraLocal(const tf::Pose& vicon_local,
                                                        tf::Pose& camera_local) const
{
    camera_local.mult(vicon_local, vicon_local_to_camera_local_);
}

void LocalCalibration::setIdentity()
{
    vicon_local_to_camera_local_.setIdentity();
}

std::string LocalCalibration::object() const
{
    return object_mesh_;
}

std::string LocalCalibration::objectDisplay() const
{
    return object_mesh_display_;
}

std::string LocalCalibration::object(const std::string &object_mesh)
{
    object_mesh_ = object_mesh;
}

std::string LocalCalibration::objectDisplay(const std::string &object_mesh_display)
{
    object_mesh_display_ = object_mesh_display;
}

