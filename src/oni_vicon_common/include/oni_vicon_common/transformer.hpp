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

#ifndef ONI_VICON_COMMON_TRANSFORMER_HPP
#define ONI_VICON_COMMON_TRANSFORMER_HPP

#include <boost/shared_ptr.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

#include "oni_vicon_common/types.hpp"
#include "oni_vicon_common/local_calibration.hpp"
#include "oni_vicon_common/global_calibration.hpp"

namespace oni_vicon
{
    class Transformer
    {
    public:
        typedef boost::shared_ptr<Transformer> Ptr;

    public:
        Transformer();
        Transformer(const geometry_msgs::Pose& global_calib_transform,
                    const geometry_msgs::Pose& local_calib_transform);
        virtual ~Transformer();

        void calibrateGlobally(sensor_msgs::CameraInfoConstPtr camera_info,
                               const tf::Pose& vicon_reference_frame,
                               const tf::Pose& depth_sensor_reference_frame);
        void calibrateLocally(const tf::Pose& vicon_reference_frame,
                              const tf::Pose& depth_sensor_reference_frame,
                              const std::string object,
                              const std::string object_display);

        tf::Transform globalTransform() const;
        tf::Transform localTransform() const;
        const CameraIntrinsics& cameraIntrinsics() const;
        tf::Pose viconPoseToCameraPose(const tf::Pose& vicon) const;

        std::string object() const;
        std::string objectDisplay() const;

    public:
        static void toMsgPose(const tf::Pose& tf_pose, geometry_msgs::Pose& msg_pose);
        static void toTfPose(const geometry_msgs::Pose& msg_pose, tf::Pose& tf_pose);
        static void toTfTransform(const geometry_msgs::Pose& msg_pose, tf::Transform &tf_trasnform);
        static void toCameraIntrinsics(sensor_msgs::CameraInfoConstPtr msg_camera_info,
                                       CameraIntrinsics& camera_intrinsics);
        static void toCameraInfo(const CameraIntrinsics& camera_intrinsics,
                                 sensor_msgs::CameraInfoPtr msg_camera_info);
        static geometry_msgs::Pose toMsgPose(const tf::Pose& tf_pose);
        static tf::Pose toTfPose(const geometry_msgs::Pose& msg_pose);
        static tf::Transform toTfTransform(const geometry_msgs::Pose& msg_pose);
        static CameraIntrinsics toCameraIntrinsics(sensor_msgs::CameraInfoConstPtr msg_camera_info);
        static sensor_msgs::CameraInfoPtr toCameraInfo(const CameraIntrinsics& camera_intrinsics);

    private:


    private:
        GlobalCalibration global_calibration_;
        LocalCalibration local_calibration_;
    };
}

#endif
