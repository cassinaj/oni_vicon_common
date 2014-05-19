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

#ifndef ONI_VICON_COMMON_CALIBRATION_GLOBAL_CALIBRATION_HPP
#define ONI_VICON_COMMON_CALIBRATION_GLOBAL_CALIBRATION_HPP

#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include "oni_vicon_common/types.hpp"

namespace oni_vicon
{
    class GlobalCalibration
    {
    public:
        /**
         * @brief GlobalCalibration Constructs an identity calibration
         */
        GlobalCalibration();

        /**
         * @brief calibrate Calibrates the Vicon and the Camera.
         *
         * Computes the transform between the Vicon global to camera global frame given a vicon
         * reference frame and a camera reference frame. Both reference frames must be identical.
         * This can be achieved using a special calibration object. The Local frame of the model
         * of this object must exactly match the local frame defined in the vicon system.
         *
         * The Vicon Tracker software is able place the object local frame according to the marker
         * positions in the object model frame. This approach does not achive a highly accurate
         * calibration but it works fine within the calibration region.
         *
         * @param camera_info
         * @param vicon_reference_frame
         * @param depth_sensor_reference_frame
         */
        void calibrate(sensor_msgs::CameraInfoConstPtr camera_info,
                       const tf::Pose& vicon_reference_frame,
                       const tf::Pose& depth_sensor_reference_frame);

        /**
         * Transforms a pose given in the vicon global frame to a pose in the camera global frame
         *
         * @param vicon_pose
         * @return
         */
        tf::Pose transformViconToCamera(const tf::Pose& vicon_pose) const;

        /**
         * Transforms a pose given in the vicon global frame to a pose in the camera global frame
         *
         * @param [in]  vicon_pose      Pose in the vicon global frame
         * @param [out] camera_pose     Resulting pose in camera global frame
         */
        void transformViconToCamera(const tf::Pose& vicon_pose, tf::Pose& camera_pose) const;

        /**
         * @brief setIdentity sets the vicon to camera transform to identity
         */
        void setIdentity();

        /**
         * @return Camera intrinsics
         */
        const CameraIntrinsics& cameraIntrinsics() const;

        /**
         * @return Global transform from vicon to the camera
         */
        const tf::Transform& viconToCameraTransform() const;

        /**
         * @brief cameraIntrinsics sets camera_intrinsics
         *
         * @param [in] camera_intrinsics    Camera intrisics
         */
        void cameraIntrinsics(const CameraIntrinsics& camera_intrinsics);

        /**
         * @brief viconToCameraTransform sets vicon_to_camera_transform
         *
         * @param [in] vicon_to_camera_transform    Global transformation
         */
        void viconToCameraTransform(const tf::Transform& vicon_to_camera_transform);

    private:
        /**
         * @brief camera_intrinsics_ Camera intrinsic parameters used to capture the depth images
         */
        CameraIntrinsics camera_intrinsics_;

        /**
         * @brief vicon_to_camera_transform_ Transformation from vicon coordinates into camera
         * coordinates
         */
        tf::Transform vicon_to_camera_transform_;
    };
}

#endif
