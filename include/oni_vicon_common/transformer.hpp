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

#include <ros/ros.h>
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
        /**
         * @brief Transformer identity constructor
         */
        Transformer();

        /**
         * @brief viconPoseToCameraPose Transforms a pose fron the Vicon frame to the camera frame
         *
         * @param [in] vicon_pose       Pose in the Vicon global frame
         *
         * @return Pose in the camera global frame
         */
        tf::Pose viconPoseToCameraPose(const tf::Pose& vicon_pose) const;

        /**
         * Computes global calibration between Vicon and the camera
         *
         * @param camera_info               Camera info used to capture the depth images
         * @param vicon_reference_frame     Vicon local frame expressed in Vicon frame
         * @param camera_reference_frame    Object local frame expressed in camera frame
         */
        void calibrateGlobally(sensor_msgs::CameraInfoConstPtr camera_info,
                               const tf::Pose& vicon_reference_frame,
                               const tf::Pose& camera_reference_frame);

        /**
         * Computes the transform within the object
         *
         * @param vicon_reference_frame     Vicon local frame expressed in Vicon frame
         * @param camera_reference_frame    Object local frame expressed in camera frame
         * @param object                    Object Wavefront mesh file name used to calibrate
         * @param object_display            Object Wavefront mesh file name used to display object
         */
        void calibrateLocally(const tf::Pose& vicon_reference_frame,
                              const tf::Pose& camera_reference_frame,
                              const std::string& object,
                              const std::string& object_display);

        /**
         * @return Global calibration
         */
        const GlobalCalibration& globalCalibration() const;

        /**
         * @return Local calibration
         */
        const LocalCalibration& localCalibration() const;

        /**
         * @brief globalCalibration Sets the global calibration.
         *
         * @param [in] global_calibration
         */
        void globalCalibration(const GlobalCalibration& global_calibration);

        /**
         * @brief localCalibration Sets the local calibration
         *
         * @param [in] local_calibration
         */
        void localCalibration(const LocalCalibration& local_calibration);

        /**
         * @return Camera intrinsics used to capture the depth images
         */
        const CameraIntrinsics& cameraIntrinsics() const;

        /**
         * @return Object Wavefront mesh file name used to calibrate
         */
        std::string object() const;

        /**
         * @return Object Wavefront mesh file name used to display object
         */
        std::string objectDisplay() const;

    private:
        GlobalCalibration global_calibration_;
        LocalCalibration local_calibration_;
    };
}

#endif
