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

#ifndef ONI_VICON_COMMON_CALIBRATION_LOCAL_CALIBRATION_HPP
#define ONI_VICON_COMMON_CALIBRATION_LOCAL_CALIBRATION_HPP

#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include "oni_vicon_common/types.hpp"

namespace oni_vicon
{
    class LocalCalibration
    {
    public:
        /**
         * @brief LocalCalibration
         */
        LocalCalibration();

        /**
         * Calibrates the local transform from the local frame defined by the Vicon system to the
         * local frame defined by the object model used by the camera
         *
         * @param [in]  vicon_reference_frame       Vicon local reference frame in camera global
         *                                          frame.
         * @param [in]  camera_reference_frame      Object local reference frame in camera global
         * @param [in]  object_mesh                 Used object mesh model to calibrate and track
         * @param [in]  object_mesh_display         Used object mesh model for display purposes
         */
        void calibrate(const tf::Pose& vicon_reference_frame,
                       const tf::Pose& camera_reference_frame,
                       const std::string &object_mesh,
                       const std::string &object_mesh_display);

        /**
         * Transform a pose given in the local frame defined by the Vicon system into the local
         * frame defined by the object model used by the camera.
         *
         * @param [in]  vicon_local     Pose in the Vicon local frame
         *
         * @return Resulting pose in the object local frame
         */
        tf::Pose transformViconLocalToCameraLocal(const tf::Pose& vicon_local) const;

        /**
         * Transform a pose given in the local frame defined by the Vicon system into the local
         * frame defined by the object model used by the camera.
         *
         * @param [in]  vicon_local     Pose in the Vicon local frame
         * @param [out] camera_local    Resulting pose in the object local frame
         */
        void transformViconLocalToCameraLocal(const tf::Pose& vicon_local,
                                              tf::Pose &camera_local) const;

        /**
         * Sets the local transform to identity. That is both systems, Vicon and the camera have the
         * same local frame within the object
         */
        void setIdentity();

        /**
         * @return the transformation from the vicon local frame to the local frame define by the
         * object model
         */
        const tf::Transform& viconLocalToCameraLocal() const;

        /**
         * Sets the transformation from the vicon local frame to the local frame define by the
         * object model
         *
         * @param [in] vicon_local_to_camera_local  local transform
         */
        void viconLocalToCameraLocal(const tf::Transform& vicon_local_to_camera_local) const;

        /**
         * Sets the used object mesh Wavefront obj file to calibrate. This shall be used for pose estimation
         * and tracking. This has usually lower details.
         *
         * @param [in]  object_mesh   Used object mesh for calibration and tracking.
         */
        void object(const std::string& object_mesh);

        /**
         * Sets the used object mesh Wavefront obj file to calibrate. This shall be used for pose estimation
         * and tracking. This has usually lower details.
         *
         * @param [in]  object_mesh_display     Used object mesh for display rendering only
         */
        void objectDisplay(const std::string& object_mesh_display);

        /**
         * @return Used object mesh Wavefront obj file to calibrate. This shall be used for pose estimation
         * and tracking. This has usually lower details.
         */
        std::string object() const;

        /**
         * @return * Used object mesh Wavefront obj file for display rendering. This is usually higher in
         * details.
         */
        std::string objectDisplay() const;

    private:
        /**
         * Transformation from the local frame defined in the object model (used by the camera)
         * to the local frame defined by the vicon system. This is due to the fact that these two
         * systems never use the same local frame within the object.
         */
        tf::Transform vicon_local_to_camera_local_;

        /**
         * Used object mesh Wavefront obj file to calibrate. This shall be used for pose estimation
         * and tracking. This has usually lower details.
         */
        std::string object_mesh_;

        /**
         * Used object mesh Wavefront obj file for display rendering. This is usually higher in
         * details.
         */
        std::string object_mesh_display_;
    };
}

#endif
