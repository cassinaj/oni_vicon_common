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

#ifndef ONI_VICON_COMMON_CALIBRATION_WRITER_HPP
#define ONI_VICON_COMMON_CALIBRATION_WRITER_HPP

#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>

#include <yaml-cpp/yaml.h>

#include "oni_vicon_common/types.hpp"
#include "oni_vicon_common/local_calibration.hpp"
#include "oni_vicon_common/global_calibration.hpp"

namespace oni_vicon
{
    class CalibrationWriter
    {
    public:
        /**
         * @brief saveLocalCalibration
         * @param destination
         * @param local_calibration
         * @return
         */
        bool saveLocalCalibration(const std::string& destination,
                                  const LocalCalibration &local_calibration);

        /**
         * @brief saveLocalCalibration
         * @param destination
         * @param global_calibration
         * @return
         */
        bool saveGlobalCalibration(const std::string& destination,
                                  const GlobalCalibration &global_calibration);
    private:
        /**
         * @brief globalCalibrationToYaml
         * @param global_calibration
         * @param doc
         */
        void globalCalibrationToYaml(const GlobalCalibration& global_calibration,
                                     YAML::Emitter &doc);

        /**
         * @brief localCalibrationToYaml
         * @param local_calibration
         * @param doc
         */
        void localCalibrationToYaml(const LocalCalibration& local_calibration,
                                    YAML::Emitter &doc);

        /**
         * @brief saveCalibration
         * @param destination
         * @param doc
         * @return
         */
        bool saveCalibration(const std::string& destination,
                             const YAML::Emitter &doc);
    };

    inline YAML::Emitter& operator << (YAML::Emitter& doc, const tf::Vector3& translation)
    {
       doc << YAML::BeginMap
           << YAML::Key << "x" << YAML::Value << translation.getX()
           << YAML::Key << "y" << YAML::Value << translation.getY()
           << YAML::Key << "z" << YAML::Value << translation.getZ()
           << YAML::EndMap;

       return doc;
    }

    inline YAML::Emitter& operator <<(YAML::Emitter& doc, const tf::Quaternion &rotation)
    {
        doc << YAML::BeginMap
            << YAML::Key << "w" << YAML::Value << rotation.getW()
            << YAML::Key << "x" << YAML::Value << rotation.getX()
            << YAML::Key << "y" << YAML::Value << rotation.getY()
            << YAML::Key << "z" << YAML::Value << rotation.getZ()
            << YAML::EndMap;

        return doc;
    }

    inline YAML::Emitter& operator <<(YAML::Emitter& doc, const tf::Transform& transform)
    {
        doc << YAML::BeginMap
            << YAML::Key << "origin" << YAML::Value << transform.getOrigin()
            << YAML::Key << "orientation" << YAML::Value << transform.getRotation()
            << YAML::EndMap;

        return doc;
    }

    inline YAML::Emitter& operator <<(YAML::Emitter& doc, const CameraIntrinsics& camera_intrinsics)
    {
        doc << YAML::BeginMap
            << YAML::Key << "f" << YAML::Value << camera_intrinsics.f
            << YAML::Key << "cx" << YAML::Value << camera_intrinsics.cx
            << YAML::Key << "cy" << YAML::Value << camera_intrinsics.cy
            << YAML::EndMap;

        return doc;
    }
}

#endif
