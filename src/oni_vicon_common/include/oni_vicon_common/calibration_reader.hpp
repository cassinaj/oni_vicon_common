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

#ifndef ONI_VICON_COMMON_CALIBRATION_READER_HPP
#define ONI_VICON_COMMON_CALIBRATION_READER_HPP

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
    class CalibrationReader
    {
    public:
        /**
         * @brief loadGlobalCalibration
         * @param source
         * @param global_calibration
         * @return
         */
        bool loadGlobalCalibration(const std::string& source,
                                   GlobalCalibration& global_calibration);

        /**
         * @brief loadLocalCalibration
         * @param source
         * @param local_calibration
         * @return
         */
        bool loadLocalCalibration(const std::string& source,
                                  LocalCalibration& local_calibration);
    private:
        /**
         * @brief globalCalibrationFromYaml
         * @param doc
         * @param global_calibration
         */
        void globalCalibrationFromYaml(const YAML::Node& doc,
                                       GlobalCalibration& global_calibration);

        /**
         * @brief localCalibrationFromYaml
         * @param doc
         * @param local_calibration
         */
        void localCalibrationFromYaml(const YAML::Node& doc,
                                      LocalCalibration& local_calibration);

        /**
         * @brief loadCalibration
         * @param source
         * @param doc
         * @return
         */
        bool loadCalibration(const std::string& source, YAML::Node &doc);
    };

    inline void operator >>(const YAML::Node &node, tf::Vector3& translation)
    {
        tfScalar x, y, z;
        node["x"] >> x;
        node["y"] >> y;
        node["z"] >> z;
        translation.setValue(x, y, z);
    }

    inline void operator >>(const YAML::Node &node, tf::Quaternion &rotation)
    {
        tfScalar w, x, y, z;
        node["w"] >> w;
        node["x"] >> x;
        node["y"] >> y;
        node["z"] >> z;
        rotation.setValue(x, y, z, w);
    }

    inline void operator >>(const YAML::Node& node, tf::Transform& transform)
    {
        tf::Vector3 origin;
        tf::Quaternion rotation;
        node["origin"] >> origin;
        node["orientation"] >> rotation;
        transform.setOrigin(origin);
        transform.setRotation(rotation);
    }

    inline void operator >>(const YAML::Node& node, CameraIntrinsics& camera_intrinsics)
    {
        node["f"] >> camera_intrinsics.f;
        node["cx"] >> camera_intrinsics.cx;
        node["cy"] >> camera_intrinsics.cy;
    }
}

#endif
