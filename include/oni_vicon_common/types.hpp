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

#ifndef ONI_VICON_COMMON_CAMERA_INTRINSICS_HPP
#define ONI_VICON_COMMON_CAMERA_INTRINSICS_HPP

#include <ni/XnCppWrapper.h>

#include <limits>

namespace oni_vicon
{
    enum Unit
    {
        Millimeter,
        Centimeter,
        Meter
    };

    /**
     * @brief Minimal camera intrinsic parameters. This is mainly used to construct 3d point clouds.
     */
    struct CameraIntrinsics
    {
        /**
         * @brief f focal length
         */
        double f;

        /**
         * @brief cx x-coordinate component of the principal point
         */
        double cx;

        /**
         * @brief cy y-coordinate component of the principal point
         */
        double cy;
    };

    /**
     * @brief simple low cost vector (alternatives, tf::Point/tf::Vector3d)
     */
    struct Point3d
    {
        float x;
        float y;
        float z;
    };

    /**
     * @brief Basic generator value used in image conversion
     */
    struct GeneratorProperties
    {
        GeneratorProperties()
        {
            value_for_no_sample = std::numeric_limits<float>::quiet_NaN();
            value_for_shadow = std::numeric_limits<float>::quiet_NaN();
            value_for_zero = std::numeric_limits<float>::quiet_NaN();
        }

        /**
         * @brief no_sample_value is the value defined by the generator on a missing sample
         */
        XnUInt64 no_sample_value;

        /**
         * @brief shadow_value is the value defined by the generator for shadow area
         */
        XnUInt64 shadow_value;

        /**
         * @brief value_for_no_sample is the value which a <no_sample_value> will be mapped onto
         */
        float value_for_no_sample;

        /**
         * @brief value_for_shadow is the value which a <shadow_value> will be mapped onto
         */
        float value_for_shadow;

        /**
         * @brief value_for_zero is the value which <0> will be mapped onto
         */
        float value_for_zero;
    };
}

#endif
