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
 * @date 05/04/2014
 * @author Jan Issac (jan.issac@gmail.com)
 * Max-Planck-Institute for Intelligent Systems, University of Southern California (USC),
 *   Karlsruhe Institute of Technology (KIT)
 */


#ifndef ONI_VICON_PLAYER_VICON_PLAYER_HPP
#define ONI_VICON_PLAYER_VICON_PLAYER_HPP

#include <depth_sensor_vicon_calibration/transform.hpp>

// C++/STD
#include <vector>
#include <map>

// ros
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Scalar.h>
#include <tf/LinearMath/Transform.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace oni_vicon_player
{
    class ViconPlayer
    {
    public:
        enum
        {
            CLOSEST,
            INTERPOLATE
        };

        struct RawRecord
        {
            int64_t vicon_time;
            int64_t vicon_frame;
            int64_t depth_sensor_time;
            int64_t depth_sensor_frame;
            int64_t vicon_frame_id;
            float translation_x;
            float translation_y;
            float translation_z;
            float orientation_w;
            float orientation_x;
            float orientation_y;
            float orientation_z;
        };

        struct PoseRecord
        {
            ros::Time stamp;
            tf::Pose pose;
        };

    private:
        typedef depth_sensor_vicon_calibration::Transformer CalibrationTransform;

    public:
        ViconPlayer(ros::NodeHandle& node_handle);
        virtual ~ViconPlayer();

        bool load(const std::string &source_file,
                  const CalibrationTransform& calibration_transform,
                  boost::function<void(int64_t)> updateCb);

        const PoseRecord& poseRecord(int64_t frame);

        void publish(const PoseRecord& pose_record, sensor_msgs::ImagePtr corresponding_image, const std::string &object_display);

        int64_t countViconFrames();
        int64_t countDepthSensorFrames();

    private:        
        RawRecord closestViconFrame(const RawRecord& oni_frame);

        std::vector<RawRecord> raw_data_;
        std::map<int64_t, PoseRecord> data_;

        ros::Publisher object_publisher_;

        int64_t start_offset_;
        bool time_is_in_ms_;
    };
}

#endif
