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

#include <string>

#include "oni_vicon_player/oni_player.hpp"

#define CHECK_RC(rc)                                          \
if (rc != XN_STATUS_OK)                                       \
{                                                             \
    ROS_ERROR("%s failed: %s\n", #rc, xnGetStatusString(rc)); \
    return false;                                             \
}

using namespace depth_sensor_vicon_calibration;
using namespace oni_vicon_player;

OniPlayer::OniPlayer(const ros::NodeHandle& node_handle,
                     const std::string& depth_frame_id,
                     const std::string& camera_info_topic,
                     const std::string& point_cloud_topic):
    node_handle_(node_handle),
    image_transport_(node_handle),
    depth_frame_id_(depth_frame_id),
    camera_info_topic_(camera_info_topic),
    point_cloud_topic_(point_cloud_topic)
{
    pub_depth_image_ = image_transport_.advertise("depth/image", 100);
    pub_depth_info_ = node_handle_.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 100);
    pub_point_cloud_ = node_handle_.advertise<sensor_msgs::PointCloud2>("depth/points", 100);

    camera_intrinsics_.cx = 1;
    camera_intrinsics_.cy = 1;
    camera_intrinsics_.f = 1;
}

OniPlayer::~OniPlayer()
{

}

bool OniPlayer::open(const std::string& source_file, const CameraIntrinsics& camera_intrinsics)
{
    ROS_INFO("Opening ONI");

    CHECK_RC(context_.Init());
    CHECK_RC(context_.OpenFileRecording(source_file.c_str(), player_));
    CHECK_RC(depth_generator_.Create(context_));
    CHECK_RC(player_.GetNumFrames(depth_generator_.GetName(), frames_));
    CHECK_RC(depth_generator_.GetIntProperty((XnChar*)"NoSampleValue", no_sample_value_));
    CHECK_RC(depth_generator_.GetIntProperty((XnChar*)"ShadowValue", shadow_value_));
    CHECK_RC(context_.StartGeneratingAll());
    CHECK_RC(player_.SetPlaybackSpeed(1.0));
    CHECK_RC(player_.SetRepeat(true));

    ROS_INFO("ONI file <%s> loaded", source_file.c_str());

    camera_intrinsics_ = camera_intrinsics;

    return true;
}

bool OniPlayer::process(ros::Time time)
{
    boost::mutex::scoped_lock lock(capture_mutex_);

    CHECK_RC(depth_generator_.WaitAndUpdateData());

    if (!depth_generator_.IsDataNew()) // not sure yet if this required
    {
        return (current_frame_ != frames_);
    }

    // get depth frame
    depth_generator_.GetMetaData(depth_meta_data_);

    current_frame_ = depth_meta_data_.FrameID();

    // convert and publish depth image and cloud point as needed
    sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image>();
    toMsgImage(depth_meta_data_, depth_msg);
    depth_msg->header.frame_id = depth_frame_id_;
    depth_msg->header.stamp = time;

    sensor_msgs::CameraInfoPtr camera_info = CalibrationTransform::toCameraInfo(camera_intrinsics_);
    camera_info->header.frame_id = depth_msg->header.frame_id;
    camera_info->header.stamp = depth_msg->header.stamp;
    camera_info->height = depth_msg->height;
    camera_info->width = depth_msg->width;
    pub_depth_info_.publish(camera_info);

    if (pub_depth_image_.getNumSubscribers () > 0)
    {
        pub_depth_image_.publish(depth_msg);
    }

    if (pub_point_cloud_.getNumSubscribers () > 0)
    {
        sensor_msgs::PointCloud2Ptr points_msg = boost::make_shared<sensor_msgs::PointCloud2>();
        toMsgPointCloud(depth_msg, points_msg);

        pub_point_cloud_.publish(points_msg);
    }

    return (current_frame_ != frames_);
}

bool OniPlayer::close()
{
    ROS_INFO("Closing ONI player");

    player_.Release();
    depth_generator_.Release();
    context_.StopGeneratingAll();
    context_.Release();
}

XnUInt32 OniPlayer::currentFrame() const
{
    return current_frame_;
}

XnUInt32 OniPlayer::countFrames() const
{
    return frames_;
}

bool OniPlayer::seekToFrame(XnInt32 frame)
{
    if (player_.IsValid())
    {
        player_.SeekToFrame(depth_generator_.GetName(), frame, XN_PLAYER_SEEK_SET);
        return true;
    }

    return false;
}

bool OniPlayer::setPlaybackSpeed(double speed)
{
    if (player_.IsValid())
    {
        player_.SetPlaybackSpeed(speed);
        return true;
    }

    return false;
}

void OniPlayer::toMsgImage(const xn::DepthMetaData& depth_meta_data,
                           sensor_msgs::ImagePtr image) const
{
    // all depth data is relative to the rgb frame since we take registered data by default
    image->header.frame_id = depth_frame_id_;
    image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    image->height = depth_meta_data.YRes();
    image->width = depth_meta_data.XRes();
    image->step = image->width * sizeof(float);
    image->data.resize (image->height * image->step);

    // copy and convert data data
    float* data = reinterpret_cast<float*>(&image->data[0]);    
    for (unsigned int i = 0, k = 0; i < image->height; i++)
    {
        for (unsigned int j = 0; j < image->width; ++j, ++k, ++data)
        {
            *data = toMeter(depth_meta_data[k]);
        }
    }
}

void OniPlayer::toMsgPointCloud(const sensor_msgs::ImagePtr& image,
                                sensor_msgs::PointCloud2Ptr points)
{
    points->header.frame_id = image->header.frame_id;
    points->header.stamp = image->header.stamp;
    points->height = image->height;
    points->width = image->width;
    points->is_dense = false;
    points->is_bigendian = false;
    points->fields.resize(3);
    points->fields[0].name = "x";
    points->fields[1].name = "y";
    points->fields[2].name = "z";

    int offset = 0;
    for (size_t d = 0; d < points->fields.size(); ++d, offset += sizeof(float))
    {
        points->fields[d].offset = offset;
        points->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
        points->fields[d].count  = 1;
    }

    points->point_step = offset;
    points->row_step = points->point_step * points->width;
    points->data.resize(points->width * points->height * points->point_step);

    const float* depth_data = reinterpret_cast<const float*>(&image->data[0]);
    float* point_data = reinterpret_cast<float*>(&points->data[0]);

    Point3d p;
    for(int  y = 0, k = 0; y < image->height; ++y)
    {
        for(int x = 0; x < image->width; ++x, ++k, ++point_data)
        {            
            p = toPoint3d(depth_data[k], x, y);

            *point_data++ = p.x;
            *point_data++ = p.y;
            *point_data = p.z;
        }
    }
}

float OniPlayer::toMeter(const XnDepthPixel& depth_pixel) const
{
    static float bad_point = std::numeric_limits<float>::quiet_NaN();

    if (depth_pixel == 0 || depth_pixel == no_sample_value_ || depth_pixel == shadow_value_)
    {
        return bad_point;
    }

    return float(depth_pixel) * 1e-3f;
}

OniPlayer::Point3d OniPlayer::toPoint3d(float depth, float x, float y) const
{
    Point3d point;

    static float bad_point = std::numeric_limits<float>::quiet_NaN();

    if (depth == bad_point)
    {
        point.x = point.y = point.z = bad_point;
        return point;
    }

    point.z = depth;
    point.x = (x - camera_intrinsics_.cx) * depth / camera_intrinsics_.f;
    point.y = (y - camera_intrinsics_.cy) * depth / camera_intrinsics_.f;

    return point;
}
