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


#include <oni_vicon_common/type_conversion.hpp>
#include "oni_vicon_player/oni_player.hpp"

#define CHECK_RC(rc)                                          \
if (rc != XN_STATUS_OK)                                       \
{                                                             \
    ROS_ERROR("%s failed: %s\n", #rc, xnGetStatusString(rc)); \
    return false;                                             \
}

using namespace oni_vicon;
using namespace oni_vicon_player;

OniPlayer::OniPlayer():
    msg_image_dirty_(true),
    msg_pointcloud_dirty_(true)
{
}

OniPlayer::~OniPlayer()
{
}

bool OniPlayer::open(const std::string& source_file, const CameraIntrinsics& camera_intrinsics)
{
    CHECK_RC(context_.Init());
    CHECK_RC(context_.OpenFileRecording(source_file.c_str(), player_));
    CHECK_RC(depth_generator_.Create(context_));
    CHECK_RC(player_.GetNumFrames(depth_generator_.GetName(), frames_));
    CHECK_RC(depth_generator_.GetIntProperty((XnChar*)"NoSampleValue", no_sample_value_));
    CHECK_RC(depth_generator_.GetIntProperty((XnChar*)"ShadowValue", shadow_value_));
    CHECK_RC(context_.StartGeneratingAll());
    CHECK_RC(player_.SetPlaybackSpeed(1.0));
    CHECK_RC(player_.SetRepeat(true));

    camera_intrinsics_ = camera_intrinsics;

    return true;
}

bool OniPlayer::processNextFrame()
{
    boost::mutex::scoped_lock lock(read_write_mutex_);

    CHECK_RC(depth_generator_.WaitAndUpdateData());

//    if (!depth_generator_.IsDataNew()) // not sure yet if this required
//    {
//        return (current_frame_ != frames_);
//    }

    // get depth frame
    depth_generator_.GetMetaData(depth_meta_data_);

    msg_image_dirty_ = true;
    msg_pointcloud_dirty_ = true;

    return (depth_meta_data_.FrameID() != frames_);
}

bool OniPlayer::close()
{
    player_.Release();
    depth_generator_.Release();
    context_.StopGeneratingAll();
    context_.Release();
}

const xn::DepthMetaData& OniPlayer::currentDepthMetaData() const
{
    return depth_meta_data_;
}

sensor_msgs::ImagePtr OniPlayer::currentDepthImageMsg()
{
    boost::mutex::scoped_lock lock(read_write_mutex_);

    if (msg_image_dirty_)
    {
        cache_msg_image_ = boost::make_shared<sensor_msgs::Image>();
        toMsgImage(currentDepthMetaData(), cache_msg_image_);

        msg_image_dirty_ = false;
    }

    return cache_msg_image_;
}

sensor_msgs::PointCloud2Ptr OniPlayer::currentPointCloud2Msg()
{
    sensor_msgs::ImagePtr msg_image = currentDepthImageMsg();

    boost::mutex::scoped_lock lock(read_write_mutex_);

    if (msg_pointcloud_dirty_)
    {
        cache_msg_pointcloud_ = boost::make_shared<sensor_msgs::PointCloud2>();
        oni_vicon::toMsgPointCloud(msg_image, camera_intrinsics_, cache_msg_pointcloud_);
    }

    return cache_msg_pointcloud_;
}

XnUInt32 OniPlayer::currentFrameID() const
{
    return depth_meta_data_.FrameID();
}

XnUInt32 OniPlayer::countFrames() const
{
    return frames_;
}

bool OniPlayer::seekToFrame(XnInt32 frameID)
{
    if (player_.IsValid())
    {
        player_.SeekToFrame(depth_generator_.GetName(), frameID, XN_PLAYER_SEEK_SET);
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

float OniPlayer::toMeter(const XnDepthPixel& depth_pixel) const
{
    static float bad_point = std::numeric_limits<float>::quiet_NaN();

    if (depth_pixel == 0 ||
        depth_pixel == no_sample_value_ ||
        depth_pixel == shadow_value_)
    {
        return bad_point;
    }

    return float(depth_pixel) * 1e-3f;
}

float OniPlayer::toMillimeter(const XnDepthPixel &depth_pixel) const
{
    static float bad_point = std::numeric_limits<float>::quiet_NaN();

    if (depth_pixel == 0 ||
        depth_pixel == no_sample_value_ ||
        depth_pixel == shadow_value_)
    {
        return bad_point;
    }

    return float(depth_pixel);
}
