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

#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <yaml-cpp/yaml.h>

#include "oni_vicon_player/oni_player.hpp"

using namespace oni_vicon_player;

OniPlayer::OniPlayer():
    image_width_(KINECT_IMAGE_COLS),
    image_height_(KINECT_IMAGE_ROWS),
    depth_width_(KINECT_IMAGE_COLS),
    depth_height_(KINECT_IMAGE_ROWS)
{
    node_handle_ = ros::NodeHandle("~");

    node_handle_.param ("rgb_frame_id", rgb_frame_id_, std::string (""));
    if (rgb_frame_id_.empty ())
    {
        rgb_frame_id_ = "/camera_rgb_optical_frame";
        ROS_INFO ("'rgb_frame_id' not set. using default: '%s'", rgb_frame_id_.c_str());
    }
    else
    {
        ROS_INFO ("rgb_frame_id = '%s' ", rgb_frame_id_.c_str());
    }

    node_handle_.param ("depth_frame_id", depth_frame_id_, std::string (""));
    if (depth_frame_id_.empty ())
    {
        depth_frame_id_ = "/camera_depth_optical_frame";
        ROS_INFO ("'depth_frame_id' not set. using default: '%s'", depth_frame_id_.c_str());
    }
    else
    {
        ROS_INFO ("depth_frame_id = '%s' ", depth_frame_id_.c_str());
    }

    // Init all publishers
    it_ = new image_transport::ImageTransport(node_handle_);
    pub_depth_image_ = it_->advertise ("depth/image", 5);

    pub_depth_info_ = node_handle_.advertise<sensor_msgs::CameraInfo> ("depth/camera_info", 5);
    pub_point_cloud_ = node_handle_.advertise<sensor_msgs::PointCloud2> ("depth/points", 5);
}

OniPlayer::~OniPlayer()
{

}

bool OniPlayer::process()
{
    boost::mutex::scoped_lock lock (capture_mutex_);

    if(kinect_capture(device_)!=0)
    {
        ROS_ERROR("Error during capture!\n");
        return false;
    }

    ros::Time time = ros::Time::now ();

    if(device_->depth_dirty)
    {
        if (pub_depth_info_.getNumSubscribers () > 0)
        {
            pub_depth_info_.publish (fillCameraInfo (time, false));
        }

        if (pub_depth_image_.getNumSubscribers () > 0)
        {
            publishDepthImage(time);
        }

        if (pub_point_cloud_.getNumSubscribers () > 0)
        {
            publishXYZPointCloud(time);
        }
    }

    ros::spinOnce();

    return true;
}

void OniPlayer::publishDepthImage(ros::Time time)
{
    sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image> ();
    depth_msg->header.stamp         = time;
    // all depth data is relative to the rgb frame since we take registered data by default
    depth_msg->header.frame_id      = rgb_frame_id_;
    depth_msg->encoding             = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_msg->height               = depth_height_;
    depth_msg->width                = depth_width_;
    depth_msg->step                 = depth_msg->width * sizeof (float);
    depth_msg->data.resize (depth_msg->height * depth_msg->step);

    kinect_get_f_depth_buffer(device_,
                              reinterpret_cast<float*>(&depth_msg->data[0]),
                               depth_msg->width * depth_msg->height *sizeof(float));

    if (pub_depth_image_.getNumSubscribers () > 0)
    {
        pub_depth_image_.publish (depth_msg);
    }
}


void OniPlayer::publishXYZPointCloud(ros::Time time)
{

  sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image > ();
  depth_msg->header.stamp         = time;
  // all depth data is relative to the rgb frame since we take registered data by default
  depth_msg->header.frame_id      = rgb_frame_id_;
  depth_msg->encoding             = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_msg->height               = depth_height_;
  depth_msg->width                = depth_width_;
  depth_msg->step                 = depth_msg->width * sizeof (float);
  depth_msg->data.resize (depth_msg->height * depth_msg->step);

  kinect_get_f_depth_buffer(device_,
                reinterpret_cast<float*>(&depth_msg->data[0]),
                depth_msg->width * depth_msg->height *sizeof(float));

  float *point_buffer = (float*)calloc(1,KINECT_IMAGE_COLS*KINECT_IMAGE_ROWS*sizeof(float)*3);
  kinect_f_depth_to_xyz_buffer(device_,
                   reinterpret_cast<float*>(&depth_msg->data[0]), depth_msg->width * depth_msg->height *sizeof(float),
                   NULL, 0,
                   point_buffer, KINECT_IMAGE_COLS*KINECT_IMAGE_ROWS*sizeof(float),
                   NULL, 0);

  sensor_msgs::PointCloud2Ptr points = boost::make_shared<sensor_msgs::PointCloud2 > ();
  // all depth data is relative to the rgb frame since we take registered data by default
  points->header.frame_id = rgb_frame_id_;
  points->header.stamp = time;
  points->width        = KINECT_IMAGE_COLS;
  points->height       = KINECT_IMAGE_ROWS;
  points->is_dense     = false;
  points->is_bigendian = false;
  points->fields.resize( 3 );
  points->fields[0].name = "x";
  points->fields[1].name = "y";
  points->fields[2].name = "z";
  int offset = 0;
  for (size_t d = 0; d < points->fields.size (); ++d, offset += sizeof(float)) {
    points->fields[d].offset = offset;
    points->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
    points->fields[d].count  = 1;
  }

  points->point_step = offset;
  points->row_step   =
    points->point_step * points->width;

  points->data.resize (points->width *
              points->height *
              points->point_step);


  for(int i=0; i<KINECT_IMAGE_ROWS; i++)
    {
      for(int j=0; j<KINECT_IMAGE_COLS; j++)
    {

      memcpy (&points->data[i * points->row_step + j * points->point_step + points->fields[0].offset],
          &point_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 0], sizeof (float));
      memcpy (&points->data[i * points->row_step + j * points->point_step + points->fields[1].offset],
          &point_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 1], sizeof (float));
      memcpy (&points->data[i * points->row_step + j * points->point_step + points->fields[2].offset],
          &point_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 2], sizeof (float));

      /*
      std::cout << "x y z " << point_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 0] << " "
            << point_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 1] << " "
                << point_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 2] << " " << std::endl;
      */
    }
    }

  if (  pub_point_cloud_.getNumSubscribers () > 0)
    pub_point_cloud_.publish (points);

  free(point_buffer);
}


sensor_msgs::CameraInfoPtr OniPlayer::fillCameraInfo (ros::Time time, bool is_rgb)
{
  sensor_msgs::CameraInfoPtr info_msg = boost::make_shared<sensor_msgs::CameraInfo > ();
  info_msg->header.stamp    = time;
  info_msg->header.frame_id = is_rgb ? rgb_frame_id_ : depth_frame_id_;
  info_msg->width           = is_rgb ? image_width_ : depth_width_;
  info_msg->height          = is_rgb ? image_height_ : depth_height_;

#if ROS_VERSION_MINIMUM(1, 3, 0)
  info_msg->D = std::vector<double>(5, 0.0);
  info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
#else
  info_msg->D.assign (0.0);
#endif
  info_msg->K.assign (0.0);
  info_msg->R.assign (0.0);
  info_msg->P.assign (0.0);
  // Simple camera matrix: square pixels, principal point at center
  double f = is_rgb ? device_->params.rgb_focal_length : device_->params.ir_focal_length;
  info_msg->K[0] = info_msg->K[4] = f;
  info_msg->K[2] = is_rgb ?  device_->params.rgb_camera_center.x : device_->params.ir_camera_center.x;
  info_msg->K[5] = is_rgb ?  device_->params.rgb_camera_center.y : device_->params.ir_camera_center.y;
  info_msg->K[8] = 1.0;
  // no rotation: identity
  info_msg->R[0] = info_msg->R[4] = info_msg->R[8] = 1.0;
  // no rotation, no translation => P=K(I|0)=(K|0)
  info_msg->P[0] = info_msg->P[5] = info_msg->K[0];
  info_msg->P[2] = info_msg->K[2];
  info_msg->P[6] = info_msg->K[5];
  info_msg->P[10] = 1.0;

  return info_msg;
}

/*
 *
 *
int main(int argc, char* argv[])
{
    if(argc < 2 )
    {
        std::cout << ">> Error: missing file argument!" << std::endl;
        return -1;
    }

    // initialize rainbow coloring
    rainbow_init();

    Context context;
    int nRetVal = context.Init();
    CHECK_RC(nRetVal, "Init");

    std::string sourceFile = argv[1];

    Player player;
    context.OpenFileRecording( sourceFile.c_str(), player );

    DepthGenerator depthGenerator;
    depthGenerator.Create(context);

    XnUInt32 uFrames;
    player.GetNumFrames( depthGenerator.GetName(), uFrames );
    std::cout << "Total " << uFrames << " frames" << std::endl;

    // create window / gui
    std::string windowName = "Kinect Display";
    cv::namedWindow(windowName, 1);
    int tbPos;
    cv::createTrackbar("MyTrackbar", windowName, &tbPos, uFrames);
    cv::createButton("button6", callbackButton2, NULL, cv::QT_PUSH_BUTTON, 1);
    cv::createButton("button6", callbackButton2, NULL, cv::QT_RADIOBOX, 1);
    cv::createButton("button6", callbackButton2, NULL, cv::QT_CHECKBOX, 1);
    cv::createButton("button6", callbackButton2, NULL, cv::QT_CHECKBOX, 1);
    cv::displayOverlay(windowName, "some stuff to display overlayed");


    // create the test images
    cv::Mat depth_image(KINECT_IMAGE_ROWS, KINECT_IMAGE_COLS, CV_8UC3);
    cv::Mat display(KINECT_IMAGE_ROWS, KINECT_IMAGE_COLS, CV_8UC3);
    uint16_t *depth_buffer = (uint16_t*)calloc(1,KINECT_IMAGE_COLS*KINECT_IMAGE_ROWS*sizeof(uint16_t));
    cv::Rect rect;

    context.StartGeneratingAll();

    const int ESC=27;
    char pressed_key = 0;
    uint8_t *dst;
    uint16_t val, min_val, max_val;
    DepthMetaData xDepthMap;

    player.SetPlaybackSpeed(1.0);

    //for( unsigned int i = 0; i < uFrames; ++ i )
    while(true)
    {
        player.SeekToFrame(depthGenerator.GetName(), tbPos, XN_PLAYER_SEEK_SET);

        depthGenerator.WaitAndUpdateData();

        // get value
        depthGenerator.GetMetaData( xDepthMap );
        //cout << "Frame " << uFrames << ", frameID " << xDepthMap.FrameID() << ", timestamp " << xDepthMap.Timestamp() << endl;

        memcpy(depth_buffer, (char*)xDepthMap.Data(), sizeof(uint16_t) * KINECT_IMAGE_COLS * KINECT_IMAGE_ROWS);

        min_val = 65535;
        max_val = 0;
        for(int i=0; i<KINECT_IMAGE_ROWS; i++)
        {
            for(int j=0; j<KINECT_IMAGE_COLS; j++)
            {
                val = depth_buffer[i*KINECT_IMAGE_COLS + j];
                if(val < min_val)
                    min_val = val;
                if(val > max_val)
                    max_val = val;
            }
        }

        for(int i=0; i<KINECT_IMAGE_ROWS; i++)
        {
            for(int j=0; j<KINECT_IMAGE_COLS; j++)
            {
                dst = (uint8_t*)(&depth_image.data[i*KINECT_IMAGE_COLS*3 + j*3]);
                val = depth_buffer[i*KINECT_IMAGE_COLS + j];
                rainbow_set(dst, (max_val - val)*1.0, min_val*1.0, max_val*1.0);
            }
        }

        // form the display image
        display = cv::Scalar(0);
        rect.x = 0;
        rect.y = 0;
        rect.width = KINECT_IMAGE_COLS;
        rect.height = KINECT_IMAGE_ROWS;
        cv::Mat roi_depth(display, rect);
        depth_image.convertTo(roi_depth, roi_depth.type(), 1, 0);

        pressed_key = cv::waitKey(2);
        if(pressed_key == ESC)
        {
            break;
        }
        cv::imshow(windowName, display);
    }

    context.StopGeneratingAll();
    context.Release();

    return 0;
}
*/
