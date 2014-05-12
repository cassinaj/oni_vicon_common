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

#define CHECK_RC(rc)									    \
if (rc != XN_STATUS_OK)											\
{																\
    ROS_ERROR("%s failed: %s\n", #rc, xnGetStatusString(rc));  \
    return false;												\
}

using namespace oni_vicon_player;

OniPlayer::OniPlayer(const ros::NodeHandle& node_handle,
                     const std::string& depth_frame_id):
    node_handle_(node_handle),
    image_transport_(node_handle),
    depth_frame_id_(depth_frame_id)
{
    pub_depth_image_ = image_transport_.advertise("depth/image", 5);
    pub_depth_info_ = node_handle_.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 5);
    pub_point_cloud_ = node_handle_.advertise<sensor_msgs::PointCloud2>("depth/points", 5);
}

OniPlayer::~OniPlayer()
{
}

bool OniPlayer::open(const std::string& source_file)
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
    CHECK_RC(player_.SetRepeat(false));

    ROS_INFO("ONI file <%s> loaded", source_file.c_str());

    return true;
}

bool OniPlayer::process()
{
    boost::mutex::scoped_lock lock(capture_mutex_);

//    if (pub_depth_image_.getNumSubscribers () > 0 ||
//        pub_point_cloud_.getNumSubscribers () > 0)
//    {
        CHECK_RC(depth_generator_.WaitAndUpdateData());

        if (!depth_generator_.IsDataNew()) // not sure yet if this required
        {
            return !player_.IsEOF();
        }

        // get depth frame
        depth_generator_.GetMetaData(depth_meta_data_);

        current_frame_ = depth_meta_data_.FrameID();

        // convert and publish depth image and cloudgitk
        sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image>();
        toMsgImage(depth_meta_data_, depth_msg);

        //sensor_msgs::PointCloud2Ptr points_msg = boost::make_shared<sensor_msgs::PointCloud2>();
        //toMsgPointCloud(depth_msg, points_msg);

        // pub_depth_info_.publish (); // publish camera info

        if (pub_depth_image_.getNumSubscribers () > 0)
        {
            pub_depth_image_.publish(depth_msg);
        }

        if (pub_point_cloud_.getNumSubscribers () > 0)
        {
            //publishXYZPointCloud(ros::Time::now());
        }
//    }

    return !player_.IsEOF();
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

void OniPlayer::toMsgImage(const xn::DepthMetaData& depth_meta_data,
                           sensor_msgs::ImagePtr image) const
{
    // TODO REPLACE THIS WITH RECORDING TIMES!!!
    ros::Time time = ros::Time::now ();

    image->header.stamp         = time;
    // all depth data is relative to the rgb frame since we take registered data by default
    image->header.frame_id      = depth_frame_id_;
    image->encoding             = sensor_msgs::image_encodings::TYPE_32FC1;
    image->height               = depth_meta_data.YRes();
    image->width                = depth_meta_data.XRes();
    image->step                 = image->width * sizeof(float);
    image->data.resize (image->height * image->step);

    // copy data

    float* data = reinterpret_cast<float*>(&image->data[0]);    
    for (unsigned int i = 0; i < image->height; i++)
    {
        for (unsigned int j = 0, k = 0; j < image->width; j++, k += image->step, data++)
        {
            *data = toMeter(depth_meta_data[i * image->width + j]);
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

    for(int  i = 0; i < image->height; i++)
    {
        for(int j = 0; j < image->width; j++)
        {



            /*
            // get the 3d location of the pt first
            if (depth_data[i * image->width + j] == 0
                 || depth_data[i * image->width + j] == no_sample_value_
                 || depth_data[i * image->width + j] == shadow_value_)
            {
                ir_pt.x = bad_point;
                ir_pt.y = bad_point;
                ir_pt.z = bad_point;
            }
            else
            {
                // from millimeters to meters
                depth = depth_buffer[i*KINECT_IMAGE_COLS + j]*0.001;

                // z = baseline * focal_length / disparity
                // x = image_pt.x * z / focal_length
                // y = image_pt.y * z / focal_length

                z = depth;
                x = 1.0*(j-self->params.ir_camera_center.x) * z / self->params.ir_focal_length;
                y = 1.0*(i-self->params.ir_camera_center.y) * z / self->params.ir_focal_length;

                /*
            std::cout << "ir focal length " << self->params.ir_focal_length << std::endl;

            std::cout << "depth " << z << " x " << x << " y " << y << std::endl;


                // as seen in ir frame
                ir_pt.x = x;
                ir_pt.y = y;
                ir_pt.z = z;
            }

            xyz_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 0] = ir_pt.x;
            xyz_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 1] = ir_pt.y;
            xyz_buffer[i*KINECT_IMAGE_COLS*3 + j*3 + 2] = ir_pt.z;


            // if no color buffers provided, continue
            if(!color_buffer)
                continue;
            if(!xyz_rgb_buffer)
                continue;


            cv::Point3f pt(ir_pt.x, ir_pt.y, ir_pt.z);
            kinect::points.at(i*KINECT_IMAGE_COLS+j) = pt;
            */
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

OniPlayer::Point3d OniPlayer::toPoint3d(const XnDepthPixel& depth_pixel, float x, float y) const
{
    OniPlayer::Point3d point;

    static float bad_point = std::numeric_limits<float>::quiet_NaN();

    if (depth_pixel == 0 || depth_pixel == no_sample_value_ || depth_pixel == shadow_value_)
    {
        point.x = point.y = point.z = bad_point;
        return point;
    }

    point.z = float(depth_pixel) * 1e-3f;
    point.x = (x - camera_intrinsics_.cx) * point.z / camera_intrinsics_.f;
    point.y = (y - camera_intrinsics_.cy) * point.z / camera_intrinsics_.f;

    return point;
}


void OniPlayer::publishXYZPointCloud(ros::Time time)
{
/*
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

    }
    }

  if (  pub_point_cloud_.getNumSubscribers () > 0)
    pub_point_cloud_.publish (points);

  free(point_buffer);
  */
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
