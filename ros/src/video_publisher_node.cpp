/* video_publisher_node.cpp
 *
 * Copyright (C) 2014 Santosh Thoduka
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#include <video_publisher/video_publisher_node.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

VideoPublisherNode::VideoPublisherNode(ros::NodeHandle &nh) : nh_(nh), it_(nh)
{
    image_publisher_ = it_.advertise("output_image", 1);

    std::string input_file;    

    if (!nh_.getParam("input_file", input_file))
    {
        ROS_ERROR("Input video file needs to be specified");
    }
    else
    {
        double start_time;
        double stop_time;
        nh_.param<double>("start_time", start_time, 0.0);
        nh_.param<double>("stop_time", stop_time, -1.0);
        publish_video(input_file, start_time, stop_time);
    }
}

VideoPublisherNode::~VideoPublisherNode()
{
}

void VideoPublisherNode::publish_video(const bfs::path &video_file_path, double start_time, double stop_time)
{
    cv::VideoCapture capture(video_file_path.string());
      
    if (!capture.isOpened())
    {
        ROS_ERROR("Cound not read %s", video_file_path.string().c_str());
        return;
    }

    double frames_per_second = capture.get(CV_CAP_PROP_FPS);    
    ros::Rate loop_rate(frames_per_second);

    capture.set(CV_CAP_PROP_POS_MSEC, start_time);

    cv::Mat frame;
    int number_of_frames = 0;

    while(ros::ok())
    {
        capture >> frame;
        if (frame.empty())
        {
            break;
        }
        number_of_frames++;

        cv_bridge::CvImage frame_msg;
        frame_msg.encoding = sensor_msgs::image_encodings::BGR8;
        frame_msg.image = frame;
        image_publisher_.publish(frame_msg.toImageMsg());

        if (stop_time > 0.0 && capture.get(CV_CAP_PROP_POS_MSEC) >= stop_time)
        {
            break;
        }
        
        loop_rate.sleep();
    }
    ROS_INFO("Number of frames processed: %i", number_of_frames);
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "video_publisher");

    ros::NodeHandle n("~");

    ROS_INFO("[video_publisher] node started");

    VideoPublisherNode video_publisher_node(n); 

    return 0;
}
