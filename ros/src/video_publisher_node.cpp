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
        publish_video(input_file);
    }
}

VideoPublisherNode::~VideoPublisherNode()
{
}

void VideoPublisherNode::publish_video(const bfs::path &video_file_path)
{
    cv::VideoCapture capture(video_file_path.string());
      
    if (!capture.isOpened())
    {
        ROS_ERROR("Cound not read %s", video_file_path.string().c_str());
        return;
    }

    double frames_per_second = capture.get(CV_CAP_PROP_FPS);    
    ros::Rate loop_rate(frames_per_second);

    cv::Mat frame;

    while(ros::ok())
    {
        capture >> frame;
        if (frame.empty())
        {
            return;
        }             

        cv_bridge::CvImage frame_msg;
        frame_msg.encoding = sensor_msgs::image_encodings::BGR8;
        frame_msg.image = frame;
        image_publisher_.publish(frame_msg.toImageMsg());        
        
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{  
    ros::init(argc, argv, "video_publisher");

    ros::NodeHandle n("~");

    ROS_INFO("[video_publisher] node started");

    VideoPublisherNode video_publisher_node(n); 

    return 0;
}
