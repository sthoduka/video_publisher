#ifndef VIDEO_PUBLISHER_NODE_H_
#define VIDEO_PUBLISHER_NODE_H_

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <image_transport/image_transport.h>

namespace bfs = boost::filesystem;

/**
 * This class publishes frames from an input video file to a sensor_msgs/Image topic at the frame rate of the video
 * The video is read using OpenCV and converted to the ros Image message using cv_bridge
 */
class VideoPublisherNode
{
    public:
        /**
         * Constructor
         *
         * @param nh
         * ROS NodeHandle object
         */
        VideoPublisherNode(ros::NodeHandle &nh);

        /**
         * Destructor
         */
        virtual ~VideoPublisherNode();


    private:
        /**
         * Reads video file at video_file_path and publishes frames to output image topic
         *
         * @param video_file_path
         * Full path to input video file
         */
        void publish_video(const bfs::path &video_file_path);

    private:
        /**
         * ROS NodeHandle object
         */
        ros::NodeHandle nh_;

        /**
         * ImageTransport object
         */
        image_transport::ImageTransport it_;
        
        /**
         * Image publisher for frames from the video
         */
        image_transport::Publisher image_publisher_;
};

#endif
