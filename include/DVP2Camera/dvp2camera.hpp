#ifndef DVP2CAMERA_HPP
#define DVP2CAMERA_HPP

#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <iostream>
#include "DVPCamera.h" /* DVP API 依赖 */

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <signal.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace dvp2_camera
{

    struct ThreadData
    {
        char *cameraName;
        ros::Publisher *publisher;
    };
    
    class DVP2Camera
    {
    public:
        DVP2Camera(ros::NodeHandle nh, ros::NodeHandle nh_private);
        ~DVP2Camera();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        ros::Publisher pub_camera_image_;

        static void *threadEntryFunction(void *p);
    };
} // namespace dvp2_camera

#endif // DVP2CAMERA_HPP
