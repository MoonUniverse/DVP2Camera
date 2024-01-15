#include "dvp2camera.hpp"

namespace dvp2_camera
{
    DVP2Camera::DVP2Camera(ros::NodeHandle nh, ros::NodeHandle nh_private) : nh_(nh),
                                                                             nh_private_(nh_private)
    {
        dvpUint32 count = 0;
        dvpInt32 num = -1;
        dvpCameraInfo info[8];
        dvpStatus status;
        std::string strInput;
        /* 枚举设备 */
        dvpRefresh(&count);
        if (count > 8)
            count = 8;

        for (int i = 0; i < count; i++)
        {
            if (dvpEnum(i, &info[i]) == DVP_STATUS_OK)
            {
                printf("[%d]-Camera FriendlyName : %s\r\n", i, info[i].FriendlyName);
            }
        }

        if (count == 0)
        {
            std::cout << "No camera found!" << std::endl;
            return;
        }

        pub_camera_image_ = nh_.advertise<sensor_msgs::Image>("image", 1);

        pthread_t tidp;
        ThreadData* data = new ThreadData{info[0].FriendlyName, &pub_camera_image_};
        int r = pthread_create(&tidp, NULL, threadEntryFunction, data);
        pthread_join(tidp, NULL);
    }
    DVP2Camera::~DVP2Camera()
    {
    }

    void *DVP2Camera::threadEntryFunction(void *p)
    {
        dvpStatus status;
        dvpHandle h;
        ThreadData* data = static_cast<ThreadData*>(p);
        bool trigMode = false;
        char *name = data->cameraName;
        ros::Publisher *pub_camera_image = data->publisher;

        printf("test start,camera is %s\r\n", name);
        do
        {
            /* 打开设备 */
            status = dvpOpenByName(name, OPEN_NORMAL, &h);
            if (status != DVP_STATUS_OK)
            {
                printf("dvpOpenByName failed with err:%d\r\n", status);
                break;
            }

            dvpRegion region;
            double exp;
            float gain;

            /* 打印ROI信息 */
            status = dvpGetRoi(h, &region);
            if (status != DVP_STATUS_OK)
            {
                printf("dvpGetRoi failed with err:%d\r\n", status);
                break;
            }
            printf("%s, region: x:%d, y:%d, w:%d, h:%d\r\n", name, region.X, region.Y, region.W, region.H);

            /* 打印曝光增益信息 */
            status = dvpGetExposure(h, &exp);
            if (status != DVP_STATUS_OK)
            {
                printf("dvpGetExposure failed with err:%d\r\n", status);
                break;
            }

            status = dvpGetAnalogGain(h, &gain);
            if (status != DVP_STATUS_OK)
            {
                printf("dvpGetAnalogGain failed with err:%d\r\n", status);
                break;
            }

            printf("%s, exposure: %lf, gain: %f\r\n", name, exp, gain);

            uint32_t v;
            /* 帧信 */
            dvpFrame frame;
            /* 帧数据首地址，用户不需要申请释放内 */
            void *p;

            /* 开始视频流 */
            status = dvpStart(h);
            if (status != DVP_STATUS_OK)
            {
                break;
            }

            /* 抓帧 */
            while (ros::ok())
            {
                /* 当前案例没有设置相机的曝光增益等参数，只展示在默认的ROI区域显示帧信 */
                status = dvpGetFrame(h, &frame, &p, 3000);
                if (status != DVP_STATUS_OK)
                {
                    if (trigMode)
                        continue;
                    else
                        break;
                }

                /* 显示帧数和帧 */
                dvpFrameCount framecount;
                status = dvpGetFrameCount(h, &framecount);
                if (status != DVP_STATUS_OK)
                {
                    printf("get framecount failed\n");
                }
                printf("framecount: %d, framerate: %f\n", framecount.uFrameCount, framecount.fFrameRate);

                /* 显示帧信 */
                printf("%s, frame:%lu, timestamp:%lu, %d*%d, %dbytes, format:%d\r\n",
                       name,
                       frame.uFrameID,
                       frame.uTimestamp,
                       frame.iWidth,
                       frame.iHeight,
                       frame.uBytes,
                       frame.format);

                if (frame.format == FORMAT_BGR24)
                {
                    cv::Mat img(frame.iHeight, frame.iWidth, CV_8UC3, p);
                    // cv::imshow("img", img);
                    // cv::waitKey(1);
                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
                    msg.get()->header.stamp = ros::Time::now();
                    msg.get()->header.frame_id = "dvp2_camera";
                    pub_camera_image->publish(msg);
                }

                ros::spinOnce();
            }

            /* 停止视频 */
            status = dvpStop(h);
            if (status != DVP_STATUS_OK)
            {
                break;
            }
        } while (0);

        dvpClose(h);

        printf("test quit, %s, status:%d\r\n", name, status);

        return NULL;
    }

} // namespace dvp2_camera