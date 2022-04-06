//#include <iostream>
//#include <opencv2/opencv.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/CompressedImage.h>
//#include <ros/ros.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>
//#include <boost/bind/bind.hpp>
//#include <tf/transform_broadcaster.h>
//#include <eigen3/Eigen/Dense>
#include "../include/processor_plugins.h"
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>
#include <thread>
//定义全局变量便于在订阅者回调与发布者之间传递信息，便于函数与函数间传递信息（相当于多个返回值）

//从包接收图像信息并找出灯条轨迹
PLUGINLIB_EXPORT_CLASS(processor::Processor, nodelet::Nodelet)
namespace processor {
    std::vector<std::vector<cv::Point>> Processor::findArmor(const sensor_msgs::ImageConstPtr &msg2) {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg2, "bgr8");
        cv_ptr->image.copyTo(img_);
        cv::Mat gray;
        cv::cvtColor(img_, gray, CV_BGR2GRAY);
        cv::Mat threshImage;
        cv::threshold(gray, threshImage, 150, 255, CV_THRESH_BINARY);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(threshImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        return contours;
    }

//找出灯条的最高点
    std::vector<cv::Point> Processor::findHighest(std::vector<std::vector<cv::Point>> contours) {
        int b = 0, d = 0;
        std::vector<cv::Point> highest(5);
        for (int i = 0; i < 4; i++) {
            highest[i] = cv::Point(0, 0);
        }
        for (int i = 0; i < contours.size(); i++) {
            double length = cv::arcLength(contours[i], 1);
            double area = cv::contourArea(contours[i]);
            cv::Rect box = cv::boundingRect(contours[i]);
            if (length > 50 && area < 1000 && box.height / box.width > 1) {
                for (int j = 0; j < contours[i].size(); j++) {
                    b = 0;
                    for (int k = 0; k < contours[i].size(); k++) {

                        if (contours[i][j].y > contours[i][k].y) {
                            b = 1;
                        }
                    }
                    if (b == 0) {
                        highest[d] = contours[i][j];
                        d++;
                        break;
                    }
                }
            }
        }
        return highest;
    }

//找出灯条的最低点
    std::vector<cv::Point> Processor::findLowest(std::vector<std::vector<cv::Point>> contours) {
        int a = 0, c = 0;
        std::vector<cv::Point> lowest(5);
        for (int i = 0; i < 4; i++) {
            lowest[i] = cv::Point(0, 0);
        }
        for (int i = 0; i < contours.size(); i++) {
            double length = cv::arcLength(contours[i], 1);
            double area = cv::contourArea(contours[i]);
            cv::Rect box = cv::boundingRect(contours[i]);
            if (length > 50 && area < 1000 && box.height / box.width > 1) {
                for (int j = 0; j < contours[i].size(); j++) {
                    a = 0;
                    for (int k = 0; k < contours[i].size(); k++) {
                        if (contours[i][j].y < contours[i][k].y) {
                            a = 1;
                        }
                    }
                    if (a == 0) {
                        lowest[c] = contours[i][j];
                        c++;
                        break;
                    }
                }
            }
        }
        return lowest;
    }

//通过图像点与模拟三维平面计算坐标轴并标记
    void Processor::axisMarker(cv::Mat K, std::vector<double, std::allocator<void>::rebind<double>::other> D,
                               std::vector<cv::Point2f> d2, cv::Mat &img) {
        std::vector<cv::Point3f> d3;
        d3.push_back(cv::Point3f(0, 61, 0));
        d3.push_back(cv::Point3f(235, 61, 0));
        d3.push_back(cv::Point3f(235, 0, 0));
        d3.push_back(cv::Point3f(0, 0, 0));
        cv::solvePnP(d3, d2, K, D, r_vec_, t_vec_);
        std::vector<cv::Point3f> d1;
        std::vector<cv::Point2f> d4(4);
        d1.push_back(cv::Point3f(235 / 2, 61 / 2, 0));
        d1.push_back(cv::Point3f(235 / 2 + 20, 61 / 2, 0));
        d1.push_back(cv::Point3f(235 / 2, 61 / 2 + 20, 0));
        d1.push_back(cv::Point3f(235 / 2, 61 / 2, 20));
        cv::projectPoints(d1, r_vec_, t_vec_, K, D, d4);
        cv::line(img, d4[0], d4[1], cv::Scalar(0, 0, 255), 5);
        cv::line(img, d4[0], d4[2], cv::Scalar(0, 255, 0), 5);
        cv::line(img, d4[0], d4[3], cv::Scalar(255, 0, 0), 5);
    }

//配对灯条并寻找装甲板
    void Processor::armorMarker(std::vector<cv::Point> highest, std::vector<cv::Point> lowest, cv::Mat &img, cv::Mat K,
                                std::vector<double, std::allocator<void>::rebind<double>::other> D) {
        int f = 0, g = 0, h = 0, e = 0;
        for (int i = 0; i < 4; i++) {
            e = 0;
            if (highest[i] != cv::Point(0, 0) && lowest[i] != cv::Point(0, 0)) {
                for (int j = 0; j < 4; j++) {
                    if (highest[j] != cv::Point(0, 0) && lowest[j] != cv::Point(0, 0) && j != i &&
                        abs(highest[j].x - highest[i].x) > 0) {
                        for (int k = 0; k < 4; k++) {
                            //通过两次遍历灯条最高点与第一次遍历的差值进行比较，找出与当前灯条最接近的灯条
                            if (highest[k] != cv::Point(0, 0) && lowest[k] != cv::Point(0, 0) && k != i) {
                                if (abs(highest[j].x - highest[i].x) > abs(highest[k].x - highest[i].x)) {
                                    e = 1;
                                }
                            }
                        }
                        if (e == 0) {
                            //再进行一次排除最近的灯条的循环，找出次接近的灯条，并与等前灯条i构成一个矩形
                            for (int l = 0; l < 4; l++) {
                                f = 0;
                                if (highest[l] != cv::Point(0, 0) && lowest[l] != cv::Point(0, 0) && l != i && l != j) {
                                    for (int k = 0; k < 4; k++) {
                                        if (highest[k] != cv::Point(0, 0) && lowest[k] != cv::Point(0, 0) && k != i &&
                                            k != j) {
                                            if (abs(highest[l].x - highest[i].x) > abs(highest[k].x - highest[i].x)) {
                                                f = 1;
                                            }
                                        }
                                    }
                                    if (f == 0 && abs(highest[l].x - highest[i].x) < 400) {
                                        g = -10;
                                        std::vector<cv::Point2f> d2;
                                        if (highest[i].x > highest[l].x) {
                                            g = (highest[i].x - highest[l].x) - (lowest[i].x - lowest[l].x);
                                            d2.push_back(highest[i]);
                                            d2.push_back(highest[l]);
                                            d2.push_back(lowest[l]);
                                            d2.push_back(lowest[i]);
                                        }
                                        if (highest[i].x < highest[l].x) {
                                            g = (highest[l].x - highest[i].x) - (lowest[l].x - lowest[i].x);
                                            d2.push_back(highest[l]);
                                            d2.push_back(highest[i]);
                                            d2.push_back(lowest[i]);
                                            d2.push_back(lowest[l]);
                                        }
                                        if (g > -8) {
                                            cv::line(img, highest[i], highest[l], cv::Scalar(0, 0, 255), 2);
                                            cv::line(img, lowest[i], lowest[l], cv::Scalar(0, 0, 255), 2);
                                            cv::line(img, highest[i], lowest[i], cv::Scalar(0, 0, 255), 2);
                                            cv::line(img, highest[l], lowest[l], cv::Scalar(0, 0, 255), 2);

                                            axisMarker(K, D, d2, img);
                                        }
                                    }
                                }
                            }
                        }
                        //将最接近的灯条与当前灯条i构成矩形
                        if (e == 0) {
                            g = -10;
                            std::vector<cv::Point2f> d2;
                            if (highest[i].x > highest[j].x) {
                                g = (highest[i].x - highest[j].x) - (lowest[i].x - lowest[j].x);
                                d2.push_back(highest[i]);
                                d2.push_back(highest[j]);
                                d2.push_back(lowest[j]);
                                d2.push_back(lowest[i]);
                            }
                            if (highest[i].x < highest[j].x) {
                                g = (highest[j].x - highest[i].x) - (lowest[j].x - lowest[i].x);
                                d2.push_back(highest[j]);
                                d2.push_back(highest[i]);
                                d2.push_back(lowest[i]);
                                d2.push_back(lowest[j]);
                            }
                            if (g > -6) {
                                cv::line(img, highest[i], highest[j], cv::Scalar(0, 0, 255), 2);
                                cv::line(img, lowest[i], lowest[j], cv::Scalar(0, 0, 255), 2);
                                cv::line(img, highest[i], lowest[i], cv::Scalar(0, 0, 255), 2);
                                cv::line(img, highest[j], lowest[j], cv::Scalar(0, 0, 255), 2);

                                axisMarker(K, D, d2, img);
                            }
                        }
                    }
                }
            }
        }
    }

//通过solvePnP()得出的旋转矩阵转化为四元数，平移矩阵转化为tf格式，通过全局变量传递到主函数发布者使用
    void Processor::tfBroadcast() {
        Eigen::Quaterniond quaternion;
        Eigen::Matrix3d rotation_matrix;

        for (int m = 0; m < 3; m++) {
            rotation_matrix(m, 0) = r_vec_.at<double>(0, m);
        }

        t_vec1_.setX(t_vec_.at<double>(0, 0) / 2000);
        t_vec1_.setY(t_vec_.at<double>(0, 1) / 2000);
        t_vec1_.setZ(t_vec_.at<double>(0, 2) / 2000);


//        tf::Vector3 vec;
//        quaternion = rotation_matrix;
//        tf::quatRotate(quaternion_, vec);
//        quaternion_.setX(quaternion.x());
//        quaternion_.setY(quaternion.y());
//        quaternion_.setZ(quaternion.z());
//        quaternion_.setW(quaternion.w());

        double phi = sqrt(r_vec_.dot(r_vec_));
        double w = 1, x = 0, y = 0, z = 0;
        if (phi > 1e-6)
        {
            w = cos(phi / 2.);
            r_vec_ *= (sin(phi / 2.) / phi);  // (r_vec /= phi) *= sin(phi / 2.)
            x = r_vec_(0, 0);
            y = r_vec_(1, 0);
            z = r_vec_(2, 0);
        }
        this->quaternion_ = { x, y, z, w };
    }

    void Processor::infoCallback1(const sensor_msgs::ImageConstPtr &msg2, const sensor_msgs::CameraInfoConstPtr &msg) {
        boost::array<double, 9> K1 = msg->K;

        double K2[9];
        for (int i = 0; i < 9; i++) {
            K2[i] = K1.at(i);
        }
        cv::Mat K(3, 3, CV_64FC1, K2);
        std::vector<double> D = msg->D;

        std::vector<std::vector<cv::Point>> contours = findArmor(msg2);

        std::vector<cv::Point> lowest(5), highest(5);
        highest = findHighest(contours);
        lowest = findLowest(contours);

        armorMarker(highest, lowest, img_, K, D);

        tfBroadcast();

        cv::imshow("a", img_);
        cv::waitKey(1);

        sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8,
                                                        img_).toImageMsg();
        publisher.publish(msg3);

        if (r_vec_.empty() != true) {
            transform_.setOrigin(t_vec1_);
            transform_.setRotation(quaternion_);
            broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "odom", "base_robot"));
        }

    }


    void Processor::onInit(){
            auto& n = getPrivateNodeHandle();
//            message_filters::Subscriber<sensor_msgs::CameraInfo> sub_imu_accel(n, "/galaxy_camera/camera_info", 1, ros::TransportHints().tcpNoDelay());
//            message_filters::Subscriber<sensor_msgs::CompressedImage> sub_imu_gyro(n, "/galaxy_camera/image_raw/compressed", 1, ros::TransportHints().tcpNoDelay());
//
//            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::CompressedImage> syncPolicy;
//            message_filters::Synchronizer<syncPolicy> sync(syncPolicy(1000), sub_imu_accel, sub_imu_gyro);
//
//            sync.registerCallback(boost::bind(&Processor::infoCallback1, this, _1, _2));
            Processor::reciever(n);
//            ros::Publisher publisher = n.advertise<sensor_msgs::Image>("camera/image", 1);
//
//            tf::TransformBroadcaster broadcaster;
//            tf::Transform transform;
//
//            ros::Rate rate(2000);
//            static ros::CallbackQueue my_queue;
//            n.setCallbackQueue(&my_queue);
//            std::thread my_thread_ = std::thread([]() {
//            ros::SingleThreadedSpinner spinner;
//            spinner.spin(&my_queue);
//        });
//            while ((n.ok())){
//                if (r_vec_.empty() != true) {
//                    transform.setOrigin(t_vec1_);
//                    transform.setRotation(quaternion_);
//                    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_robot"));
//                }
//                sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8,
//                                                                img_).toImageMsg();
//                publisher.publish(msg3);
//                ros::spinOnce();
//                rate.sleep();
//            }
    }
}

