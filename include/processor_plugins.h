//
// Created by irving on 2022/4/5.
//

#ifndef SRC_PROCESSOR_PLUGIN_H
#define SRC_PROCESSOR_PLUGIN_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind/bind.hpp>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <nodelet/nodelet.h>
#include <pluginlib/class_loader.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#endif //SRC_PROCESSOR_PLUGIN_H

namespace processor
{
class Processor : public nodelet::Nodelet
{
public:
    std::vector<std::vector<cv::Point>> findArmor(const sensor_msgs::ImageConstPtr& msg2);

    std::vector<cv::Point> findHighest(std::vector<std::vector<cv::Point>> contours);

    std::vector<cv::Point> findLowest(std::vector<std::vector<cv::Point>> contours);

    void axisMarker(cv::Mat K,std::vector<double,std::allocator<void>::rebind<double>::other> D,std::vector<cv::Point2f> d2,cv::Mat& img);

    void armorMarker(std::vector<cv::Point>highest,std::vector<cv::Point> lowest,cv::Mat& img,cv::Mat K,
                     std::vector<double,std::allocator<void>::rebind<double>::other> D);

    void tfBroadcast();

    virtual void onInit();

    void reciever(ros::NodeHandle& p_nh)
    {
                it_ = std::make_shared<image_transport::ImageTransport>(p_nh);
                cam_sub_ = it_->subscribeCamera("/galaxy_camera/image_raw", 1000,&processor::Processor::infoCallback1, this);
                p_nh_ = ros::NodeHandle(p_nh,"reciever_node");
    }
private:
    ros::NodeHandle p_nh_;
    ros::Publisher publisher = p_nh_.advertise<sensor_msgs::Image>("camera/image", 1);
    std::shared_ptr<image_transport::ImageTransport> it_;
    cv::Mat img_;
    tf::Vector3 t_vec1_;
    tf::Quaternion quaternion_;
    cv::Mat_<double> r_vec_, t_vec_;
    image_transport::CameraSubscriber cam_sub_;
    void infoCallback1(const sensor_msgs::ImageConstPtr &msg2,const sensor_msgs::CameraInfoConstPtr &msg);
    tf::TransformBroadcaster broadcaster_;
    tf::Transform transform_;
};
}
