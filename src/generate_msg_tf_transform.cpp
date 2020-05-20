#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <iostream>

#include "convert_2d_to_3d/Result.h"

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>


darknet_ros_msgs::BoundingBoxes gBoundingboxes;
double gx = 0.0;
double gy = 0.0;
double gz = 0.0;

void boundingbox_callback(const darknet_ros_msgs::BoundingBoxes& boundingbox) {
    gBoundingboxes.bounding_boxes.clear();
    for(int i = 0; i < boundingbox.bounding_boxes.size(); i++) {
        gBoundingboxes.bounding_boxes.push_back(boundingbox.bounding_boxes[i]);
    }

	tf2_ros::TransformBroadcaster tfb_;
    // create static tf
    geometry_msgs::TransformStamped transformStamped;
    std::string object_coordinate = "object_coordinate";
    transformStamped.header.frame_id = "camera_color_optical_frame";
    transformStamped.child_frame_id = object_coordinate;

    transformStamped.transform.translation.x = gx;
    transformStamped.transform.translation.y = gy;
    transformStamped.transform.translation.z = gz;

    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;

    transformStamped.header.stamp = ros::Time::now();
    tfb_.sendTransform(transformStamped);

}
void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud)
{
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl::fromROSMsg(*pointcloud, pcl_cloud);

    if(gBoundingboxes.bounding_boxes.empty())
        return;

    ros::NodeHandle r;
	ros::Publisher pub_result_;
    pub_result_ = r.advertise<convert_2d_to_3d::Result>("detected_object", 10);
//    printf("Bounding boxes size : %d\n", gBoundingboxes.bounding_boxes.size());

    std::string id = "";

    for(int i = 0; i < gBoundingboxes.bounding_boxes.size(); i++) {
        pcl::PointXYZRGB p = pcl_cloud( (gBoundingboxes.bounding_boxes[i].xmin + gBoundingboxes.bounding_boxes[i].xmax)/2,
                                        (gBoundingboxes.bounding_boxes[i].ymin + gBoundingboxes.bounding_boxes[i].ymax)/2);
        gx = p.x;
        gy = p.y;
        gz = p.z;
        id = gBoundingboxes.bounding_boxes[i].Class;
        std::cout <<"point : " << p << std::endl;
        std::cout <<" ==> " << p.x << " , " << p.y << " , " << p.z << std::endl;

        // Publish result
        convert_2d_to_3d::Result msg;
        msg.type = "detected_object";
        msg.data = id;

        msg.pose.header.stamp = ros::Time::now();
        msg.pose.header.frame_id = "object_coordinate";
        msg.pose.pose.position.x = gx;
        msg.pose.pose.position.y = gy;
        msg.pose.pose.position.z = gz;

        msg.pose.pose.orientation.x = 0;
        msg.pose.pose.orientation.y = 0;
        msg.pose.pose.orientation.z = 0;
        msg.pose.pose.orientation.w = 1;

        pub_result_.publish(msg);
    }



    gBoundingboxes.bounding_boxes.clear();
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "convert_boundingbox_node");

    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_ = nh_.subscribe("/camera/depth_registered/points", 10, pointcloud_callback);
    ros::Subscriber boundingboxes_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 10, boundingbox_callback);
//    pub_result_ = nh_.advertise<convert_2d_to_3d::Result>("detected_object", 10);

    ros::spin();

    delete pointcloud_sub_;
    delete boundingboxes_sub_;

    return 0;
}

