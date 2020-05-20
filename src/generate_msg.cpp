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

class ConvertBoundingBoxNode
{
    public:
        ConvertBoundingBoxNode()
        {
			pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &ConvertBoundingBoxNode::pointcloud_callback, this);
			boundingboxes_sub_ = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, &ConvertBoundingBoxNode::boundingbox_callback, this);
			pub_result_ = nh_.advertise<convert_2d_to_3d::Result>("detected_object", 1);

            //cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
            ROS_INFO("[%s] initialized...", ros::this_node::getName().c_str());
        }
        ~ConvertBoundingBoxNode()
        {
            delete pointcloud_sub_;
            delete boundingboxes_sub_;
        }

	void boundingbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& boundingbox) {
		gBoundingboxes.bounding_boxes.clear();
		for(int i = 0; i < boundingbox->bounding_boxes.size(); i++) {
		    gBoundingboxes.bounding_boxes.push_back(boundingbox->bounding_boxes[i]);
		}

	}

	void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud)
	{
		pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
		pcl::fromROSMsg(*pointcloud, pcl_cloud);

		if(gBoundingboxes.bounding_boxes.empty())
		    return;

	//    printf("Bounding boxes size : %d\n", gBoundingboxes.bounding_boxes.size());

		double x = 0.0;
		double y = 0.0;
		double z = 0.0;
		std::string id = "";

		for(int i = 0; i < gBoundingboxes.bounding_boxes.size(); i++) {
		    pcl::PointXYZRGB p = pcl_cloud( (gBoundingboxes.bounding_boxes[i].xmin + gBoundingboxes.bounding_boxes[i].xmax)/2,
		                                    (gBoundingboxes.bounding_boxes[i].ymin + gBoundingboxes.bounding_boxes[i].ymax)/2);
		    x = p.x;
		    y = p.y;
		    z = p.z;
		    id = gBoundingboxes.bounding_boxes[i].Class;
//		    std::cout <<"point : " << p << std::endl;
//		    std::cout <<" ==> " << p.x << " , " << p.y << " , " << p.z << std::endl;

		    // create static tf
/*
		    geometry_msgs::TransformStamped transformStamped;
		    std::string object_coordinate = "object_coordinate";
		    transformStamped.header.frame_id = "camera_color_optical_frame";
		    transformStamped.child_frame_id = object_coordinate;

		    transformStamped.transform.translation.x = x;
		    transformStamped.transform.translation.y = y;
		    transformStamped.transform.translation.z = z;

		    transformStamped.transform.rotation.x = 0;
		    transformStamped.transform.rotation.y = 0;
		    transformStamped.transform.rotation.z = 0;
		    transformStamped.transform.rotation.w = 1;

		    transformStamped.header.stamp = ros::Time::now();
		    tfb_.sendTransform(transformStamped);
*/
		    // Publish result
		    convert_2d_to_3d::Result msg;
		    msg.type = "detected_object";
		    msg.data = id;

		    msg.pose.header.stamp = ros::Time::now();
		    msg.pose.header.frame_id = "object_coordinate";
		    msg.pose.pose.position.x = x;
		    msg.pose.pose.position.y = y;
		    msg.pose.pose.position.z = z;

		    msg.pose.pose.orientation.x = 0;
		    msg.pose.pose.orientation.y = 0;
		    msg.pose.pose.orientation.z = 0;
		    msg.pose.pose.orientation.w = 1;

		    pub_result_.publish(msg);
		}

		ros::Duration(0.01).sleep();
	}

    private:
        ros::NodeHandle nh_;
        tf2_ros::TransformBroadcaster tfb_;
	ros::Subscriber pointcloud_sub_;
	ros::Subscriber boundingboxes_sub_;
	ros::Publisher pub_result_;
	darknet_ros_msgs::BoundingBoxes gBoundingboxes;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "convert_boundingbox_node");
    ConvertBoundingBoxNode m = ConvertBoundingBoxNode();
    ros::spin();
    return 0;
}


