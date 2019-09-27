#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/fpfh.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <string>
#include <sstream>
#include <unistd.h>
#include <vector>
#include <twodto3d/twodto3d.h>
#include <pcl/common/common.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float32MultiArray.h"
#include "tf/message_filter.h"
#include <baxter_core_msgs/DigitalIOState.h>
#include <twodto3d/mymessage.h>
#include <std_msgs/Time.h>
#include <stdio.h>
using namespace sensor_msgs;
using namespace message_filters;


//typedef pcl::PointXYZ PointT;
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);

ros::Publisher t_pub;
void cuffcallback(const baxter_core_msgs::DigitalIOState& cuff_msg)
{

 
 twodto3d::mymessage timer;
 
 timer.header.frame_id = "/world";
 timer.data = 0.0;

  // sensor_msgs::Image output_image;
  // output_image.header.stamp     = ros::Time(0);
  // output_image.height           = 320;
  // output_image.width            = 280;
  // output_image.encoding         = "rgb8";
  // output_image.is_bigendian     = false;
  // output_image.step             = 3 * 320;

 char waitkey ;
 //std::cin >> waitkey;
 if(cuff_msg.state)
 //if(waitkey)
 {
  ros::Duration(0.5).sleep();
  timer.header.stamp = ros::Time::now();
  t_pub.publish(timer);

 ROS_INFO("cuff pressed.");
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "transform_pub");
  ros::NodeHandle n;

  ROS_INFO("Ready to capture. Press any key to continue...");

  // image_transport::ImageTransport it(n);
  // image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, rgbcallback);
  // ros::Subscriber sub_pcd = n.subscribe("/camera/depth/points", 1, pcdcallback);
  
  // tf::TransformListener tf_;
  // std::string target_frame_ = "/base";

  // std::cout << "target_frame_";
  // tf::MessageFilter<geometry_msgs::PointStamped> * tf_filter_;
  //   message_filters::Subscriber<geometry_msgs::PointStamped> point_sub(n, "/left_hand", 10);
    
  //   tf_filter_ = new tf::MessageFilter<geometry_msgs::PointStamped>(point_sub, tf_, target_frame_, 10);
  //   tf_filter_->registerCallback( boost::bind(&msgCallback, _1) );
    t_pub = n.advertise<twodto3d::mymessage>("/cuff_status", 1);

    ros::Subscriber sub_cuff = n.subscribe("/robot/digital_io/right_lower_button/state", 1, cuffcallback);

  
    //t_pub.publish(transform);
 

  

  // message_filters::Subscriber<Image> image_sub(n, "/camera/rgb/image_raw", 10);
  // message_filters::Subscriber<PointCloud2> pcd_sub(n, "/camera/depth/points", 10);
  // message_filters::Subscriber<CameraInfo> info_sub(n, "/camera/depth/camera_info", 10);
  

  

  //  typedef sync_policies::ApproximateTime<Image, CameraInfo, geometry_msgs::PointStamped> MySyncPolicy;
  
  // // // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  // Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image_sub, info_sub, *tf_filter_);
  // sync.registerCallback(boost::bind(&rgbcallback, _1, _2,_3 ));


  ros::spin();

  return 0;
}
