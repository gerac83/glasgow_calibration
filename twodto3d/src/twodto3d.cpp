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
#include <camera_info_manager/camera_info_manager.h>
#include <baxter_core_msgs/DigitalIOState.h>
#include <std_msgs/Time.h>
#include <twodto3d/mymessage.h>
using namespace sensor_msgs;
using namespace message_filters;


//typedef pcl::PointXYZ PointT;
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> cloud_pcl;

cv_bridge::CvImagePtr cv_ptr;
geometry_msgs::TransformStamped transformConverter;
ros::Publisher pub_info, pub_img, pub;
bool status = false;

//void pcdcallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::PointCloud2ConstPtr& msg, const CameraInfoConstPtr& cam_info, const boost::shared_ptr<const geometry_msgs::PointStamped>& point_ptr)

// void pcdcallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::PointCloud2ConstPtr& msg, const CameraInfoConstPtr& cam_info)

// {
  
//   pcl::fromROSMsg(*msg, cloud_pcl);
//   std::cout<< "pcd time "<< msg->header.stamp << std::endl;
//   std::cout<< std::endl << "image time "<< img->header.stamp;
  
  //baxter_core_msgs::DigitalIOState iff;
// }
// void cuffcallback(const baxter_core_msgs::DigitalIOState& cuff_msg)
// {

//   ROS_INFO("Press Cuff to Capture.");
//  status = bool(cuff_msg.state);
//  if(status)
//  ROS_INFO("cuff pressed.");

// }
//void rgbcallback(const sensor_msgs::ImageConstPtr& msg, const CameraInfoConstPtr& cam_info)

void rgbcallback(const sensor_msgs::ImageConstPtr& msg, const CameraInfoConstPtr& cam_info, const twodto3d::mymessageConstPtr& status)
{

    std::cout<< std::endl << "image time "<< msg->header.stamp;
    std::cout<< std::endl << "info time "<< cam_info->header.stamp;
  
    //ros::Duration(5.0).sleep();

       tf::TransformListener listener;
       bool success = false;
  //ros::Rate rate(10.0);
 // while (!success){
    tf::StampedTransform transform;
    try{
      
      listener.waitForTransform("/torso", "/right_gripper", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("/torso", "/right_gripper", ros::Time(0), transform);
      std::cout<< std::endl << "transform time "<< transform.stamp_ << std::endl;
      //ROS_INFO("Successful.");
    //  success = true;
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
   //   success = false;
    }

    

    ros::Time current_time, last_time;
        current_time = ros::Time::now();
        last_time = ros::Time::now();
    //ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>("/leader/tf", 1);

    // OK GOOD TO GO, let me knwo the outcomes :)
       // SO IT WAS CREATEQUATERIONTFTOMSG, NOW THIS IS quaternionTFToMsg, INTENTIONALLY?


    double yaw; 


ros::Rate loopRate(10);

    yaw =   tf::getYaw(transform.getRotation()); 

 
double th = yaw;
 geometry_msgs::Quaternion odom_quat;
 tf::quaternionTFToMsg(transform.getRotation(), odom_quat);
    transformConverter.header.stamp = current_time; 
    transformConverter.header.frame_id = transform.frame_id_;
    transformConverter.child_frame_id = transform.child_frame_id_;
    transformConverter.transform.translation.x = transform.getOrigin().x();
    transformConverter.transform.translation.y = transform.getOrigin().y();
    transformConverter.transform.translation.z = transform.getOrigin().z();
    transformConverter.transform.rotation = odom_quat;
    ROS_INFO_STREAM("X pose " <<  transformConverter.transform.translation.x );
    ROS_INFO_STREAM("Y pose " << transformConverter.transform.translation.y);
    ROS_INFO_STREAM("Z pose " << transformConverter.transform.translation.z );
    ROS_INFO_STREAM("Yaw "<< yaw );

    pub.publish(transformConverter);


    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_ptr->header.stamp = current_time;
    

    //msg->header.stamp = 0;
   // cam_info->header.stamp = current_time;

    sensor_msgs::CameraInfo info;
    info = *cam_info;
    // Camera_info_manager::CameraInfoManager info_mgr;
    // // get camera parameters 
    //         sensor_msgs::CameraInfo info = info_mgr.getCameraInfo();
             info.header.stamp = current_time;
    //         info.header.frame_id = cam_info->header.frame_id; // whatever it is called
    //         info.width = cv_ptr->cols;
    //         info.height = cv_ptr->rows;





      pub_img.publish(cv_ptr->toImageMsg());
      pub_info.publish(info);

}

bool twod_to_3d(twodto3d::twodto3d::Request &req, twodto3d::twodto3d::Response &res)


{

	  std_msgs::Float32MultiArray twod_array;
	  std_msgs::Float32MultiArray threed_array;
	  twod_array = req.two_d_array;
	  int TwoDCoordinate = 0;
	  twod_array.data.begin();
	  float x, y;



	  for(std::vector<float>::const_iterator it = req.two_d_array.data.begin(); it != req.two_d_array.data.end(); ++it)
	{
		if(TwoDCoordinate < 1)
			x = *it;
		else if(TwoDCoordinate <2 && TwoDCoordinate > 1)
		{
			y = *it;
			TwoDCoordinate = 0;
			pcl::PointXYZ dpoint = cloud_pcl(x,y);
			threed_array.data.push_back(dpoint.x);
			threed_array.data.push_back(dpoint.y);
			threed_array.data.push_back(dpoint.z);

		}


		}
		res.three_d_array = threed_array;
	

 // //  for (int i = 0; i < fpfhs->points.size (); i++)
 // //  {
 // //    for(int j = 0; j < fpfhLength; j++){
 // //      //desc_d[i][j] = fpfhs->points[i].histogram[j];
 // //      fpfhs_array.data.push_back(fpfhs->points[i].histogram[j]);
 // //    }
 // //  }

 //  res.fpfh_feature = fpfhs_array;
 //  res.centroid_array = centroid_array;
  return true;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_2d_to_3d");
  ros::NodeHandle n;
  pub = n.advertise<geometry_msgs::TransformStamped>("/transform_GrippertoBase", 1);
  pub_img = n.advertise<Image>("/Image_for_Calibration", 1);
  pub_info = n.advertise<CameraInfo>("/Cam_info_for_Calibration", 1);
  ros::ServiceServer service = n.advertiseService("converting2dto3d", twod_to_3d);
  ROS_INFO("Ready to convert.");

  // image_transport::ImageTransport it(n);
  // image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, rgbcallback);
  //ros::Subscriber sub_cuff = n.subscribe("/robot/digital_io/right_lower_cuff/state", 1, cuffcallback);



  message_filters::Subscriber<Image> image_sub(n, "/zed/zed_node/rgb_raw/image_raw_color", 1);
  message_filters::Subscriber<CameraInfo> info_sub(n, "/zed/zed_node/rgb_raw/camera_info", 1);
  message_filters::Subscriber<twodto3d::mymessage> cuff_sub(n, "/cuff_status", 1);

  

   //typedef sync_policies::ApproximateTime<Image, CameraInfo> MySyncPolicy;
   typedef sync_policies::ApproximateTime<Image, CameraInfo, twodto3d::mymessage> MySyncPolicy;
  
  // // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub, cuff_sub);
  //Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub);
  sync.registerCallback(boost::bind(&rgbcallback, _1, _2, _3));
  //pub.publish(transformConverter);


  ros::spin();

  return 0;
}
