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

ros::Publisher t_pub;

// *********** Uncomment code below while using Baxter!
// void cuffcallback(const baxter_core_msgs::DigitalIOState &cuff_msg)
// {
//     twodto3d::mymessage timer;

//     timer.header.frame_id = "/doesntmatter";
//     timer.data = 0.0;

//     if (cuff_msg.state)
//     {
//         ros::Duration(0.5).sleep();
//         timer.header.stamp = ros::Time::now();
//         t_pub.publish(timer);

//         ROS_INFO("cuff pressed.");
//     }
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_pub");
    ros::NodeHandle n;

    t_pub = n.advertise<twodto3d::mymessage>("/cuff_status", 1);

    ROS_INFO("Ready to capture!");

    // uncomment both lines below For Baxter only
    // ros::Subscriber sub_cuff = n.subscribe("/robot/digital_io/right_lower_button/state", 1, cuffcallback);
    // ros::spin();

    // Uncomment for other robots
    ros::Rate loop_rate(10);
    twodto3d::mymessage timer;

    timer.header.frame_id = "/doesntmatter";
    timer.data = 0.0;
    while (ros::ok())
    {
        char waitkey;
        ROS_INFO("Press ENTER to start capturing!")
        std::cin >> waitkey; // This will wait until the user presses ENTER
        if (waitkey)
        {
            ros::Duration(0.5).sleep();
            timer.header.stamp = ros::Time::now();
            t_pub.publish(timer);

            ROS_INFO("Enter pressed.");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
