#include "ros/ros.h"

//darknet_ros_msgs
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/ObjectCount.h"

static int nBoxes = 0;

void nBoxesCallback(const darknet_ros_msgs::ObjectCount::ConstPtr& msg)
{
  nBoxes = msg->count;
}


void msgCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  // 메시지 표시
  for(int i=0; i<nBoxes; i++)
  {
    ROS_INFO("xmin = %ld", msg->bounding_boxes[i].xmin);
    ROS_INFO("xmax = %ld", msg->bounding_boxes[i].xmax);
    ROS_INFO("ymin = %ld", msg->bounding_boxes[i].ymin);
    ROS_INFO("ymax = %ld", msg->bounding_boxes[i].ymax);
  }
  ROS_INFO("%d", nBoxes);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscribe_sub");
  ros::NodeHandle nh;
  
  ros::Subscriber sub_nboxes = nh.subscribe("/darknet_ros/found_object", 100, nBoxesCallback);
  ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes", 100, msgCallback);
  ros::spin();
  return 0;
}
