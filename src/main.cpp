
#include <ros/ros.h>

#include "../include/SlamUtil.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slamUtil");

  SlamUtil slamutil;


  ros::spin();

  return(0);
}
