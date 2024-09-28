
#include "SlamUtil.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>

#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>

#include <slamUtil/Trajectory.h>



SlamUtil::SlamUtil() {

  ros::NodeHandle node;
    

  ROS_INFO("Hello");
  lap_pub = node.advertise<std_msgs::String>("/lap", 10);   
  marker_pub = node.advertise<visualization_msgs::MarkerArray>("/waypoint", 10);
  hector_msg_pub = node.advertise<std_msgs::String>("syscommand", 10);

  pose_sub = node.subscribe("/slam_out_pose",100, &SlamUtil::poseCallback, this);




    lap = 0;
    waypointCount = 0;


}

SlamUtil::~SlamUtil() {
    lap = 0;

}


void SlamUtil::poseCallback(const geometry_msgs::PoseStamped& _pose) {


    ros::Duration(0.5).sleep();
    geometry_msgs::Pose pose_data;
    pose_data = _pose.pose;

    markerPublisher(pose_data);





}


void SlamUtil::markerPublisher(const geometry_msgs::Pose& slam_pose) {


    pointMarker.header.frame_id = "map";
    pointMarker.header.stamp = ros::Time::now();

    pointMarker.type = visualization_msgs::Marker::SPHERE;
    pointMarker.id = waypointCount;
    pointMarker.action = visualization_msgs::Marker::ADD;

    pointMarker.pose.orientation.w = slam_pose.orientation.w;
    pointMarker.pose.position.x = slam_pose.position.x;
    pointMarker.pose.position.y = slam_pose.position.y;

    

    pointMarker.color.r = 0.8;
    pointMarker.color.g = 0.1;
    pointMarker.color.b = 0.1;
    pointMarker.color.a = 1.0;
    
    pointMarker.scale.x = 0.1;
    pointMarker.scale.y = 0.1;
    pointMarker.scale.z = 0.1;


    trajectoryMarker.markers.push_back(pointMarker);
    marker_pub.publish(trajectoryMarker);

    waypointCount++;
}


void SlamUtil::changeLapAndResetMap() {
    // lap을 증가시키고 새로운 lap에 대한 작업 수행
    lap++;
    // 현재까지의 맵을 저장

    
    std::string map_file_name = "/home/ak47/maps/map" + std::to_string(lap-1) + "";
    saveCurrentMap(map_file_name);

    // Hector SLAM의 맵을 초기화하고, 새로운 lap을 시작
    
    resetSlamProcessor();


    // 이전 lap에서 저장한 맵을 불러와서 SLAM 프로세서에 로드
    std::string load_map_file = "/home/ak47/maps/map" + std::to_string(lap-1) + ".yaml";

    loadPreviousMap(load_map_file);
}

void SlamUtil::saveCurrentMap(const std::string& map_file_name) {
    // 현재 맵을 저장하기 위한 로직을 구현합니다.
    // map_server를 사용하여 맵을 파일로 저장할 수 있습니다.
    ROS_INFO("Saving current map to %s", map_file_name.c_str());

    // 예시 코드: map_server를 호출하여 맵 저장
    system(("rosrun map_server map_saver map:=/map1s -f " + map_file_name).c_str());
}

void SlamUtil::resetSlamProcessor() {
    // Hector SLAM 프로세서를 리셋하여 새로운 SLAM을 시작할 수 있도록 합니다.
    
    std_msgs::String reset;

    reset.data = "reset";

    //reset slam Processor
    //slamProcessor->reset();


    //ROS_INFO("%f", slamProcessor->poseInfoContainer_.getPoseStamped().pose.position.x);
    //SlamUtil::resetPose(initial_pose);
    //initial_pose_ = Eigen::Vector3f(origin_x, origin_y, steering);
    
}

void SlamUtil::loadPreviousMap(const std::string& map_file_name) {
    
    ROS_INFO("[WIP] Loading previous map from %s", map_file_name.c_str());
    //system(("rosrun map_server map_server map:=/map1s " + map_file_name).c_str());


    // map_server -> 직접 systemd에서 map_server을 진행한다. (LOAD)
    system(("rosrun map_server map_server map:=/map " + map_file_name +" &").c_str());
}