
#include "SlamUtil.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>

#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>


#include <slamUtil/Trajectory.h>


#define ORIGIN_RANGE 1.0 // 1.0 중심반경
#define DISTNACE_INTERVAL 0.5 // 0.5m 간격으로 점을 찍도록 하마......... hippo
float distance_from_Origin; // 초기 지점과의 거리 -> used loop closure
float distance_from_previous_position;
bool isCenter = false; 
bool changelap = false;
bool isFirst = true;
int lap = 0;
int waypointCount = 0;






SlamUtil::SlamUtil() {

  ros::NodeHandle node;
    
  lap = 0;

  ROS_INFO("Hello");
  lap_pub = node.advertise<std_msgs::String>("/lap", 10);   
  marker_pub = node.advertise<visualization_msgs::MarkerArray>("/waypoint", 10);
  hector_msg_pub = node.advertise<std_msgs::String>("syscommand", 10);

  pose_sub = node.subscribe("/slam_out_pose",100, &SlamUtil::poseCallback, this);
 

}

SlamUtil::~SlamUtil() {

}


void SlamUtil::poseCallback(const geometry_msgs::PoseStamped& _pose) {
    ROS_INFO("%d", 1);

    double x,y;





    pose_data = _pose.pose;

    

    x = pose_data.position.x - previous_pose_data.position.x;
    y = pose_data.position.y - previous_pose_data.position.y;

    double circle = sqrt(pow(x, 2) + pow(y, 2));


    
    if(circle > pow(DISTNACE_INTERVAL, 2))  // 0.5m 이상 경계 외 지역에 있다면
    {
        markerPublisher(pose_data);
        previous_pose_data = _pose.pose;
    }


    x = origin_pose.position.x - pose_data.position.x;
    y = origin_pose.position.y - pose_data.position.y;

    


     //for loop closure -> 거리 확인
    distance_from_Origin = sqrt(
        pow(x,2) + pow(y,2)
        ); 

        // 중심까지의 거리를 확인하여라!
    if (distance_from_Origin > ORIGIN_RANGE)  isCenter = false; 
    else isCenter = true;

    if(isCenter) {
        changelap = true;   
    }

    if(!isCenter && (changelap && ( distance_from_Origin > ORIGIN_RANGE + 0.6))) {
        // 중심에서 벗어난 뒤 바로 실행할 경우, 경계면에서 문제 발생 -ㅣ> 경계면에서 벗어난 이후에 add lap
        
        changelap = false;
        if(isFirst) {
            isFirst = false;
        }else {
            
            origin_pose.position.x = (-1) * _pose.pose.position.x;
            origin_pose.position.y = (-1) * _pose.pose.position.y;
            origin_pose.position.z = 0;
            
            changeLapAndResetMap();
            ROS_INFO("123");
        }      


        
    }




  std_msgs::String lapData;

  lapData.data = std::to_string(lap);
  lap_pub.publish(lapData);



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

    

    pointMarker.color.r = 0.3;
    pointMarker.color.g = 0.1;
    pointMarker.color.b = 0.9;
    pointMarker.color.a = 1.0;
    
    pointMarker.scale.x = 0.1;
    pointMarker.scale.y = 0.1;
    pointMarker.scale.z = 0.1;


    trajectoryMarker.markers.push_back(pointMarker);
    marker_pub.publish(trajectoryMarker);

    waypointCount++;
}


void SlamUtil::changeLapAndResetMap() {
    lap++;

    //SAVE
    std::string map_file_name = "/home/ak47/maps/map" + std::to_string(lap-1) + "";
    saveCurrentMap(map_file_name);

    
    resetSlamProcessor();

    //LOAD
    std::string load_map_file = "/home/ak47/maps/map" + std::to_string(lap-1) + ".yaml";
    loadPreviousMap(load_map_file);
}

void SlamUtil::saveCurrentMap(const std::string& map_file_name) {
    // 현재 맵을 저장하기 위한 로직을 구현합니다.
    // map_server를 사용하여 맵을 파일로 저장할 수 있습니다.
    ROS_INFO("Saving current map to %s", map_file_name.c_str());

    // 예시 코드: map_server를 호출하여 맵 저장
    system(("rosrun map_server map_saver map:=/map -f " + map_file_name).c_str());
}

void SlamUtil::resetSlamProcessor() {
    // Hector SLAM 프로세서를 리셋하여 새로운 SLAM을 시작할 수 있도록 합니다.
    
    std_msgs::String reset;
    reset.data = "reset";
    hector_msg_pub.publish(reset);

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
    system(("rosrun map_server map_server map:=/map1s " + map_file_name +" &").c_str());
}