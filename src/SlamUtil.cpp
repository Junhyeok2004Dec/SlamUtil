
#include "SlamUtil.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>

#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>

#include <ackermann_msgs/AckermannDriveStamped.h>


#include <slamUtil/Trajectory.h>
#include <slamUtil/Position.h>
#include <sensor_msgs/LaserScan.h>


double ORIGIN_RANGE = 2.0;// 1.0 중심반경
double DISTNACE_INTERVAL = 0.5; // 0.5m 간격으로 점을 찍도록 하마......... hippo
double DEAD_ZONE = 0.7;
double MAP_CHANGE_RANGE = 0.6;

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

  lap_pub = node.advertise<std_msgs::String>("/lap", 10);   
  marker_pub = node.advertise<slamUtil::Trajectory>("/trajectoryMarker", 10);
  trajectoryList_pub = node.advertise<slamUtil::Trajectory>("/trajectoryList", 100);

  hector_msg_pub = node.advertise<std_msgs::String>("syscommand", 10);  
  center_msg_pub = node.advertise<std_msgs::String>("/origin", 10);
  drive_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);
  pose_pub = node.advertise<geometry_msgs::PoseStamped>("/slam_out_pose", 100);
  
  scan_sub_ = node.subscribe("/scan", 10, &SlamUtil::scanCallback, this);


  //Todo drive pub

  pose_sub = node.subscribe("/slam_out_pose",100, &SlamUtil::poseCallback, this);
 
  origin_pose.position.x = 0;
  origin_pose.position.y = 0;
  origin_pose.position.z = 0;


  float ORIGIN_RANGE, DISTNACE_INTERVAL, DEAD_ZONE, MAP_CHANGE_RANGE;

  node.getParam("ORIGIN_RANGE", ORIGIN_RANGE);
  node.getParam("DISTNACE_INTERVAL", DISTNACE_INTERVAL);
  node.getParam("DEAD_ZONE", DEAD_ZONE);
  node.getParam("MAP_CHANGE_RANGE", MAP_CHANGE_RANGE);
  

}


SlamUtil::~SlamUtil() {
    

}


void SlamUtil::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // 정지코드 구현

        std::vector<float> valid_ranges;
        std::vector<float> angles;

        // 유효한 범위 데이터 및 해당 각도 추출
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            float range = scan_msg->ranges[i];
           
            if (range >= scan_msg->range_min && range <= scan_msg->range_max) {
                valid_ranges.push_back(range);
                angles.push_back(scan_msg->angle_min + i * scan_msg->angle_increment);

                if (scan_msg->ranges[i] < DEAD_ZONE) {
                    ackermann_msgs::AckermannDriveStamped stop_msg;
                    stop_msg.drive.speed = 0.0;
                    stop_msg.drive.steering_angle = 0.0;
                    drive_pub.publish(stop_msg);
            
                
            }

            }


        }

}

void SlamUtil::poseCallback(const geometry_msgs::PoseStamped& _pose) {
    ROS_INFO("%d", 1);

    double x,y;

    pose_data = _pose.pose;

    x = pose_data.position.x - previous_marker_pose_data.position.x;
    y = pose_data.position.y - previous_marker_pose_data.position.y;

    double circle = sqrt(pow(x, 2) + pow(y, 2));


    
    if(circle > 2.0 * pow(DISTNACE_INTERVAL, 2))  // 0.5m 이상 경계 외 지역에 있다면
    {
        if(lap < 2) {
            
        markerPublisher(pose_data);
        previous_marker_pose_data = _pose.pose;
        }
    }
    
    previous_lap_pose_data = _pose.pose;

    x = origin_pose.position.x - previous_lap_pose_data.position.x;
    y = origin_pose.position.y - previous_lap_pose_data.position.y;

         //for loop closure -> 거리 확인
    distance_from_Origin = sqrt(
        pow(x,2) + pow(y,2)
        ); 


    origin_pose.position.x = (-1)* pose_data.position.x;
    origin_pose.position.y = (-1) * pose_data.position.y;
    origin_pose.orientation.w = pose_data.orientation.w;
    origin_pose.position.z = 0;
    

    
        // 중심까지의 거리를 확인하여라!
    if (distance_from_Origin > ORIGIN_RANGE)  isCenter = false; 
    else isCenter = true;

    if(isCenter) {
        changelap = true;   
    }

    if(!isCenter && (changelap && ( distance_from_Origin > ORIGIN_RANGE + MAP_CHANGE_RANGE))) {
     
     	
        changelap = false;

	previous_lap_pose_data = _pose.pose;
	
        std_msgs::String centerMsg;
        std::string centerString = "";

        centerString = std::to_string(origin_pose.position.x) ;
        centerString += ",";
        centerString += std::to_string(origin_pose.position.y) ;

        centerMsg.data = centerString;
        changeLapAndResetMap();
        center_msg_pub.publish(centerMsg);
        pose_pub.publish(_pose); // load orientation before map refresh



    }

  std_msgs::String lapData;
  lapData.data = std::to_string(lap);

  if(lap == 0) {
  lapData.data = "1"; // 0번째 lap 또한 1번째 바퀴이므로 1번째로 publish하도록 통일

  }
  lap_pub.publish(lapData);

}

int trajectoryCount = 0;
void SlamUtil::trajectoryListPublisher(const geometry_msgs::Pose& _pose)
{
    slamUtil::Trajectory trajectoryList;
    slamUtil::Position position; 

    // 현재 위치 좌표를 설정
    position.x = _pose.position.x;
    position.y = _pose.position.y;

    // Trajectory 메시지에 좌표를 추가
    trajectoryList.position.push_back(position);

    // Trajectory 메시지의 ID 설정
    trajectoryList.id = trajectoryCount++;

    // Trajectory 메시지를 publish
    trajectoryList_pub.publish(trajectoryList);
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
    if(lap == 0) {
        lap++;

    } else {
        lap++;

    //SAVE
    //std::string map_file_name = "/home/user/maps/map" + std::to_string(lap-1) + "";
    std::string map_file_name = "/home/user/maps/mapPrev";
    saveCurrentMap(map_file_name);

    //resetSlamProcessor();

    //LOAD
    //std::string load_map_file = "/home/user/maps/map" + std::to_string(lap-1) + ".yaml";
    std::string load_map_file = "/home/user/maps/mapPrev.yaml";
    loadPreviousMap(load_map_file);
    }
}

void SlamUtil::saveCurrentMap(const std::string& map_file_name) {
    // 현재 맵을 저장하기 위한 로직을 구현합니다.
    // map_server를 사용하여 맵을 파일로 저장할 수 있습니다.
    ROS_INFO("Saving current map to %s", map_file_name.c_str());

    // 예시 코드: map_server를 호출하여 맵 저장
    system(("rosrun map_server map_saver -f map:=/map "+ map_file_name).c_str());
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
    
    ROS_INFO("Loading previous map from %s", map_file_name.c_str());
    //system(("rosrun map_server map_server map:=/map1s " + map_file_name).c_str());
    // map_server -> 직접 systemd에서 map_server을 진행한다. (LOAD)
    system(("rosrun map_server map_server map:=/mapOK " + map_file_name  +" &").c_str());
}
