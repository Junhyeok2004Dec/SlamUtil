#include <fstream>
#include <vector>
#include <thread>

#include "ros/ros.h"

#include <slamUtil/Trajectory.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class SlamUtil
{

public:
    SlamUtil();
    ~SlamUtil();


    // 폴더에서 OccupancyGrid 형식의 맵을 
    // system(..) 으로 로드한다.
    void changeLapAndResetMap();
    void saveCurrentMap(const std::string& map_file_name);
    void resetSlamProcessor();
    void loadPreviousMap(const std::string& map_file_name);
    void markerPublisher(const geometry_msgs::Pose& slam_pose);
    void poseCallback(const geometry_msgs::PoseStamped& _pose);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

protected:
    ros::NodeHandle node;

    ros::Publisher lap_pub; // 몇 번쨰 lap인지 확인하기 위함
    ros::Publisher trajectory_pub;
    ros::Publisher hector_msg_pub;
    ros::Publisher marker_pub; // waypoint 가시화 하기 위함
    ros::Publisher center_msg_pub;
    ros::Publisher drive_pub;

    geometry_msgs::Pose pose_data, previous_pose_data, origin_pose;
    
    slamUtil::Trajectory trajectory;

    ros::Subscriber pose_sub;
    ros::Subscriber scan_sub_;


    visualization_msgs::Marker pointMarker;
    visualization_msgs::MarkerArray trajectoryMarker;

float distance_from_Origin; // 초기 지점과의 거리 -> used loop closure
bool isCenter; 
bool changelap;
bool isFirst;
int lap;
int waypointCount;
    
    
};