/*
  Программа предназначена для перемещения ВДРК по окружности радиусом radiusTubeFromUser.
  В качестве аргументов - скорость перемещения роботов.
  Зная расстояние от маркера до камеры, 
  сперва перемещается передний робот до заданного расстояния между роботами,
  затем его догоняет задний робот.
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/ModelStates.h>
#include <boost/asio.hpp>

#define MIN_ROBOTS_DIST       1.48442                             // минимальное  расстояние между камерой и маркером   [м]
#define MAX_ROBOTS_DIST       1.77312                             // максимальное расстояние между камерой и маркером   [м]
#define ROBOTS_DIST_PRECISION 0.00001                             // точность оценочного взаимного расположения роботов [м]

geometry_msgs::PoseStamped crntArOdomPose;                        // текущее   фактическое положение _маркера_ относительно ГСК     
geometry_msgs::PoseStamped crntCamOdomPose;                       // текущее   фактическое положение _камеры_  относительно ГСК  
geometry_msgs::Twist       velVdrkMsg;                            // сообщение скорости для роботов [м/с]
ros::Publisher             velCmdCamPub;                          // паблишер  скорости для камеры
ros::Publisher             velCmdArPub;                           // паблишер  скорости для маркера
ros::Subscriber            crntOdomPoseSub;

double vel4VdrkFromUser       = 0;                                // скорость для роботов от пользователя [м/с]
double radiusTubeFromUser     = 0;                                // радиус трубы от пользователя [м]
double lenOfAr2CamOffset      = 0;                                // оценочное расстояние между маркером и камерой [м]
bool camera_is_stop           = true;
bool marker_is_stop           = true;
bool getOdomPoses             = false;                            // получены фактические положения относительно ГСК

void setStopVdrk(){
  velVdrkMsg.linear.x  = 0.0;
  velVdrkMsg.linear.y  = 0.0;
  velVdrkMsg.linear.z  = 0.0;
  velVdrkMsg.angular.x = 0.0;
  velVdrkMsg.angular.y = 0.0;
  velVdrkMsg.angular.z = 0.0;
}

void marker_go(){
  if ((ROBOTS_DIST_PRECISION < (MAX_ROBOTS_DIST - lenOfAr2CamOffset)) && camera_is_stop) {
    velVdrkMsg.linear.y  = vel4VdrkFromUser;
    velVdrkMsg.angular.z = (-1.0) * vel4VdrkFromUser / radiusTubeFromUser;
    velCmdArPub.publish(velVdrkMsg);
    marker_is_stop       = false;
  } else {
    setStopVdrk();
    velCmdArPub.publish(velVdrkMsg);
    marker_is_stop       = true;
    std::this_thread::sleep_for(std::chrono::seconds(0));
  }
}

void camera_go(){
  if (((lenOfAr2CamOffset - MIN_ROBOTS_DIST) > ROBOTS_DIST_PRECISION) && marker_is_stop) {
    velVdrkMsg.linear.y  = vel4VdrkFromUser;
    velVdrkMsg.angular.z = (-1.0) * vel4VdrkFromUser / radiusTubeFromUser;
    velCmdCamPub.publish(velVdrkMsg);
    camera_is_stop       = false;
  } else {
    setStopVdrk();
    velCmdCamPub.publish(velVdrkMsg);
    camera_is_stop       = true;
    std::this_thread::sleep_for(std::chrono::seconds(0));
  }
}

// расстояние между точками
double getDistance(const double t1, const double t2) {                
  return t1 - t2;
}

void findAr2CamOffset(geometry_msgs::Vector3& ar2CamOffset){
  ar2CamOffset.x = abs(getDistance(crntArOdomPose.pose.position.x, crntCamOdomPose.pose.position.x));
  ar2CamOffset.y = abs(getDistance(crntArOdomPose.pose.position.y, crntCamOdomPose.pose.position.y));
  ar2CamOffset.z = abs(getDistance(crntArOdomPose.pose.position.z, crntCamOdomPose.pose.position.z));
}

void findLenOfAr2CamOffset(){
  geometry_msgs::Vector3 ar2CamOffset;
  findAr2CamOffset(ar2CamOffset);
  lenOfAr2CamOffset = std::sqrt(std::pow(ar2CamOffset.x, 2) +
                                std::pow(ar2CamOffset.y, 2) +
                                std::pow(ar2CamOffset.z, 2));
  ROS_INFO("\nlenOfAr2CamOffset = %.5f\n", lenOfAr2CamOffset);
}

void go_vdrk_in_radius(){
  findLenOfAr2CamOffset();
  marker_go();
  camera_go();
}

// получаем текущее фактическое положение _маркера_ относительно ГСК И
// получаем текущее фактическое положение _камеры_  относительно ГСК 
void getCrntOdomPoseHandler(const gazebo_msgs::ModelStates& arucoGazeboMsg) {
  crntArOdomPose.pose.position.x     = arucoGazeboMsg.pose[1].position.x;
  crntArOdomPose.pose.position.y     = arucoGazeboMsg.pose[1].position.y;
  crntArOdomPose.pose.position.z     = arucoGazeboMsg.pose[1].position.z;
  crntArOdomPose.pose.orientation.w  = arucoGazeboMsg.pose[1].orientation.w;
  crntArOdomPose.pose.orientation.x  = arucoGazeboMsg.pose[1].orientation.x;
  crntArOdomPose.pose.orientation.y  = arucoGazeboMsg.pose[1].orientation.y;
  crntArOdomPose.pose.orientation.z  = arucoGazeboMsg.pose[1].orientation.z;
  crntCamOdomPose.pose.position.x    = arucoGazeboMsg.pose[2].position.x;
  crntCamOdomPose.pose.position.y    = arucoGazeboMsg.pose[2].position.y;
  crntCamOdomPose.pose.position.z    = arucoGazeboMsg.pose[2].position.z;
  crntCamOdomPose.pose.orientation.w = arucoGazeboMsg.pose[2].orientation.w;
  crntCamOdomPose.pose.orientation.x = arucoGazeboMsg.pose[2].orientation.x;
  crntCamOdomPose.pose.orientation.y = arucoGazeboMsg.pose[2].orientation.y;
  crntCamOdomPose.pose.orientation.z = arucoGazeboMsg.pose[2].orientation.z;
  getOdomPoses = true;
}

void setup(ros::NodeHandle& node) {
  crntOdomPoseSub      = node.subscribe("/gazebo/model_states",  0, getCrntOdomPoseHandler);
  velCmdCamPub         = node.advertise<geometry_msgs::Twist>("/camera_cmd_vel", 0);
  velCmdArPub          = node.advertise<geometry_msgs::Twist>("/aruco_cmd_vel",  0);
  velVdrkMsg.linear.x  = 0.0;
  velVdrkMsg.linear.y  = 0.0;
  velVdrkMsg.linear.z  = 0.0;
  velVdrkMsg.angular.x = 0.0;
  velVdrkMsg.angular.y = 0.0;
  velVdrkMsg.angular.z = 0.0;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "go_in_radius");
  ros::NodeHandle node;
  ros::param::param<double>("~Velocity",    vel4VdrkFromUser,   0);
  ros::param::param<double>("~Radius_tube", radiusTubeFromUser, 0);
  setup(node);
  ros::Rate loop_rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    if (!getOdomPoses) continue;
    getOdomPoses = false;
    go_vdrk_in_radius();
    loop_rate.sleep();
  }
  return 0;
}