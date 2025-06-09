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

#define MIN_ROBOTS_DIST       1                               // минимальное  расстояние между камерой и маркером   [м]
#define MAX_ROBOTS_DIST       3                               // максимальное расстояние между камерой и маркером   [м]
#define ROBOTS_DIST_PRECISION 0.01                            // точность оценочного взаимного расположения роботов [м]

double kP = 0.1;         // Коэффициент П-регулятора для угловой скорости

geometry_msgs::PoseStamped crntArOdomPose;                        // текущее   фактическое положение _маркера_ относительно ГСК     
geometry_msgs::PoseStamped crntCamOdomPose;                       // текущее   фактическое положение _камеры_  относительно ГСК  
geometry_msgs::Twist       velVdrkMsg;                            // сообщение скорости для роботов [м/с]
ros::Publisher             velCmdCamPub;                          // паблишер  скорости для камеры
ros::Publisher             velCmdArPub;                           // паблишер  скорости для маркера
ros::Subscriber            crntOdomPoseSub;                       // текущее положение

double vel4VdrkFromUser       = 0;                                // скорость для роботов от пользователя [м/с]
double radiusTubeFromUser     = 0;                                // радиус трубы от пользователя [м]
double lenOfAr2CamOffset      = 0;                                // точное расстояние между маркером и камерой [м]
bool camera_is_stay           = true;
bool marker_is_stay           = true;
bool getOdomPoses             = false;                            // получены фактические положения относительно ГСК

void findAr2CamOffset(geometry_msgs::Vector3 &ar2CamOffset);

void setStopVdrk(){
  velVdrkMsg.linear.x  = 0.0;
  velVdrkMsg.linear.y  = 0.0;
  velVdrkMsg.linear.z  = 0.0;
  velVdrkMsg.angular.x = 0.0;
  velVdrkMsg.angular.y = 0.0;
  velVdrkMsg.angular.z = 0.0;
}

void marker_go(){
  if ((ROBOTS_DIST_PRECISION < (MAX_ROBOTS_DIST - lenOfAr2CamOffset)) && camera_is_stay) {
    velVdrkMsg.linear.y  = vel4VdrkFromUser;
    velVdrkMsg.angular.z = (-1.0) * vel4VdrkFromUser / radiusTubeFromUser;

    ROS_INFO("marker_go velVdrkMsg.angular.z: [%f]", velVdrkMsg.angular.z);

    velCmdArPub.publish(velVdrkMsg);
    marker_is_stay       = false;
  } else {
    setStopVdrk();
    velCmdArPub.publish(velVdrkMsg);
    marker_is_stay       = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(0));
  }
}

// Где находится маркер относительно камеры в реальности
void tryToGetAr2CamTransform(tf::TransformListener& listener, tf::StampedTransform& transform){
  try{
    geometry_msgs::Vector3 ar2CamOffset;
    findAr2CamOffset(ar2CamOffset);

    transform.setOrigin(tf::Vector3(ar2CamOffset.x, ar2CamOffset.y, ar2CamOffset.z));

    ROS_INFO("Transform: [%f, %f, %f]", 
             transform.getOrigin().x(),
             transform.getOrigin().y(),
             transform.getOrigin().z());
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

void camera_go(){
  if ((ROBOTS_DIST_PRECISION < (lenOfAr2CamOffset - MIN_ROBOTS_DIST)) && marker_is_stay) {
    velVdrkMsg.linear.y  = vel4VdrkFromUser;
    velVdrkMsg.angular.z = (-1.0) * vel4VdrkFromUser / radiusTubeFromUser;

    ROS_INFO("camera_go velVdrkMsg.angular.z: [%f]", velVdrkMsg.angular.z);

    velCmdCamPub.publish(velVdrkMsg);

    camera_is_stay       = false;
  } else {
    setStopVdrk();
    velCmdCamPub.publish(velVdrkMsg);
    camera_is_stay       = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(0));
  }
}

void findAr2CamOffset(geometry_msgs::Vector3 &ar2CamOffset){
  ar2CamOffset.x = crntArOdomPose.pose.position.x - crntCamOdomPose.pose.position.x;
  ar2CamOffset.y = crntArOdomPose.pose.position.y - crntCamOdomPose.pose.position.y;
  ar2CamOffset.z = crntArOdomPose.pose.position.z - crntCamOdomPose.pose.position.z;
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
void getCrntOdomPoseHandler(const gazebo_msgs::ModelStates& gazebo_msg) {
  crntArOdomPose.pose.position.x     = gazebo_msg.pose[1].position.x;
  crntArOdomPose.pose.position.y     = gazebo_msg.pose[1].position.y;
  crntArOdomPose.pose.position.z     = gazebo_msg.pose[1].position.z;
  crntArOdomPose.pose.orientation.w  = gazebo_msg.pose[1].orientation.w;
  crntArOdomPose.pose.orientation.x  = gazebo_msg.pose[1].orientation.x;
  crntArOdomPose.pose.orientation.y  = gazebo_msg.pose[1].orientation.y;
  crntArOdomPose.pose.orientation.z  = gazebo_msg.pose[1].orientation.z;
  crntCamOdomPose.pose.position.x    = gazebo_msg.pose[2].position.x;
  crntCamOdomPose.pose.position.y    = gazebo_msg.pose[2].position.y;
  crntCamOdomPose.pose.position.z    = gazebo_msg.pose[2].position.z;
  crntCamOdomPose.pose.orientation.w = gazebo_msg.pose[2].orientation.w;
  crntCamOdomPose.pose.orientation.x = gazebo_msg.pose[2].orientation.x;
  crntCamOdomPose.pose.orientation.y = gazebo_msg.pose[2].orientation.y;
  crntCamOdomPose.pose.orientation.z = gazebo_msg.pose[2].orientation.z;
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
  ros::NodeHandle nh("~");

  nh.getParam("velocity", vel4VdrkFromUser);
  nh.getParam("radius_tube", radiusTubeFromUser);

  ROS_INFO("\nvelocity = %.5f\n", vel4VdrkFromUser);
  ROS_INFO("\nradius_tube = %.5f\n", radiusTubeFromUser);

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