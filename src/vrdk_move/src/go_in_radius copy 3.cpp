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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

#define MIN_ROBOTS_DIST       1.9                               // минимальное  расстояние между камерой и маркером   [м]
#define MAX_ROBOTS_DIST       2.5                               // максимальное расстояние между камерой и маркером   [м]
#define ROBOTS_DIST_PRECISION 0.01                            // точность оценочного взаимного расположения роботов [м]

double kP = 0.1;         // Коэффициент П-регулятора для угловой скорости

bool setup_yet = false;

geometry_msgs::Pose crntArOdomPose;                               // текущее   фактическое положение _маркера_ относительно ГСК     
geometry_msgs::Pose crntCamOdomPose;                              // текущее   фактическое положение _камеры_  относительно ГСК  

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


geometry_msgs::Pose desired_pose;    // желаемая поза для второго робота относительно первого


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
  setStopVdrk();  // обнуляем все скорости
  if ((ROBOTS_DIST_PRECISION < (MAX_ROBOTS_DIST - lenOfAr2CamOffset)) && camera_is_stay) {
    velVdrkMsg.linear.y  = vel4VdrkFromUser;
    velVdrkMsg.angular.z = (-1.0) * vel4VdrkFromUser / radiusTubeFromUser;

    ROS_INFO("marker_go velVdrkMsg.angular.z: [%f]", velVdrkMsg.angular.z);

    velCmdArPub.publish(velVdrkMsg);
    marker_is_stay       = false;
  } else {
    velCmdArPub.publish(velVdrkMsg);
    marker_is_stay       = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(0));
  }
}

// Где находится маркер относительно камеры в реальности в мировой системе координат
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

double getYawFromPose(const geometry_msgs::Pose& pose) {
    // Создаём tf2::Quaternion из сообщения ROS
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    );

    ROS_INFO("getYawFromPose px py pz: [%f, %f, %f]", 
             pose.position.x,
             pose.position.y,
             pose.position.z);

    // Преобразуем кватернион в матрицу вращения 3x3
    tf2::Matrix3x3 m(q);
    
    // Получаем углы Эйлера (roll, pitch, yaw)
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;  // Возвращаем угол поворота вокруг Z (в радианах)
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle <= -M_PI) angle += 2 * M_PI;
    return angle;
}


void camera_go(){

  if (!marker_is_stay) return;
  setStopVdrk();  // обнуляем все скорости

  tf::TransformListener listener;
  tf::StampedTransform transform;
  tryToGetAr2CamTransform(listener, transform);

  double current_yaw_cam = getYawFromPose(crntCamOdomPose); // находим текущий угол камеры
  double yaw_degrees = current_yaw_cam * (180.0 / M_PI);
  ROS_INFO("in camera_go current_yaw_cam: %f rad (%f deg)", current_yaw_cam, yaw_degrees);

  double dx = transform.getOrigin().x();                  // фактическое положение маркера относительно камеры в ГСК
  double dy = transform.getOrigin().y();                  // фактическое положение маркера относительно камеры в ГСК
  double cur_yaw = atan2(dy, dx) + current_yaw_cam;       // фактический угол между камерой и маркером в СК камеры

  double alpha = atan2(dy, dx) - getYawFromPose(crntArOdomPose);    // угол между нормалью робота и прямой, соединяющий камеру и маркер
  double r_desired = std::sqrt(std::pow(desired_pose.position.x, 2) + std::pow(desired_pose.position.y, 2)); // желаемое расстояние между роботом и маркером
  
  double dx_des_gl = r_desired * cos(alpha);
  double dy_des_gl = r_desired * sin(alpha);

  double error_dx = dx - dx_des_gl;         // ошибка по дельта х
  double error_dy = dy - dy_des_gl;         // ошибка по дельта y

  // double error_yaw = cur_yaw - getYawFromPose(desired_pose); // ошибка по углу

  ROS_INFO("in camera_go atan2(dy, dx): [%f]", atan2(dy, dx));   // 78.03 градусов
  ROS_INFO("in camera_go dx, dy, cur_yaw: [%f, %f, %f]", dx, dy, cur_yaw);
  ROS_INFO("in camera_go getYawFromPose(desired_pose): [%f]", getYawFromPose(desired_pose));
  ROS_INFO("in camera_go error_dx, error_dy,: [%f, %f]", error_dx, error_dy);

  if (abs(error_dx) > ROBOTS_DIST_PRECISION){
    velVdrkMsg.linear.x =  error_dx;
  }

  if (abs(error_dy) > ROBOTS_DIST_PRECISION){
    velVdrkMsg.linear.y =  error_dy;
  }

  // if (abs(error_yaw) > 0.001){
  //   // error_yaw = normalizeAngle(error_yaw);
  //   velVdrkMsg.angular.z = (-1.0) * error_yaw;
  // }

  ROS_INFO("camera_go velVdrkMsg.angular.z: [%f]", velVdrkMsg.angular.z);
  velCmdCamPub.publish(velVdrkMsg);
  camera_is_stay = false;

  if (abs(error_dx) <= ROBOTS_DIST_PRECISION && abs(error_dy) <= ROBOTS_DIST_PRECISION ){ //&& abs(error_yaw) < 0.001
    setStopVdrk();
    velCmdCamPub.publish(velVdrkMsg);
    camera_is_stay       = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(0));
  }
}

void findAr2CamOffset(geometry_msgs::Vector3 &ar2CamOffset){
  ar2CamOffset.x = crntArOdomPose.position.x - crntCamOdomPose.position.x;
  ar2CamOffset.y = crntArOdomPose.position.y - crntCamOdomPose.position.y;
  ar2CamOffset.z = crntArOdomPose.position.z - crntCamOdomPose.position.z;
}

void findLenOfAr2CamOffset(){
  geometry_msgs::Vector3 ar2CamOffset;
  findAr2CamOffset(ar2CamOffset);
  lenOfAr2CamOffset = std::sqrt(std::pow(ar2CamOffset.x, 2) +
                                std::pow(ar2CamOffset.y, 2) +
                                std::pow(ar2CamOffset.z, 2));
  ROS_INFO("\nlenOfAr2CamOffset = %.5f\n", lenOfAr2CamOffset);

  ROS_INFO("ar2CamOffset x y z: [%f, %f, %f]", 
            ar2CamOffset.x,
            ar2CamOffset.y,
            ar2CamOffset.z);


  double dx = ar2CamOffset.x;
  double dy = ar2CamOffset.y;
  ROS_INFO("atan2(dy, dx): [%f]", atan2(dy, dx));

}

void go_vdrk_in_radius(){
  findLenOfAr2CamOffset();
  marker_go();
  camera_go();
}

// получаем текущее фактическое положение _маркера_ относительно ГСК И
// получаем текущее фактическое положение _камеры_  относительно ГСК 
void getCrntOdomPoseHandler(const gazebo_msgs::ModelStates& gazebo_msg) {

  crntArOdomPose = gazebo_msg.pose[1];
  crntCamOdomPose = gazebo_msg.pose[2];

  getOdomPoses = true;
}

void setup(ros::NodeHandle& node) {
  setup_yet = true;
  velCmdCamPub         = node.advertise<geometry_msgs::Twist>("/camera_cmd_vel", 0);
  velCmdArPub          = node.advertise<geometry_msgs::Twist>("/aruco_cmd_vel",  0);
  velVdrkMsg.linear.x  = 0.0;
  velVdrkMsg.linear.y  = 0.0;
  velVdrkMsg.linear.z  = 0.0;
  velVdrkMsg.angular.x = 0.0;
  velVdrkMsg.angular.y = 0.0;
  velVdrkMsg.angular.z = 0.0;

  geometry_msgs::Vector3 ar2CamOffset;
  findAr2CamOffset(ar2CamOffset);

  double dx = ar2CamOffset.x;
  double dy = ar2CamOffset.y;
  ROS_INFO("in setup dx, dy: [%f, %f]", dx, dy);
  ROS_INFO("in setup atan2(dy, dx): [%f]", atan2(dy, dx));

  desired_pose.position.x = dx;
  desired_pose.position.y = dy;
  desired_pose.position.z = 0.0;
  desired_pose.orientation.x = 0.0;
  desired_pose.orientation.y = 0.0;
  desired_pose.orientation.z = sin(atan2(dy, dx) / 2.0);  // Ось Z
  desired_pose.orientation.w = cos(atan2(dy, dx) / 2.0);  // Угол поворота        // 78 градусов

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "go_in_radius");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  nh.getParam("velocity", vel4VdrkFromUser);
  nh.getParam("radius_tube", radiusTubeFromUser);

  ROS_INFO("\nvelocity = %.5f\n", vel4VdrkFromUser);
  ROS_INFO("\nradius_tube = %.5f\n", radiusTubeFromUser);

  crntOdomPoseSub = node.subscribe("/gazebo/model_states",  0, getCrntOdomPoseHandler);
  
  ros::Rate loop_rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    if (!getOdomPoses) continue;
    getOdomPoses = false;

    if (!setup_yet) setup(node);

    go_vdrk_in_radius();
    loop_rate.sleep();
  }
  return 0;
}