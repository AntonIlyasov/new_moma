#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>

// Параметры (можно получать из параметров ROS или аргументов)
const double radiusTubeFromUser  = 5.0;         // радиус окружности
const double vel4VdrkFromUser  = 1.0;           // скорость первого робота (м/с)
const double dist_0 = 1.0;                      // расстояние между роботами

// Частота обновления (Гц)
const double loop_rate_hz = 20.0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "follower_robot_controller");
    ros::NodeHandle nh;

    // Топик для публикации скорости второго робота
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("second_robot/cmd_vel", 10);

    ros::Rate rate(loop_rate_hz);

    // Начальное положение второго робота (предполагаем в начале в точке (radiusTubeFromUser  - dist_0, 0))
    geometry_msgs::Pose2D follower_pose;
    follower_pose.x = radiusTubeFromUser  - dist_0; // Начальная позиция вдоль оси X
    follower_pose.y = 0.0;                          // Начальная позиция вдоль оси Y (по направлению движения)
    follower_pose.theta = 0.0;                      // Угол направления движения второго робота

    double time_start = ros::Time::now().toSec();

    while (ros::ok())
    {
        double t = ros::Time::now().toSec() - time_start;

        // Angular velocity первого робота
        double omega = vel4VdrkFromUser  / radiusTubeFromUser ;

        // Позиция первого робота на окружности
        double leader_x = radiusTubeFromUser  * sin(omega * t); // Изменяем по оси X (перпендикулярно движению)
        double leader_y = radiusTubeFromUser  * cos(omega * t); // Изменяем по оси Y (по направлению движения)

        // Направление движения первого робота (вдоль оси Y)
        double leader_vx = radiusTubeFromUser  * omega * cos(omega * t); // Изменение вдоль оси X
        double leader_vy = -radiusTubeFromUser  * omega * sin(omega * t); // Изменение вдоль оси Y

        // Нормируем вектор скорости, чтобы получить направление движения
        double leader_speed = sqrt(leader_vx * leader_vx + leader_vy * leader_vy);
        double dir_x = leader_vx / leader_speed;
        double dir_y = leader_vy / leader_speed;

        // Целевая позиция второго робота — позади первого на расстоянии dist_0 по направлению движения
        double target_x = leader_x + dir_x * dist_0; // Изменяем по оси X (перпендикулярно)
        double target_y = leader_y + dir_y * dist_0; // Изменяем по оси Y (вперед)

        // Вектор ошибки позиции второго робота относительно цели
        double error_x = target_x - follower_pose.x;
        double error_y = target_y - follower_pose.y;

        // Простейший П-регулятор для скорости (коэффициенты можно настроить)
        double Kp_linear = 1.0;
        double Kp_angular = 4.0;

        // Вычисляем желаемую скорость линейную (по направлению на цель)
        double distance_error = sqrt(error_x * error_x + error_y * error_y);

        // Угол до цели
        double angle_to_target = atan2(error_y, error_x);

        // Угол поворота робота
        double angle_error = angle_to_target - follower_pose.theta;
        // Нормируем угол в диапазон [-pi, pi]
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        // Формируем сообщения скорости
        geometry_msgs::Twist vel_msg;

        // Линейная скорость пропорциональна проекции ошибки на направление взгляда робота
        vel_msg.linear.x = Kp_linear * distance_error * cos(angle_error);
        if (vel_msg.linear.x > vel4VdrkFromUser ) vel_msg.linear.x = vel4VdrkFromUser ; // ограничение максимальной скорости

        // Угловая скорость пропорциональна углу ошибки
        vel_msg.angular.z = Kp_angular * angle_error;

        // Обновляем положение второго робота (простая интеграция для симуляции, можно убрать если есть реальная локализация)
        double dt = 1.0 / loop_rate_hz;
        follower_pose.theta += vel_msg.angular.z * dt;
        // Нормируем угол
        while (follower_pose.theta > M_PI) follower_pose.theta -= 2 * M_PI;
        while (follower_pose.theta < -M_PI) follower_pose.theta += 2 * M_PI;

        follower_pose.x += vel_msg.linear.x * cos(follower_pose.theta) * dt;
        follower_pose.y += vel_msg.linear.x * sin(follower_pose.theta) * dt;

        // Публикуем скорость
        vel_pub.publish(vel_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}





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




  [ INFO] [1747922693.064520586, 50.660000000]: Transform: [0.027676, 3.991303, -0.000000]
  [ INFO] [1747922790.483269689, 147.815000000]: Transform: [0.027676, 3.991308, 0.000001]


    try{
    // listener.lookupTransform("camera_link_base", "aruco_link_base",
    //                           ros::Time(0), transform);

    listener.waitForTransform("camera_link_base", "aruco_link_base", 
                            ros::Time(0), ros::Duration(1.0));
    listener.lookupTransform("camera_link_base", "aruco_link_base", 
                          ros::Time(0), transform);

    ROS_INFO("Transform: [%f, %f, %f]", 
             transform.getOrigin().x(),
             transform.getOrigin().y(),
             transform.getOrigin().z());
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }