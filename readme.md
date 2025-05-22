Проект МОМА одометрии для восстановления траектории МТ.

Порядок сборки проекта:
1. sudo apt-get install ros-noetic-gazebo-msgs
2. sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
3. catkin_make
4. source devel/setup.bash

Если все собралось, можно переходить к сл. этапу.

Убить все процессы Gazebo:
killall gzserver gzclient roslaunch rosmaster

Порядок запуска симуляции:
1. roslaunch my_gazebo_world track.launch
2. roslaunch vrdk_move go_in_radius.launch   - движение по окружности
3. 
4. 
5. 
6. 