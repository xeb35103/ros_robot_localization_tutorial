# README.md


## 使い方
$ source /opt/ros/melodic/setup.bash
$ catkin build
$ source ./devel/setup.bash
$ roslaunch robot_localization_demo  robot_localization_demo.launch

## TODO
- バッグしても前進しようとする
  - turtlesimの出力トピック/turtle1/pose/linear_velocityが負にならないため/turtle1_odometry_nodeでおかしくなる
- base_linkのトピックが出力されない
  - base_link_frameにbeacon_linkを指定しているため?
- twistの原点が反映されていない？




