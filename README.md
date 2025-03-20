ROS2 nav2 costmap plugin 공부중

RGB-D 이미지 -> 색 추출 -> pointcloud 생성 -> cost 생성

카메라 바닥 이미지를 활용, 
Pointcloud와 LaserScan을 합쳐서 2D SLAM/Localization 알고리즘에 사용하는게 목표

message_filter에서 lifecycle node를 못받아서 분리
