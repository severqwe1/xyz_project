# xyz_project

파일 위치
~/ros2_ws/src/doosan-robot2/dsr_example2/dsr_example/dsr_example/simple 여기서 ls

그럼 test_realsense.py 이파일이 핵심 파일



*** 모든 명령어는 각자 워크스페이스에서 + source install/setup.bash 이후 실행하기 ***

ros2 launch realsense2_camera rs_align_depth_launch.py depth_module.depth_profile:=640x480x30 rgb_camera.color_profile:=640x480x30 initial_reset:=true align_depth.enable:=true enable_rgbd:=true pointcloud.enable:=true # realsense 실행

ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=110.120.1.xx port:=12345 model:=e0509
​
로봇팔 연결(host 각자 팀 ip에 맞게 변경)
ros2 run dsr_example test_realsense 
​
예제 실행
