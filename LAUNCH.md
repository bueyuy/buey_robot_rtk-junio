RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch buey_robot motor_gateway.launch.py

ros2 launch buey_robot real_zed.launch.py

python3 src/buey_robot/tools/camera_stream_sdk.py
