#!/bin/bash
# Launch BlueLily IMU bridge in a ROS2 Humble Docker container
# Run this on the Jetson: ./launch_bluelily_bridge.sh [start|stop|status|logs]

CONTAINER_NAME="bluelily_bridge"
IMAGE="arm64v8/ros:humble-ros-base"
WS_PATH="$HOME/HowYouSeeMe/ros2_ws"

case "${1:-start}" in
  start)
    docker stop $CONTAINER_NAME 2>/dev/null
    docker rm $CONTAINER_NAME 2>/dev/null
    echo "Starting BlueLily ROS2 IMU bridge..."
    docker run -d \
        --name $CONTAINER_NAME \
        --restart unless-stopped \
        --device /dev/ttyACM0:/dev/ttyACM0 \
        --network host \
        -v $WS_PATH:/ros2_ws \
        $IMAGE \
        bash -c "source /ros2_ws/install/setup.bash && \
                 ros2 run bluelily_bridge bluelily_imu_node \
                 --ros-args -p port:=/dev/ttyACM0 -p baud_rate:=115200"
    echo "Bridge started. Check: docker logs $CONTAINER_NAME"
    ;;
  stop)
    docker stop $CONTAINER_NAME && docker rm $CONTAINER_NAME
    echo "Bridge stopped."
    ;;
  status)
    docker exec $CONTAINER_NAME bash -c \
        "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
         timeout 4 ros2 topic hz /imu/data"
    ;;
  logs)
    docker logs -f $CONTAINER_NAME
    ;;
  *)
    echo "Usage: $0 [start|stop|status|logs]"
    ;;
esac
