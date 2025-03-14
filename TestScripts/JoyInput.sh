ros2 run joy joy_node &
JOY_PID=$!
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' &
TELEOP_PID=$!

cleanup() {
    echo "Shutting down..."
    kill $JOY_PID $TELEOP_PID
    wait
    exit 0
}

trap cleanup SIGINT

echo -e "\e[0;31m Right bumper is deadman \e[0m"
ros2 topic echo /cmd_vel