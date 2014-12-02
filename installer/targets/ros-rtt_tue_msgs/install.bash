if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    echo "Error: There are no rtt_tue_msgs in groovy"
else
    tue-install-ros git https://github.com/tue-robotics/rtt_tue_msgs
fi
