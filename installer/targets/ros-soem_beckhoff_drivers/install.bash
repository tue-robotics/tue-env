if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    tue-install-ros-rosbuild git http://git.mech.kuleuven.be/robotics/soem.git soem_beckhoff_drivers d9a96dd71885f6f147aa7522b3131974718a305a
elif [[ $TUE_ROS_DISTRO == "hydro" ]]
then
    tue-install-ros git https://github.com/orocos/rtt_soem soem_beckhoff_drivers
fi
