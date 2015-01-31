if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    tue-install-ros-rosbuild git http://git.mech.kuleuven.be/robotics/soem.git soem_master d9a96dd71885f6f147aa7522b3131974718a305a
    tue-install-apply-patch patch_groovy.patch
else
    tue-install-ros git https://github.com/orocos/rtt_soem soem_master
fi

# Install orocos
tue-install-target orocos
