if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    tue-install-ros-rosbuild svn https://svn.code.sf.net/p/cmu-ros-pkg/code/trunk/3rdparty/ann2 .
    tue-install-apply-patch patch_groovy.patch
fi
