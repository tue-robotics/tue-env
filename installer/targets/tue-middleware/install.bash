# GROOVY specific packages
if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    tue-install-target ros-amigo_arm_navigation
    tue-install-target ros-pein_face_recognition
    tue-install-target ros-pein_face_segmentation
    tue-install-target ros-pein_odufinder
    tue-install-target ros-tue_move_base_3d
    tue-install-target ros-pein_ar_pose
fi
