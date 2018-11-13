login with ubuntu user:

sudo adduser --disabled-password --gecos "" amigo
sudo adduser amigo 
sudo usermod -a -G adm,dialout,sudo,audio,video amigo
sudo passwd amigo
echo '%sudo ALL=(ALL) NOPASSWD:ALL' | sudo tee --append /etc/sudoers
sudo reboot

login with amigo user:

ssh-keygen
source <(wget -O- https://raw.githubusercontent.com/tue-robotics/tue-env/master/installer/scripts/bootstrap-ros-kinetic)
tue-get install ros
tue-get install openpose
tue-get install ros-image_recognition_openpose
tue-make
source ~/.bashrc
roscd image_recognition_openpose
ln -s ~/openpose 
tue-make --pre-clean image_recognition_openpose

test with:

export ROS_MASTER_URI=http://<hostname>.local:11311
rosrun image_recognition_openpose image_recognition_openpose_node _net_input_width:=368 _net_input_height:=368 _net_output_width:=368 _net_output_height:=368 _model_folder:=/home/amigo/openpose/models/ __ns:=amigo/pose_detector

Make sure the hosts can be found ping from amigo1 to jetson and from jetson to amigo1 should work based on hostnames, not IP

optionally: instsall service:

sudo install $TUE_DIR/installer/targets/ros-image_recognition_openpose/image_recognition_openpose.service /etc/systemd/system/
sudo systemctl enable image_recognition_openpose
sudo systemctl start image_recognition_openpose

For use on amigo:
tue-get install amigo4
