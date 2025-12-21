# scout_mini_ros2
Agilex Scout Mini base packages, compatible with ROS2 Jammy, Ubuntu 24.04 and Gazebo

### First steps

ssh-keygen -b 4096 -t rsa
less /home/scout/.ssh/id_rsa.pub

mkdir ~/Git
sudo apt install git

git clone git@github.com:h3ct0r/ros2_devcontainer_docker_compose.git
cd ros2_devcontainer_docker_compose/local_mount/
mkdir src
cd src
git clone git@github.com:h3ct0r/scout_mini_ros2_dev.git

sudo apt install python3.12-venv
python3 -m venv ~/py3.12
source ~/py3.12/bin/activate
pip install ansible

echo -e "\nsource ~/py3.12/bin/activate" >> ~/.bashrc

Run ansible configuration for host embeded PC:

- `ansible-playbook -c local -i localhost, robot_ubuntu_playbook.yaml -vvv  -e "ansible_become_password=scout"`


## Run the robot

ros2 launch scout_base scout_base.launch.py 
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# sim

ros2 launch agilex_scout simulate_control_gazebo.launch.py lidar_type:=3d rviz:=true

$ ros2 launch scout_nav2 nav2.launch.py simulation:=true slam:=<true|false> localization:=<amcl|slam_toolbox>


## Nav2

ros2 launch scout_nav2 nav2.launch.py simulation:=false slam:=true


tar --use-compress-program=lz4 -cf lapinha_rosbag2_2025_12_19-11_04_23.tar


## Compile

MAKEFLAGS="-j6" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release