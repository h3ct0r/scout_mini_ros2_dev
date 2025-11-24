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
