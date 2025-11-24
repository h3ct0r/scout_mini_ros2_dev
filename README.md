# scout_mini_ros2
Agilex Scout Mini base packages, compatible with ROS2 Jammy, Ubuntu 24.04 and Gazebo

Run ansible configuration for host embeded PC:

- `ansible-playbook -c local -i localhost, robot_ubuntu_playbook.yaml -vvv  -e "ansible_become_password=scout"`