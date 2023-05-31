export ROS_DISTRO=noetic                                 # Set ROS distribution
mkdir -p /tmp/ros_tutorials_ws/src                       # Create workspace
cd /tmp/ros_tutorials_ws/src                             # Navigate to source space
rosinstall_generator --deps ros_tutorials > .rosinstall  # Get list of packages
wstool update                                            # Checkout all packages
cd /tmp/ros_tutorials_ws                                 # Navigate to ros workspace root
catkin init                                              # Initialize workspace
