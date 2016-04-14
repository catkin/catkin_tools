source /opt/ros/indigo/setup.bash          # Source ROS indigo to use Catkin
mkdir -p /tmp/quickstart_ws/src            # Make a new workspace and source space
cd /tmp/quickstart_ws                      # Navigate to the workspace root
catkin init                                # Initialize it with a hidden marker file
cd /tmp/quickstart_ws/src                  # Navigate to the source space
catkin create pkg pkg_a                    # Populate the source space with packages...
catkin create pkg pkg_b
catkin create pkg pkg_c --catkin-deps pkg_a
catkin create pkg pkg_d --catkin-deps pkg_a pkg_b
catkin list                                # List the packages in the workspace
catkin build                               # Build all packages in the workspace
source /tmp/quickstart_ws/devel/setup.bash # Load the workspace's environment
catkin clean                               # Clean all the build products
