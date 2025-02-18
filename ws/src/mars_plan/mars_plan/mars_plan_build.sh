rm -rf log/ build/ install/
colcon build --symlink-install
source install/setup.bash