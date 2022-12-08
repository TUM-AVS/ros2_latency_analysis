#!/bin/bash

set -e

awsim_ver="v1.0.1_custom"
map_ver="v1.0.0"

rootdir=$PWD

echo "MA Experiment Workspace is configured with:"
echo "- awsim_ver......: ${awsim_ver}"
echo "- map_ver........: ${map_ver}"
echo "- rootdir........: ${rootdir}"
# Ensure project is in the correct location
test "$(realpath $rootdir)" = "$(realpath /home/$USER/Max_MA)" || (echo "The root directory (currently $rootdir) has to be /home/$USER/Max_MA" && exit 1)

source /opt/ros/galactic/setup.bash

# Clone Autoware and tools

vcs import < dependencies.repos
# Check if expected files/folders exist, print error and exit otherwise.
stat autoware > /dev/null
stat scenario_runner > /dev/null
stat ros2multitopic > /dev/null

# Download and unpack customized AWSIM

awsim_zip_name="AWSIM_${awsim_ver}.zip"
wget "https://github.com/mojomex/AWSIM/releases/download/v1.0.1_custom/${awsim_zip_name}"
mkdir AWSIM
unzip -d AWSIM "${awsim_zip_name}"
rm "${awsim_zip_name}"
stat AWSIM/AWSIM.headless.x86_64 > /dev/null

# Download and unpack map files

map_zip_name="nishishinjuku_autoware_map.zip"
wget "https://github.com/tier4/AWSIM/releases/download/${map_ver}/${map_zip_name}"
unzip ${map_zip_name}
stat nishishinjuku_autoware_map > /dev/null
mv nishishinjuku_autoware_map map
rm ${map_zip_name}

# Set up Autoware

cd autoware
./setup-dev-env.sh
mkdir src
vcs import src < autoware.repos
vcs import src < "$rootdir/tracing.repos"
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --symlink-install --packages-up-to tracetools ros2trace tracetools_read tracetools_analysis
stat install/setup.bash > /dev/null
cd $rootdir

# Set up ROS2 Multitopic

cd ros2multitopic
pip3 install -r requirements.txt
colcon build --symlink-install
stat install/setup.bash > /dev/null
cd $rootdir

# Set up scenario runner

cd scenario_runner
pip3 install -r requirements.txt
vcs import src < dependencies.repos
colcon build --symlink-install
cd $rootdir
