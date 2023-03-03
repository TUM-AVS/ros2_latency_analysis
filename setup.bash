#!/bin/bash

set -e

awsim_ver="v1.0.1_custom"
map_ver="v1.0.0"

rootdir=$PWD

install_runner=1
install_autoware=1
install_awsim=1

# If no specific install options are given, install everything by default.
# Else, install AWSIM and Autoware only if arguments are present.
if [ $# -eq "0" ]
then
    install_autoware=0
    install_awsim=0
fi

# Read install options from command line arguments
while [ $# -gt 0 ]
do
    case "$1" in
        --sim)
            install_awsim=1
            ;;
        --aw)
            install_autoware=1
            ;;
        *) 
            echo "Unknown argument: '$1'. Ignoring."
            ;;
    esac
    shift
done

# Print configuration
echo "MA Experiment Workspace is configured with:"

if [ $install_awsim ]
then
    echo "- AWSIM..........: INSTALL"
    echo "  - awsim_ver....: ${awsim_ver}"
    echo "  - map_ver......: ${map_ver}"
else
    echo "- AWSIM..........: SKIP"
fi

if [ $install_autoware ]
then
    echo "- Autoware.......: INSTALL"
else
    echo "- Autoware.......: SKIP"
fi

if [ $install_runner ]
then
    echo "- Scenario Runner: INSTALL"
else
    echo "- Scenario Runner: SKIP"
fi

echo "- rootdir........: ${rootdir}"

# Install APT packages
sudo apt-get install screen
sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev
sudo apt-get install python3-babeltrace python3-lttng
sudo apt-get install numactl

# Ensure ROS2 environment is available during install
source /opt/ros/galactic/setup.bash

# Download and unpack customized AWSIM (x86 only)

if [ $install_awsim = "0" ]
then
    echo "Skipping AWSIM installation."
elif [ "$(uname -i)" = "x86_64" ]
then
    echo "Downloading and unpacking AWSIM"
    awsim_zip_name="AWSIM_${awsim_ver}.zip"
    rm -f $awsim_zip_name
    wget "https://github.com/TUM-AVS/ros2_latency_analysis/releases/download/v1.0.1_custom/${awsim_zip_name}"
    rm -rf AWSIM
    mkdir AWSIM
    unzip -d AWSIM $awsim_zip_name
    rm -f $awsim_zip_name
    stat AWSIM/AWSIM.headless.x86_64 > /dev/null

    # Download and unpack map files
    echo "Downloading and unpacking map files"
    map_zip_name="nishishinjuku_autoware_map.zip"
    rm -f $map_zip_name
    rm -rf nishishinjuku_autoware_map
    wget "https://github.com/tier4/AWSIM/releases/download/${map_ver}/${map_zip_name}"
    unzip $map_zip_name
    stat nishishinjuku_autoware_map > /dev/null
    rm -rf map
    mv nishishinjuku_autoware_map map
    rm -f $map_zip_name
else
    echo "Skipping AWSIM installation (architecture is not x86_64)."
fi


if [ $install_autoware = "0" ]
then
    echo "Skipping Autoware installation."
else
    # Clone Autoware and tools
    echo "Cloning Autoware repository"
    vcs import --repos < aw.repos
    # Check if expected files/folders exist, print error and exit otherwise.
    stat autoware > /dev/null

    # Set up Autoware
    echo "Setting up Autoware"
    # Add 16GiB of swap space to make Autoware compile without OutOfMemory errors
    swap_filename="temporary_autoware_compile_swapfile"
    sudo dd if=/dev/zero of=./"$swap_filename" bs=1G count=16
    sudo chmod 600 ./"$swap_filename"
    sudo mkswap ./"$swap_filename"
    sudo swapon ./"$swap_filename"

    cd autoware
    ./setup-dev-env.sh
    mkdir -p src
    vcs import src < autoware.repos
    vcs import src --repos < "$rootdir/tracing.repos"
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    stat install/setup.bash > /dev/null
    cd "$rootdir"

    sudo swapoff ./"$swap_filename"
    sudo rm ./"$swap_filename"
fi

if [ $install_runner = "0" ]
then
    echo "Skipping Scenario Runner installation."
else
    echo "Cloning Scenario Runner repository"
    vcs import --repos < runner.repos
    stat scenario_runner > /dev/null

    # Set up scenario runner
    echo "Setting up Scenatrio Runner"
    cd scenario_runner
    pip3 install -r requirements.txt
    vcs import --workers 1 --repos src < dependencies.repos
    colcon build --symlink-install
    cd "$rootdir"
fi

echo "$rootdir" > "$HOME/.ros2_dataflow_root"

echo "================="
echo "Setup successful."
