# Prepare workspace
cd ~/Documents/ || exit
mkdir -p steer_drive_ros/src
cd steer_drive_ros/src || exit

# Clone repository and change branch
git clone git@github.com:CIR-KIT/steer_drive_ros.git
cd steer_drive_ros || exit
git checkout melodic-devel

# catkin_make
cd ..
cp -r steer_drive_ros/* .
rm -rf steer_drive_ros
cd ..
catkin_make

# Register to .bashrc
# typoでbashを上書きしないように！！！！！
## source /home/ogiwara/Documents/steer_drive_ros/devel/setup.bash