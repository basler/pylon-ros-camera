
if [ -z "$ROS_DISTRO" ]; then
  echo "Please set \$ROS_DISTRO variable."
  exit
fi

THIS_FILE=$BASH_SOURCE
THIS_PROJECT_ROOT=$(realpath $(dirname $(realpath $THIS_FILE)))

# Install ROS dependency
## From git repos
echo "Start to install dependent repos(vcs)"
catkin_ws=$(realpath ${THIS_PROJECT_ROOT}/../../)

vcs import \
  --recursive \
  --input ${THIS_PROJECT_ROOT}/build_depends.repos \
  ${catkin_ws}/src
vcs pull ${catkin_ws}/src

## From apt repositories
echo "Start to install dependent binarys(rosdep )"
rosdep update
rosdep install -r -y -i \
  --from-paths ${catkin_ws}/src \
  --rosdistro $ROS_DISTRO

unset catkin_ws
unset THIS_FILE
unset THIS_PROJECT_ROOT
unset THIS_REPOSITORY_NAME
echo "finish to install dependencies!"