rosversion() {
  if [ $# = 0 ]
  then
    echo $ROS_VERSION
    return
  fi
  if [ $1 = "-q" ]
  then
    ROS_VERSION_QUIET=1
    shift
  fi
  export ROS_VERSION=$1

  if [ "$ROS_VERSION_QUIET" != "1" ]
  then
    echo "Setting ros version to $ROS_VERSION"
  fi
  . /opt/ros/$ROS_VERSION/setup.bash
  
  export ROS_PACKAGE_PATH=~/ros_workspace:$ROS_PACKAGE_PATH

  export PYTHONPATH=~/ros_workspace/python:$PYTHONPATH
}

rosversion -q fuerte

set_master_uri() {
if [ $ROBOT = "sim" ]; then
  export ROS_HOSTNAME=localhost
  export ROS_MASTER_URI=http://localhost:11311
else
  export ROS_HOSTNAME=surgical5.cs.berkeley.edu
  export ROS_MASTER_URI=http://raven.cs.berkeley.edu:11311
fi
}

setsim() {
  export ROBOT=sim
  echo sim > ~/.ros/robot
  set_master_uri
}

setraven() {
  export ROBOT=raven
  echo raven > ~/.ros/robot
  set_master_uri
}

if [ -f ~/.ros/robot ]; then
  export ROBOT=`cat ~/.ros/robot`
  set_master_uri
else
  setraven
fi

rviz() {
  rosrun_in fuerte rviz rviz
}

view_frames() {
  CURR_WD=$(pwd)
  cd /tmp
  rosrun tf view_frames && evince frames.pdf &
  cd $CURR_WD
}

rospush() {
  SSH_CMD=ssh
  PRE_CMDS=
  if [ "$1" = "clean" ]; then
    shift
    NOBUILD=
    if [ "$1" = "nobuild" ]; then
      NOBUILD=$1
      shift
    fi
    rospush nobuild --delete -n "$@"
    read -p "Continue?" -n 1 -r
    if [[ $REPLY =~ ^[Yy]$ ]]; then
      rospush $NOBUILD "$@" --delete
    fi
  fi
  if [ "$1" = "nobuild" ]; then
    SSH_CMD=true
    shift
  fi
  rsync -avz --exclude-from=$HOME/ros_workspace/.rospushignore "$@" ~/ros_workspace/ biorobotics@raven.cs.berkeley.edu:/home/biorobotics/$USER/ && $SSH_CMD -t biorobotics@raven.cs.berkeley.edu "ROS_PACKAGE_PATH=/home/biorobotics/$USER:\$ROS_PACKAGE_PATH rosmake raven_2_control" && echo rospush completed: $(date)
}

#export EDITOR=npp
export EDITOR=gedit

execute_in () {
  CURR_ROS_VERSION=$(rosversion)
  rosversion -q $1
  shift
  $*
  rosversion -q $CURR_ROS_VERSION
  CURR_ROS_VERSION=
}

rosrun_in () {
  THE_ROS_VER=$1
  shift
  execute_in $THE_ROS_VER rosrun "$@"
}

dynparam () {
  execute_in electric rosrun dynamic_reconfigure dynparam "$@"
}

raven() {
  ssh biorobotics@raven.cs.berkeley.edu
}

runraven() {
  if [ "$1" != "-f" ]
  then
    rospush
    if [ $? -ne 0 ]; then
      echo Push or build failed!
      return
    fi
  else
    shift
  fi
  ssh -t biorobotics@raven.cs.berkeley.edu "ROS_PACKAGE_PATH=/home/biorobotics/$USER:\$ROS_PACKAGE_PATH sudo roslaunch raven_2_control raven_2.launch arm:=green $@"
}

reloadmodel() {
  rospush nobuild && 
  ssh -t biorobotics@raven.cs.berkeley.edu "ROS_PACKAGE_PATH=/home/biorobotics/$USER:\$ROS_PACKAGE_PATH roslaunch raven_2_control raven_2.pub.launch arm:=green $@"
}

hydras() {
  rosrun sixense driver
}
