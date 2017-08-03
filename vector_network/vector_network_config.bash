
# Set this to whatever physical port you are using to communicate externally 
# (eg. eth0, eth1, wlan0,...etc)
# For simulation                   : lo
# For robot                        : br0
# For remote PC connected to robot : its likely wlan0 or eth0

export ROBOT_NETWORK=lo

# Does not change
export ROS_IP=$(ip -4 address show $ROBOT_NETWORK | grep 'inet' | sed 's/.*inet \([0-9\.]\+\).*/\1/')

# Change to http://10.66.171.1:11311/ for remote PC connected to robot
export ROS_MASTER_URI=http://$ROS_IP:11311/
