# This configures the environment variables for a vector simulation
# This is necessary to run before starting the simulation 
#

# If there is an onboard PC powered by the system this will run the watchdog
# to make sure it gets gracefully shutdown before the system power cuts out.
export VECTOR_POWERS_PC_ONBOARD=false

#Default Vector RMP_V3 network 
export VECTOR_IP_ADDRESS=10.66.171.5
export VECTOR_IP_PORT_NUM=8080

# The reference frame for all the accessories and sensors
export VECTOR_PARENT_LINK=base_chassis_link

#Determines if the platform should use 2D or 3D odometry
export VECTOR_USE_2D_ODOMETRY=true

#
# Determines if we should use robot_localization EKF or the onboard odometry;
# set to true for onboard odometry. Platform odometry is fairly accurate and will
# improve in the next few releases, but the robot_localization package offers an alternative
# form of localization that can be used to fuse other forms of odometry sensor data. Such as Visual,
# laser scan matching and landmarking information sources. If this is set to false we use robot
# localization to fuse the wheel odometry, SIC IMU, external IMU, etc.
#
export VECTOR_USE_PLATFORM_ODOMETRY=true

#
# This enables the laser scan matcher which generates a /vector/lsm/pose_stamped in the odom frame
#
export VECTOR_ENABLE_LSM=true

#
# Set this if you want the platform odometry to be corrected by LSM. This is stable indoors, but should be tested with
# teleoperation before using it
#
export VECTOR_USE_LSM_TO_CORRECT_ODOMETRY=true

# Joystick configurations for joystick set VECTOR_JOY_IS_ATTACHED if the joystick
# is physically attached to this PC
export VECTOR_JOY_IS_ATTACHED=false
export VECTOR_JOY_DEV=/dev/input/js0
export VECTOR_JOY_MAPPING=extreme3D
export VECTOR_JOY_DEADZONE=0.1

# laser configuration (only supports two by default)
export VECTOR_LASER1_IP=10.66.171.8
export VECTOR_LASER1_PORT=2112
export LASER1_XYZ="0.3075 0 0.1439106"
export LASER1_RPY="3.1415 0 0"
export LASER1_MAX_RANGE=10.0
export LASER1_MIN_RANGE=0.01
export LASER1_MAX_ANGLE=2.0
export LASER1_MIN_ANGLE=-2.0
export LASER1_PREFIX="front"

export VECTOR_LASER2_IP=10.66.171.9
export VECTOR_LASER2_PORT=2112
export LASER2_XYZ="-0.3075 0 0.1439106"
export LASER2_RPY="3.1415 0 3.1415"
export LASER2_MAX_RANGE=10.0
export LASER2_MIN_RANGE=0.01
export LASER2_MAX_ANGLE=2.0
export LASER2_MIN_ANGLE=-2.0
export LASER2_PREFIX="rear"


