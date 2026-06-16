# Mixed message exercising scalars, strings, arrays, sequences, constants
uint8 STATE_IDLE=0
uint8 STATE_ACTIVE=1
string ROBOT=turtlebot

std_msgs/Header header
uint8 state
string name
float64[36] covariance
int32[] readings
geometry_msgs/Point[] waypoints
