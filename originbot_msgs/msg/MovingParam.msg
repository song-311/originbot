std_msgs/Header header

uint8 moving_type
uint8 MOVING_UNKNOWN=0
uint8 MOVING_STOP=1
uint8 MOVING_FORWARD=2
uint8 MOVING_LEFT=3
uint8 MOVING_RIGHT=4

# Distance (m) for FORWARD, angle (rad) for LEFT/RIGHT; 0.0 lets the executor decide
float32 moving_value
