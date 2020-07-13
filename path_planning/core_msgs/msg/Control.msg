Header header

#Control Mode
# NO_CONTROL = 0
# EMERGENCY_BRAKE = 1
# NORMAL_S = 2 # normal s path following mode
# NORMAL_C = 3 # normal c point following mode
# SLOW_C = 4 # c poin following with low speed
# U_TURN = 5 # when u turn is activated
# PARK_FORWARD = 6
# PARK_REVERSE = 7 # when returning to main lane from parking lot
string control_mode

bool is_auto
bool estop
int8 gear
int16 brake
float32 speed
float32 steer
