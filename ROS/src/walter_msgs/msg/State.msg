# This represents the state the robot is in 
# The header contains the time at which this message is send and from which frame_id. 
# The walter_state is a unsigned int used to carry the state which are declared above. 
Header header  


# Walter has recieved an error
uint8 WALTER_STATE_ERROR = 0
# Walter is idle
uint8 WALTER_STATE_IDLE = 1
# Walter is collecting nuts
uint8 WALTER_STATE_AUTONOMOUS_DRIVING = 2
# Walter is being driven mannually
uint8 WALTER_STATE_MANUAL_DRIVING = 3
# Walter is charging
uint8 WALTER_STATE_CHARGING = 4
# Walter is scanning the area to construct a good map
uint8 WALTER_STATE_SCANNING_AREA = 5
# Walter is can't move 
uint8 WALTER_STATE_STUCK = 6
# Walter is not connected to server
uint8 WALTER_STATE_OFFLINE = 7

uint8 walter_state 