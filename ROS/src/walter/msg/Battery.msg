# This represents the percentage of the battery of the walter bot. 
# The header contains the time at which this message is send and from which frame_id. 
# The percentage contains a unsigned 1 byte int ranging from 0 to 256.  
# The seq is a sequence id, consecutively increasing ID. 
Header header 
uint8 percentage 
uint32 seq 