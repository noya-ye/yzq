uint8 CMD_UNKNOWN=0
uint8 CMD_START=1
uint8 CMD_PAUSE=2
uint8 CMD_RESUME=3
uint8 CMD_RTL=4
uint8 CMD_LAND=5
uint8 CMD_RESET=6
uint8 CMD_CLEAR_TRACK=7
uint8 CMD_GOTO=8

uint8 cmd_type

bool has_goal
float32 x
float32 y
float32 z

string raw