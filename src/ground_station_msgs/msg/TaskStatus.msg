uint8 STATE_WAITING=0
uint8 STATE_TAKEOFF=1
uint8 STATE_PATROL=2
uint8 STATE_BLIND_CHECK=3
uint8 STATE_RETURNING=4
uint8 STATE_LANDING=5
uint8 STATE_FINISHED=6
uint8 STATE_ERROR=7

uint32 plan_id
uint8 state

# 从0开始，对应MissionPlan.route_cells数组下标
uint32 current_index

# 当前目标网格，例如A3B5
string current_cell

bool mission_done