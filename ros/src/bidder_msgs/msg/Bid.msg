# Bid for task assignment

# Identify for which task and robot this proposal is
commons_msgs/Goal task
string robot_id

# State the proposed cost
uint32 cost

# Information on where to include the task in the schedule
# edge_cost_{in,out} : cost of the new edges that are connected to the new task
# insert_at_id : insert the task before the scheduleItem with this id
# append : flag to be used to insert the task at the end of the schedule
uint32 edge_cost_in
uint32 edge_cost_out
string insert_at_id
bool after
