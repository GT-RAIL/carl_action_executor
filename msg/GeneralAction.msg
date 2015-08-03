#actions
uint8 NAVIGATE=0
uint8 READY_ARM=1
uint8 RETRACT_ARM=2
uint8 PICKUP=3
uint8 STORE=4
uint8 SEGMENT=5
uint8 SEGMENT_AND_RECOGNIZE=6

#error codes
uint8 SUCCESS=0
uint8 NAVIGATION_PLANNING_FAILURE=1
uint8 NAVIGATION_EXECUTION_FAILURE=2
uint8 MANIPULATION_PLANNING_FAILURE=3
uint8 MANIPULATION_EXECUTION_FAILURE=4
uint8 SENSING_FAILURE=5
uint8 OBJECT_NOT_FOUND=6
uint8 PREEMPTED=7

uint8 action_type                               # type of action
geometry_msgs/PoseStamped nav_pose              # pose goal for navigation
int32 nav_location                              # nav goal specified as a rail_lab_location_server location, only used if nav_goal pose is unspecified
string object_name                              # name of object to be acted on
geometry_msgs/PoseStamped manipulation_pose     # end-effector pose for manipulation actions
string surface_frame                            # surface tf frame for tabletop actions