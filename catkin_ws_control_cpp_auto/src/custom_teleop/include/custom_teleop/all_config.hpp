#ifndef ALL_CONFIG_HPP
#define ALL_CONFIG_HPP
// Vehicle Geometry
static constexpr double  WB = 0.650; // m
static constexpr double  V_LENGTH = 0.980 + 0.2; // m since hybrid A* currently does not support local costmap added 0.3
static constexpr double  V_WIDTH = 0.745 + 0.2; // m since hybrid A* currently does not support local costmap added 0.3
static constexpr double  REAR_AXLE_DISTANCE = 0.165; // m


// Controller
static constexpr int GOAL_X = 44.0; // goalx
static constexpr int GOAL_Y = 0.0; // goaly
static constexpr int GOAL_Z = 0.0; // goalz

static constexpr int NX = 4; // states
static constexpr int NU = 2; // input
static constexpr int N_IND_SEARCH = 20; // ahead index for splice planner
static constexpr double DT = 0.02; // 0.01 for sim // 0.02 for real robot //s reduce when speeds is lower than 0.5
static constexpr double  MAX_STEER = 33 * M_PI / 180; // rad
static constexpr double  MAX_DSTEER = 0.52; // max ddelta rad/s -  30 deg / s
static constexpr double  MAX_SPEED = 1.0; // the real robot is 5.4; 1.0 for safety // m / s
static constexpr double  MIN_SPEED = 0; //-HunterV2Params::max_linear_speed;
static constexpr double  MAX_ACCEL = 1; // m/ss max accelration
static constexpr double GOAL_DIS = 1.5;  // goal distance m
static constexpr double STOP_SPEED = 0.125; // stop speed m/s
static constexpr double TARGET_SPEED = 0.5; // m/s

// Global Planner
// Note: current global planner just uses /map as a static map must be develop to support dynamic map /global_costmap/costmap
// Note: it uses internal dynamic model instead of odometry

static constexpr int steering_angle_discrete_num = 5.0; // Number of discrete steering angle steps used in planning (e.g., finer steps for smooth path>
static constexpr double segment_length = 1.5; // Length of each motion primitive (trajectory segment) in meters.
static constexpr int segment_length_discrete_num = 8; // Number of discrete steps for segment length, which affects path resolution.
static constexpr double steering_penalty = 1.1; //Cost multiplier for using steering, encouraging straighter paths.
static constexpr double steering_change_penalty = 1.5; // Penalty for changing steering angle, preventing sharp turns if lower.
static constexpr double reversing_penalty = 3.0; // Extra cost for reversing, discouraging unnecessary backward movements.
static constexpr double shot_distance = 5.0; // Threshold distance for direct connection to the goal, improving efficiency if close enough.


#endif
