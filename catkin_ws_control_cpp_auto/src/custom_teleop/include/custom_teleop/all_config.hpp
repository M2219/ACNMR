#ifndef ALL_CONFIG_HPP
#define ALL_CONFIG_HPP
// Vehicle Geometry

/*
   SetVehicleShape(4.7, 2.0, 1.3);
    <node pkg="hybrid_a_star" type="run_hybrid_astar" name="run_hybrid_astar" output="screen">
        <param name="planner/steering_angle" value="15.0"/>
        <param name="planner/steering_angle_discrete_num" value="1"/>
        <param name="planner/wheel_base" value="2.0"/>
        <param name="planner/segment_length" value="1.6"/>
        <param name="planner/segment_length_discrete_num" value="8"/>
        <param name="planner/steering_penalty" value="1.5"/>
        <param name="planner/reversing_penalty" value="3.0"/>
        <param name="planner/steering_change_penalty" value="2.0"/>
        <param name="planner/shot_distance" value="5.0"/>#

*/
static constexpr double  WB = 0.650; // m
static constexpr double  V_LENGTH = 0.980 + 0.2; // m since hybrid A* currently does not support local costmap added 0.2
static constexpr double  V_WIDTH = 0.745 + 0.2; // m since hybrid A* currently does not support local costmap added 0.2
static constexpr double  REAR_AXLE_DISTANCE = 0.165+0.0; // m

// Controller
static constexpr int GOAL_X = 48.0; //44 goalx
static constexpr int GOAL_Y = 2.5; // goaly
static constexpr int GOAL_Z = 0.0; // goalz

static constexpr int NX = 4; // states
static constexpr int NU = 2; // input
static constexpr int N_IND_SEARCH = 10; // ahead index for splice planner
static constexpr double DT = 0.02; // simulation time
static constexpr double  MAX_STEER = 33 * M_PI / 180; // rad
static constexpr double  MAX_DSTEER = 0.52; // max ddelta rad/s -  30 deg / s
static constexpr double  MAX_SPEED = 1.0; // the real robot is 5.4; 1.0 for safety // m / s
static constexpr double  MIN_SPEED = 0; //-HunterV2Params::max_linear_speed;
static constexpr double  MAX_ACCEL = 2.0; // m/ss max accelration
static constexpr double GOAL_DIS = 1.5;  // goal distance m
static constexpr double STOP_SPEED = 0.125; // stop speed m/s
static constexpr double TARGET_SPEED = 0.5; // m/s

// Global Planner
// Note: current global planner just uses /map as a static map must be develop to support dynamic map /global_costmap/costmap
// Note: it uses internal dynamic model instead of odometry

static constexpr int steering_angle_discrete_num = 5.5; // Number of discrete steering angle steps used in planning (e.g., finer steps for smooth path>
static constexpr double segment_length = 1.6; // Length of each motion primitive (trajectory segment) in meters.
static constexpr int segment_length_discrete_num = 8; // Number of discrete steps for segment length, which affects path resolution.
static constexpr double steering_penalty = 1.5; //Cost multiplier for using steering, encouraging straighter paths.
static constexpr double steering_change_penalty = 2.0; // Penalty for changing steering angle, preventing sharp turns if lower.
static constexpr double reversing_penalty = 3.0; // Extra cost for reversing, discouraging unnecessary backward movements.
static constexpr double shot_distance = 5.0; // Threshold distance for direct connection to the goal, improving efficiency if close enough.

#endif
