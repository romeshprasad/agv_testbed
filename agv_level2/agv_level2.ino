#include "Arduino_Alvik.h"
#include <math.h>
#include <stdint.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <std_msgs/msg/string.h>

Arduino_Alvik alvik;

// =====================================================
// COMMAND ENCODING
// =====================================================
// All path knowledge lives on the ROS side.
// The robot receives a flat byte array: [opcode, param, opcode, param, ...]
// and executes each command in order.

#define CMD_MOVE_NORTH        1
#define CMD_MOVE_SOUTH        2
#define CMD_MOVE_EAST         3
#define CMD_MOVE_WEST         4
#define CMD_ENTER_WORKSTATION 5
#define CMD_RETURN_TO_DEPOT   6   // param = number of reds to travel west before going south to blue

#define CMD_QUEUE_MAX  64

struct Command {
  uint8_t opcode;
  uint8_t param;
};

Command cmd_queue[CMD_QUEUE_MAX];
uint8_t cmd_queue_head = 0;
uint8_t cmd_queue_size = 0;

// =====================================================
// GRID HEADINGS (Alvik yaw frame, CCW positive)
// =====================================================
const float YAW_N = 0.0;
const float YAW_E = 270.0;
const float YAW_S = 180.0;
const float YAW_W = 90.0;

float leg_target_yaw = YAW_N;

// =====================================================
// TUNING VALUES
// =====================================================

const int TAPE_THRESHOLD = 275;

const float BASE_SPEED = 60.0;
const float YELLOW_SEARCH_SPEED = 30.0;
const float WORKSTATION_APPROACH_SPEED = 20.0;
const float KP = 50.0;
const float MAX_CORRECTION = 40.0;

const float STICKER_CROSS_SPEED = 30.0;

const float KP_HEADING = 1.5;
const float MAX_HEADING_CORRECTION = 25.0;

const float REVERSE_DOCK_SPEED = 20.0;

const float REVERSE_DOCK_ABSOLUTE_YAW = 0.0;
const float FINAL_DEPOT_YAW = 0.0;

const float YAW_TOLERANCE = 1.0;
const float TURN_MIN_SPEED = 15.0;
const float TURN_MAX_SPEED = 60.0;
const unsigned long TURN_CONTROL_MS = 5;

const unsigned long TURN_CENTERING_MS = 120;
const unsigned long PRE_CENTER_BRAKE_MS = 100;
const unsigned long PRE_ROTATE_BRAKE_MS = 200;
const unsigned long POST_ROTATE_BRAKE_MS = 200;

const unsigned long POST_TURN_EXIT_MS = 250;
const unsigned long YELLOW_POST_TURN_EXIT_MS = 150;
const float POST_TURN_EXIT_SPEED = 30.0;

const int RED_STABLE_SAMPLES = 4;
const int YELLOW_STABLE_SAMPLES = 6;
const int BLUE_STABLE_SAMPLES = 16;

const unsigned long MARKER_IGNORE_AFTER_TURN_MS = 600;
const unsigned long MARKER_IGNORE_AFTER_RED_PASS_MS = 750;

const unsigned long YELLOW_ARM_AFTER_ENTRY_RED_MS = 750;

const unsigned long YELLOW_ENTRY_DEPART_IGNORE_MS = 250;
const unsigned long YELLOW_IGNORE_AFTER_YAW_ALIGN_MS = 300;
const unsigned long EXIT_WORKSTATION_IGNORE_MS = 1000;

const unsigned long WORKSTATION_WAIT_MS = 1000;
const unsigned long WORKSTATION_BLINK_MS = 250;

const unsigned long LOST_LINE_FAILSAFE_MS = 1500;
const unsigned long LOOP_DELAY_MS = 15;

// =====================================================
// STATE MACHINE
// =====================================================

enum RobotState {
  WAIT_FOR_MISSION,
  EXECUTE_COMMAND,
  MOVE_DRIVE,
  DEPOT_SOUTH,
  COMMAND_DONE,

  DRIVE_TO_YELLOW_ENTRY,
  DRIVE_TO_YELLOW_WORKSTATION,
  REVERSE_TO_WORKSTATION_YELLOW,
  WORKSTATION_WAIT,
  DRIVE_OUT_TO_YELLOW_ENTRY,

  TURN_GENERIC,
  DONE,
  EMERGENCY_STOP
};

RobotState robot_state = WAIT_FOR_MISSION;
RobotState after_turn_state = DONE;
RobotState after_move_state = COMMAND_DONE;

enum TurnPhase {
  TURN_IDLE,
  TURN_PRE_CENTER_BRAKE,
  TURN_CENTER_FORWARD,
  TURN_PRE_ROTATE_BRAKE,
  TURN_ROTATING,
  TURN_POST_ROTATE_BRAKE,
  TURN_POST_EXIT
};

TurnPhase turn_phase = TURN_IDLE;

enum TargetColor {
  TARGET_RED,
  TARGET_YELLOW,
  TARGET_BLUE
};

// =====================================================
// GLOBAL VARIABLES
// =====================================================

float x, y, yaw;

unsigned long lost_line_start_ms = 0;
unsigned long marker_ignore_until_ms = 0;

TargetColor last_marker_target = TARGET_RED;
int marker_stable_count = 0;

int move_red_target = 0;
int move_red_count  = 0;

unsigned long turn_phase_start_ms = 0;
unsigned long turn_start_ms = 0;
unsigned long last_turn_control_ms = 0;

float turn_start_yaw = 0.0;
float turn_target_yaw = 0.0;
float pending_turn_angle = 0.0;

bool pending_turn_absolute = false;
float pending_absolute_yaw = 0.0;

unsigned long pending_center_ms = 0;
unsigned long pending_post_exit_ms = 0;
unsigned long pending_marker_ignore_ms = MARKER_IGNORE_AFTER_TURN_MS;

unsigned long workstation_wait_start_ms = 0;

// =====================================================
// micro-ROS OBJECTS
// =====================================================

rcl_node_t          ros_node;
rclc_support_t      ros_support;
rcl_allocator_t     ros_allocator;
rclc_executor_t     ros_executor;

rcl_subscription_t  cmd_sub;
rcl_publisher_t     status_pub;

std_msgs__msg__UInt8MultiArray  cmd_msg;
std_msgs__msg__String           status_msg;

uint8_t cmd_data_buf[CMD_QUEUE_MAX * 2];
char    status_buf[32];

bool ros_ready = false;

// =====================================================
// FUNCTION PROTOTYPES
// =====================================================

void commandsCallback(const void* msg_in);
void publishStatus(const char* text);

void waitForMissionState();
void executeCommandState();
void moveDriveState();
void depotSouthState();
void commandDoneState();

void driveToYellowEntryState();
void driveToYellowWorkstationState();
void reverseToWorkstationYellowState();
void workstationWaitState();
void driveOutToYellowEntryState();

bool driveForwardUntilColor(TargetColor target, float drive_speed);
bool reverseStraightUntilColor(TargetColor target);
void followLineOrDriveStraight(int left, int center, int right, float base_speed);
float calculateCenterError(int left, int center, int right);
bool isOnTape(int left, int center, int right);
bool isIntersection(int left, int center, int right);
float headingCorrection();

void beginTurn(float angle, RobotState next_state, unsigned long center_ms,
               unsigned long post_exit_ms, unsigned long marker_ignore_ms);
void beginTurnToYaw(float target_yaw, RobotState next_state, unsigned long center_ms,
                    unsigned long post_exit_ms, unsigned long marker_ignore_ms);
void turnGenericState();
void finishTurn();
float normalizeYaw(float angle);
float yawError(float target, float current);
void startRotateRelative(float relativeAngle);
void startRotateAbsolute(float targetYaw);
bool updateRotateTo();

bool targetColorDetectedStable(TargetColor target, bool red_now, bool yellow_now, bool blue_now);
void resetMarkerStable();
bool isRed(float h, float s, float v);
float colorChroma(float a, float b, float c);
bool isYellow(float h, float s, float v, float nr, float ng, float nb, int left, int center, int right);
bool isBlue(float h, float s, float v);

void checkLineFailsafe(bool tape_now, bool red_now, bool yellow_now, bool blue_now);
void setLEDOff();
void setLEDRed();
void setLEDGreen();
void setLEDBlue();
void setLEDYellow();

// =====================================================
// SETUP
// =====================================================

void setup() {
  alvik.begin();
  alvik.reset_pose(0, 0, 0, CM, DEG);

  set_microros_wifi_transports("AGV_SWARM", "ISECap123", "192.168.1.143", 8888);

  ros_allocator = rcl_get_default_allocator();
  rclc_support_init(&ros_support, 0, NULL, &ros_allocator);
  rclc_node_init_default(&ros_node, "agv_node", "", &ros_support);

  rclc_publisher_init_default(
    &status_pub, &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/agv/status");

  rclc_subscription_init_default(
    &cmd_sub, &ros_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
    "/agv/commands");

  cmd_msg.data.data     = cmd_data_buf;
  cmd_msg.data.capacity = sizeof(cmd_data_buf);
  cmd_msg.data.size     = 0;

  status_msg.data.data     = status_buf;
  status_msg.data.capacity = sizeof(status_buf);
  status_msg.data.size     = 0;

  rclc_executor_init(&ros_executor, &ros_support.context, 1, &ros_allocator);
  rclc_executor_add_subscription(&ros_executor, &cmd_sub, &cmd_msg,
                                 &commandsCallback, ON_NEW_DATA);

  ros_ready = true;
  robot_state = WAIT_FOR_MISSION;
  leg_target_yaw = YAW_N;

  publishStatus("IDLE");
  setLEDBlue();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  if (alvik.get_touch_cancel() && robot_state != EMERGENCY_STOP) {
    alvik.brake();
    robot_state = EMERGENCY_STOP;
    publishStatus("ERROR");
  }

  if (ros_ready) {
    rclc_executor_spin_some(&ros_executor, RCL_MS_TO_NS(0));
  }

  switch (robot_state) {
    case WAIT_FOR_MISSION:
      waitForMissionState();
      break;

    case EXECUTE_COMMAND:
      executeCommandState();
      break;

    case MOVE_DRIVE:
      moveDriveState();
      break;

    case DEPOT_SOUTH:
      depotSouthState();
      break;

    case COMMAND_DONE:
      commandDoneState();
      break;

    case DRIVE_TO_YELLOW_ENTRY:
      driveToYellowEntryState();
      break;

    case DRIVE_TO_YELLOW_WORKSTATION:
      driveToYellowWorkstationState();
      break;

    case REVERSE_TO_WORKSTATION_YELLOW:
      reverseToWorkstationYellowState();
      break;

    case WORKSTATION_WAIT:
      workstationWaitState();
      break;

    case DRIVE_OUT_TO_YELLOW_ENTRY:
      driveOutToYellowEntryState();
      break;

    case TURN_GENERIC:
      turnGenericState();
      break;

    case DONE:
      alvik.brake();
      setLEDGreen();
      break;

    case EMERGENCY_STOP:
      alvik.brake();
      setLEDRed();
      break;
  }

  delay(LOOP_DELAY_MS);
}

// =====================================================
// micro-ROS CALLBACK
// =====================================================

void commandsCallback(const void* msg_in) {
  if (robot_state != WAIT_FOR_MISSION && robot_state != DONE) {
    publishStatus("BUSY");
    return;
  }

  const std_msgs__msg__UInt8MultiArray* msg =
    (const std_msgs__msg__UInt8MultiArray*)msg_in;

  uint8_t pair_count = (uint8_t)(msg->data.size / 2);
  if (pair_count > CMD_QUEUE_MAX) {
    pair_count = CMD_QUEUE_MAX;
  }

  cmd_queue_size = pair_count;
  cmd_queue_head = 0;

  for (uint8_t i = 0; i < pair_count; i++) {
    cmd_queue[i].opcode = msg->data.data[i * 2];
    cmd_queue[i].param  = msg->data.data[i * 2 + 1];
  }

  move_red_target = 0;
  move_red_count  = 0;
  lost_line_start_ms = 0;
  marker_ignore_until_ms = millis() + 500;
  resetMarkerStable();

  alvik.reset_pose(0, 0, 0, CM, DEG);
  leg_target_yaw = YAW_N;

  robot_state = EXECUTE_COMMAND;
  publishStatus("EXECUTING");
}

// =====================================================
// NEW STATE HANDLERS
// =====================================================

void waitForMissionState() {
  alvik.brake();
  setLEDBlue();
}

void executeCommandState() {
  if (cmd_queue_head >= cmd_queue_size) {
    robot_state = DONE;
    publishStatus("DONE");
    setLEDGreen();
    return;
  }

  Command cmd = cmd_queue[cmd_queue_head];

  switch (cmd.opcode) {
    case CMD_MOVE_NORTH:
      move_red_target = (cmd.param > 0) ? cmd.param : 1;
      move_red_count  = 0;
      after_move_state = COMMAND_DONE;
      beginTurnToYaw(YAW_N, MOVE_DRIVE, TURN_CENTERING_MS,
                     POST_TURN_EXIT_MS, MARKER_IGNORE_AFTER_TURN_MS);
      break;

    case CMD_MOVE_SOUTH:
      move_red_target = (cmd.param > 0) ? cmd.param : 1;
      move_red_count  = 0;
      after_move_state = COMMAND_DONE;
      beginTurnToYaw(YAW_S, MOVE_DRIVE, TURN_CENTERING_MS,
                     POST_TURN_EXIT_MS, MARKER_IGNORE_AFTER_TURN_MS);
      break;

    case CMD_MOVE_EAST:
      move_red_target = (cmd.param > 0) ? cmd.param : 1;
      move_red_count  = 0;
      after_move_state = COMMAND_DONE;
      beginTurnToYaw(YAW_E, MOVE_DRIVE, TURN_CENTERING_MS,
                     POST_TURN_EXIT_MS, MARKER_IGNORE_AFTER_TURN_MS);
      break;

    case CMD_MOVE_WEST:
      move_red_target = (cmd.param > 0) ? cmd.param : 1;
      move_red_count  = 0;
      after_move_state = COMMAND_DONE;
      beginTurnToYaw(YAW_W, MOVE_DRIVE, TURN_CENTERING_MS,
                     POST_TURN_EXIT_MS, MARKER_IGNORE_AFTER_TURN_MS);
      break;

    case CMD_ENTER_WORKSTATION:
      marker_ignore_until_ms = millis() + YELLOW_ARM_AFTER_ENTRY_RED_MS;
      resetMarkerStable();
      robot_state = DRIVE_TO_YELLOW_ENTRY;
      break;

    case CMD_RETURN_TO_DEPOT:
      move_red_target  = (cmd.param > 0) ? cmd.param : 1;
      move_red_count   = 0;
      after_move_state = DEPOT_SOUTH;
      beginTurnToYaw(YAW_W, MOVE_DRIVE, TURN_CENTERING_MS,
                     POST_TURN_EXIT_MS, MARKER_IGNORE_AFTER_TURN_MS);
      break;

    default:
      publishStatus("ERROR");
      robot_state = EMERGENCY_STOP;
      break;
  }
}

void moveDriveState() {
  if (driveForwardUntilColor(TARGET_RED, BASE_SPEED)) {
    move_red_count++;

    if (move_red_count >= move_red_target) {
      robot_state = after_move_state;
    } else {
      marker_ignore_until_ms = millis() + MARKER_IGNORE_AFTER_RED_PASS_MS;
      resetMarkerStable();
    }
  }
}

void depotSouthState() {
  if (driveForwardUntilColor(TARGET_BLUE, BASE_SPEED)) {
    beginTurnToYaw(FINAL_DEPOT_YAW, COMMAND_DONE, 0, 0,
                   MARKER_IGNORE_AFTER_TURN_MS);
  }
}

void commandDoneState() {
  publishStatus("EXECUTING");
  cmd_queue_head++;
  robot_state = EXECUTE_COMMAND;
}

// =====================================================
// WORKSTATION DOCKING STATES
// =====================================================

void driveToYellowEntryState() {
  if (driveForwardUntilColor(TARGET_YELLOW, YELLOW_SEARCH_SPEED)) {
    beginTurnToYaw(YAW_S, DRIVE_TO_YELLOW_WORKSTATION, TURN_CENTERING_MS,
                   YELLOW_POST_TURN_EXIT_MS, YELLOW_ENTRY_DEPART_IGNORE_MS);
  }
}

void driveToYellowWorkstationState() {
  if (driveForwardUntilColor(TARGET_YELLOW, WORKSTATION_APPROACH_SPEED)) {
    beginTurnToYaw(REVERSE_DOCK_ABSOLUTE_YAW, REVERSE_TO_WORKSTATION_YELLOW, 0, 0,
                   YELLOW_IGNORE_AFTER_YAW_ALIGN_MS);
  }
}

void reverseToWorkstationYellowState() {
  if (reverseStraightUntilColor(TARGET_YELLOW)) {
    workstation_wait_start_ms = millis();
    robot_state = WORKSTATION_WAIT;
  }
}

void workstationWaitState() {
  alvik.brake();

  unsigned long now = millis();
  unsigned long elapsed = now - workstation_wait_start_ms;

  if (((elapsed / WORKSTATION_BLINK_MS) % 2) == 0) {
    setLEDYellow();
  } else {
    setLEDOff();
  }

  if (elapsed >= WORKSTATION_WAIT_MS) {
    resetMarkerStable();
    setLEDGreen();

    beginTurnToYaw(YAW_N, DRIVE_OUT_TO_YELLOW_ENTRY, 0, 0,
                   EXIT_WORKSTATION_IGNORE_MS);
  }
}

void driveOutToYellowEntryState() {
  if (driveForwardUntilColor(TARGET_YELLOW, YELLOW_SEARCH_SPEED)) {
    beginTurnToYaw(YAW_E, COMMAND_DONE, TURN_CENTERING_MS,
                   POST_TURN_EXIT_MS, MARKER_IGNORE_AFTER_TURN_MS);
  }
}

// =====================================================
// DRIVE HELPERS
// =====================================================

bool driveForwardUntilColor(TargetColor target, float drive_speed) {
  int left, center, right;
  float h, s, v;
  float nr, ng, nb;

  alvik.get_line_sensors(left, center, right);
  alvik.get_color(h, s, v, HSV);
  alvik.get_color(nr, ng, nb, RGB);

  bool tape_now   = isOnTape(left, center, right);
  bool red_now    = isRed(h, s, v);
  bool yellow_now = isYellow(h, s, v, nr, ng, nb, left, center, right);
  bool blue_now   = isBlue(h, s, v);

  if (targetColorDetectedStable(target, red_now, yellow_now, blue_now)) {
    alvik.brake();
    return true;
  }

  followLineOrDriveStraight(left, center, right, drive_speed);
  checkLineFailsafe(tape_now, red_now, yellow_now, blue_now);

  return false;
}

bool reverseStraightUntilColor(TargetColor target) {
  int left, center, right;
  float h, s, v;
  float nr, ng, nb;

  alvik.get_line_sensors(left, center, right);
  alvik.get_color(h, s, v, HSV);
  alvik.get_color(nr, ng, nb, RGB);

  bool tape_now   = isOnTape(left, center, right);
  bool red_now    = isRed(h, s, v);
  bool yellow_now = isYellow(h, s, v, nr, ng, nb, left, center, right);
  bool blue_now   = isBlue(h, s, v);

  if (targetColorDetectedStable(target, red_now, yellow_now, blue_now)) {
    alvik.brake();
    return true;
  }

  float h_corr = headingCorrection();
  alvik.set_wheels_speed(-REVERSE_DOCK_SPEED - h_corr, -REVERSE_DOCK_SPEED + h_corr, RPM);
  checkLineFailsafe(tape_now, red_now, yellow_now, blue_now);

  return false;
}

float headingCorrection() {
  alvik.get_pose(x, y, yaw, CM, DEG);
  float err = yawError(leg_target_yaw, yaw);
  return constrain(KP_HEADING * err, -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);
}

bool isIntersection(int left, int center, int right) {
  (void)center;
  return (left > TAPE_THRESHOLD && right > TAPE_THRESHOLD);
}

void followLineOrDriveStraight(int left, int center, int right, float base_speed) {
  bool tape_now        = isOnTape(left, center, right);
  bool intersection_now = isIntersection(left, center, right);

  if (!tape_now || intersection_now) {
    float speed  = tape_now ? base_speed : STICKER_CROSS_SPEED;
    float h_corr = headingCorrection();
    alvik.set_wheels_speed(speed - h_corr, speed + h_corr, RPM);
    return;
  }

  float error      = calculateCenterError(left, center, right);
  float correction = constrain(error * KP, -MAX_CORRECTION, MAX_CORRECTION);

  alvik.set_wheels_speed(base_speed - correction, base_speed + correction, RPM);
}

float calculateCenterError(int left, int center, int right) {
  float sum_weight = left + center + right;

  if (sum_weight <= 0.0) {
    return 0.0;
  }

  float centroid = (left + center * 2.0 + right * 3.0) / sum_weight;
  return -centroid + 2.0;
}

bool isOnTape(int left, int center, int right) {
  return (left > TAPE_THRESHOLD || center > TAPE_THRESHOLD || right > TAPE_THRESHOLD);
}

// =====================================================
// TURN CONTROL
// =====================================================

void beginTurn(float angle,
               RobotState next_state,
               unsigned long center_ms,
               unsigned long post_exit_ms,
               unsigned long marker_ignore_ms) {
  alvik.brake();
  setLEDYellow();

  pending_turn_absolute   = false;
  pending_turn_angle      = angle;
  after_turn_state        = next_state;
  pending_center_ms       = center_ms;
  pending_post_exit_ms    = post_exit_ms;
  pending_marker_ignore_ms = marker_ignore_ms;

  turn_phase          = TURN_PRE_CENTER_BRAKE;
  turn_phase_start_ms = millis();
  robot_state         = TURN_GENERIC;

  resetMarkerStable();
}

void beginTurnToYaw(float target_yaw,
                    RobotState next_state,
                    unsigned long center_ms,
                    unsigned long post_exit_ms,
                    unsigned long marker_ignore_ms) {
  alvik.brake();
  setLEDYellow();

  pending_turn_absolute   = true;
  pending_absolute_yaw    = normalizeYaw(target_yaw);
  pending_turn_angle      = 0.0;
  after_turn_state        = next_state;
  pending_center_ms       = center_ms;
  pending_post_exit_ms    = post_exit_ms;
  pending_marker_ignore_ms = marker_ignore_ms;

  turn_phase          = TURN_PRE_CENTER_BRAKE;
  turn_phase_start_ms = millis();
  robot_state         = TURN_GENERIC;

  resetMarkerStable();
}

void turnGenericState() {
  unsigned long now = millis();

  switch (turn_phase) {
    case TURN_IDLE:
      turn_phase = TURN_PRE_CENTER_BRAKE;
      turn_phase_start_ms = now;
      break;

    case TURN_PRE_CENTER_BRAKE:
      alvik.brake();
      setLEDYellow();

      if (now - turn_phase_start_ms >= PRE_CENTER_BRAKE_MS) {
        if (pending_center_ms > 0) {
          alvik.set_wheels_speed(POST_TURN_EXIT_SPEED, POST_TURN_EXIT_SPEED, RPM);
          turn_phase = TURN_CENTER_FORWARD;
          turn_phase_start_ms = now;
        } else {
          turn_phase = TURN_PRE_ROTATE_BRAKE;
          turn_phase_start_ms = now;
        }
      }
      break;

    case TURN_CENTER_FORWARD:
      setLEDYellow();

      if (now - turn_phase_start_ms >= pending_center_ms) {
        alvik.brake();
        turn_phase = TURN_PRE_ROTATE_BRAKE;
        turn_phase_start_ms = now;
      }
      break;

    case TURN_PRE_ROTATE_BRAKE:
      alvik.brake();
      setLEDYellow();

      if (now - turn_phase_start_ms >= PRE_ROTATE_BRAKE_MS) {
        if (pending_turn_absolute) {
          startRotateAbsolute(pending_absolute_yaw);
        } else {
          startRotateRelative(pending_turn_angle);
        }
        turn_phase = TURN_ROTATING;
      }
      break;

    case TURN_ROTATING:
      setLEDYellow();

      if (updateRotateTo()) {
        turn_phase = TURN_POST_ROTATE_BRAKE;
        turn_phase_start_ms = millis();
      }
      break;

    case TURN_POST_ROTATE_BRAKE:
      alvik.brake();
      setLEDYellow();

      if (now - turn_phase_start_ms >= POST_ROTATE_BRAKE_MS) {
        if (pending_post_exit_ms > 0) {
          alvik.set_wheels_speed(POST_TURN_EXIT_SPEED, POST_TURN_EXIT_SPEED, RPM);
          turn_phase = TURN_POST_EXIT;
          turn_phase_start_ms = now;
        } else {
          finishTurn();
        }
      }
      break;

    case TURN_POST_EXIT: {
      setLEDYellow();

      int left, center, right;
      alvik.get_line_sensors(left, center, right);

      bool tape_now        = isOnTape(left, center, right);
      bool intersection_now = isIntersection(left, center, right);

      if (tape_now && !intersection_now) {
        followLineOrDriveStraight(left, center, right, POST_TURN_EXIT_SPEED);
      } else {
        alvik.get_pose(x, y, yaw, CM, DEG);
        float err    = yawError(pending_turn_absolute ? pending_absolute_yaw : turn_target_yaw, yaw);
        float h_corr = constrain(KP_HEADING * err, -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);
        alvik.set_wheels_speed(POST_TURN_EXIT_SPEED - h_corr, POST_TURN_EXIT_SPEED + h_corr, RPM);
      }

      if (now - turn_phase_start_ms >= pending_post_exit_ms) {
        alvik.brake();
        finishTurn();
      }
      break;
    }
  }
}

void finishTurn() {
  turn_phase = TURN_IDLE;
  marker_ignore_until_ms = millis() + pending_marker_ignore_ms;
  resetMarkerStable();

  if (pending_turn_absolute) {
    leg_target_yaw = pending_absolute_yaw;
  } else {
    leg_target_yaw = turn_target_yaw;
  }

  robot_state = after_turn_state;
}

float normalizeYaw(float angle) {
  angle = fmod(angle + 360.0, 360.0);
  if (angle < 0.0) angle += 360.0;
  return angle;
}

float yawError(float target, float current) {
  target  = normalizeYaw(target);
  current = normalizeYaw(current);
  return fmod((target - current + 540.0), 360.0) - 180.0;
}

void startRotateRelative(float relativeAngle) {
  alvik.get_pose(x, y, yaw, CM, DEG);
  turn_start_yaw  = yaw;
  turn_target_yaw = normalizeYaw(yaw + relativeAngle);
  turn_start_ms   = millis();
  last_turn_control_ms = 0;
}

void startRotateAbsolute(float targetYaw) {
  alvik.get_pose(x, y, yaw, CM, DEG);
  turn_start_yaw  = yaw;
  turn_target_yaw = normalizeYaw(targetYaw);
  turn_start_ms   = millis();
  last_turn_control_ms = 0;
}

bool updateRotateTo() {
  unsigned long now = millis();

  if (last_turn_control_ms != 0 && now - last_turn_control_ms < TURN_CONTROL_MS) {
    return false;
  }

  last_turn_control_ms = now;
  alvik.get_pose(x, y, yaw, CM, DEG);

  float current_yaw = normalizeYaw(yaw);
  float error       = yawError(turn_target_yaw, current_yaw);

  if (fabs(error) <= YAW_TOLERANCE) {
    alvik.brake();
    return true;
  }

  float scale = fabs(error) / 90.0;
  if (scale > 1.0) scale = 1.0;

  float speed = TURN_MIN_SPEED + (TURN_MAX_SPEED - TURN_MIN_SPEED) * scale;

  if (error > 0.0) {
    alvik.set_wheels_speed(-speed, speed, RPM);
  } else {
    alvik.set_wheels_speed(speed, -speed, RPM);
  }

  return false;
}

// =====================================================
// MARKER DETECTION
// =====================================================

bool targetColorDetectedStable(TargetColor target, bool red_now, bool yellow_now, bool blue_now) {
  if (millis() < marker_ignore_until_ms) {
    resetMarkerStable();
    return false;
  }

  bool target_now = false;
  if      (target == TARGET_RED)    target_now = red_now;
  else if (target == TARGET_YELLOW) target_now = yellow_now;
  else if (target == TARGET_BLUE)   target_now = blue_now;

  int needed_samples = RED_STABLE_SAMPLES;
  if      (target == TARGET_YELLOW) needed_samples = YELLOW_STABLE_SAMPLES;
  else if (target == TARGET_BLUE)   needed_samples = BLUE_STABLE_SAMPLES;

  if (target_now) {
    if (last_marker_target != target) {
      marker_stable_count = 0;
      last_marker_target  = target;
    }
    marker_stable_count++;
  } else {
    marker_stable_count = 0;
    last_marker_target  = target;
  }

  if (marker_stable_count >= needed_samples) {
    resetMarkerStable();
    return true;
  }

  return false;
}

void resetMarkerStable() {
  marker_stable_count = 0;
}

bool isRed(float h, float s, float v) {
  bool hue_red     = (h > 340.0 || h < 20.0);
  bool saturated   = s > 0.40;
  bool bright_enough = v > 0.04;
  return hue_red && saturated && bright_enough;
}

float colorChroma(float a, float b, float c) {
  float max_val = a;
  if (b > max_val) max_val = b;
  if (c > max_val) max_val = c;

  float min_val = a;
  if (b < min_val) min_val = b;
  if (c < min_val) min_val = c;

  return max_val - min_val;
}

bool isYellow(float h, float s, float v, float nr, float ng, float nb, int left, int center, int right) {
  float chroma   = colorChroma(nr, ng, nb);
  bool  tape_now = isOnTape(left, center, right);

  bool black_tape_false_yellow =
    tape_now &&
    h > 65.0 && h < 95.0 &&
    s < 0.25 &&
    v < 0.30 &&
    chroma < 0.05;

  if (black_tape_false_yellow) return false;

  bool hue_yellow       = (h > 32.0 && h < 50.0);
  bool strongly_saturated = s > 0.60;
  bool bright_enough    = v > 0.08;
  bool colorful_enough  = chroma > 0.075;

  return hue_yellow && strongly_saturated && bright_enough && colorful_enough;
}

bool isBlue(float h, float s, float v) {
  bool hue_blue    = (h > 200.0 && h < 220.0);
  bool saturated   = s > 0.60;
  bool bright_enough = v > 0.08;
  return hue_blue && saturated && bright_enough;
}

// =====================================================
// LINE FAILSAFE
// =====================================================

void checkLineFailsafe(bool tape_now, bool red_now, bool yellow_now, bool blue_now) {
  if (!tape_now && !red_now && !yellow_now && !blue_now) {
    if (lost_line_start_ms == 0) {
      lost_line_start_ms = millis();
    }

    if (millis() - lost_line_start_ms > LOST_LINE_FAILSAFE_MS) {
      alvik.brake();
      publishStatus("ERROR");
      robot_state = EMERGENCY_STOP;
    }
  } else {
    lost_line_start_ms = 0;
  }
}

// =====================================================
// micro-ROS STATUS PUBLISHER
// =====================================================

void publishStatus(const char* text) {
  if (!ros_ready) return;
  snprintf(status_buf, sizeof(status_buf), "%s", text);
  status_msg.data.size = strlen(status_buf);
  rcl_publish(&status_pub, &status_msg, NULL);
}

// =====================================================
// LED HELPERS
// =====================================================

void setLEDOff() {
  alvik.left_led.set_color(0, 0, 0);
  alvik.right_led.set_color(0, 0, 0);
}

void setLEDRed() {
  alvik.left_led.set_color(1, 0, 0);
  alvik.right_led.set_color(1, 0, 0);
}

void setLEDGreen() {
  alvik.left_led.set_color(0, 1, 0);
  alvik.right_led.set_color(0, 1, 0);
}

void setLEDBlue() {
  alvik.left_led.set_color(0, 0, 1);
  alvik.right_led.set_color(0, 0, 1);
}

void setLEDYellow() {
  alvik.left_led.set_color(1, 1, 0);
  alvik.right_led.set_color(1, 1, 0);
}
