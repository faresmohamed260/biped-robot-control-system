
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <ESPmDNS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>

namespace {
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t CONTROL_INTERVAL_MS = 20;
constexpr uint32_t WIFI_RETRY_MS = 12000;
constexpr uint32_t WIFI_SERVICE_INTERVAL_MS = 100;
constexpr const char* DEVICE_TYPE = "robot_biped";
constexpr const char* DEVICE_MODEL = "Biped Robot";
constexpr const char* FIRMWARE_VERSION = "2026.05-biped1";
constexpr int DEFAULT_ANALOG_DEADZONE = 48;
constexpr int AXIS_MAX_MAGNITUDE = 512;
constexpr int TRIGGER_MAX_MAGNITUDE = 1023;
constexpr float BIPED_L1 = 5.0f;
constexpr float BIPED_L2 = 5.7f;
constexpr float BIPED_STEP_CLEARANCE = 1.0f;
constexpr float BIPED_STEP_HEIGHT = 10.0f;
constexpr float BIPED_STAND_Z = 10.7f;
constexpr float BIPED_STEP_INCREMENT = 0.5f;
constexpr float BIPED_INIT_INCREMENT = 0.1f;
constexpr int BIPED_DEFAULT_STEP_DELAY_MS = 15;

const char* DEFAULT_AP_SSID = "Biped-Setup";
const char* DEFAULT_AP_PASSWORD = "robotarm123";
const char* DEFAULT_HOSTNAME = "dume-biped";

enum JointIndex {
  BASE = 0,
  SHOULDER,
  ELBOW,
  WRIST_PITCH,
  WRIST_ROTATE,
  GRIPPER,
  JOINT_COUNT
};

constexpr int BIPED_POSE_COUNT = 5;
constexpr int BIPED_DEFAULT_POSE_DURATION_MS = 400;
constexpr int BIPED_DEFAULT_INTERP_STEPS = 20;
constexpr int BIPED_DEFAULT_HOLD_MS = 80;
const char* const BIPED_POSE_NAMES[BIPED_POSE_COUNT] = {"stand", "left_forward", "right_forward", "shift_left", "shift_right"};

enum ControlMode {
  CONTROL_NONE = 0,
  CONTROL_AXIS = 1,
  CONTROL_BUTTONS = 2
};

enum MotorType {
  MOTOR_POSITIONAL = 0,
  MOTOR_CONTINUOUS = 1
};

enum AxisSource {
  AXIS_NONE = 0,
  AXIS_LX = 1,
  AXIS_LY = 2,
  AXIS_RX = 3,
  AXIS_RY = 4,
  AXIS_DPAD_X = 5,
  AXIS_DPAD_Y = 6,
  AXIS_TRIGGERS = 7
};

enum ButtonSource {
  BTN_NONE = 0,
  BTN_UP = 1,
  BTN_DOWN = 2,
  BTN_LEFT = 3,
  BTN_RIGHT = 4,
  BTN_SQUARE = 5,
  BTN_CROSS = 6,
  BTN_CIRCLE = 7,
  BTN_TRIANGLE = 8,
  BTN_L1 = 9,
  BTN_R1 = 10,
  BTN_L2 = 11,
  BTN_R2 = 12,
  BTN_SHARE = 13,
  BTN_OPTIONS = 14,
  BTN_L3 = 15,
  BTN_R3 = 16,
  BTN_PS = 17,
  BTN_TOUCHPAD = 18
};

struct Joint {
  const char* name;
  const char* label;
  const char* key;
  Servo servo;
  uint8_t pin;
  uint8_t motorType;
  int minAngle;
  int maxAngle;
  int homeAngle;
  int step;
  int pulseMin;
  int pulseMax;
  int physicalMinAngle;
  int physicalMaxAngle;
  int neutralOutput;
  int stopDeadband;
  int maxSpeedScale;
  bool inverted;
  int position;
  int velocity;
  int startupTarget;
  int rawOutput;
  int storedMinAngle;
  int storedMaxAngle;
  int storedHomeAngle;
  int storedPhysicalMinAngle;
  int storedPhysicalMaxAngle;
  int storedPosition;
  bool manualContinuousControl;
  bool attached;
  uint8_t controlMode;
  uint8_t axisSource;
  uint8_t positiveButton;
  uint8_t negativeButton;
  bool inputInvert;
};

struct ControllerSettings {
  bool enabled = true;
  bool allowNewConnections = true;
  uint8_t ledR = 0;
  uint8_t ledG = 0;
  uint8_t ledB = 255;
  uint8_t rumbleForce = 0;
  uint8_t rumbleDuration = 0;
  int axisCenterLX = 0;
  int axisCenterLY = 0;
  int axisCenterRX = 0;
  int axisCenterRY = 0;
  int axisDeadzone = DEFAULT_ANALOG_DEADZONE;
  uint8_t homeAllButton = BTN_NONE;
  bool homeAllLatched = false;
};

enum ControllerWorkflowState : uint8_t {
  CTL_DISABLED = 0,
  CTL_IDLE = 1,
  CTL_SCANNING = 2,
  CTL_PAIRING = 3,
  CTL_BONDED = 4,
  CTL_RECONNECTING = 5,
  CTL_CONNECTED = 6,
  CTL_ERROR = 7,
};

struct ControllerIdentity {
  String name = "";
  String type = "";
  String btAddress = "";
};

struct WifiSettings {
  String staSsid = "";
  String staPassword = "";
  String apSsid = DEFAULT_AP_SSID;
  String apPassword = DEFAULT_AP_PASSWORD;
  String hostname = DEFAULT_HOSTNAME;
};

struct ConnectedControllerInfo {
  bool connected = false;
  String modelName = "";
  String typeName = "";
  String btAddress = "";
  uint8_t battery = 0;
};

Joint joints[JOINT_COUNT] = {
  {"base",         "Left Ankle", "bas", Servo(), 18, MOTOR_POSITIONAL, 0,   180, 80, 2, 500, 2400, 0, 180, 90, 3, 100, false, 80, 0, 80, 80, 0, 180, 80, 0, 180, 80, false, false, CONTROL_AXIS, AXIS_LX, BTN_NONE, BTN_NONE, false},
  {"shoulder",     "Left Knee", "sho", Servo(), 19, MOTOR_POSITIONAL, 0,   180, 90, 2, 500, 2400, 0, 180, 90, 3, 100, false, 90, 0, 90, 90, 0, 180, 90, 0, 180, 90, false, false, CONTROL_AXIS, AXIS_LY, BTN_NONE, BTN_NONE, true},
  {"elbow",        "Left Hip", "elb", Servo(), 23, MOTOR_POSITIONAL, 0,   180, 110, 2, 500, 2400, 0, 180, 90, 3, 100, false, 110, 0, 110, 110, 0, 180, 110, 0, 180, 110, false, false, CONTROL_AXIS, AXIS_RX, BTN_NONE, BTN_NONE, false},
  {"wrist_pitch",  "Right Knee", "wpi", Servo(), 21, MOTOR_POSITIONAL, 0,   180, 25, 2, 500, 2400, 0, 180, 90, 3, 100, false, 25, 0, 25, 25, 0, 180, 25, 0, 180, 25, false, false, CONTROL_AXIS, AXIS_RY, BTN_NONE, BTN_NONE, true},
  {"wrist_rotate", "Right Ankle", "wro", Servo(), 22, MOTOR_POSITIONAL, 0,   180, 40, 2, 500, 2400, 0, 180, 90, 3, 100, false, 40, 0, 40, 40, 0, 180, 40, 0, 180, 40, false, false, CONTROL_BUTTONS, AXIS_NONE, BTN_UP, BTN_DOWN, false},
  {"gripper",      "Right Hip", "gri", Servo(), 25, MOTOR_POSITIONAL, 0,  180, 135, 2, 500, 2400, 0, 180, 90, 3, 100, false, 135, 0, 135, 135, 0, 180, 135, 0, 180, 135, false, false, CONTROL_BUTTONS, AXIS_NONE, BTN_CIRCLE, BTN_CROSS, false},
};

int bipedPoseAngles[BIPED_POSE_COUNT][JOINT_COUNT] = {
    {80, 90, 110, 25, 40, 135},
    {110, 100, 180, 60, 40, 135},
    {80, 55, 110, 15, 10, 66},
    {80, 90, 110, 25, 40, 135},
    {80, 90, 110, 25, 40, 135},
};

Preferences preferences;
ControllerSettings controllerSettings;
WifiSettings wifiSettings;
ConnectedControllerInfo connectedInfo;
WebServer server(80);
ControllerPtr connectedControllers[BP32_MAX_GAMEPADS];
ControllerPtr activeController = nullptr;
ControllerIdentity rememberedController;
String localBluetoothMac = "";
String lastWifiResult = "boot";
String lastWifiFailure = "";
String mdnsHostname = "";
bool mdnsActive = false;
ControllerWorkflowState controllerWorkflowState = CTL_IDLE;
String controllerStatusText = "idle";
String lastControllerError = "";
uint32_t controllerStateSinceMs = 0;
uint32_t controllerReconnectStartedMs = 0;
uint32_t controllerPairingStartedMs = 0;
uint32_t lastControlTick = 0;
uint32_t wifiConnectStartedMs = 0;
bool wifiConnectInProgress = false;
SemaphoreHandle_t stateMutex = nullptr;
TaskHandle_t controlTaskHandle = nullptr;
bool wifiReconnectRequested = false;
bool rebootRequested = false;

void updateControllerInputs();
void applyVelocityMotion();
void serviceWifiConnection();
void handleBiped();

String prefKey(const Joint& joint, const char* suffix) {
  return String(joint.key) + suffix;
}

void lockState() {
  if (stateMutex != nullptr) {
    xSemaphoreTakeRecursive(stateMutex, portMAX_DELAY);
  }
}

void unlockState() {
  if (stateMutex != nullptr) {
    xSemaphoreGiveRecursive(stateMutex);
  }
}

String jsonQuote(const String& value) {
  String escaped = "\"";
  for (int i = 0; i < value.length(); ++i) {
    char c = value[i];
    if (c == '\\' || c == '"') {
      escaped += '\\';
      escaped += c;
    } else if (c == '\n') {
      escaped += "\\n";
    } else if (c == '\r') {
      escaped += "\\r";
    } else if (c == '\t') {
      escaped += "\\t";
    } else {
      escaped += c;
    }
  }
  escaped += '"';
  return escaped;
}

String boolJson(bool value) {
  return value ? "true" : "false";
}

String htmlEscape(const String& value) {
  String escaped;
  escaped.reserve(value.length() + 16);
  for (int i = 0; i < value.length(); ++i) {
    char c = value[i];
    if (c == '&') escaped += "&amp;";
    else if (c == '<') escaped += "&lt;";
    else if (c == '>') escaped += "&gt;";
    else if (c == '"') escaped += "&quot;";
    else escaped += c;
  }
  return escaped;
}

String formatBdAddress(const uint8_t* addr) {
  if (addr == nullptr) {
    return "";
  }
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02x:%02x:%02x:%02x:%02x:%02x",
           addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
  return String(buffer);
}

String sanitizeMdnsHostname(const String& hostname) {
  String value = hostname;
  value.toLowerCase();
  String sanitized;
  sanitized.reserve(value.length());
  bool previousDash = false;
  for (int i = 0; i < value.length(); ++i) {
    char c = value[i];
    bool valid = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
    if (valid) {
      sanitized += c;
      previousDash = false;
    } else if (!previousDash && sanitized.length() > 0) {
      sanitized += '-';
      previousDash = true;
    }
  }
  while (sanitized.endsWith("-")) {
    sanitized.remove(sanitized.length() - 1);
  }
  if (sanitized.length() == 0) {
    sanitized = DEFAULT_HOSTNAME;
  }
  return sanitized;
}

String wifiStatusString(wl_status_t status) {
  switch (status) {
    case WL_CONNECTED: return "connected";
    case WL_NO_SSID_AVAIL: return "no_ssid";
    case WL_CONNECT_FAILED: return "connect_failed";
    case WL_CONNECTION_LOST: return "connection_lost";
    case WL_DISCONNECTED: return "disconnected";
    case WL_IDLE_STATUS: return "idle";
    default: return "unknown";
  }
}

void stopMdns() {
  if (mdnsActive) {
    MDNS.end();
    mdnsActive = false;
  }
}

void startMdnsIfReady() {
  stopMdns();
  mdnsHostname = sanitizeMdnsHostname(wifiSettings.hostname);
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  if (MDNS.begin(mdnsHostname.c_str())) {
    MDNS.addService("http", "tcp", 80);
    mdnsActive = true;
  } else {
    lastWifiFailure = "mdns_start_failed";
  }
}

const char* controllerTypeName(int type) {
  switch (type) {
    case Controller::CONTROLLER_TYPE_PS4Controller: return "ps4";
    case Controller::CONTROLLER_TYPE_PS5Controller: return "ps5";
    case Controller::CONTROLLER_TYPE_XBoxOneController: return "xbox_one";
    case Controller::CONTROLLER_TYPE_XBox360Controller: return "xbox_360";
    case Controller::CONTROLLER_TYPE_SwitchProController: return "switch_pro";
    case Controller::CONTROLLER_TYPE_SwitchJoyConLeft: return "joycon_left";
    case Controller::CONTROLLER_TYPE_SwitchJoyConRight: return "joycon_right";
    case Controller::CONTROLLER_TYPE_SwitchJoyConPair: return "joycon_pair";
    case Controller::CONTROLLER_TYPE_PS3Controller: return "ps3";
    default: return "generic";
  }
}

const char* controllerWorkflowStateName(ControllerWorkflowState state) {
  switch (state) {
    case CTL_DISABLED: return "disabled";
    case CTL_IDLE: return "idle";
    case CTL_SCANNING: return "scanning";
    case CTL_PAIRING: return "pairing";
    case CTL_BONDED: return "bonded";
    case CTL_RECONNECTING: return "reconnecting";
    case CTL_CONNECTED: return "connected";
    case CTL_ERROR: return "error";
    default: return "unknown";
  }
}

void setControllerWorkflowState(ControllerWorkflowState state, const String& statusText) {
  if (controllerWorkflowState != state || controllerStatusText != statusText) {
    controllerWorkflowState = state;
    controllerStatusText = statusText;
    controllerStateSinceMs = millis();
  }
}

void clearControllerError() {
  lastControllerError = "";
}

void setControllerError(const String& errorText) {
  lastControllerError = errorText;
  setControllerWorkflowState(CTL_ERROR, errorText);
}

void saveRememberedController() {
  preferences.putString("ctl_mem_name", rememberedController.name);
  preferences.putString("ctl_mem_type", rememberedController.type);
  preferences.putString("ctl_mem_addr", rememberedController.btAddress);
}

void forgetRememberedController() {
  rememberedController = ControllerIdentity();
  saveRememberedController();
}

const char* controlModeName(uint8_t value) {
  switch (value) {
    case CONTROL_AXIS: return "axis";
    case CONTROL_BUTTONS: return "buttons";
    default: return "none";
  }
}

const char* motorTypeName(uint8_t value) {
  switch (value) {
    case MOTOR_CONTINUOUS: return "continuous_360";
    default: return "positional_180";
  }
}

const char* jointCoordinateSpaceName(const Joint& joint) {
  return joint.motorType == MOTOR_CONTINUOUS ? "speed_percent" : "servo_angle_degrees";
}

const char* axisSourceName(uint8_t value) {
  switch (value) {
    case AXIS_LX: return "left_stick_x";
    case AXIS_LY: return "left_stick_y";
    case AXIS_RX: return "right_stick_x";
    case AXIS_RY: return "right_stick_y";
    case AXIS_DPAD_X: return "dpad_x";
    case AXIS_DPAD_Y: return "dpad_y";
    case AXIS_TRIGGERS: return "triggers";
    default: return "none";
  }
}

const char* buttonSourceName(uint8_t value) {
  switch (value) {
    case BTN_UP: return "up";
    case BTN_DOWN: return "down";
    case BTN_LEFT: return "left";
    case BTN_RIGHT: return "right";
    case BTN_SQUARE: return "square";
    case BTN_CROSS: return "cross";
    case BTN_CIRCLE: return "circle";
    case BTN_TRIANGLE: return "triangle";
    case BTN_L1: return "l1";
    case BTN_R1: return "r1";
    case BTN_L2: return "l2";
    case BTN_R2: return "r2";
    case BTN_SHARE: return "share";
    case BTN_OPTIONS: return "options";
    case BTN_L3: return "l3";
    case BTN_R3: return "r3";
    case BTN_PS: return "ps";
    case BTN_TOUCHPAD: return "touchpad";
    default: return "none";
  }
}

void sendOk(const String& extra = "{}") {
  server.send(200, "application/json", "{\"ok\":true,\"result\":" + extra + "}");
}

void sendError(const String& message, int code = 400) {
  server.send(code, "application/json", "{\"ok\":false,\"error\":" + jsonQuote(message) + "}");
}

int clampAngle(const Joint& joint, int value) {
  return constrain(value, joint.minAngle, joint.maxAngle);
}

float logicalToPhysicalAngle(const Joint& joint, int logicalValue) {
  if (joint.motorType == MOTOR_CONTINUOUS) {
    return static_cast<float>(logicalValue);
  }
  int logicalMin = joint.minAngle;
  int logicalMax = joint.maxAngle;
  if (logicalMax == logicalMin) {
    return static_cast<float>(joint.physicalMinAngle);
  }
  float ratio = static_cast<float>(logicalValue - logicalMin) / static_cast<float>(logicalMax - logicalMin);
  return static_cast<float>(joint.physicalMinAngle) +
         ratio * static_cast<float>(joint.physicalMaxAngle - joint.physicalMinAngle);
}

int logicalToRawOutput(const Joint& joint, int value) {
  if (joint.motorType == MOTOR_CONTINUOUS) {
    int speed = joint.inverted ? -value : value;
    speed = (speed * joint.maxSpeedScale) / 100;
    speed = constrain(speed, -100, 100);
    if (abs(speed) <= joint.stopDeadband) {
      return constrain(joint.neutralOutput, 0, 180);
    }
    if (speed > 0) {
      int start = min(180, joint.neutralOutput + 1);
      return map(speed, joint.stopDeadband + 1, 100, start, 180);
    }
    int start = max(0, joint.neutralOutput - 1);
    return map(speed, -joint.stopDeadband - 1, -100, start, 0);
  }
  return joint.inverted ? (180 - value) : value;
}

void normalizeJointConfig(Joint& joint) {
  joint.motorType = constrain(joint.motorType, MOTOR_POSITIONAL, MOTOR_CONTINUOUS);
  int commandMin = joint.motorType == MOTOR_CONTINUOUS ? -100 : 0;
  int commandMax = joint.motorType == MOTOR_CONTINUOUS ? 100 : 180;
  joint.minAngle = constrain(joint.minAngle, commandMin, commandMax);
  joint.maxAngle = constrain(joint.maxAngle, commandMin, commandMax);
  if (joint.maxAngle < joint.minAngle) {
    int swap = joint.minAngle;
    joint.minAngle = joint.maxAngle;
    joint.maxAngle = swap;
  }
  if (joint.motorType == MOTOR_CONTINUOUS && joint.homeAngle == 90) {
    joint.homeAngle = 0;
  }
  if (joint.motorType == MOTOR_CONTINUOUS) {
    joint.minAngle = -100;
    joint.maxAngle = 100;
    joint.homeAngle = 0;
    joint.physicalMinAngle = -100;
    joint.physicalMaxAngle = 100;
  }
  joint.homeAngle = clampAngle(joint, joint.homeAngle);
  joint.position = clampAngle(joint, joint.position);
  joint.startupTarget = clampAngle(joint, joint.homeAngle);
  joint.velocity = 0;
  if (joint.motorType != MOTOR_CONTINUOUS) {
    joint.manualContinuousControl = false;
  }
  joint.step = max(1, joint.step);
  joint.pulseMin = max(100, joint.pulseMin);
  joint.pulseMax = max(joint.pulseMin + 100, joint.pulseMax);
  joint.neutralOutput = constrain(joint.neutralOutput, 0, 180);
  joint.stopDeadband = constrain(joint.stopDeadband, 0, 20);
  joint.maxSpeedScale = constrain(joint.maxSpeedScale, 1, 100);
  joint.controlMode = constrain(joint.controlMode, CONTROL_NONE, CONTROL_BUTTONS);
  joint.axisSource = constrain(joint.axisSource, AXIS_NONE, AXIS_TRIGGERS);
  joint.positiveButton = constrain(joint.positiveButton, BTN_NONE, BTN_TOUCHPAD);
  joint.negativeButton = constrain(joint.negativeButton, BTN_NONE, BTN_TOUCHPAD);
  joint.rawOutput = logicalToRawOutput(joint, joint.position);
}

int axisToCommand(const Joint& joint, int value, int maxMagnitude, bool invertInput) {
  int signedValue = invertInput ? -value : value;
  int clamped = constrain(signedValue, -maxMagnitude, maxMagnitude);
  if (joint.motorType == MOTOR_CONTINUOUS) {
    if (abs(clamped) < controllerSettings.axisDeadzone) {
      return 0;
    }
    return map(clamped, -maxMagnitude, maxMagnitude, joint.minAngle, joint.maxAngle);
  }
  int deadzoneApplied = abs(clamped) < controllerSettings.axisDeadzone ? 0 : clamped;
  return map(deadzoneApplied, -maxMagnitude, maxMagnitude, joint.minAngle, joint.maxAngle);
}

void attachJoint(Joint& joint, bool reattach = false) {
  lockState();
  if (joint.attached && !reattach) {
    unlockState();
    return;
  }
  if (joint.attached && reattach) {
    joint.servo.detach();
    joint.attached = false;
  }
  if (!joint.attached) {
    joint.servo.setPeriodHertz(50);
    joint.servo.attach(joint.pin, joint.pulseMin, joint.pulseMax);
    joint.attached = true;
  }
  unlockState();
}

void detachJoint(Joint& joint) {
  lockState();
  if (!joint.attached) {
    unlockState();
    return;
  }
  joint.servo.detach();
  joint.attached = false;
  unlockState();
}

void writeJoint(Joint& joint, int requestedPosition, bool forceAttach = true) {
  lockState();
  joint.position = clampAngle(joint, requestedPosition);
  joint.rawOutput = logicalToRawOutput(joint, joint.position);
  if (forceAttach) {
    attachJoint(joint);
    joint.servo.write(joint.rawOutput);
  }
  unlockState();
}

void writeAllJoints() {
  for (int i = 0; i < JOINT_COUNT; ++i) {
    writeJoint(joints[i], joints[i].position);
  }
}

int bipedPoseIndexByName(const String& name) {
  for (int i = 0; i < BIPED_POSE_COUNT; ++i) {
    if (name.equalsIgnoreCase(BIPED_POSE_NAMES[i])) {
      return i;
    }
  }
  return -1;
}

String bipedPosePrefKey(int poseIndex, int jointIndex) {
  return "bp_" + String(poseIndex) + "_" + String(jointIndex);
}

void setBipedPoseDefaults() {
  const int defaults[BIPED_POSE_COUNT][JOINT_COUNT] = {
      {80, 90, 110, 25, 40, 135},
      {110, 100, 180, 60, 40, 135},
      {80, 55, 110, 15, 10, 66},
      {80, 90, 110, 25, 40, 135},
      {80, 90, 110, 25, 40, 135},
  };
  for (int poseIndex = 0; poseIndex < BIPED_POSE_COUNT; ++poseIndex) {
    for (int jointIndex = 0; jointIndex < JOINT_COUNT; ++jointIndex) {
      bipedPoseAngles[poseIndex][jointIndex] = defaults[poseIndex][jointIndex];
    }
  }
}

void loadBipedPoseSettings() {
  setBipedPoseDefaults();
  for (int poseIndex = 0; poseIndex < BIPED_POSE_COUNT; ++poseIndex) {
    for (int jointIndex = 0; jointIndex < JOINT_COUNT; ++jointIndex) {
      String key = bipedPosePrefKey(poseIndex, jointIndex);
      if (preferences.isKey(key.c_str())) {
        bipedPoseAngles[poseIndex][jointIndex] = preferences.getInt(key.c_str(), bipedPoseAngles[poseIndex][jointIndex]);
      }
    }
  }
}

void saveBipedPoseSettings(int poseIndex) {
  if (poseIndex < 0 || poseIndex >= BIPED_POSE_COUNT) {
    return;
  }
  for (int jointIndex = 0; jointIndex < JOINT_COUNT; ++jointIndex) {
    String key = bipedPosePrefKey(poseIndex, jointIndex);
    preferences.putInt(key.c_str(), bipedPoseAngles[poseIndex][jointIndex]);
  }
}

void saveAllBipedPoseSettings() {
  for (int poseIndex = 0; poseIndex < BIPED_POSE_COUNT; ++poseIndex) {
    saveBipedPoseSettings(poseIndex);
  }
}

void captureCurrentBipedPose(int poseIndex) {
  if (poseIndex < 0 || poseIndex >= BIPED_POSE_COUNT) {
    return;
  }
  for (int jointIndex = 0; jointIndex < JOINT_COUNT; ++jointIndex) {
    bipedPoseAngles[poseIndex][jointIndex] = joints[jointIndex].position;
  }
  saveBipedPoseSettings(poseIndex);
}

void setExplicitBipedPose(int poseIndex, const int values[JOINT_COUNT]) {
  if (poseIndex < 0 || poseIndex >= BIPED_POSE_COUNT) {
    return;
  }
  for (int jointIndex = 0; jointIndex < JOINT_COUNT; ++jointIndex) {
    bipedPoseAngles[poseIndex][jointIndex] = clampAngle(joints[jointIndex], values[jointIndex]);
  }
  saveBipedPoseSettings(poseIndex);
}

void moveToBipedPoseAngles(const int targetAngles[JOINT_COUNT], int durationMs, int interpolationSteps) {
  int startAngles[JOINT_COUNT];
  for (int jointIndex = 0; jointIndex < JOINT_COUNT; ++jointIndex) {
    startAngles[jointIndex] = joints[jointIndex].position;
  }

  int safeSteps = max(1, interpolationSteps);
  if (durationMs <= 0 || safeSteps <= 1) {
    for (int jointIndex = 0; jointIndex < JOINT_COUNT; ++jointIndex) {
      writeJoint(joints[jointIndex], clampAngle(joints[jointIndex], targetAngles[jointIndex]));
    }
    return;
  }

  int stepDelayMs = durationMs / safeSteps;
  for (int stepIndex = 1; stepIndex <= safeSteps; ++stepIndex) {
    float t = static_cast<float>(stepIndex) / static_cast<float>(safeSteps);
    for (int jointIndex = 0; jointIndex < JOINT_COUNT; ++jointIndex) {
      float interpolated = startAngles[jointIndex] + (targetAngles[jointIndex] - startAngles[jointIndex]) * t;
      writeJoint(joints[jointIndex], clampAngle(joints[jointIndex], static_cast<int>(roundf(interpolated))));
    }
    if (stepDelayMs > 0) {
      delay(stepDelayMs);
    }
  }
}

void executeBipedPose(int poseIndex, int durationMs, int interpolationSteps, int holdMs) {
  if (poseIndex < 0 || poseIndex >= BIPED_POSE_COUNT) {
    return;
  }
  moveToBipedPoseAngles(bipedPoseAngles[poseIndex], durationMs, interpolationSteps);
  if (holdMs > 0) {
    delay(holdMs);
  }
}

bool playBipedSequence(const String& namesCsv, int durationMs, int interpolationSteps, int holdMs, int repeatCount, String& error) {
  String remaining = namesCsv;
  remaining.trim();
  if (remaining.length() == 0) {
    error = "empty_sequence";
    return false;
  }

  int safeRepeatCount = max(1, repeatCount);
  for (int repeatIndex = 0; repeatIndex < safeRepeatCount; ++repeatIndex) {
    String cursor = remaining;
    while (cursor.length() > 0) {
      int commaIndex = cursor.indexOf(',');
      String token = commaIndex >= 0 ? cursor.substring(0, commaIndex) : cursor;
      token.trim();
      if (token.length() > 0) {
        int poseIndex = bipedPoseIndexByName(token);
        if (poseIndex < 0) {
          error = "unknown_pose:" + token;
          return false;
        }
        executeBipedPose(poseIndex, durationMs, interpolationSteps, holdMs);
      }
      if (commaIndex < 0) {
        break;
      }
      cursor = cursor.substring(commaIndex + 1);
      cursor.trim();
    }
  }
  return true;
}

String bipedPoseJson(int poseIndex) {
  String json = "{";
  json += "\"name\":" + jsonQuote(BIPED_POSE_NAMES[poseIndex]);
  json += ",\"angles\":{";
  for (int jointIndex = 0; jointIndex < JOINT_COUNT; ++jointIndex) {
    if (jointIndex > 0) {
      json += ",";
    }
    json += jsonQuote(joints[jointIndex].name);
    json += ":";
    json += String(bipedPoseAngles[poseIndex][jointIndex]);
  }
  json += "}}";
  return json;
}

String bipedJson() {
  String json = "{\"poses\":[";
  for (int poseIndex = 0; poseIndex < BIPED_POSE_COUNT; ++poseIndex) {
    if (poseIndex > 0) {
      json += ",";
    }
    json += bipedPoseJson(poseIndex);
  }
  json += "]}";
  return json;
}

void saveJointSettings(const Joint& joint) {
  preferences.putUChar(prefKey(joint, "_pin").c_str(), joint.pin);
  preferences.putUChar(prefKey(joint, "_type").c_str(), joint.motorType);
  preferences.putInt(prefKey(joint, "_min").c_str(), joint.minAngle);
  preferences.putInt(prefKey(joint, "_max").c_str(), joint.maxAngle);
  preferences.putInt(prefKey(joint, "_home").c_str(), joint.homeAngle);
  preferences.putInt(prefKey(joint, "_phmin").c_str(), joint.physicalMinAngle);
  preferences.putInt(prefKey(joint, "_phmax").c_str(), joint.physicalMaxAngle);
  preferences.putInt(prefKey(joint, "_step").c_str(), joint.step);
  preferences.putInt(prefKey(joint, "_pmin").c_str(), joint.pulseMin);
  preferences.putInt(prefKey(joint, "_pmax").c_str(), joint.pulseMax);
  preferences.putInt(prefKey(joint, "_neu").c_str(), joint.neutralOutput);
  preferences.putInt(prefKey(joint, "_dead").c_str(), joint.stopDeadband);
  preferences.putInt(prefKey(joint, "_scale").c_str(), joint.maxSpeedScale);
  preferences.putBool(prefKey(joint, "_inv").c_str(), joint.inverted);
  preferences.putInt(prefKey(joint, "_pos").c_str(), joint.position);
  preferences.putUChar(prefKey(joint, "_cm").c_str(), joint.controlMode);
  preferences.putUChar(prefKey(joint, "_ax").c_str(), joint.axisSource);
  preferences.putUChar(prefKey(joint, "_pb").c_str(), joint.positiveButton);
  preferences.putUChar(prefKey(joint, "_nb").c_str(), joint.negativeButton);
  preferences.putBool(prefKey(joint, "_iinv").c_str(), joint.inputInvert);
}

void saveControllerSettings() {
  preferences.putBool("ctl_enabled", controllerSettings.enabled);
  preferences.putBool("ctl_pair", controllerSettings.allowNewConnections);
  preferences.putUChar("ctl_led_r", controllerSettings.ledR);
  preferences.putUChar("ctl_led_g", controllerSettings.ledG);
  preferences.putUChar("ctl_led_b", controllerSettings.ledB);
  preferences.putUChar("ctl_r_force", controllerSettings.rumbleForce);
  preferences.putUChar("ctl_r_dur", controllerSettings.rumbleDuration);
  preferences.putInt("ctl_c_lx", controllerSettings.axisCenterLX);
  preferences.putInt("ctl_c_ly", controllerSettings.axisCenterLY);
  preferences.putInt("ctl_c_rx", controllerSettings.axisCenterRX);
  preferences.putInt("ctl_c_ry", controllerSettings.axisCenterRY);
  preferences.putInt("ctl_dead", controllerSettings.axisDeadzone);
  preferences.putUChar("ctl_home_btn", controllerSettings.homeAllButton);
  saveRememberedController();
}

void saveWifiSettings() {
  preferences.putString("wifi_sta_ssid", wifiSettings.staSsid);
  preferences.putString("wifi_sta_pw", wifiSettings.staPassword);
  preferences.putString("wifi_ap_ssid", wifiSettings.apSsid);
  preferences.putString("wifi_ap_pw", wifiSettings.apPassword);
  preferences.putString("wifi_host", wifiSettings.hostname);
}

void saveAllSettings() {
  for (int i = 0; i < JOINT_COUNT; ++i) {
    saveJointSettings(joints[i]);
  }
  saveControllerSettings();
  saveWifiSettings();
}

void loadJointSettings(Joint& joint) {
  joint.pin = preferences.getUChar(prefKey(joint, "_pin").c_str(), joint.pin);
  joint.motorType = preferences.getUChar(prefKey(joint, "_type").c_str(), joint.motorType);
  joint.minAngle = preferences.getInt(prefKey(joint, "_min").c_str(), joint.minAngle);
  joint.maxAngle = preferences.getInt(prefKey(joint, "_max").c_str(), joint.maxAngle);
  joint.homeAngle = preferences.getInt(prefKey(joint, "_home").c_str(), joint.homeAngle);
  joint.physicalMinAngle = preferences.getInt(prefKey(joint, "_phmin").c_str(), joint.physicalMinAngle);
  joint.physicalMaxAngle = preferences.getInt(prefKey(joint, "_phmax").c_str(), joint.physicalMaxAngle);
  joint.step = preferences.getInt(prefKey(joint, "_step").c_str(), joint.step);
  joint.pulseMin = preferences.getInt(prefKey(joint, "_pmin").c_str(), joint.pulseMin);
  joint.pulseMax = preferences.getInt(prefKey(joint, "_pmax").c_str(), joint.pulseMax);
  joint.neutralOutput = preferences.getInt(prefKey(joint, "_neu").c_str(), joint.neutralOutput);
  joint.stopDeadband = preferences.getInt(prefKey(joint, "_dead").c_str(), joint.stopDeadband);
  joint.maxSpeedScale = preferences.getInt(prefKey(joint, "_scale").c_str(), joint.maxSpeedScale);
  joint.inverted = preferences.getBool(prefKey(joint, "_inv").c_str(), joint.inverted);
  joint.position = preferences.getInt(prefKey(joint, "_pos").c_str(), joint.position);
  joint.controlMode = preferences.getUChar(prefKey(joint, "_cm").c_str(), joint.controlMode);
  joint.axisSource = preferences.getUChar(prefKey(joint, "_ax").c_str(), joint.axisSource);
  joint.positiveButton = preferences.getUChar(prefKey(joint, "_pb").c_str(), joint.positiveButton);
  joint.negativeButton = preferences.getUChar(prefKey(joint, "_nb").c_str(), joint.negativeButton);
  joint.inputInvert = preferences.getBool(prefKey(joint, "_iinv").c_str(), joint.inputInvert);
  joint.storedMinAngle = joint.minAngle;
  joint.storedMaxAngle = joint.maxAngle;
  joint.storedHomeAngle = joint.homeAngle;
  joint.storedPhysicalMinAngle = joint.physicalMinAngle;
  joint.storedPhysicalMaxAngle = joint.physicalMaxAngle;
  joint.storedPosition = joint.position;
  normalizeJointConfig(joint);
}

void loadControllerSettings() {
  controllerSettings.enabled = preferences.getBool("ctl_enabled", controllerSettings.enabled);
  controllerSettings.allowNewConnections = preferences.getBool("ctl_pair", controllerSettings.allowNewConnections);
  controllerSettings.ledR = preferences.getUChar("ctl_led_r", controllerSettings.ledR);
  controllerSettings.ledG = preferences.getUChar("ctl_led_g", controllerSettings.ledG);
  controllerSettings.ledB = preferences.getUChar("ctl_led_b", controllerSettings.ledB);
  controllerSettings.rumbleForce = preferences.getUChar("ctl_r_force", controllerSettings.rumbleForce);
  controllerSettings.rumbleDuration = preferences.getUChar("ctl_r_dur", controllerSettings.rumbleDuration);
  controllerSettings.axisCenterLX = preferences.getInt("ctl_c_lx", controllerSettings.axisCenterLX);
  controllerSettings.axisCenterLY = preferences.getInt("ctl_c_ly", controllerSettings.axisCenterLY);
  controllerSettings.axisCenterRX = preferences.getInt("ctl_c_rx", controllerSettings.axisCenterRX);
  controllerSettings.axisCenterRY = preferences.getInt("ctl_c_ry", controllerSettings.axisCenterRY);
  controllerSettings.axisDeadzone = constrain(preferences.getInt("ctl_dead", controllerSettings.axisDeadzone), 0, 200);
  controllerSettings.homeAllButton = constrain(preferences.getUChar("ctl_home_btn", controllerSettings.homeAllButton), BTN_NONE, BTN_TOUCHPAD);
  rememberedController.name = preferences.getString("ctl_mem_name", "");
  rememberedController.type = preferences.getString("ctl_mem_type", "");
  rememberedController.btAddress = preferences.getString("ctl_mem_addr", "");
}

void loadWifiSettings() {
  wifiSettings.staSsid = preferences.getString("wifi_sta_ssid", wifiSettings.staSsid);
  wifiSettings.staPassword = preferences.getString("wifi_sta_pw", wifiSettings.staPassword);
  wifiSettings.apSsid = preferences.getString("wifi_ap_ssid", wifiSettings.apSsid);
  wifiSettings.apPassword = preferences.getString("wifi_ap_pw", wifiSettings.apPassword);
  wifiSettings.hostname = preferences.getString("wifi_host", wifiSettings.hostname);
  if (wifiSettings.apSsid.length() == 0) {
    wifiSettings.apSsid = DEFAULT_AP_SSID;
  }
  if (wifiSettings.apPassword.length() < 8) {
    wifiSettings.apPassword = DEFAULT_AP_PASSWORD;
  }
  if (wifiSettings.hostname.length() == 0) {
    wifiSettings.hostname = DEFAULT_HOSTNAME;
  }
}

void loadAllSettings() {
  for (int i = 0; i < JOINT_COUNT; ++i) {
    loadJointSettings(joints[i]);
  }
  loadControllerSettings();
  loadWifiSettings();
}

void selectActiveController() {
  lockState();
  activeController = nullptr;
  connectedInfo = ConnectedControllerInfo();
  for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
    if (connectedControllers[i] != nullptr && connectedControllers[i]->isConnected() && connectedControllers[i]->isGamepad()) {
      activeController = connectedControllers[i];
      connectedInfo.connected = true;
      connectedInfo.modelName = activeController->getModelName();
      connectedInfo.typeName = controllerTypeName(activeController->getModel());
      ControllerProperties properties = activeController->getProperties();
      connectedInfo.btAddress = formatBdAddress(properties.btaddr);
      connectedInfo.battery = activeController->battery();
      break;
    }
  }
  unlockState();
}

void refreshControllerWorkflowState() {
  lockState();
  if (!controllerSettings.enabled) {
    setControllerWorkflowState(CTL_DISABLED, "controller input disabled");
    unlockState();
    return;
  }
  if (connectedInfo.connected) {
    setControllerWorkflowState(CTL_CONNECTED, "controller connected");
    unlockState();
    return;
  }
  if (controllerSettings.allowNewConnections) {
    uint32_t elapsed = controllerPairingStartedMs > 0 ? millis() - controllerPairingStartedMs : 0;
    if (elapsed > 0 && elapsed < 2000) {
      setControllerWorkflowState(CTL_PAIRING, "waiting for selected controller to connect");
    } else {
      setControllerWorkflowState(CTL_SCANNING, "pairing mode enabled; put the controller in pairing mode");
    }
    unlockState();
    return;
  }
  if (rememberedController.btAddress.length() > 0) {
    setControllerWorkflowState(CTL_RECONNECTING, "waiting for remembered controller to reconnect");
    unlockState();
    return;
  }
  setControllerWorkflowState(CTL_IDLE, "ready to pair a controller");
  unlockState();
}

void initializeJointsForStartup() {
  for (int i = 0; i < JOINT_COUNT; ++i) {
    Joint& joint = joints[i];
    normalizeJointConfig(joint);
    joint.startupTarget = joint.motorType == MOTOR_CONTINUOUS ? 0 : joint.homeAngle;
    joint.velocity = 0;
    attachJoint(joint, true);
    writeJoint(joint, joint.startupTarget, false);
    joint.servo.write(joint.rawOutput);
    joint.position = joint.startupTarget;
  }
}

void controlTask(void* parameter) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t intervalTicks = pdMS_TO_TICKS(CONTROL_INTERVAL_MS);
  for (;;) {
    BP32.update();

    if (!connectedInfo.connected) {
      if (controllerSettings.allowNewConnections && controllerPairingStartedMs > 0 && millis() - controllerPairingStartedMs > 45000) {
        lastControllerError = "pairing_timeout";
      } else if (!controllerSettings.allowNewConnections && rememberedController.btAddress.length() > 0 &&
                 controllerReconnectStartedMs > 0 && millis() - controllerReconnectStartedMs > 15000 && lastControllerError.length() == 0) {
        lastControllerError = "reconnect_timeout_turn_on_remembered_controller";
      }
    }

    refreshControllerWorkflowState();
    updateControllerInputs();
    applyVelocityMotion();
    lastControlTick = millis();
    vTaskDelayUntil(&lastWake, intervalTicks);
  }
}

void applyControllerFeedback() {
  if (activeController == nullptr || !activeController->isConnected()) {
    return;
  }
  activeController->setColorLED(controllerSettings.ledR, controllerSettings.ledG, controllerSettings.ledB);
  if (controllerSettings.rumbleForce > 0 && controllerSettings.rumbleDuration > 0) {
    activeController->setRumble(controllerSettings.rumbleForce, controllerSettings.rumbleDuration);
  }
}

void onConnectedController(ControllerPtr ctl) {
  lockState();
  for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
    if (connectedControllers[i] == nullptr) {
      connectedControllers[i] = ctl;
      break;
    }
  }
  selectActiveController();
  if (connectedInfo.connected) {
    rememberedController.name = connectedInfo.modelName;
    rememberedController.type = connectedInfo.typeName;
    rememberedController.btAddress = connectedInfo.btAddress;
    saveRememberedController();
    controllerReconnectStartedMs = 0;
    clearControllerError();
    setControllerWorkflowState(CTL_CONNECTED, "controller connected");
  }
  unlockState();
  applyControllerFeedback();
}

void onDisconnectedController(ControllerPtr ctl) {
  lockState();
  for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
    if (connectedControllers[i] == ctl) {
      connectedControllers[i] = nullptr;
      break;
    }
  }
  selectActiveController();
  if (controllerSettings.enabled && rememberedController.btAddress.length() > 0 && !controllerSettings.allowNewConnections) {
    controllerReconnectStartedMs = millis();
  }
  unlockState();
  refreshControllerWorkflowState();
}

void startControllerStack() {
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableNewBluetoothConnections(controllerSettings.allowNewConnections);
  localBluetoothMac = formatBdAddress(BP32.localBdAddress());
  controllerPairingStartedMs = controllerSettings.allowNewConnections ? millis() : 0;
  controllerReconnectStartedMs = (!controllerSettings.allowNewConnections && rememberedController.btAddress.length() > 0) ? millis() : 0;
  refreshControllerWorkflowState();
}

void startWifi() {
  stopMdns();
  WiFi.disconnect(false, false);
  delay(200);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setHostname(wifiSettings.hostname.c_str());
  bool apStarted = WiFi.softAP(wifiSettings.apSsid.c_str(), wifiSettings.apPassword.c_str());
  lastWifiResult = apStarted ? "ap_started" : "ap_start_failed";
  lastWifiFailure = apStarted ? "" : "ap_start_failed";
  if (wifiSettings.staSsid.length() > 0) {
    WiFi.begin(wifiSettings.staSsid.c_str(), wifiSettings.staPassword.c_str());
    wifiConnectStartedMs = millis();
    wifiConnectInProgress = true;
    lastWifiResult = "sta_connecting";
    lastWifiFailure = "";
  } else {
    lastWifiResult = "sta_not_configured";
    lastWifiFailure = "sta_not_configured";
    wifiConnectInProgress = false;
  }
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("STA IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("mDNS: ");
  Serial.println(mdnsActive ? String(mdnsHostname + ".local") : String("inactive"));
}

void serviceWifiConnection() {
  if (!wifiConnectInProgress) {
    if (WiFi.status() == WL_CONNECTED && !mdnsActive) {
      startMdnsIfReady();
    }
    return;
  }
  wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    wifiConnectInProgress = false;
    lastWifiResult = "sta_connected";
    lastWifiFailure = "";
    startMdnsIfReady();
    return;
  }
  if (millis() - wifiConnectStartedMs >= WIFI_RETRY_MS) {
    wifiConnectInProgress = false;
    lastWifiResult = "sta_failed";
    lastWifiFailure = wifiStatusString(status);
  }
}

int dpadValueX(ControllerPtr controller) {
  if (controller == nullptr) {
    return 0;
  }
  int direction = 0;
  if (controller->dpad() & DPAD_RIGHT) direction += AXIS_MAX_MAGNITUDE;
  if (controller->dpad() & DPAD_LEFT) direction -= AXIS_MAX_MAGNITUDE;
  return direction;
}

int dpadValueY(ControllerPtr controller) {
  if (controller == nullptr) {
    return 0;
  }
  int direction = 0;
  if (controller->dpad() & DPAD_UP) direction += AXIS_MAX_MAGNITUDE;
  if (controller->dpad() & DPAD_DOWN) direction -= AXIS_MAX_MAGNITUDE;
  return direction;
}

int getVelocity(int value, int maxMagnitude, bool invert = false) {
  int magnitude = abs(value);
  if (magnitude < controllerSettings.axisDeadzone || maxMagnitude <= controllerSettings.axisDeadzone) {
    return 0;
  }
  int scaled = map(magnitude, controllerSettings.axisDeadzone, maxMagnitude, 1, 6);
  int direction = (value > 0) ? 1 : -1;
  return (invert ? -direction : direction) * scaled;
}

bool buttonPressed(uint8_t button) {
  if (activeController == nullptr || !activeController->isConnected()) {
    return false;
  }
  switch (button) {
    case BTN_UP: return activeController->dpad() & DPAD_UP;
    case BTN_DOWN: return activeController->dpad() & DPAD_DOWN;
    case BTN_LEFT: return activeController->dpad() & DPAD_LEFT;
    case BTN_RIGHT: return activeController->dpad() & DPAD_RIGHT;
    case BTN_SQUARE: return activeController->x();
    case BTN_CROSS: return activeController->a();
    case BTN_CIRCLE: return activeController->b();
    case BTN_TRIANGLE: return activeController->y();
    case BTN_L1: return activeController->l1();
    case BTN_R1: return activeController->r1();
    case BTN_L2: return activeController->l2();
    case BTN_R2: return activeController->r2();
    case BTN_SHARE: return activeController->miscBack();
    case BTN_OPTIONS: return activeController->miscHome();
    case BTN_L3: return activeController->thumbL();
    case BTN_R3: return activeController->thumbR();
    case BTN_PS: return activeController->miscSystem();
    case BTN_TOUCHPAD: return false;
    default: return false;
  }
}

String controllerInputsJson() {
  int rawLX = 0;
  int rawLY = 0;
  int rawRX = 0;
  int rawRY = 0;
  int rawThrottle = 0;
  int rawBrake = 0;
  int centeredLX = 0;
  int centeredLY = 0;
  int centeredRX = 0;
  int centeredRY = 0;
  int triggerDifference = 0;
  int dpadX = 0;
  int dpadY = 0;

  bool up = false;
  bool down = false;
  bool left = false;
  bool right = false;
  bool square = false;
  bool cross = false;
  bool circle = false;
  bool triangle = false;
  bool l1 = false;
  bool r1 = false;
  bool l2 = false;
  bool r2 = false;
  bool share = false;
  bool options = false;
  bool l3 = false;
  bool r3 = false;
  bool ps = false;
  bool touchpad = false;

  if (activeController != nullptr && activeController->isConnected()) {
    rawLX = activeController->axisX();
    rawLY = -activeController->axisY();
    rawRX = activeController->axisRX();
    rawRY = -activeController->axisRY();
    rawThrottle = activeController->throttle();
    rawBrake = activeController->brake();

    centeredLX = rawLX - controllerSettings.axisCenterLX;
    centeredLY = rawLY - controllerSettings.axisCenterLY;
    centeredRX = rawRX - controllerSettings.axisCenterRX;
    centeredRY = rawRY - controllerSettings.axisCenterRY;
    triggerDifference = rawThrottle - rawBrake;

    up = activeController->dpad() & DPAD_UP;
    down = activeController->dpad() & DPAD_DOWN;
    left = activeController->dpad() & DPAD_LEFT;
    right = activeController->dpad() & DPAD_RIGHT;
    dpadX = dpadValueX(activeController);
    dpadY = dpadValueY(activeController);

    square = activeController->x();
    cross = activeController->a();
    circle = activeController->b();
    triangle = activeController->y();
    l1 = activeController->l1();
    r1 = activeController->r1();
    l2 = activeController->l2();
    r2 = activeController->r2();
    share = activeController->miscBack();
    options = activeController->miscHome();
    l3 = activeController->thumbL();
    r3 = activeController->thumbR();
    ps = activeController->miscSystem();
  }

  String json = "{";
  json += "\"raw_lx\":" + String(rawLX);
  json += ",\"raw_ly\":" + String(rawLY);
  json += ",\"raw_rx\":" + String(rawRX);
  json += ",\"raw_ry\":" + String(rawRY);
  json += ",\"centered_lx\":" + String(centeredLX);
  json += ",\"centered_ly\":" + String(centeredLY);
  json += ",\"centered_rx\":" + String(centeredRX);
  json += ",\"centered_ry\":" + String(centeredRY);
  json += ",\"raw_throttle\":" + String(rawThrottle);
  json += ",\"raw_brake\":" + String(rawBrake);
  json += ",\"trigger_difference\":" + String(triggerDifference);
  json += ",\"dpad_x\":" + String(dpadX);
  json += ",\"dpad_y\":" + String(dpadY);
  json += ",\"buttons\":{";
  json += "\"up\":" + boolJson(up);
  json += ",\"down\":" + boolJson(down);
  json += ",\"left\":" + boolJson(left);
  json += ",\"right\":" + boolJson(right);
  json += ",\"square\":" + boolJson(square);
  json += ",\"cross\":" + boolJson(cross);
  json += ",\"circle\":" + boolJson(circle);
  json += ",\"triangle\":" + boolJson(triangle);
  json += ",\"l1\":" + boolJson(l1);
  json += ",\"r1\":" + boolJson(r1);
  json += ",\"l2\":" + boolJson(l2);
  json += ",\"r2\":" + boolJson(r2);
  json += ",\"share\":" + boolJson(share);
  json += ",\"options\":" + boolJson(options);
  json += ",\"l3\":" + boolJson(l3);
  json += ",\"r3\":" + boolJson(r3);
  json += ",\"ps\":" + boolJson(ps);
  json += ",\"touchpad\":" + boolJson(touchpad);
  json += "}}";
  return json;
}

int readAxisValue(uint8_t source) {
  if (activeController == nullptr || !activeController->isConnected()) {
    return 0;
  }
  switch (source) {
    case AXIS_LX: return activeController->axisX() - controllerSettings.axisCenterLX;
    case AXIS_LY: return -activeController->axisY() - controllerSettings.axisCenterLY;
    case AXIS_RX: return activeController->axisRX() - controllerSettings.axisCenterRX;
    case AXIS_RY: return -activeController->axisRY() - controllerSettings.axisCenterRY;
    case AXIS_DPAD_X: return dpadValueX(activeController);
    case AXIS_DPAD_Y: return dpadValueY(activeController);
    case AXIS_TRIGGERS: return activeController->throttle() - activeController->brake();
    default: return 0;
  }
}

int jointIndexByName(const String& name) {
  for (int i = 0; i < JOINT_COUNT; ++i) {
    if (name.equalsIgnoreCase(joints[i].name)) {
      return i;
    }
  }
  return -1;
}

String jointJson(const Joint& joint) {
  String json = "{";
  json += "\"name\":" + jsonQuote(joint.name);
  json += ",\"label\":" + jsonQuote(joint.label);
  json += ",\"coordinate_space\":" + jsonQuote(jointCoordinateSpaceName(joint));
  json += ",\"pin\":" + String(joint.pin);
  json += ",\"motor_type\":" + jsonQuote(motorTypeName(joint.motorType));
  json += ",\"min_angle\":" + String(joint.minAngle);
  json += ",\"max_angle\":" + String(joint.maxAngle);
  json += ",\"home_angle\":" + String(joint.homeAngle);
  json += ",\"step\":" + String(joint.step);
  json += ",\"pulse_min\":" + String(joint.pulseMin);
  json += ",\"pulse_max\":" + String(joint.pulseMax);
  json += ",\"physical_min_angle\":" + String(joint.physicalMinAngle);
  json += ",\"physical_max_angle\":" + String(joint.physicalMaxAngle);
  json += ",\"physical_angle\":" + String(logicalToPhysicalAngle(joint, joint.position), 2);
  json += ",\"physical_home_angle\":" + String(logicalToPhysicalAngle(joint, joint.homeAngle), 2);
  json += ",\"neutral_output\":" + String(joint.neutralOutput);
  json += ",\"stop_deadband\":" + String(joint.stopDeadband);
  json += ",\"max_speed_scale\":" + String(joint.maxSpeedScale);
  json += ",\"invert\":" + boolJson(joint.inverted);
  json += ",\"position\":" + String(joint.position);
  json += ",\"startup_target\":" + String(joint.startupTarget);
  json += ",\"raw_output\":" + String(joint.rawOutput);
  json += ",\"stored_min_angle\":" + String(joint.storedMinAngle);
  json += ",\"stored_max_angle\":" + String(joint.storedMaxAngle);
  json += ",\"stored_home_angle\":" + String(joint.storedHomeAngle);
  json += ",\"stored_physical_min_angle\":" + String(joint.storedPhysicalMinAngle);
  json += ",\"stored_physical_max_angle\":" + String(joint.storedPhysicalMaxAngle);
  json += ",\"stored_position\":" + String(joint.storedPosition);
  json += ",\"attached\":" + boolJson(joint.attached);
  json += ",\"velocity\":" + String(joint.velocity);
  json += ",\"control_mode\":" + jsonQuote(controlModeName(joint.controlMode));
  json += ",\"axis_source\":" + jsonQuote(axisSourceName(joint.axisSource));
  json += ",\"positive_button\":" + jsonQuote(buttonSourceName(joint.positiveButton));
  json += ",\"negative_button\":" + jsonQuote(buttonSourceName(joint.negativeButton));
  json += ",\"input_invert\":" + boolJson(joint.inputInvert);
  json += "}";
  return json;
}

String controllerJson() {
  int batteryRaw = connectedInfo.connected ? activeController->battery() : 0;
  int batteryPercent = 0;
  if (batteryRaw == 255) {
    batteryPercent = 100;
  } else if (batteryRaw > 1) {
    batteryPercent = map(batteryRaw, 2, 254, 1, 99);
  }
  String json = "{";
  json += "\"enabled\":" + boolJson(controllerSettings.enabled);
  json += ",\"allow_new_connections\":" + boolJson(controllerSettings.allowNewConnections);
  json += ",\"state\":" + jsonQuote(controllerWorkflowStateName(controllerWorkflowState));
  json += ",\"status_text\":" + jsonQuote(controllerStatusText);
  json += ",\"last_error\":" + jsonQuote(lastControllerError);
  json += ",\"scanning_in_progress\":" + boolJson(controllerWorkflowState == CTL_SCANNING || controllerWorkflowState == CTL_PAIRING);
  json += ",\"reconnect_in_progress\":" + boolJson(controllerWorkflowState == CTL_RECONNECTING);
  json += ",\"connected\":" + boolJson(connectedInfo.connected);
  json += ",\"esp32_bt_mac\":" + jsonQuote(localBluetoothMac);
  json += ",\"controller_name\":" + jsonQuote(connectedInfo.modelName);
  json += ",\"controller_type\":" + jsonQuote(connectedInfo.typeName);
  json += ",\"controller_bt_addr\":" + jsonQuote(connectedInfo.btAddress);
  json += ",\"remembered_name\":" + jsonQuote(rememberedController.name);
  json += ",\"remembered_type\":" + jsonQuote(rememberedController.type);
  json += ",\"remembered_bt_addr\":" + jsonQuote(rememberedController.btAddress);
  json += ",\"led_r\":" + String(controllerSettings.ledR);
  json += ",\"led_g\":" + String(controllerSettings.ledG);
  json += ",\"led_b\":" + String(controllerSettings.ledB);
  json += ",\"rumble_force\":" + String(controllerSettings.rumbleForce);
  json += ",\"rumble_duration\":" + String(controllerSettings.rumbleDuration);
  json += ",\"axis_deadzone\":" + String(controllerSettings.axisDeadzone);
  json += ",\"axis_center_lx\":" + String(controllerSettings.axisCenterLX);
  json += ",\"axis_center_ly\":" + String(controllerSettings.axisCenterLY);
  json += ",\"axis_center_rx\":" + String(controllerSettings.axisCenterRX);
  json += ",\"axis_center_ry\":" + String(controllerSettings.axisCenterRY);
  json += ",\"home_all_button\":" + jsonQuote(buttonSourceName(controllerSettings.homeAllButton));
  json += ",\"battery\":" + String(batteryPercent);
  json += ",\"battery_raw\":" + String(batteryRaw);
  json += ",\"inputs\":" + controllerInputsJson();
  json += "}";
  return json;
}

String wifiJson() {
  wl_status_t staStatus = WiFi.status();
  String json = "{";
  json += "\"hostname\":" + jsonQuote(wifiSettings.hostname);
  json += ",\"mdns_hostname\":" + jsonQuote(mdnsHostname.length() > 0 ? mdnsHostname + ".local" : "");
  json += ",\"mdns_active\":" + boolJson(mdnsActive);
  json += ",\"ap_active\":" + boolJson(WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA);
  json += ",\"ap_ssid\":" + jsonQuote(wifiSettings.apSsid);
  json += ",\"ap_ip\":" + jsonQuote(WiFi.softAPIP().toString());
  json += ",\"sta_ssid\":" + jsonQuote(wifiSettings.staSsid);
  json += ",\"sta_connected\":" + boolJson(staStatus == WL_CONNECTED);
  json += ",\"sta_ip\":" + jsonQuote(staStatus == WL_CONNECTED ? WiFi.localIP().toString() : "");
  json += ",\"sta_status\":" + jsonQuote(wifiStatusString(staStatus));
  json += ",\"last_result\":" + jsonQuote(lastWifiResult);
  json += ",\"last_failure\":" + jsonQuote(lastWifiFailure);
  json += "}";
  return json;
}

String identifyJson() {
  String json = "{\"ok\":true";
  json += ",\"device_type\":" + jsonQuote(DEVICE_TYPE);
  json += ",\"device_model\":" + jsonQuote(DEVICE_MODEL);
  json += ",\"firmware_version\":" + jsonQuote(FIRMWARE_VERSION);
  json += ",\"hostname\":" + jsonQuote(wifiSettings.hostname);
  json += ",\"mdns_hostname\":" + jsonQuote(mdnsHostname.length() > 0 ? mdnsHostname + ".local" : "");
  json += ",\"ip_address\":" + jsonQuote(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString());
  json += ",\"ap_ip\":" + jsonQuote(WiFi.softAPIP().toString());
  json += ",\"sta_connected\":" + boolJson(WiFi.status() == WL_CONNECTED);
  json += ",\"mac\":" + jsonQuote(WiFi.macAddress());
  json += "}";
  return json;
}

String fullStateJson() {
  String json = "{\"ok\":true";
  json += ",\"device_type\":" + jsonQuote(DEVICE_TYPE);
  json += ",\"device_model\":" + jsonQuote(DEVICE_MODEL);
  json += ",\"firmware_version\":" + jsonQuote(FIRMWARE_VERSION);
  json += ",\"wifi\":" + wifiJson();
  json += ",\"ps4\":" + controllerJson();
  json += ",\"biped\":" + bipedJson();
  json += ",\"joints\":[";
  for (int i = 0; i < JOINT_COUNT; ++i) {
    if (i > 0) {
      json += ",";
    }
    json += jointJson(joints[i]);
  }
  json += "]}";
  return json;
}

bool setJointField(Joint& joint, const String& field, int value) {
  if (field.equalsIgnoreCase("pin")) {
    joint.pin = static_cast<uint8_t>(constrain(value, 0, 255));
    attachJoint(joint, true);
  } else if (field.equalsIgnoreCase("motor_type")) {
    joint.motorType = constrain(value, MOTOR_POSITIONAL, MOTOR_CONTINUOUS);
    if (joint.motorType == MOTOR_CONTINUOUS) {
      joint.minAngle = -100;
      joint.maxAngle = 100;
      joint.homeAngle = 0;
      joint.physicalMinAngle = -100;
      joint.physicalMaxAngle = 100;
      joint.position = 0;
    } else {
      joint.minAngle = 0;
      joint.maxAngle = 180;
      joint.homeAngle = 90;
      joint.physicalMinAngle = 0;
      joint.physicalMaxAngle = 180;
      joint.position = 90;
    }
  } else if (field.equalsIgnoreCase("min")) {
    joint.minAngle = constrain(value, joint.motorType == MOTOR_CONTINUOUS ? -100 : 0, joint.motorType == MOTOR_CONTINUOUS ? 100 : 180);
    if (joint.minAngle > joint.maxAngle) {
      joint.maxAngle = joint.minAngle;
    }
  } else if (field.equalsIgnoreCase("max")) {
    joint.maxAngle = constrain(value, joint.motorType == MOTOR_CONTINUOUS ? -100 : 0, joint.motorType == MOTOR_CONTINUOUS ? 100 : 180);
    if (joint.maxAngle < joint.minAngle) {
      joint.minAngle = joint.maxAngle;
    }
  } else if (field.equalsIgnoreCase("home")) {
    joint.homeAngle = value;
  } else if (field.equalsIgnoreCase("step")) {
    joint.step = max(1, value);
  } else if (field.equalsIgnoreCase("pulse_min")) {
    joint.pulseMin = max(100, value);
    joint.pulseMax = max(joint.pulseMin + 100, joint.pulseMax);
    attachJoint(joint, true);
  } else if (field.equalsIgnoreCase("pulse_max")) {
    joint.pulseMax = max(value, joint.pulseMin + 100);
    attachJoint(joint, true);
  } else if (field.equalsIgnoreCase("physical_min_angle")) {
    joint.physicalMinAngle = value;
  } else if (field.equalsIgnoreCase("physical_max_angle")) {
    joint.physicalMaxAngle = value;
  } else if (field.equalsIgnoreCase("neutral_output")) {
    joint.neutralOutput = constrain(value, 0, 180);
  } else if (field.equalsIgnoreCase("stop_deadband")) {
    joint.stopDeadband = constrain(value, 0, 20);
  } else if (field.equalsIgnoreCase("max_speed_scale")) {
    joint.maxSpeedScale = constrain(value, 1, 100);
  } else if (field.equalsIgnoreCase("invert")) {
    joint.inverted = value != 0;
  } else if (field.equalsIgnoreCase("position")) {
    joint.position = value;
  } else {
    return false;
  }
  normalizeJointConfig(joint);
  writeJoint(joint, joint.position);
  saveJointSettings(joint);
  return true;
}

void updateControllerInputs() {
  lockState();
  if (!controllerSettings.enabled || activeController == nullptr || !activeController->isConnected()) {
    for (int i = 0; i < JOINT_COUNT; ++i) {
      joints[i].velocity = 0;
      if (joints[i].motorType == MOTOR_CONTINUOUS && !joints[i].manualContinuousControl && joints[i].position != joints[i].homeAngle) {
        writeJoint(joints[i], 0, joints[i].attached);
      }
    }
    controllerSettings.homeAllLatched = false;
    unlockState();
    return;
  }

  connectedInfo.battery = activeController->battery();
  bool homePressed = controllerSettings.homeAllButton != BTN_NONE && buttonPressed(controllerSettings.homeAllButton);
  if (homePressed && !controllerSettings.homeAllLatched) {
    for (int i = 0; i < JOINT_COUNT; ++i) {
      joints[i].velocity = 0;
      int homeTarget = joints[i].motorType == MOTOR_CONTINUOUS ? 0 : joints[i].homeAngle;
      writeJoint(joints[i], homeTarget);
    }
    controllerSettings.homeAllLatched = true;
    unlockState();
    return;
  }
  controllerSettings.homeAllLatched = homePressed;

  for (int i = 0; i < JOINT_COUNT; ++i) {
    Joint& joint = joints[i];
    joint.velocity = 0;

    if (joint.controlMode == CONTROL_AXIS) {
      int axisValue = readAxisValue(joint.axisSource);
      int maxMagnitude = (joint.axisSource == AXIS_TRIGGERS) ? TRIGGER_MAX_MAGNITUDE : AXIS_MAX_MAGNITUDE;
      if (joint.motorType == MOTOR_CONTINUOUS) {
        joint.manualContinuousControl = false;
        writeJoint(joint, axisToCommand(joint, axisValue, maxMagnitude, joint.inputInvert), joint.attached);
      } else {
        joint.velocity = getVelocity(axisValue, maxMagnitude, joint.inputInvert);
      }
    } else if (joint.controlMode == CONTROL_BUTTONS) {
      int direction = 0;
      if (buttonPressed(joint.positiveButton)) direction += 1;
      if (buttonPressed(joint.negativeButton)) direction -= 1;
      int stepAmount = max(1, joint.step);
      if (joint.motorType == MOTOR_CONTINUOUS) {
        int signedDirection = joint.inputInvert ? -direction : direction;
        int requestedSpeed = signedDirection * stepAmount * 10;
        if (direction == 0) {
          requestedSpeed = 0;
        }
        joint.manualContinuousControl = false;
        writeJoint(joint, requestedSpeed, joint.attached);
      } else {
        joint.velocity = joint.inputInvert ? (-direction * stepAmount) : (direction * stepAmount);
      }
    } else if (joint.motorType == MOTOR_CONTINUOUS && !joint.manualContinuousControl && joint.position != joint.homeAngle) {
      writeJoint(joint, 0, joint.attached);
    }
  }
  unlockState();
}

void applyVelocityMotion() {
  lockState();
  for (int i = 0; i < JOINT_COUNT; ++i) {
    if (joints[i].velocity == 0) {
      continue;
    }
    int requested = joints[i].position + joints[i].velocity;
    writeJoint(joints[i], requested, joints[i].attached);
  }
  unlockState();
}

void handleState() {
  lockState();
  server.send(200, "application/json", fullStateJson());
  unlockState();
}

void handleIdentify() {
  server.send(200, "application/json", identifyJson());
}

void handleSystem() {
  String cmd = server.arg("cmd");
  if (cmd == "save") {
    saveAllSettings();
    saveAllBipedPoseSettings();
    sendOk();
  } else if (cmd == "load") {
    loadAllSettings();
    loadBipedPoseSettings();
    initializeJointsForStartup();
    BP32.enableNewBluetoothConnections(controllerSettings.allowNewConnections);
    startWifi();
    applyControllerFeedback();
    sendOk();
  } else if (cmd == "home_all") {
    for (int i = 0; i < JOINT_COUNT; ++i) {
      if (joints[i].motorType == MOTOR_CONTINUOUS) {
        joints[i].manualContinuousControl = false;
        writeJoint(joints[i], 0);
        joints[i].startupTarget = 0;
      } else {
        writeJoint(joints[i], joints[i].homeAngle);
        joints[i].startupTarget = joints[i].homeAngle;
      }
    }
    sendOk();
  } else if (cmd == "reboot") {
    rebootRequested = true;
    sendOk();
  } else {
    sendError("unknown_system_command");
  }
}

void handleJoint() {
  int index = jointIndexByName(server.arg("joint"));
  if (index < 0) {
    sendError("unknown_joint");
    return;
  }
  Joint& joint = joints[index];
  String cmd = server.arg("cmd");
  if (cmd == "move") {
    if (joint.motorType == MOTOR_CONTINUOUS) {
      joint.manualContinuousControl = true;
    }
    writeJoint(joint, server.arg("value").toInt());
    sendOk(jointJson(joint));
  } else if (cmd == "nudge") {
    if (joint.motorType == MOTOR_CONTINUOUS) {
      joint.manualContinuousControl = true;
    }
    writeJoint(joint, joint.position + server.arg("value").toInt());
    sendOk(jointJson(joint));
  } else if (cmd == "home") {
    int homeTarget = joint.motorType == MOTOR_CONTINUOUS ? 0 : joint.homeAngle;
    joint.manualContinuousControl = joint.motorType == MOTOR_CONTINUOUS;
    writeJoint(joint, homeTarget);
    joint.startupTarget = homeTarget;
    sendOk(jointJson(joint));
  } else if (cmd == "attach") {
    attachJoint(joint, true);
    writeJoint(joint, joint.position);
    sendOk(jointJson(joint));
  } else if (cmd == "detach") {
    detachJoint(joint);
    sendOk(jointJson(joint));
  } else if (cmd == "apply") {
    setJointField(joint, "pin", server.arg("pin").toInt());
    setJointField(joint, "motor_type", server.arg("motor_type").toInt());
    setJointField(joint, "min", server.arg("min").toInt());
    setJointField(joint, "max", server.arg("max").toInt());
    setJointField(joint, "home", server.arg("home").toInt());
    setJointField(joint, "step", server.arg("step").toInt());
    setJointField(joint, "pulse_min", server.arg("pulse_min").toInt());
    setJointField(joint, "pulse_max", server.arg("pulse_max").toInt());
    if (server.hasArg("physical_min_angle")) {
      setJointField(joint, "physical_min_angle", server.arg("physical_min_angle").toInt());
    }
    if (server.hasArg("physical_max_angle")) {
      setJointField(joint, "physical_max_angle", server.arg("physical_max_angle").toInt());
    }
    setJointField(joint, "neutral_output", server.arg("neutral_output").toInt());
    setJointField(joint, "stop_deadband", server.arg("stop_deadband").toInt());
    setJointField(joint, "max_speed_scale", server.arg("max_speed_scale").toInt());
    setJointField(joint, "invert", server.arg("invert").toInt());
    joint.controlMode = constrain(server.arg("control_mode").toInt(), CONTROL_NONE, CONTROL_BUTTONS);
    joint.axisSource = constrain(server.arg("axis_source").toInt(), AXIS_NONE, AXIS_TRIGGERS);
    joint.positiveButton = constrain(server.arg("positive_button").toInt(), BTN_NONE, BTN_TOUCHPAD);
    joint.negativeButton = constrain(server.arg("negative_button").toInt(), BTN_NONE, BTN_TOUCHPAD);
    joint.inputInvert = server.arg("input_invert").toInt() != 0;
    saveJointSettings(joint);
    sendOk(jointJson(joint));
  } else {
    sendError("unknown_joint_command");
  }
}

void handleBiped() {
  String cmd = server.arg("cmd");
  long requestedDurationMs =
      server.hasArg("duration_ms") ? server.arg("duration_ms").toInt() : static_cast<long>(BIPED_DEFAULT_POSE_DURATION_MS);
  int durationMs = requestedDurationMs < 0L ? 0 : static_cast<int>(requestedDurationMs);
  long requestedInterpolationSteps =
      server.hasArg("interp_steps") ? server.arg("interp_steps").toInt() : static_cast<long>(BIPED_DEFAULT_INTERP_STEPS);
  int interpolationSteps = requestedInterpolationSteps < 1L ? 1 : static_cast<int>(requestedInterpolationSteps);
  long requestedHoldMs =
      server.hasArg("hold_ms") ? server.arg("hold_ms").toInt() : static_cast<long>(BIPED_DEFAULT_HOLD_MS);
  int holdMs = requestedHoldMs < 0L ? 0 : static_cast<int>(requestedHoldMs);

  if (cmd == "state" || cmd == "list_poses") {
    sendOk(bipedJson());
    return;
  }

  if (cmd == "stand") {
    executeBipedPose(bipedPoseIndexByName("stand"), durationMs, interpolationSteps, holdMs);
    sendOk(fullStateJson());
    return;
  }

  if (cmd == "initialize") {
    executeBipedPose(bipedPoseIndexByName("stand"), durationMs, interpolationSteps, holdMs);
    sendOk(fullStateJson());
    return;
  }

  if (cmd == "run_pose") {
    int poseIndex = bipedPoseIndexByName(server.arg("name"));
    if (poseIndex < 0) {
      sendError("unknown_biped_pose");
      return;
    }
    executeBipedPose(poseIndex, durationMs, interpolationSteps, holdMs);
    sendOk(fullStateJson());
    return;
  }

  if (cmd == "save_pose") {
    int poseIndex = bipedPoseIndexByName(server.arg("name"));
    if (poseIndex < 0) {
      sendError("unknown_biped_pose");
      return;
    }
    if (server.arg("current") == "1") {
      captureCurrentBipedPose(poseIndex);
    } else {
      int values[JOINT_COUNT] = {joints[BASE].position, joints[SHOULDER].position, joints[ELBOW].position,
                                 joints[WRIST_PITCH].position, joints[WRIST_ROTATE].position, joints[GRIPPER].position};
      if (server.hasArg("base")) values[BASE] = server.arg("base").toInt();
      if (server.hasArg("shoulder")) values[SHOULDER] = server.arg("shoulder").toInt();
      if (server.hasArg("elbow")) values[ELBOW] = server.arg("elbow").toInt();
      if (server.hasArg("wrist_pitch")) values[WRIST_PITCH] = server.arg("wrist_pitch").toInt();
      if (server.hasArg("wrist_rotate")) values[WRIST_ROTATE] = server.arg("wrist_rotate").toInt();
      if (server.hasArg("gripper")) values[GRIPPER] = server.arg("gripper").toInt();
      setExplicitBipedPose(poseIndex, values);
    }
    sendOk(bipedJson());
    return;
  }

  if (cmd == "play_sequence" || cmd == "walk_forward" || cmd == "walk_backward") {
    long requestedRepeatCount = server.hasArg("repeat") ? server.arg("repeat").toInt() : 1L;
    int repeatCount = requestedRepeatCount < 1L ? 1 : static_cast<int>(requestedRepeatCount);
    String names = server.arg("names");
    if (cmd == "walk_forward" && names.length() == 0) {
      names = "stand,left_forward,stand,right_forward,stand";
    } else if (cmd == "walk_backward" && names.length() == 0) {
      names = "stand,right_forward,stand,left_forward,stand";
    }
    String error;
    if (!playBipedSequence(names, durationMs, interpolationSteps, holdMs, repeatCount, error)) {
      sendError(error);
      return;
    }
    sendOk(fullStateJson());
    return;
  }

  sendError("unknown_biped_command");
}

void handlePs4() {
  String cmd = server.arg("cmd");
  if (cmd == "enable") {
    controllerSettings.enabled = server.arg("value").toInt() != 0;
    clearControllerError();
    if (!controllerSettings.enabled) {
      setControllerWorkflowState(CTL_DISABLED, "controller input disabled");
    } else if (rememberedController.btAddress.length() > 0 && !controllerSettings.allowNewConnections) {
      controllerReconnectStartedMs = millis();
      setControllerWorkflowState(CTL_RECONNECTING, "waiting for remembered controller to reconnect");
    } else {
      refreshControllerWorkflowState();
    }
    saveControllerSettings();
    sendOk(controllerJson());
  } else if (cmd == "pair_mode") {
    controllerSettings.allowNewConnections = server.arg("value").toInt() != 0;
    BP32.enableNewBluetoothConnections(controllerSettings.allowNewConnections);
    clearControllerError();
    controllerPairingStartedMs = controllerSettings.allowNewConnections ? millis() : 0;
    controllerReconnectStartedMs = (!controllerSettings.allowNewConnections && rememberedController.btAddress.length() > 0) ? millis() : 0;
    refreshControllerWorkflowState();
    saveControllerSettings();
    sendOk(controllerJson());
  } else if (cmd == "remember_current") {
    if (activeController == nullptr || !activeController->isConnected()) {
      sendError("no_controller_connected");
      return;
    }
    rememberedController.name = connectedInfo.modelName;
    rememberedController.type = connectedInfo.typeName;
    rememberedController.btAddress = connectedInfo.btAddress;
    saveRememberedController();
    clearControllerError();
    refreshControllerWorkflowState();
    sendOk(controllerJson());
  } else if (cmd == "forget_target") {
    bool hadRemembered = rememberedController.btAddress.length() > 0;
    forgetRememberedController();
    clearControllerError();
    if (!connectedInfo.connected) {
      setControllerWorkflowState(CTL_IDLE, "ready to pair a controller");
    } else {
      refreshControllerWorkflowState();
    }
    if (!hadRemembered) {
      lastControllerError = "no_remembered_controller";
    }
    sendOk(controllerJson());
  } else if (cmd == "forget") {
    BP32.forgetBluetoothKeys();
    forgetRememberedController();
    clearControllerError();
    setControllerWorkflowState(CTL_IDLE, "bluetooth bond data cleared");
    sendOk(controllerJson());
  } else if (cmd == "disconnect") {
    if (activeController != nullptr && activeController->isConnected()) {
      activeController->disconnect();
    }
    refreshControllerWorkflowState();
    sendOk(controllerJson());
  } else if (cmd == "led") {
    controllerSettings.ledR = static_cast<uint8_t>(constrain(server.arg("r").toInt(), 0, 255));
    controllerSettings.ledG = static_cast<uint8_t>(constrain(server.arg("g").toInt(), 0, 255));
    controllerSettings.ledB = static_cast<uint8_t>(constrain(server.arg("b").toInt(), 0, 255));
    saveControllerSettings();
    applyControllerFeedback();
    sendOk(controllerJson());
  } else if (cmd == "rumble") {
    controllerSettings.rumbleForce = static_cast<uint8_t>(constrain(server.arg("force").toInt(), 0, 255));
    controllerSettings.rumbleDuration = static_cast<uint8_t>(constrain(server.arg("duration").toInt(), 0, 255));
    saveControllerSettings();
    applyControllerFeedback();
    sendOk(controllerJson());
  } else if (cmd == "deadzone") {
    controllerSettings.axisDeadzone = constrain(server.arg("value").toInt(), 0, 200);
    saveControllerSettings();
    sendOk(controllerJson());
  } else if (cmd == "calibrate_center") {
    if (activeController == nullptr || !activeController->isConnected()) {
      setControllerError("no_controller_connected");
      sendError("no_controller_connected");
      return;
    }
    controllerSettings.axisCenterLX = activeController->axisX();
    controllerSettings.axisCenterLY = -activeController->axisY();
    controllerSettings.axisCenterRX = activeController->axisRX();
    controllerSettings.axisCenterRY = -activeController->axisRY();
    saveControllerSettings();
    sendOk(controllerJson());
  } else if (cmd == "home_button") {
    controllerSettings.homeAllButton = constrain(server.arg("value").toInt(), BTN_NONE, BTN_TOUCHPAD);
    saveControllerSettings();
    sendOk(controllerJson());
  } else {
    sendError("unknown_ps4_command");
  }
}

void handleWifi() {
  String cmd = server.arg("cmd");
  if (cmd == "set") {
    String hostname = server.arg("hostname");
    String apSsid = server.arg("ap_ssid");
    String apPassword = server.arg("ap_password");
    if (hostname.length() > 0) {
      wifiSettings.hostname = hostname;
    }
    if (apSsid.length() > 0) {
      wifiSettings.apSsid = apSsid;
    }
    if (server.hasArg("ap_password") && apPassword.length() >= 8) {
      wifiSettings.apPassword = apPassword;
    }
    if (server.hasArg("sta_ssid")) {
      wifiSettings.staSsid = server.arg("sta_ssid");
    }
    if (server.hasArg("sta_password")) {
      wifiSettings.staPassword = server.arg("sta_password");
    }
    saveWifiSettings();
    wifiReconnectRequested = true;
    sendOk(wifiJson());
  } else if (cmd == "reconnect") {
    wifiReconnectRequested = true;
    sendOk(wifiJson());
  } else {
    sendError("unknown_wifi_command");
  }
}

void handleRoot() {
  String page;
  page.reserve(2600);
  page += "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  page += "<title>Robot Arm Setup</title>";
  page += "<style>body{font-family:Arial,sans-serif;max-width:760px;margin:24px auto;padding:0 16px;background:#111;color:#eee}";
  page += "input{width:100%;padding:12px;margin:8px 0 16px;border:1px solid #444;border-radius:8px;background:#1c1c1c;color:#fff}";
  page += "button{padding:12px 16px;border:0;border-radius:8px;background:#2d6cdf;color:#fff;font-weight:600;width:100%}";
  page += ".card{background:#1a1a1a;padding:20px;border-radius:14px;margin-bottom:18px}.muted{color:#bbb}.ok{color:#7bd88f}</style></head><body>";
  page += "<div class='card'><h2>Robot Arm Setup</h2>";
  page += "<p class='muted'>Connect the ESP32 to your home Wi-Fi here. After that, use the Streamlit dashboard from any device on the same network.</p>";
  page += "<p><strong>Setup AP:</strong> " + htmlEscape(wifiSettings.apSsid) + " (" + WiFi.softAPIP().toString() + ")</p>";
  page += "<p><strong>Controller pairing:</strong> Put the DualShock 4 in pairing mode with SHARE + PS while pairing mode is enabled in the dashboard.</p>";
  page += "<p><strong>ESP32 Bluetooth MAC:</strong> " + htmlEscape(localBluetoothMac) + "</p>";
  page += "<p><strong>Station status:</strong> ";
  page += (WiFi.status() == WL_CONNECTED) ? "<span class='ok'>Connected</span>" : "Not connected";
  page += "</p>";
  if (WiFi.status() == WL_CONNECTED) {
    page += "<p><strong>Station IP:</strong> " + WiFi.localIP().toString() + "</p>";
    page += "<p><strong>mDNS:</strong> " + htmlEscape(mdnsHostname + ".local") + "</p>";
  }
  page += "</div>";
  page += "<div class='card'><form method='get' action='/setup'>";
  page += "<label>Home Wi-Fi SSID</label><input name='sta_ssid' value='" + htmlEscape(wifiSettings.staSsid) + "' placeholder='Your router SSID'>";
  page += "<label>Home Wi-Fi Password</label><input name='sta_password' type='password' placeholder='Your router password'>";
  page += "<label>Hostname</label><input name='hostname' value='" + htmlEscape(wifiSettings.hostname) + "' placeholder='biped-robot'>";
  page += "<label>Setup AP SSID</label><input name='ap_ssid' value='" + htmlEscape(wifiSettings.apSsid) + "'>";
  page += "<label>Setup AP Password</label><input name='ap_password' type='password' placeholder='At least 8 characters'>";
  page += "<button type='submit'>Save and reconnect</button></form></div>";
  page += "<div class='card'><p><strong>API:</strong> <a style='color:#8ab4ff' href='/api/state'>/api/state</a></p>";
  page += "<p><strong>Identify:</strong> <a style='color:#8ab4ff' href='/api/identify'>/api/identify</a></p>";
  page += "<p class='muted'>The dashboard also lets you enable pairing mode, inspect the connected controller, and map every motor input.</p></div>";
  page += "</body></html>";
  server.send(200, "text/html", page);
}

void handleSetupPage() {
  if (server.hasArg("hostname") && server.arg("hostname").length() > 0) {
    wifiSettings.hostname = server.arg("hostname");
  }
  if (server.hasArg("ap_ssid") && server.arg("ap_ssid").length() > 0) {
    wifiSettings.apSsid = server.arg("ap_ssid");
  }
  if (server.hasArg("ap_password") && server.arg("ap_password").length() >= 8) {
    wifiSettings.apPassword = server.arg("ap_password");
  }
  if (server.hasArg("sta_ssid")) {
    wifiSettings.staSsid = server.arg("sta_ssid");
  }
  if (server.hasArg("sta_password") && server.arg("sta_password").length() > 0) {
    wifiSettings.staPassword = server.arg("sta_password");
  }
  saveWifiSettings();
  wifiReconnectRequested = true;

  String page;
  page.reserve(950);
  page += "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  page += "<title>Saved</title><style>body{font-family:Arial,sans-serif;max-width:640px;margin:24px auto;padding:0 16px;background:#111;color:#eee}";
  page += "a{color:#8ab4ff}.card{background:#1a1a1a;padding:20px;border-radius:14px}</style></head><body><div class='card'>";
  page += "<h2>Wi-Fi settings saved</h2><p>The ESP32 is reconnecting now.</p>";
  page += "<p><strong>Setup AP:</strong> " + htmlEscape(wifiSettings.apSsid) + "</p>";
  page += "<p><strong>Home SSID:</strong> " + htmlEscape(wifiSettings.staSsid) + "</p>";
  page += "<p>After a few seconds, reopen <a href='/'>the setup page</a> or check <a href='/api/state'>/api/state</a>.</p>";
  page += "</div></body></html>";
  server.send(200, "text/html", page);
}

void configureRoutes() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/setup", HTTP_GET, handleSetupPage);
  server.on("/api/identify", HTTP_GET, handleIdentify);
  server.on("/api/state", HTTP_GET, handleState);
  server.on("/api/system", HTTP_GET, handleSystem);
  server.on("/api/joint", HTTP_GET, handleJoint);
  server.on("/api/biped", HTTP_GET, handleBiped);
  server.on("/api/ps4", HTTP_GET, handlePs4);
  server.on("/api/wifi", HTTP_GET, handleWifi);
  server.onNotFound([]() { sendError("not_found", 404); });
}
}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  stateMutex = xSemaphoreCreateRecursiveMutex();
  preferences.begin("robot-arm", false);
  loadAllSettings();
  loadBipedPoseSettings();
  initializeJointsForStartup();
  startControllerStack();
  startWifi();
  configureRoutes();
  server.begin();
  xTaskCreatePinnedToCore(controlTask, "controlTask", 8192, nullptr, 3, &controlTaskHandle, 1);

  Serial.println("=== Robot Arm Wi-Fi Control Ready ===");
  Serial.print("Connect Streamlit to: http://");
  Serial.println(WiFi.softAPIP());
  Serial.print("Bluetooth MAC: ");
  Serial.println(localBluetoothMac);
}

void loop() {
  server.handleClient();
  serviceWifiConnection();

  if (wifiReconnectRequested) {
    wifiReconnectRequested = false;
    startWifi();
  }

  if (rebootRequested) {
    delay(250);
    ESP.restart();
  }

  delay(1);
}
