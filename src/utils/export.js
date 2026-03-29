import { formatNumber } from "./math.js";

export const EXPORT_OPTIONS = [
  {
    id: "json",
    label: "JSON Snapshot",
    extension: "json",
    description: "Machine-readable snapshot of links, limits, calibration, state, and waypoints."
  },
  {
    id: "arduino-servo",
    label: "Arduino Servo",
    extension: "ino",
    description: "Blocking moveToXY servo scaffold with IK, limits, zero offsets, and direction inversion."
  },
  {
    id: "arduino-stepper",
    label: "Arduino Stepper",
    extension: "ino",
    description: "AccelStepper-based moveToXY scaffold for driver-style steppers on Arduino."
  },
  {
    id: "esp32-stepper",
    label: "ESP32 Stepper",
    extension: "ino",
    description: "ESP32-friendly motion scaffold with commandMoveToXY and loop-driven serviceMotion."
  },
  {
    id: "python",
    label: "Python Controller",
    extension: "py",
    description: "Python class with move_to, IK solving, servo conversion, and stepper target helpers."
  }
];

function boolLiteral(value) {
  return value ? "true" : "false";
}

function pythonBool(value) {
  return value ? "True" : "False";
}

function indentJson(value) {
  return JSON.stringify(value, null, 2);
}

export function createRobotExportPayload(config, robotState, waypoints) {
  return {
    generatedAt: new Date().toISOString(),
    robot: {
      links: {
        L1: config.L1,
        L2: config.L2
      },
      jointLimitsRad: {
        theta1Min: config.jointLimits.theta1Min,
        theta1Max: config.jointLimits.theta1Max,
        theta2Min: config.jointLimits.theta2Min,
        theta2Max: config.jointLimits.theta2Max
      },
      calibration: config.calibration
    },
    state: {
      mode: config.mode,
      elbowMode: config.elbowMode,
      target: {
        x: config.targetX,
        y: config.targetY
      },
      jointAnglesRad: {
        theta1: robotState.activeTheta1,
        theta2: robotState.activeTheta2
      },
      servoCommandsDeg: robotState.servoCommands,
      endEffector: robotState.fk.endEffector,
      reachable: robotState.ik.reachable,
      withinJointLimits: robotState.theta1WithinLimits && robotState.theta2WithinLimits,
      nearSingularity: robotState.nearSingularity
    },
    trajectory: {
      waypoints
    }
  };
}

function sharedCppConstants(config, robotState) {
  return `constexpr float L1 = ${formatNumber(config.L1, 6)}f;
constexpr float L2 = ${formatNumber(config.L2, 6)}f;

constexpr float THETA1_MIN_RAD = ${formatNumber(config.jointLimits.theta1Min, 6)}f;
constexpr float THETA1_MAX_RAD = ${formatNumber(config.jointLimits.theta1Max, 6)}f;
constexpr float THETA2_MIN_RAD = ${formatNumber(config.jointLimits.theta2Min, 6)}f;
constexpr float THETA2_MAX_RAD = ${formatNumber(config.jointLimits.theta2Max, 6)}f;

constexpr float JOINT1_ZERO_OFFSET_DEG = ${formatNumber(config.calibration.joint1OffsetDeg, 4)}f;
constexpr float JOINT2_ZERO_OFFSET_DEG = ${formatNumber(config.calibration.joint2OffsetDeg, 4)}f;
constexpr bool JOINT1_INVERTED = ${boolLiteral(config.calibration.joint1Invert)};
constexpr bool JOINT2_INVERTED = ${boolLiteral(config.calibration.joint2Invert)};

constexpr float HOME_TARGET_X = ${formatNumber(config.targetX, 6)}f;
constexpr float HOME_TARGET_Y = ${formatNumber(config.targetY, 6)}f;
constexpr float HOME_THETA1_RAD = ${formatNumber(robotState.activeTheta1, 6)}f;
constexpr float HOME_THETA2_RAD = ${formatNumber(robotState.activeTheta2, 6)}f;
constexpr bool DEFAULT_ELBOW_UP = ${boolLiteral(config.elbowMode === "up")};`;
}

function sharedCppHelpers() {
  return `struct PoseRad {
  float theta1;
  float theta2;
};

float clampf(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}

bool withinJointLimits(const PoseRad& pose) {
  return pose.theta1 >= THETA1_MIN_RAD && pose.theta1 <= THETA1_MAX_RAD &&
         pose.theta2 >= THETA2_MIN_RAD && pose.theta2 <= THETA2_MAX_RAD;
}

bool solveIK(float x, float y, bool elbowUp, PoseRad& pose) {
  const float radius2 = x * x + y * y;
  const float rawCosTheta2 = (radius2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  if (rawCosTheta2 < -1.0f || rawCosTheta2 > 1.0f) {
    return false;
  }

  const float cosTheta2 = clampf(rawCosTheta2, -1.0f, 1.0f);
  const float sinMagnitude = sqrtf(fmaxf(0.0f, 1.0f - cosTheta2 * cosTheta2));
  const float sinTheta2 = elbowUp ? sinMagnitude : -sinMagnitude;

  pose.theta2 = atan2f(sinTheta2, cosTheta2);
  pose.theta1 = atan2f(y, x) - atan2f(L2 * sinTheta2, L1 + L2 * cosTheta2);
  return withinJointLimits(pose);
}`;
}

function createArduinoServoController(config, robotState) {
  return `#include <Arduino.h>
#include <Servo.h>
#include <math.h>

namespace PlanarArmServo {
${sharedCppConstants(config, robotState)}

constexpr int JOINT1_SERVO_PIN = 9;
constexpr int JOINT2_SERVO_PIN = 10;

Servo joint1Servo;
Servo joint2Servo;

${sharedCppHelpers()}

float simToServoDeg(float thetaRad, float offsetDeg, bool inverted) {
  const float thetaDeg = thetaRad * 180.0f / PI;
  return offsetDeg + (inverted ? -thetaDeg : thetaDeg);
}

bool writeJointPoseRad(const PoseRad& pose) {
  if (!withinJointLimits(pose)) {
    return false;
  }

  const float servo1Deg = simToServoDeg(pose.theta1, JOINT1_ZERO_OFFSET_DEG, JOINT1_INVERTED);
  const float servo2Deg = simToServoDeg(pose.theta2, JOINT2_ZERO_OFFSET_DEG, JOINT2_INVERTED);

  joint1Servo.write(constrain(servo1Deg, 0.0f, 180.0f));
  joint2Servo.write(constrain(servo2Deg, 0.0f, 180.0f));
  return true;
}

bool moveToJointPose(float theta1Rad, float theta2Rad) {
  PoseRad pose = {theta1Rad, theta2Rad};
  return writeJointPoseRad(pose);
}

bool moveToXY(float x, float y, bool elbowUp = DEFAULT_ELBOW_UP, uint16_t settleMs = 400) {
  PoseRad pose;
  if (!solveIK(x, y, elbowUp, pose)) {
    return false;
  }

  const bool accepted = writeJointPoseRad(pose);
  if (accepted) {
    delay(settleMs);
  }
  return accepted;
}

void attachActuators() {
  joint1Servo.attach(JOINT1_SERVO_PIN);
  joint2Servo.attach(JOINT2_SERVO_PIN);
}

void moveHome() {
  moveToXY(HOME_TARGET_X, HOME_TARGET_Y, DEFAULT_ELBOW_UP);
}
}  // namespace PlanarArmServo

void setup() {
  Serial.begin(115200);
  PlanarArmServo::attachActuators();
  PlanarArmServo::moveHome();
}

void loop() {
  // Example: move to a new XY target when needed.
  // PlanarArmServo::moveToXY(0.15f, 0.20f, true);
}
`;
}

function createArduinoStepperController(config, robotState) {
  return `#include <Arduino.h>
#include <AccelStepper.h>
#include <math.h>

namespace PlanarArmStepper {
${sharedCppConstants(config, robotState)}

constexpr int JOINT1_STEP_PIN = 2;
constexpr int JOINT1_DIR_PIN = 5;
constexpr int JOINT2_STEP_PIN = 3;
constexpr int JOINT2_DIR_PIN = 6;

// Update these for your real drivetrain.
constexpr float JOINT1_STEPS_PER_RAD = (200.0f * 16.0f * 5.0f) / (2.0f * PI);
constexpr float JOINT2_STEPS_PER_RAD = (200.0f * 16.0f * 5.0f) / (2.0f * PI);

AccelStepper joint1(AccelStepper::DRIVER, JOINT1_STEP_PIN, JOINT1_DIR_PIN);
AccelStepper joint2(AccelStepper::DRIVER, JOINT2_STEP_PIN, JOINT2_DIR_PIN);

${sharedCppHelpers()}

long thetaToSteps(float thetaRad, float zeroOffsetDeg, bool inverted, float stepsPerRad) {
  const float offsetRad = zeroOffsetDeg * PI / 180.0f;
  const float signedTheta = inverted ? -thetaRad : thetaRad;
  return lroundf((signedTheta + offsetRad) * stepsPerRad);
}

bool commandJointPose(const PoseRad& pose) {
  if (!withinJointLimits(pose)) {
    return false;
  }

  joint1.moveTo(thetaToSteps(pose.theta1, JOINT1_ZERO_OFFSET_DEG, JOINT1_INVERTED, JOINT1_STEPS_PER_RAD));
  joint2.moveTo(thetaToSteps(pose.theta2, JOINT2_ZERO_OFFSET_DEG, JOINT2_INVERTED, JOINT2_STEPS_PER_RAD));
  return true;
}

bool moveToXY(float x, float y, bool elbowUp = DEFAULT_ELBOW_UP) {
  PoseRad pose;
  if (!solveIK(x, y, elbowUp, pose)) {
    return false;
  }
  return commandJointPose(pose);
}

bool runMoveToXY(float x, float y, bool elbowUp = DEFAULT_ELBOW_UP) {
  if (!moveToXY(x, y, elbowUp)) {
    return false;
  }

  while (joint1.distanceToGo() != 0 || joint2.distanceToGo() != 0) {
    joint1.run();
    joint2.run();
  }
  return true;
}

void configureMotion() {
  joint1.setMaxSpeed(2200.0f);
  joint1.setAcceleration(1600.0f);
  joint2.setMaxSpeed(2200.0f);
  joint2.setAcceleration(1600.0f);
}

void moveHome() {
  runMoveToXY(HOME_TARGET_X, HOME_TARGET_Y, DEFAULT_ELBOW_UP);
}
}  // namespace PlanarArmStepper

void setup() {
  Serial.begin(115200);
  PlanarArmStepper::configureMotion();
  PlanarArmStepper::moveHome();
}

void loop() {
  // Example: PlanarArmStepper::runMoveToXY(0.18f, 0.24f, false);
}
`;
}

function createEsp32StepperController(config, robotState) {
  return `#include <Arduino.h>
#include <AccelStepper.h>
#include <math.h>

namespace PlanarArmEsp32 {
${sharedCppConstants(config, robotState)}

constexpr int JOINT1_STEP_PIN = 18;
constexpr int JOINT1_DIR_PIN = 19;
constexpr int JOINT2_STEP_PIN = 21;
constexpr int JOINT2_DIR_PIN = 22;

constexpr float JOINT1_STEPS_PER_RAD = (200.0f * 16.0f * 5.0f) / (2.0f * PI);
constexpr float JOINT2_STEPS_PER_RAD = (200.0f * 16.0f * 5.0f) / (2.0f * PI);

AccelStepper joint1(AccelStepper::DRIVER, JOINT1_STEP_PIN, JOINT1_DIR_PIN);
AccelStepper joint2(AccelStepper::DRIVER, JOINT2_STEP_PIN, JOINT2_DIR_PIN);

${sharedCppHelpers()}

long thetaToSteps(float thetaRad, float zeroOffsetDeg, bool inverted, float stepsPerRad) {
  const float offsetRad = zeroOffsetDeg * PI / 180.0f;
  const float signedTheta = inverted ? -thetaRad : thetaRad;
  return lroundf((signedTheta + offsetRad) * stepsPerRad);
}

bool commandJointPose(const PoseRad& pose) {
  if (!withinJointLimits(pose)) {
    return false;
  }

  joint1.moveTo(thetaToSteps(pose.theta1, JOINT1_ZERO_OFFSET_DEG, JOINT1_INVERTED, JOINT1_STEPS_PER_RAD));
  joint2.moveTo(thetaToSteps(pose.theta2, JOINT2_ZERO_OFFSET_DEG, JOINT2_INVERTED, JOINT2_STEPS_PER_RAD));
  return true;
}

bool commandMoveToXY(float x, float y, bool elbowUp = DEFAULT_ELBOW_UP) {
  PoseRad pose;
  if (!solveIK(x, y, elbowUp, pose)) {
    return false;
  }
  return commandJointPose(pose);
}

void configureMotion() {
  joint1.setMaxSpeed(3600.0f);
  joint1.setAcceleration(2400.0f);
  joint2.setMaxSpeed(3600.0f);
  joint2.setAcceleration(2400.0f);
}

void serviceMotion() {
  joint1.run();
  joint2.run();
}

void commandHome() {
  commandMoveToXY(HOME_TARGET_X, HOME_TARGET_Y, DEFAULT_ELBOW_UP);
}
}  // namespace PlanarArmEsp32

void setup() {
  Serial.begin(115200);
  PlanarArmEsp32::configureMotion();
  PlanarArmEsp32::commandHome();
}

void loop() {
  PlanarArmEsp32::serviceMotion();
  // Example: if (Serial.available()) { PlanarArmEsp32::commandMoveToXY(0.22f, 0.12f, true); }
}
`;
}

function createPythonController(config, robotState, waypoints) {
  return `import math
from dataclasses import dataclass

L1 = ${formatNumber(config.L1, 8)}
L2 = ${formatNumber(config.L2, 8)}

THETA1_MIN_RAD = ${formatNumber(config.jointLimits.theta1Min, 8)}
THETA1_MAX_RAD = ${formatNumber(config.jointLimits.theta1Max, 8)}
THETA2_MIN_RAD = ${formatNumber(config.jointLimits.theta2Min, 8)}
THETA2_MAX_RAD = ${formatNumber(config.jointLimits.theta2Max, 8)}

JOINT1_ZERO_OFFSET_DEG = ${formatNumber(config.calibration.joint1OffsetDeg, 6)}
JOINT2_ZERO_OFFSET_DEG = ${formatNumber(config.calibration.joint2OffsetDeg, 6)}
JOINT1_INVERTED = ${pythonBool(config.calibration.joint1Invert)}
JOINT2_INVERTED = ${pythonBool(config.calibration.joint2Invert)}

HOME_TARGET = (${formatNumber(config.targetX, 8)}, ${formatNumber(config.targetY, 8)})
DEFAULT_ELBOW_UP = ${pythonBool(config.elbowMode === "up")}
WAYPOINTS = ${indentJson(waypoints)}


@dataclass
class PoseRad:
    theta1: float
    theta2: float


class PlanarArmController:
    def __init__(self):
        self.joint1_steps_per_rad = (200.0 * 16.0 * 5.0) / (2.0 * math.pi)
        self.joint2_steps_per_rad = (200.0 * 16.0 * 5.0) / (2.0 * math.pi)

    def within_limits(self, pose: PoseRad) -> bool:
        return (
            THETA1_MIN_RAD <= pose.theta1 <= THETA1_MAX_RAD
            and THETA2_MIN_RAD <= pose.theta2 <= THETA2_MAX_RAD
        )

    def solve_ik(self, x: float, y: float, elbow_up: bool = DEFAULT_ELBOW_UP) -> PoseRad:
        radius2 = x * x + y * y
        raw_cos_theta2 = (radius2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
        if raw_cos_theta2 < -1.0 or raw_cos_theta2 > 1.0:
            raise ValueError("Target is outside the reachable workspace")

        cos_theta2 = max(-1.0, min(1.0, raw_cos_theta2))
        sin_magnitude = math.sqrt(max(0.0, 1.0 - cos_theta2 * cos_theta2))
        sin_theta2 = sin_magnitude if elbow_up else -sin_magnitude

        theta2 = math.atan2(sin_theta2, cos_theta2)
        theta1 = math.atan2(y, x) - math.atan2(L2 * sin_theta2, L1 + L2 * cos_theta2)
        pose = PoseRad(theta1=theta1, theta2=theta2)
        if not self.within_limits(pose):
            raise ValueError("Solved pose violates configured joint limits")
        return pose

    def sim_to_servo_deg(self, theta_rad: float, zero_offset_deg: float, inverted: bool) -> float:
        theta_deg = math.degrees(theta_rad)
        return zero_offset_deg + (-theta_deg if inverted else theta_deg)

    def pose_to_servo_command(self, pose: PoseRad) -> dict:
        return {
            "joint1_servo_deg": self.sim_to_servo_deg(pose.theta1, JOINT1_ZERO_OFFSET_DEG, JOINT1_INVERTED),
            "joint2_servo_deg": self.sim_to_servo_deg(pose.theta2, JOINT2_ZERO_OFFSET_DEG, JOINT2_INVERTED),
        }

    def pose_to_stepper_targets(self, pose: PoseRad) -> dict:
        joint1_signed = -pose.theta1 if JOINT1_INVERTED else pose.theta1
        joint2_signed = -pose.theta2 if JOINT2_INVERTED else pose.theta2
        joint1_offset_rad = math.radians(JOINT1_ZERO_OFFSET_DEG)
        joint2_offset_rad = math.radians(JOINT2_ZERO_OFFSET_DEG)
        return {
            "joint1_steps": round((joint1_signed + joint1_offset_rad) * self.joint1_steps_per_rad),
            "joint2_steps": round((joint2_signed + joint2_offset_rad) * self.joint2_steps_per_rad),
        }

    def move_to(self, x: float, y: float, elbow_up: bool = DEFAULT_ELBOW_UP) -> dict:
        pose = self.solve_ik(x, y, elbow_up=elbow_up)
        servo = self.pose_to_servo_command(pose)
        stepper = self.pose_to_stepper_targets(pose)
        command = {
            "target": (x, y),
            "pose_rad": pose,
            "servo": servo,
            "stepper": stepper,
        }
        self.send_command(command)
        return command

    def send_command(self, command: dict) -> None:
        # Replace this with serial, CAN, PWM, or direct driver output.
        print("MOVE COMMAND:", command)

    def move_home(self) -> dict:
        return self.move_to(*HOME_TARGET, elbow_up=DEFAULT_ELBOW_UP)


if __name__ == "__main__":
    controller = PlanarArmController()
    controller.move_home()
`;
}

export function createExportArtifact(type, config, robotState, waypoints) {
  if (type === "arduino-servo") {
    return createArduinoServoController(config, robotState);
  }
  if (type === "arduino-stepper") {
    return createArduinoStepperController(config, robotState);
  }
  if (type === "esp32-stepper") {
    return createEsp32StepperController(config, robotState);
  }
  if (type === "python") {
    return createPythonController(config, robotState, waypoints);
  }

  return JSON.stringify(createRobotExportPayload(config, robotState, waypoints), null, 2);
}

export function downloadTextFile(filename, content, mimeType = "text/plain;charset=utf-8") {
  const blob = new Blob([content], { type: mimeType });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = filename;
  link.click();
  URL.revokeObjectURL(url);
}

export async function copyText(content) {
  if (!navigator.clipboard?.writeText) {
    return false;
  }

  await navigator.clipboard.writeText(content);
  return true;
}
