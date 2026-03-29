import { formatNumber } from "./math.js";

const PLANAR2_EXPORT_OPTIONS = [
  {
    id: "json",
    label: "JSON Snapshot",
    extension: "json",
    description: "Machine-readable snapshot of links, limits, calibration, live state, and waypoints."
  },
  {
    id: "arduino-servo",
    label: "Arduino Servo",
    extension: "ino",
    description: "2 DOF Arduino servo scaffold with IK, limits, and zero offsets."
  },
  {
    id: "python",
    label: "Python Controller",
    extension: "py",
    description: "2 DOF Python controller scaffold with FK, IK, and command conversion."
  }
];

const LEG3_EXPORT_OPTIONS = [
  {
    id: "json",
    label: "JSON Snapshot",
    extension: "json",
    description: "Machine-readable snapshot of the leg geometry, limits, calibration, and current pose."
  },
  {
    id: "python",
    label: "Python Controller",
    extension: "py",
    description: "3 DOF leg controller scaffold with yaw-plus-planar IK and robot-side notes."
  }
];

function boolLiteral(value) {
  return value ? "true" : "false";
}

function pythonBool(value) {
  return value ? "True" : "False";
}

export function getExportOptionsForMode(mode) {
  return mode === "leg3" ? LEG3_EXPORT_OPTIONS : PLANAR2_EXPORT_OPTIONS;
}

export function createRobotExportPayload(mode, config, robotState, trajectory) {
  if (mode === "leg3") {
    return {
      generatedAt: new Date().toISOString(),
      simulatorMode: "3-DOF leg",
      robot: {
        links: {
          coxaL1: config.L1,
          femurL2: config.L2,
          tibiaL3: config.L3
        },
        jointLimitsRad: config.jointLimits,
        calibration: config.calibration
      },
      state: {
        solverMode: config.solverMode,
        kneeMode: config.kneeMode,
        target: robotState.activeTarget,
        jointAnglesRad: {
          theta1: robotState.activeTheta1,
          theta2: robotState.activeTheta2,
          theta3: robotState.activeTheta3
        },
        foot: robotState.fk.foot,
        knee: robotState.fk.knee,
        coxa: robotState.fk.coxa,
        servoCommandsDeg: robotState.servoCommands,
        reachable: robotState.ik.reachable,
        withinJointLimits:
          robotState.theta1WithinLimits &&
          robotState.theta2WithinLimits &&
          robotState.theta3WithinLimits,
        nearSingularity: robotState.nearSingularity
      },
      trajectory
    };
  }

  return {
    generatedAt: new Date().toISOString(),
    simulatorMode: "2-DOF planar arm",
    robot: {
      links: {
        L1: config.L1,
        L2: config.L2
      },
      jointLimitsRad: config.jointLimits,
      calibration: config.calibration
    },
    state: {
      solverMode: config.solverMode,
      elbowMode: config.elbowMode,
      target: robotState.activeTarget,
      jointAnglesRad: {
        theta1: robotState.activeTheta1,
        theta2: robotState.activeTheta2
      },
      endEffector: robotState.fk.endEffector,
      servoCommandsDeg: robotState.servoCommands,
      reachable: robotState.ik.reachable,
      withinJointLimits: robotState.theta1WithinLimits && robotState.theta2WithinLimits,
      nearSingularity: robotState.nearSingularity
    },
    trajectory
  };
}

function createPlanar2ArduinoServo(config, robotState) {
  return `#include <Arduino.h>
#include <Servo.h>
#include <math.h>

namespace PlanarArmServo {
constexpr float L1 = ${formatNumber(config.L1, 6)}f;
constexpr float L2 = ${formatNumber(config.L2, 6)}f;
constexpr float THETA1_MIN_RAD = ${formatNumber(config.jointLimits.theta1Min, 6)}f;
constexpr float THETA1_MAX_RAD = ${formatNumber(config.jointLimits.theta1Max, 6)}f;
constexpr float THETA2_MIN_RAD = ${formatNumber(config.jointLimits.theta2Min, 6)}f;
constexpr float THETA2_MAX_RAD = ${formatNumber(config.jointLimits.theta2Max, 6)}f;
constexpr float JOINT1_ZERO_OFFSET_DEG = ${formatNumber(config.calibration.joint1OffsetDeg, 4)}f;
constexpr float JOINT2_ZERO_OFFSET_DEG = ${formatNumber(config.calibration.joint2OffsetDeg, 4)}f;
constexpr bool JOINT1_INVERTED = ${boolLiteral(config.calibration.joint1Invert)};
constexpr bool JOINT2_INVERTED = ${boolLiteral(config.calibration.joint2Invert)};
constexpr bool DEFAULT_ELBOW_UP = ${boolLiteral(config.elbowMode === "up")};

struct PoseRad {
  float theta1;
  float theta2;
};

float clampf(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
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
  return pose.theta1 >= THETA1_MIN_RAD && pose.theta1 <= THETA1_MAX_RAD &&
         pose.theta2 >= THETA2_MIN_RAD && pose.theta2 <= THETA2_MAX_RAD;
}

float simToServoDeg(float thetaRad, float offsetDeg, bool inverted) {
  const float thetaDeg = thetaRad * 180.0f / PI;
  return offsetDeg + (inverted ? -thetaDeg : thetaDeg);
}
}  // namespace PlanarArmServo

void setup() {}
void loop() {}
`;
}

function createPlanar2Python(config, robotState, trajectory) {
  return `import math
from dataclasses import dataclass

L1 = ${formatNumber(config.L1, 8)}
L2 = ${formatNumber(config.L2, 8)}
HOME_TARGET = (${formatNumber(robotState.activeTarget.x, 8)}, ${formatNumber(robotState.activeTarget.y, 8)})
WAYPOINTS = ${JSON.stringify(trajectory.waypoints, null, 2)}
DEFAULT_ELBOW_UP = ${pythonBool(config.elbowMode === "up")}

@dataclass
class PoseRad:
    theta1: float
    theta2: float

def solve_ik(x: float, y: float, elbow_up: bool = DEFAULT_ELBOW_UP) -> PoseRad:
    radius2 = x * x + y * y
    raw_cos_theta2 = (radius2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if raw_cos_theta2 < -1.0 or raw_cos_theta2 > 1.0:
        raise ValueError("Target outside workspace")
    cos_theta2 = max(-1.0, min(1.0, raw_cos_theta2))
    sin_mag = math.sqrt(max(0.0, 1.0 - cos_theta2 * cos_theta2))
    sin_theta2 = sin_mag if elbow_up else -sin_mag
    theta2 = math.atan2(sin_theta2, cos_theta2)
    theta1 = math.atan2(y, x) - math.atan2(L2 * sin_theta2, L1 + L2 * cos_theta2)
    return PoseRad(theta1=theta1, theta2=theta2)

if __name__ == "__main__":
    print(solve_ik(*HOME_TARGET))
`;
}

function createLeg3Python(config, robotState) {
  return `import math
from dataclasses import dataclass

L1 = ${formatNumber(config.L1, 8)}  # coxa / hip offset
L2 = ${formatNumber(config.L2, 8)}  # femur
L3 = ${formatNumber(config.L3, 8)}  # tibia
HOME_TARGET = (
    ${formatNumber(robotState.activeTarget.x, 8)},
    ${formatNumber(robotState.activeTarget.y, 8)},
    ${formatNumber(robotState.activeTarget.z, 8)},
)
DEFAULT_KNEE_EXTENDED = ${pythonBool(config.kneeMode === "extended")}

@dataclass
class LegPoseRad:
    theta1: float
    theta2: float
    theta3: float

def solve_leg_ik(x: float, y: float, z: float, knee_extended: bool = DEFAULT_KNEE_EXTENDED) -> LegPoseRad:
    theta1 = math.atan2(y, x)
    radius_xy = math.hypot(x, y)
    plane_x = radius_xy - L1
    raw_cos_theta3 = (plane_x * plane_x + z * z - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
    if raw_cos_theta3 < -1.0 or raw_cos_theta3 > 1.0:
        raise ValueError("Foot target outside workspace")

    cos_theta3 = max(-1.0, min(1.0, raw_cos_theta3))
    sin_mag = math.sqrt(max(0.0, 1.0 - cos_theta3 * cos_theta3))
    sin_theta3 = sin_mag if knee_extended else -sin_mag
    theta3 = math.atan2(sin_theta3, cos_theta3)
    theta2 = math.atan2(z, plane_x) - math.atan2(L3 * sin_theta3, L2 + L3 * cos_theta3)
    return LegPoseRad(theta1=theta1, theta2=theta2, theta3=theta3)

if __name__ == "__main__":
    print(solve_leg_ik(*HOME_TARGET))
`;
}

export function createExportArtifact(mode, type, config, robotState, trajectory) {
  if (mode === "leg3") {
    if (type === "python") {
      return createLeg3Python(config, robotState);
    }
    return JSON.stringify(
      createRobotExportPayload(mode, config, robotState, trajectory),
      null,
      2
    );
  }

  if (type === "arduino-servo") {
    return createPlanar2ArduinoServo(config, robotState);
  }
  if (type === "python") {
    return createPlanar2Python(config, robotState, trajectory);
  }
  return JSON.stringify(
    createRobotExportPayload(mode, config, robotState, trajectory),
    null,
    2
  );
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
