import { EPSILON, SINGULARITY_RATIO } from "../constants.js";
import { clamp, radToDeg } from "./math.js";

export function forwardKinematics(L1, L2, theta1, theta2) {
  const x1 = L1 * Math.cos(theta1);
  const y1 = L1 * Math.sin(theta1);
  const x = x1 + L2 * Math.cos(theta1 + theta2);
  const y = y1 + L2 * Math.sin(theta1 + theta2);

  return {
    joint1: { x: x1, y: y1 },
    endEffector: { x, y }
  };
}

export function solveInverseKinematics({ L1, L2, x, y, elbowMode }) {
  const radiusSquared = x * x + y * y;
  const radius = Math.sqrt(radiusSquared);
  const innerRadius = Math.abs(L1 - L2);
  const outerRadius = L1 + L2;
  const rawCosTheta2 = (radiusSquared - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  const reachable =
    rawCosTheta2 >= -1 - EPSILON &&
    rawCosTheta2 <= 1 + EPSILON &&
    radius <= outerRadius + EPSILON &&
    radius >= innerRadius - EPSILON;
  const cosTheta2 = clamp(rawCosTheta2, -1, 1);
  const sinMagnitude = Math.sqrt(Math.max(0, 1 - cosTheta2 * cosTheta2));
  const sinTheta2 = elbowMode === "up" ? sinMagnitude : -sinMagnitude;
  const theta2 = Math.atan2(sinTheta2, cosTheta2);
  const theta1 =
    Math.atan2(y, x) -
    Math.atan2(L2 * sinTheta2, L1 + L2 * cosTheta2);

  return {
    theta1,
    theta2,
    sinTheta2,
    cosTheta2,
    rawCosTheta2,
    reachable,
    radius,
    innerRadius,
    outerRadius
  };
}

export function computeJacobian(L1, L2, theta1, theta2) {
  const sin1 = Math.sin(theta1);
  const cos1 = Math.cos(theta1);
  const sin12 = Math.sin(theta1 + theta2);
  const cos12 = Math.cos(theta1 + theta2);

  return [
    [-L1 * sin1 - L2 * sin12, -L2 * sin12],
    [L1 * cos1 + L2 * cos12, L2 * cos12]
  ];
}

export function determinant2x2(matrix) {
  return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
}

export function multiplyJacobian(matrix, vector) {
  return {
    xDot: matrix[0][0] * vector[0] + matrix[0][1] * vector[1],
    yDot: matrix[1][0] * vector[0] + matrix[1][1] * vector[1]
  };
}

export function withinJointLimits(theta, min, max) {
  return theta >= min - EPSILON && theta <= max + EPSILON;
}

export function convertSimulationToServoDegrees(thetaRadians, offsetDeg, inverted) {
  const signed = inverted ? -radToDeg(thetaRadians) : radToDeg(thetaRadians);
  return offsetDeg + signed;
}

export function deriveRobotState(config) {
  const ik = solveInverseKinematics({
    L1: config.L1,
    L2: config.L2,
    x: config.targetX,
    y: config.targetY,
    elbowMode: config.elbowMode
  });

  const activeTheta1 = config.mode === "ik" ? ik.theta1 : config.theta1;
  const activeTheta2 = config.mode === "ik" ? ik.theta2 : config.theta2;
  const fk = forwardKinematics(config.L1, config.L2, activeTheta1, activeTheta2);
  const jacobian = computeJacobian(config.L1, config.L2, activeTheta1, activeTheta2);
  const determinant = determinant2x2(jacobian);
  const velocity = multiplyJacobian(jacobian, [config.theta1Dot, config.theta2Dot]);
  const theta1WithinLimits = withinJointLimits(
    activeTheta1,
    config.jointLimits.theta1Min,
    config.jointLimits.theta1Max
  );
  const theta2WithinLimits = withinJointLimits(
    activeTheta2,
    config.jointLimits.theta2Min,
    config.jointLimits.theta2Max
  );
  const singularityThreshold = Math.max(EPSILON, SINGULARITY_RATIO * config.L1 * config.L2);
  const nearSingularity = Math.abs(determinant) <= singularityThreshold;
  const reachableWithLimits = ik.reachable && theta1WithinLimits && theta2WithinLimits;
  const servoCommands = {
    joint1ServoDeg: convertSimulationToServoDegrees(
      activeTheta1,
      config.calibration.joint1OffsetDeg,
      config.calibration.joint1Invert
    ),
    joint2ServoDeg: convertSimulationToServoDegrees(
      activeTheta2,
      config.calibration.joint2OffsetDeg,
      config.calibration.joint2Invert
    )
  };
  const targetDistance = Math.hypot(config.targetX - fk.endEffector.x, config.targetY - fk.endEffector.y);

  return {
    ik,
    fk,
    activeTheta1,
    activeTheta2,
    jacobian,
    determinant,
    velocity,
    theta1WithinLimits,
    theta2WithinLimits,
    nearSingularity,
    singularityThreshold,
    reachableWithLimits,
    servoCommands,
    targetDistance,
    stateLabel: describeState({
      mode: config.mode,
      ikReachable: ik.reachable,
      theta1WithinLimits,
      theta2WithinLimits,
      nearSingularity
    })
  };
}

export function describeState({ mode, ikReachable, theta1WithinLimits, theta2WithinLimits, nearSingularity }) {
  if (mode === "ik" && !ikReachable) {
    return "Target outside workspace";
  }

  if (!theta1WithinLimits || !theta2WithinLimits) {
    return "Pose violates joint limits";
  }

  if (nearSingularity) {
    return "Near singular configuration";
  }

  return "Nominal";
}
