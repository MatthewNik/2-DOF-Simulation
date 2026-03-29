import { EPSILON, SINGULARITY_RATIO } from "../constants.js";
import { clamp, radToDeg } from "./math.js";

function determinant2x2(matrix) {
  return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
}

function determinant3x3(matrix) {
  return (
    matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) -
    matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]) +
    matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
  );
}

function multiplyMatrixVector(matrix, vector) {
  return matrix.map((row) =>
    row.reduce((sum, entry, index) => sum + entry * vector[index], 0)
  );
}

function withinJointLimits(theta, min, max) {
  return theta >= min - EPSILON && theta <= max + EPSILON;
}

function convertSimulationToServoDegrees(thetaRadians, offsetDeg, inverted) {
  const signed = inverted ? -radToDeg(thetaRadians) : radToDeg(thetaRadians);
  return offsetDeg + signed;
}

export function forwardKinematics2D(L1, L2, theta1, theta2) {
  const x1 = L1 * Math.cos(theta1);
  const y1 = L1 * Math.sin(theta1);
  const x = x1 + L2 * Math.cos(theta1 + theta2);
  const y = y1 + L2 * Math.sin(theta1 + theta2);

  return {
    joint1: { x: x1, y: y1 },
    endEffector: { x, y }
  };
}

export function solveInverseKinematics2D({ L1, L2, x, y, elbowMode }) {
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

export function computeJacobian2D(L1, L2, theta1, theta2) {
  const sin1 = Math.sin(theta1);
  const cos1 = Math.cos(theta1);
  const sin12 = Math.sin(theta1 + theta2);
  const cos12 = Math.cos(theta1 + theta2);

  return [
    [-L1 * sin1 - L2 * sin12, -L2 * sin12],
    [L1 * cos1 + L2 * cos12, L2 * cos12]
  ];
}

export function derivePlanar2State(config) {
  const ik = solveInverseKinematics2D({
    L1: config.L1,
    L2: config.L2,
    x: config.targetX,
    y: config.targetY,
    elbowMode: config.elbowMode
  });

  const activeTheta1 = config.solverMode === "ik" ? ik.theta1 : config.theta1;
  const activeTheta2 = config.solverMode === "ik" ? ik.theta2 : config.theta2;
  const fk = forwardKinematics2D(config.L1, config.L2, activeTheta1, activeTheta2);
  const jacobian = computeJacobian2D(config.L1, config.L2, activeTheta1, activeTheta2);
  const determinant = determinant2x2(jacobian);
  const [xDot, yDot] = multiplyMatrixVector(jacobian, [config.theta1Dot, config.theta2Dot]);
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

  return {
    simulatorType: "planar2",
    dimensions: 2,
    ik,
    fk,
    activeTheta1,
    activeTheta2,
    activeAngles: [activeTheta1, activeTheta2],
    activeTarget: { x: config.targetX, y: config.targetY },
    jacobian,
    determinant,
    velocity: { xDot, yDot },
    theta1WithinLimits,
    theta2WithinLimits,
    jointLimitStatus: [theta1WithinLimits, theta2WithinLimits],
    nearSingularity,
    singularityThreshold,
    reachableWithLimits,
    servoCommands: {
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
    },
    targetDistance: Math.hypot(
      config.targetX - fk.endEffector.x,
      config.targetY - fk.endEffector.y
    ),
    stateLabel: describePlanar2State({
      solverMode: config.solverMode,
      ikReachable: ik.reachable,
      theta1WithinLimits,
      theta2WithinLimits,
      nearSingularity
    })
  };
}

export function forwardKinematics3D(L1, L2, L3, theta1, theta2, theta3) {
  const cos1 = Math.cos(theta1);
  const sin1 = Math.sin(theta1);
  const cos2 = Math.cos(theta2);
  const sin2 = Math.sin(theta2);
  const cos23 = Math.cos(theta2 + theta3);
  const sin23 = Math.sin(theta2 + theta3);
  const radialKnee = L1 + L2 * cos2;
  const radialFoot = radialKnee + L3 * cos23;

  const coxa = {
    x: L1 * cos1,
    y: L1 * sin1,
    z: 0
  };
  const knee = {
    x: radialKnee * cos1,
    y: radialKnee * sin1,
    z: L2 * sin2
  };
  const foot = {
    x: radialFoot * cos1,
    y: radialFoot * sin1,
    z: L2 * sin2 + L3 * sin23
  };

  return {
    coxa,
    knee,
    foot,
    sideView: {
      base: { r: 0, z: 0 },
      coxa: { r: L1, z: 0 },
      knee: { r: radialKnee, z: L2 * sin2 },
      foot: { r: radialFoot, z: L2 * sin2 + L3 * sin23 }
    },
    radialFoot
  };
}

export function solveInverseKinematics3D({ L1, L2, L3, x, y, z, kneeMode }) {
  const theta1 = Math.atan2(y, x);
  const radiusXY = Math.hypot(x, y);
  const planeX = radiusXY - L1;
  const planeY = z;
  const planeRadiusSquared = planeX * planeX + planeY * planeY;
  const planeRadius = Math.sqrt(planeRadiusSquared);
  const rawCosTheta3 = (planeRadiusSquared - L2 * L2 - L3 * L3) / (2 * L2 * L3);
  const reachable =
    rawCosTheta3 >= -1 - EPSILON &&
    rawCosTheta3 <= 1 + EPSILON &&
    planeRadius <= L2 + L3 + EPSILON &&
    planeRadius >= Math.abs(L2 - L3) - EPSILON;
  const cosTheta3 = clamp(rawCosTheta3, -1, 1);
  const sinMagnitude = Math.sqrt(Math.max(0, 1 - cosTheta3 * cosTheta3));
  const sinTheta3 = kneeMode === "extended" ? sinMagnitude : -sinMagnitude;
  const theta3 = Math.atan2(sinTheta3, cosTheta3);
  const theta2 =
    Math.atan2(planeY, planeX) -
    Math.atan2(L3 * sinTheta3, L2 + L3 * cosTheta3);

  return {
    theta1,
    theta2,
    theta3,
    cosTheta3,
    sinTheta3,
    rawCosTheta3,
    reachable,
    radiusXY,
    planeX,
    planeY,
    planeRadius
  };
}

export function computeJacobian3D(L1, L2, L3, theta1, theta2, theta3) {
  const radial = L1 + L2 * Math.cos(theta2) + L3 * Math.cos(theta2 + theta3);
  const drTheta2 = -L2 * Math.sin(theta2) - L3 * Math.sin(theta2 + theta3);
  const drTheta3 = -L3 * Math.sin(theta2 + theta3);
  const dzTheta2 = L2 * Math.cos(theta2) + L3 * Math.cos(theta2 + theta3);
  const dzTheta3 = L3 * Math.cos(theta2 + theta3);
  const cos1 = Math.cos(theta1);
  const sin1 = Math.sin(theta1);

  return [
    [-radial * sin1, cos1 * drTheta2, cos1 * drTheta3],
    [radial * cos1, sin1 * drTheta2, sin1 * drTheta3],
    [0, dzTheta2, dzTheta3]
  ];
}

export function deriveLeg3State(config) {
  const ik = solveInverseKinematics3D({
    L1: config.L1,
    L2: config.L2,
    L3: config.L3,
    x: config.targetX,
    y: config.targetY,
    z: config.targetZ,
    kneeMode: config.kneeMode
  });

  const activeTheta1 = config.solverMode === "ik" ? ik.theta1 : config.theta1;
  const activeTheta2 = config.solverMode === "ik" ? ik.theta2 : config.theta2;
  const activeTheta3 = config.solverMode === "ik" ? ik.theta3 : config.theta3;
  const fk = forwardKinematics3D(
    config.L1,
    config.L2,
    config.L3,
    activeTheta1,
    activeTheta2,
    activeTheta3
  );
  const jacobian = computeJacobian3D(
    config.L1,
    config.L2,
    config.L3,
    activeTheta1,
    activeTheta2,
    activeTheta3
  );
  const determinant = determinant3x3(jacobian);
  const [xDot, yDot, zDot] = multiplyMatrixVector(jacobian, [
    config.theta1Dot,
    config.theta2Dot,
    config.theta3Dot
  ]);
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
  const theta3WithinLimits = withinJointLimits(
    activeTheta3,
    config.jointLimits.theta3Min,
    config.jointLimits.theta3Max
  );
  const singularityThreshold = Math.max(
    EPSILON,
    SINGULARITY_RATIO * Math.max(0.1, config.L2 * config.L3 * (config.L1 + config.L2 + config.L3))
  );
  const nearSingularity = Math.abs(determinant) <= singularityThreshold;
  const reachableWithLimits =
    ik.reachable &&
    theta1WithinLimits &&
    theta2WithinLimits &&
    theta3WithinLimits;

  return {
    simulatorType: "leg3",
    dimensions: 3,
    ik,
    fk,
    activeTheta1,
    activeTheta2,
    activeTheta3,
    activeAngles: [activeTheta1, activeTheta2, activeTheta3],
    activeTarget: {
      x: config.targetX,
      y: config.targetY,
      z: config.targetZ
    },
    jacobian,
    determinant,
    velocity: { xDot, yDot, zDot },
    theta1WithinLimits,
    theta2WithinLimits,
    theta3WithinLimits,
    jointLimitStatus: [theta1WithinLimits, theta2WithinLimits, theta3WithinLimits],
    nearSingularity,
    singularityThreshold,
    reachableWithLimits,
    servoCommands: {
      joint1ServoDeg: convertSimulationToServoDegrees(
        activeTheta1,
        config.calibration.joint1OffsetDeg,
        config.calibration.joint1Invert
      ),
      joint2ServoDeg: convertSimulationToServoDegrees(
        activeTheta2,
        config.calibration.joint2OffsetDeg,
        config.calibration.joint2Invert
      ),
      joint3ServoDeg: convertSimulationToServoDegrees(
        activeTheta3,
        config.calibration.joint3OffsetDeg,
        config.calibration.joint3Invert
      )
    },
    targetDistance: Math.hypot(
      config.targetX - fk.foot.x,
      config.targetY - fk.foot.y,
      config.targetZ - fk.foot.z
    ),
    stateLabel: describeLeg3State({
      solverMode: config.solverMode,
      ikReachable: ik.reachable,
      theta1WithinLimits,
      theta2WithinLimits,
      theta3WithinLimits,
      nearSingularity
    })
  };
}

export function describePlanar2State({
  solverMode,
  ikReachable,
  theta1WithinLimits,
  theta2WithinLimits,
  nearSingularity
}) {
  if (solverMode === "ik" && !ikReachable) {
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

export function describeLeg3State({
  solverMode,
  ikReachable,
  theta1WithinLimits,
  theta2WithinLimits,
  theta3WithinLimits,
  nearSingularity
}) {
  if (solverMode === "ik" && !ikReachable) {
    return "Foot target outside workspace";
  }
  if (!theta1WithinLimits || !theta2WithinLimits || !theta3WithinLimits) {
    return "Leg pose violates limits";
  }
  if (nearSingularity) {
    return "Near planar singularity";
  }
  return "Nominal";
}
