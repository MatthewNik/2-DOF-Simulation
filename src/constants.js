import { degToRad } from "./utils/math.js";

export const STORAGE_KEYS = {
  appState: "planar-robot-app-state",
  presets: "planar-robot-presets"
};

export const DEFAULT_CONFIG = {
  mode: "fk",
  angleUnit: "deg",
  elbowMode: "down",
  L1: 1.2,
  L2: 0.9,
  theta1: degToRad(35),
  theta2: degToRad(40),
  theta1Dot: degToRad(18),
  theta2Dot: degToRad(-12),
  targetX: 1.55,
  targetY: 0.55,
  traceEnabled: true,
  animateTargetChanges: false,
  jointLimits: {
    theta1Min: degToRad(-170),
    theta1Max: degToRad(170),
    theta2Min: degToRad(-150),
    theta2Max: degToRad(150)
  },
  calibration: {
    joint1OffsetDeg: 90,
    joint2OffsetDeg: 90,
    joint1Invert: false,
    joint2Invert: true
  }
};

export const DEFAULT_TRAJECTORY = {
  waypoints: [],
  trail: []
};

export const BUILT_IN_PRESETS = [
  {
    id: "default",
    label: "Default Bench",
    config: DEFAULT_CONFIG
  },
  {
    id: "compact-servo-arm",
    label: "Compact Servo Arm",
    config: {
      ...DEFAULT_CONFIG,
      L1: 0.95,
      L2: 0.7,
      theta1: degToRad(25),
      theta2: degToRad(55),
      targetX: 1.15,
      targetY: 0.65
    }
  },
  {
    id: "long-reach",
    label: "Long Reach Demo",
    config: {
      ...DEFAULT_CONFIG,
      L1: 1.45,
      L2: 1.15,
      theta1: degToRad(40),
      theta2: degToRad(-35),
      targetX: 1.85,
      targetY: 0.8
    }
  },
  {
    id: "calibration-rig",
    label: "Calibration Rig",
    config: {
      ...DEFAULT_CONFIG,
      mode: "fk",
      L1: 1,
      L2: 1,
      theta1: degToRad(0),
      theta2: degToRad(0),
      theta1Dot: degToRad(0),
      theta2Dot: degToRad(0),
      targetX: 1.6,
      targetY: 0
    }
  }
];

export const EPSILON = 1e-9;
export const SINGULARITY_RATIO = 0.06;
