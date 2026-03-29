import { degToRad } from "./utils/math.js";

export const STORAGE_KEYS = {
  appState: "robotics-simulator-dashboard-state"
};

export const SIMULATOR_MODES = [
  {
    id: "planar2",
    label: "2 DOF Mode",
    shortLabel: "2 DOF"
  },
  {
    id: "leg3",
    label: "3 DOF Mode",
    shortLabel: "3 DOF"
  }
];

export const DEFAULT_PLANAR2_MODULE_ORDER = [
  "ikfk-settings",
  "link-lengths",
  "joint-angles",
  "target-position",
  "units-display",
  "joint-velocity",
  "joint-limits",
  "jacobian-info",
  "calibration",
  "presets-motion"
];

export const DEFAULT_LEG3_MODULE_ORDER = [
  "leg-settings",
  "link-lengths",
  "joint-angles",
  "foot-target",
  "units-display",
  "joint-limits",
  "jacobian-info",
  "calibration",
  "leg-view"
];

export const EPSILON = 1e-9;
export const SINGULARITY_RATIO = 0.06;

export function createDefaultPlanar2Config() {
  return {
    solverMode: "ik",
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
}

export function createDefaultLeg3Config() {
  return {
    solverMode: "ik",
    angleUnit: "deg",
    kneeMode: "folded",
    L1: 0.34,
    L2: 0.72,
    L3: 0.86,
    theta1: degToRad(20),
    theta2: degToRad(-18),
    theta3: degToRad(76),
    theta1Dot: degToRad(8),
    theta2Dot: degToRad(12),
    theta3Dot: degToRad(-16),
    targetX: 1.25,
    targetY: 0.42,
    targetZ: -0.48,
    footTargetEnabled: true,
    show3dPreview: true,
    cameraAzimuthDeg: 32,
    cameraElevationDeg: 24,
    jointLimits: {
      theta1Min: degToRad(-70),
      theta1Max: degToRad(70),
      theta2Min: degToRad(-95),
      theta2Max: degToRad(85),
      theta3Min: degToRad(10),
      theta3Max: degToRad(150)
    },
    calibration: {
      joint1OffsetDeg: 90,
      joint2OffsetDeg: 90,
      joint3OffsetDeg: 90,
      joint1Invert: false,
      joint2Invert: false,
      joint3Invert: true
    }
  };
}

export const PLANAR2_BUILT_IN_PRESETS = [
  {
    id: "default",
    label: "Default Bench",
    config: createDefaultPlanar2Config()
  },
  {
    id: "compact-servo-arm",
    label: "Compact Servo Arm",
    config: {
      ...createDefaultPlanar2Config(),
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
      ...createDefaultPlanar2Config(),
      L1: 1.45,
      L2: 1.15,
      theta1: degToRad(40),
      theta2: degToRad(-35),
      targetX: 1.85,
      targetY: 0.8
    }
  }
];

export const LEG3_BUILT_IN_PRESETS = [
  {
    id: "hexapod-neutral",
    label: "Hexapod Neutral",
    config: createDefaultLeg3Config()
  },
  {
    id: "quadruped-stance",
    label: "Quadruped Stance",
    config: {
      ...createDefaultLeg3Config(),
      L1: 0.28,
      L2: 0.64,
      L3: 0.78,
      theta1: degToRad(10),
      theta2: degToRad(-28),
      theta3: degToRad(88),
      targetX: 1.05,
      targetY: 0.18,
      targetZ: -0.58
    }
  },
  {
    id: "reach-forward",
    label: "Reach Forward",
    config: {
      ...createDefaultLeg3Config(),
      theta1: degToRad(30),
      theta2: degToRad(-12),
      theta3: degToRad(54),
      targetX: 1.32,
      targetY: 0.76,
      targetZ: -0.24
    }
  }
];

export function createDefaultAppState() {
  return {
    activeMode: "planar2",
    activeWorkspacePage: "analysis",
    simulators: {
      planar2: {
        config: createDefaultPlanar2Config(),
        ui: {
          moduleOrder: [...DEFAULT_PLANAR2_MODULE_ORDER]
        },
        trajectory: {
          waypoints: [],
          trail: []
        },
        presets: {
          custom: [],
          selectedId: PLANAR2_BUILT_IN_PRESETS[0].id
        }
      },
      leg3: {
        config: createDefaultLeg3Config(),
        ui: {
          moduleOrder: [...DEFAULT_LEG3_MODULE_ORDER]
        },
        trajectory: {
          waypoints: [],
          trail: []
        },
        presets: {
          custom: [],
          selectedId: LEG3_BUILT_IN_PRESETS[0].id
        }
      }
    }
  };
}
