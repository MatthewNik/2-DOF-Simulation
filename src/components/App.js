import { html, useEffect, useMemo, useRef, useState } from "../lib.js";
import {
  createDefaultLeg3Config,
  createDefaultPlanar2Config,
  LEG3_BUILT_IN_PRESETS,
  PLANAR2_BUILT_IN_PRESETS
} from "../constants.js";
import { DraggableControlPanel } from "./DraggableControlPanel.js";
import { EquationsPanel } from "./EquationsPanel.js";
import { ModeSwitcher } from "./ModeSwitcher.js";
import { RobotCanvas } from "./RobotCanvas.js";
import { StatePanel } from "./StatePanel.js";
import { deriveLeg3State, derivePlanar2State } from "../utils/kinematics.js";
import {
  distance,
  lerp,
  radToDeg,
  degToRad,
  sanitizeNumber,
  formatNumber
} from "../utils/math.js";
import { loadSavedAppState, saveAppState } from "../utils/storage.js";

function clone(value) {
  return typeof structuredClone === "function"
    ? structuredClone(value)
    : JSON.parse(JSON.stringify(value));
}

function limitTrail(nextTrail) {
  return nextTrail.slice(-1200);
}

function unitLabel(angleUnit) {
  return angleUnit === "deg" ? "deg" : "rad";
}

function angleToUi(value, angleUnit) {
  return angleUnit === "deg" ? radToDeg(value) : value;
}

function angleFromUi(value, angleUnit) {
  return angleUnit === "deg" ? degToRad(value) : value;
}

function SliderNumberControl({
  label,
  value,
  min,
  max,
  step,
  displayValue,
  onChange,
  disabled = false
}) {
  return html`
    <div className="control-row">
      <label>
        <span>${label}</span>
        <span>${displayValue}</span>
      </label>
      <div className="control-inline">
        <input
          type="range"
          min=${min}
          max=${max}
          step=${step}
          value=${value}
          onInput=${(event) => onChange(Number(event.target.value))}
          disabled=${disabled}
        />
        <input
          type="number"
          min=${min}
          max=${max}
          step=${step}
          value=${value}
          onChange=${(event) => onChange(Number(event.target.value))}
          disabled=${disabled}
        />
      </div>
    </div>
  `;
}

function ToggleButton({ active, label, onClick }) {
  return html`
    <button className=${active ? "active" : ""} type="button" onClick=${onClick}>
      ${label}
    </button>
  `;
}

export function App() {
  const [appState, setAppState] = useState(() => loadSavedAppState());
  const [isPlanarPathPlaying, setIsPlanarPathPlaying] = useState(false);
  const targetAnimationRef = useRef(null);

  const planarConfig = appState.simulators.planar2.config;
  const legConfig = appState.simulators.leg3.config;
  const planarTrajectory = appState.simulators.planar2.trajectory;
  const legTrajectory = appState.simulators.leg3.trajectory;
  const planarPresets = appState.simulators.planar2.presets;
  const legPresets = appState.simulators.leg3.presets;
  const planarState = useMemo(() => derivePlanar2State(planarConfig), [planarConfig]);
  const legState = useMemo(() => deriveLeg3State(legConfig), [legConfig]);
  const activeMode = appState.activeMode;
  const activeConfig = activeMode === "leg3" ? legConfig : planarConfig;
  const activeRobotState = activeMode === "leg3" ? legState : planarState;
  const activeTrajectory = activeMode === "leg3" ? legTrajectory : planarTrajectory;

  useEffect(() => {
    saveAppState(appState);
  }, [appState]);

  useEffect(() => {
    if (!planarConfig.traceEnabled) {
      return;
    }
    appendPlanarTrailPoint(planarState.fk.endEffector);
  }, [planarConfig.traceEnabled, planarState.fk.endEffector.x, planarState.fk.endEffector.y]);

  useEffect(() => {
    if (!isPlanarPathPlaying) {
      return undefined;
    }

    if (planarTrajectory.waypoints.length < 2) {
      setIsPlanarPathPlaying(false);
      return undefined;
    }

    cancelPlanarTargetAnimation();

    const segments = planarTrajectory.waypoints.slice(1).map((point, index) => ({
      start: planarTrajectory.waypoints[index],
      end: point,
      length: distance(planarTrajectory.waypoints[index], point)
    }));
    const totalLength = segments.reduce((sum, segment) => sum + segment.length, 0);
    if (totalLength <= 1e-9) {
      setIsPlanarPathPlaying(false);
      return undefined;
    }

    const speed = Math.max(0.3, (planarConfig.L1 + planarConfig.L2) * 0.35);
    let frameId = null;
    let previousTime = null;
    let travelled = 0;

    const step = (time) => {
      if (previousTime === null) {
        previousTime = time;
      }

      const deltaSeconds = (time - previousTime) / 1000;
      previousTime = time;
      travelled = Math.min(totalLength, travelled + deltaSeconds * speed);

      let remaining = travelled;
      let activeSegment = segments[segments.length - 1];
      for (const segment of segments) {
        if (remaining <= segment.length) {
          activeSegment = segment;
          break;
        }
        remaining -= segment.length;
      }

      const t = activeSegment.length <= 1e-9 ? 1 : remaining / activeSegment.length;
      handlePlanarTargetChange(
        {
          x: lerp(activeSegment.start.x, activeSegment.end.x, t),
          y: lerp(activeSegment.start.y, activeSegment.end.y, t)
        },
        { immediate: true }
      );

      if (travelled >= totalLength) {
        setIsPlanarPathPlaying(false);
        return;
      }

      frameId = requestAnimationFrame(step);
    };

    frameId = requestAnimationFrame(step);

    return () => {
      if (frameId !== null) {
        cancelAnimationFrame(frameId);
      }
    };
  }, [isPlanarPathPlaying, planarTrajectory.waypoints, planarConfig.L1, planarConfig.L2]);

  function updateSimulator(mode, updater) {
    setAppState((previous) => {
      const next = clone(previous);
      updater(next.simulators[mode], next);
      return next;
    });
  }

  function updateSimulatorConfig(mode, updater) {
    updateSimulator(mode, (simulator) => {
      simulator.config = updater(simulator.config);
    });
  }

  function setActiveMode(mode) {
    setAppState((previous) => ({
      ...previous,
      activeMode: mode
    }));
  }

  function cancelPlanarTargetAnimation() {
    if (targetAnimationRef.current) {
      cancelAnimationFrame(targetAnimationRef.current);
      targetAnimationRef.current = null;
    }
  }

  function appendPlanarTrailPoint(point) {
    updateSimulator("planar2", (simulator) => {
      const sample = {
        x: Number(point.x.toFixed(4)),
        y: Number(point.y.toFixed(4))
      };
      const previous = simulator.trajectory.trail;
      const last = previous[previous.length - 1];
      if (last && distance(last, sample) < 0.015) {
        return;
      }
      simulator.trajectory.trail = limitTrail([...previous, sample]);
    });
  }

  function handlePlanarTargetChange(point, options = {}) {
    setIsPlanarPathPlaying(false);
    if (planarConfig.animateTargetChanges && !options.immediate) {
      cancelPlanarTargetAnimation();
      const start =
        planarConfig.solverMode === "ik"
          ? { x: planarConfig.targetX, y: planarConfig.targetY }
          : planarState.fk.endEffector;
      const end = point;
      const durationMs = 320;
      const startedAt = performance.now();

      function step(now) {
        const t = Math.min(1, (now - startedAt) / durationMs);
        const eased = 1 - Math.pow(1 - t, 3);
        const nextPoint = {
          x: lerp(start.x, end.x, eased),
          y: lerp(start.y, end.y, eased)
        };

        updateSimulatorConfig("planar2", (previous) => ({
          ...previous,
          solverMode: "ik",
          targetX: nextPoint.x,
          targetY: nextPoint.y
        }));

        if (t < 1) {
          targetAnimationRef.current = requestAnimationFrame(step);
        } else {
          targetAnimationRef.current = null;
        }
      }

      targetAnimationRef.current = requestAnimationFrame(step);
      return;
    }

    cancelPlanarTargetAnimation();
    updateSimulatorConfig("planar2", (previous) => ({
      ...previous,
      solverMode: "ik",
      targetX: sanitizeNumber(point.x, previous.targetX),
      targetY: sanitizeNumber(point.y, previous.targetY)
    }));
  }

  function handleLegTargetChange(point) {
    updateSimulatorConfig("leg3", (previous) => ({
      ...previous,
      solverMode: "ik",
      footTargetEnabled: true,
      targetX: sanitizeNumber(point.x, previous.targetX),
      targetY: sanitizeNumber(point.y, previous.targetY),
      targetZ: sanitizeNumber(point.z, previous.targetZ)
    }));
  }

  function handlePlanarSolverModeChange(nextMode) {
    if (nextMode === planarConfig.solverMode) {
      return;
    }
    cancelPlanarTargetAnimation();
    setIsPlanarPathPlaying(false);
    updateSimulatorConfig("planar2", (previous) => {
      if (nextMode === "ik") {
        return {
          ...previous,
          solverMode: "ik",
          theta1: planarState.activeTheta1,
          theta2: planarState.activeTheta2,
          targetX: planarState.fk.endEffector.x,
          targetY: planarState.fk.endEffector.y
        };
      }
      return {
        ...previous,
        solverMode: "fk",
        theta1: planarState.activeTheta1,
        theta2: planarState.activeTheta2
      };
    });
  }

  function handleLegSolverModeChange(nextMode) {
    if (nextMode === legConfig.solverMode) {
      return;
    }
    updateSimulatorConfig("leg3", (previous) => {
      if (nextMode === "ik") {
        return {
          ...previous,
          solverMode: "ik",
          theta1: legState.activeTheta1,
          theta2: legState.activeTheta2,
          theta3: legState.activeTheta3,
          targetX: legState.fk.foot.x,
          targetY: legState.fk.foot.y,
          targetZ: legState.fk.foot.z
        };
      }
      return {
        ...previous,
        solverMode: "fk",
        theta1: legState.activeTheta1,
        theta2: legState.activeTheta2,
        theta3: legState.activeTheta3
      };
    });
  }

  function handlePlanarJointLimitChange(key, value) {
    updateSimulatorConfig("planar2", (previous) => {
      const next = clone(previous);
      next.jointLimits[key] = sanitizeNumber(value, previous.jointLimits[key]);
      if (key === "theta1Min" && next.jointLimits.theta1Min > next.jointLimits.theta1Max) {
        next.jointLimits.theta1Max = next.jointLimits.theta1Min;
      }
      if (key === "theta1Max" && next.jointLimits.theta1Max < next.jointLimits.theta1Min) {
        next.jointLimits.theta1Min = next.jointLimits.theta1Max;
      }
      if (key === "theta2Min" && next.jointLimits.theta2Min > next.jointLimits.theta2Max) {
        next.jointLimits.theta2Max = next.jointLimits.theta2Min;
      }
      if (key === "theta2Max" && next.jointLimits.theta2Max < next.jointLimits.theta2Min) {
        next.jointLimits.theta2Min = next.jointLimits.theta2Max;
      }
      return next;
    });
  }

  function handleLegJointLimitChange(key, value) {
    updateSimulatorConfig("leg3", (previous) => {
      const next = clone(previous);
      next.jointLimits[key] = sanitizeNumber(value, previous.jointLimits[key]);
      const [jointPrefix] = key.match(/^theta\d/) || [];
      if (jointPrefix) {
        const minKey = `${jointPrefix}Min`;
        const maxKey = `${jointPrefix}Max`;
        if (next.jointLimits[minKey] > next.jointLimits[maxKey]) {
          if (key.endsWith("Min")) {
            next.jointLimits[maxKey] = next.jointLimits[minKey];
          } else {
            next.jointLimits[minKey] = next.jointLimits[maxKey];
          }
        }
      }
      return next;
    });
  }

  function handlePresetLoad(mode) {
    const simulator = appState.simulators[mode];
    const builtIns = mode === "leg3" ? LEG3_BUILT_IN_PRESETS : PLANAR2_BUILT_IN_PRESETS;
    const preset = [...builtIns, ...simulator.presets.custom].find(
      (entry) => entry.id === simulator.presets.selectedId
    );
    if (!preset) {
      return;
    }

    if (mode === "planar2") {
      cancelPlanarTargetAnimation();
      setIsPlanarPathPlaying(false);
    }

    updateSimulator(mode, (nextSimulator) => {
      nextSimulator.config = clone(preset.config);
      nextSimulator.trajectory = { waypoints: [], trail: [] };
    });
  }

  function handlePresetSave(mode) {
    const simulator = appState.simulators[mode];
    const label = window.prompt(
      "Preset name",
      `${mode === "leg3" ? "Leg" : "Arm"} Preset ${simulator.presets.custom.length + 1}`
    );
    if (!label) {
      return;
    }

    const nextPreset = {
      id: `custom-${Date.now()}`,
      label: label.trim(),
      config: clone(simulator.config)
    };

    updateSimulator(mode, (nextSimulator) => {
      nextSimulator.presets.custom.push(nextPreset);
      nextSimulator.presets.selectedId = nextPreset.id;
    });
  }

  function handlePresetDelete(mode) {
    const selectedId = appState.simulators[mode].presets.selectedId;
    if (!selectedId.startsWith("custom-")) {
      return;
    }

    const defaultId =
      mode === "leg3" ? LEG3_BUILT_IN_PRESETS[0].id : PLANAR2_BUILT_IN_PRESETS[0].id;
    updateSimulator(mode, (simulator) => {
      simulator.presets.custom = simulator.presets.custom.filter(
        (preset) => preset.id !== selectedId
      );
      simulator.presets.selectedId = defaultId;
    });
  }

  function handlePlanarReset() {
    cancelPlanarTargetAnimation();
    setIsPlanarPathPlaying(false);
    updateSimulator("planar2", (simulator) => {
      simulator.config = createDefaultPlanar2Config();
      simulator.trajectory = { waypoints: [], trail: [] };
      simulator.presets.selectedId = PLANAR2_BUILT_IN_PRESETS[0].id;
    });
  }

  function handleLegReset() {
    updateSimulator("leg3", (simulator) => {
      simulator.config = createDefaultLeg3Config();
      simulator.trajectory = { waypoints: [], trail: [] };
      simulator.presets.selectedId = LEG3_BUILT_IN_PRESETS[0].id;
    });
  }

  function buildPlanarCards() {
    const angleUnitText = unitLabel(planarConfig.angleUnit);
    const theta1Ui = angleToUi(planarState.activeTheta1, planarConfig.angleUnit);
    const theta2Ui = angleToUi(planarState.activeTheta2, planarConfig.angleUnit);
    const theta1LimitMinUi = angleToUi(planarConfig.jointLimits.theta1Min, planarConfig.angleUnit);
    const theta1LimitMaxUi = angleToUi(planarConfig.jointLimits.theta1Max, planarConfig.angleUnit);
    const theta2LimitMinUi = angleToUi(planarConfig.jointLimits.theta2Min, planarConfig.angleUnit);
    const theta2LimitMaxUi = angleToUi(planarConfig.jointLimits.theta2Max, planarConfig.angleUnit);
    const velocityRange = planarConfig.angleUnit === "deg" ? 180 : Math.PI;
    const workspaceExtent = Math.max(0.5, (planarConfig.L1 + planarConfig.L2) * 1.15);
    const builtIns = PLANAR2_BUILT_IN_PRESETS;
    const presetOptions = [...builtIns, ...planarPresets.custom];
    const activePreset =
      presetOptions.find((preset) => preset.id === planarPresets.selectedId) || presetOptions[0];

    return [
      {
        id: "ikfk-settings",
        title: "IK / FK Settings",
        subtitle: planarState.stateLabel,
        content: html`
          <div className="toggle-group">
            <${ToggleButton} active=${planarConfig.solverMode === "fk"} label="Forward Kinematics" onClick=${() => handlePlanarSolverModeChange("fk")} />
            <${ToggleButton} active=${planarConfig.solverMode === "ik"} label="Inverse Kinematics" onClick=${() => handlePlanarSolverModeChange("ik")} />
          </div>
          <div className="toggle-group" style=${{ marginTop: "10px" }}>
            <${ToggleButton} active=${planarConfig.elbowMode === "down"} label="Elbow Down" onClick=${() => updateSimulatorConfig("planar2", (previous) => ({ ...previous, elbowMode: "down" }))} />
            <${ToggleButton} active=${planarConfig.elbowMode === "up"} label="Elbow Up" onClick=${() => updateSimulatorConfig("planar2", (previous) => ({ ...previous, elbowMode: "up" }))} />
          </div>
        `
      },
      {
        id: "link-lengths",
        title: "Link Lengths",
        subtitle: "Robot geometry",
        content: html`
          <div className="control-grid">
            <${SliderNumberControl} label="Link L1" value=${planarConfig.L1} min=${0.2} max=${2.5} step=${0.01} displayValue=${`${formatNumber(planarConfig.L1)} m`} onChange=${(value) => updateSimulatorConfig("planar2", (previous) => ({ ...previous, L1: Math.max(0.2, sanitizeNumber(value, previous.L1)) }))} />
            <${SliderNumberControl} label="Link L2" value=${planarConfig.L2} min=${0.2} max=${2.5} step=${0.01} displayValue=${`${formatNumber(planarConfig.L2)} m`} onChange=${(value) => updateSimulatorConfig("planar2", (previous) => ({ ...previous, L2: Math.max(0.2, sanitizeNumber(value, previous.L2)) }))} />
          </div>
        `
      },
      {
        id: "joint-angles",
        title: "Joint Angles",
        subtitle: planarConfig.solverMode === "ik" ? "Solved pose display" : "Direct FK control",
        content: html`
          <div className="control-grid">
            <${SliderNumberControl} label="Theta 1" value=${theta1Ui} min=${theta1LimitMinUi} max=${theta1LimitMaxUi} step=${planarConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(theta1Ui, planarConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`} onChange=${(value) => updateSimulatorConfig("planar2", (previous) => ({ ...previous, theta1: angleFromUi(value, previous.angleUnit) }))} disabled=${planarConfig.solverMode === "ik"} />
            <${SliderNumberControl} label="Theta 2" value=${theta2Ui} min=${theta2LimitMinUi} max=${theta2LimitMaxUi} step=${planarConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(theta2Ui, planarConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`} onChange=${(value) => updateSimulatorConfig("planar2", (previous) => ({ ...previous, theta2: angleFromUi(value, previous.angleUnit) }))} disabled=${planarConfig.solverMode === "ik"} />
          </div>
        `
      },
      {
        id: "target-position",
        title: "Target Position",
        subtitle: "Direct workspace target",
        content: html`
          <div className="control-grid">
            <${SliderNumberControl} label="Target X" value=${planarConfig.targetX} min=${-workspaceExtent} max=${workspaceExtent} step=${0.01} displayValue=${`${formatNumber(planarConfig.targetX)} m`} onChange=${(value) => handlePlanarTargetChange({ x: value, y: planarConfig.targetY }, { immediate: true })} />
            <${SliderNumberControl} label="Target Y" value=${planarConfig.targetY} min=${-workspaceExtent} max=${workspaceExtent} step=${0.01} displayValue=${`${formatNumber(planarConfig.targetY)} m`} onChange=${(value) => handlePlanarTargetChange({ x: planarConfig.targetX, y: value }, { immediate: true })} />
          </div>
        `
      },
      {
        id: "units-display",
        title: "Units and Display",
        subtitle: "Engineering readout preferences",
        content: html`
          <div className="toggle-group">
            <${ToggleButton} active=${planarConfig.angleUnit === "deg"} label="Degrees" onClick=${() => updateSimulatorConfig("planar2", (previous) => ({ ...previous, angleUnit: "deg" }))} />
            <${ToggleButton} active=${planarConfig.angleUnit === "rad"} label="Radians" onClick=${() => updateSimulatorConfig("planar2", (previous) => ({ ...previous, angleUnit: "rad" }))} />
          </div>
          <div className="toggle-group" style=${{ marginTop: "10px" }}>
            <${ToggleButton} active=${planarConfig.traceEnabled} label="Trail On" onClick=${() => updateSimulatorConfig("planar2", (previous) => ({ ...previous, traceEnabled: true }))} />
            <${ToggleButton} active=${!planarConfig.traceEnabled} label="Trail Off" onClick=${() => updateSimulatorConfig("planar2", (previous) => ({ ...previous, traceEnabled: false }))} />
          </div>
          <div className="toggle-group" style=${{ marginTop: "10px" }}>
            <${ToggleButton} active=${planarConfig.animateTargetChanges} label="Animate Target" onClick=${() => updateSimulatorConfig("planar2", (previous) => ({ ...previous, animateTargetChanges: true }))} />
            <${ToggleButton} active=${!planarConfig.animateTargetChanges} label="Instant Target" onClick=${() => updateSimulatorConfig("planar2", (previous) => ({ ...previous, animateTargetChanges: false }))} />
          </div>
        `
      },
      {
        id: "joint-velocity",
        title: "Joint Velocity",
        subtitle: "Live Jacobian velocity input",
        content: html`
          <div className="control-grid">
            <${SliderNumberControl} label="Theta 1 Dot" value=${angleToUi(planarConfig.theta1Dot, planarConfig.angleUnit)} min=${-velocityRange} max=${velocityRange} step=${planarConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(angleToUi(planarConfig.theta1Dot, planarConfig.angleUnit), planarConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}/s`} onChange=${(value) => updateSimulatorConfig("planar2", (previous) => ({ ...previous, theta1Dot: angleFromUi(value, previous.angleUnit) }))} />
            <${SliderNumberControl} label="Theta 2 Dot" value=${angleToUi(planarConfig.theta2Dot, planarConfig.angleUnit)} min=${-velocityRange} max=${velocityRange} step=${planarConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(angleToUi(planarConfig.theta2Dot, planarConfig.angleUnit), planarConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}/s`} onChange=${(value) => updateSimulatorConfig("planar2", (previous) => ({ ...previous, theta2Dot: angleFromUi(value, previous.angleUnit) }))} />
          </div>
        `
      },
      {
        id: "joint-limits",
        title: "Joint Limits",
        subtitle: "Motion envelope",
        content: html`
          <div className="control-grid">
            <${SliderNumberControl} label="Theta 1 Min" value=${theta1LimitMinUi} min=${planarConfig.angleUnit === "deg" ? -180 : -Math.PI} max=${theta1LimitMaxUi} step=${planarConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(theta1LimitMinUi, planarConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`} onChange=${(value) => handlePlanarJointLimitChange("theta1Min", angleFromUi(value, planarConfig.angleUnit))} />
            <${SliderNumberControl} label="Theta 1 Max" value=${theta1LimitMaxUi} min=${theta1LimitMinUi} max=${planarConfig.angleUnit === "deg" ? 180 : Math.PI} step=${planarConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(theta1LimitMaxUi, planarConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`} onChange=${(value) => handlePlanarJointLimitChange("theta1Max", angleFromUi(value, planarConfig.angleUnit))} />
            <${SliderNumberControl} label="Theta 2 Min" value=${theta2LimitMinUi} min=${planarConfig.angleUnit === "deg" ? -180 : -Math.PI} max=${theta2LimitMaxUi} step=${planarConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(theta2LimitMinUi, planarConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`} onChange=${(value) => handlePlanarJointLimitChange("theta2Min", angleFromUi(value, planarConfig.angleUnit))} />
            <${SliderNumberControl} label="Theta 2 Max" value=${theta2LimitMaxUi} min=${theta2LimitMinUi} max=${planarConfig.angleUnit === "deg" ? 180 : Math.PI} step=${planarConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(theta2LimitMaxUi, planarConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`} onChange=${(value) => handlePlanarJointLimitChange("theta2Max", angleFromUi(value, planarConfig.angleUnit))} />
          </div>
        `
      },
      {
        id: "jacobian-info",
        title: "Jacobian / Singularity Info",
        subtitle: "Realtime conditioning feedback",
        content: html`
          <div className="info-grid">
            <div className="info-card">
              <strong>Determinant</strong>
              <span>${formatNumber(planarState.determinant, 6)}</span>
            </div>
            <div className="info-card">
              <strong>Reachability</strong>
              <span>${planarState.ik.reachable ? "Reachable" : "Warning"}</span>
            </div>
            <div className="info-card">
              <strong>Conditioning</strong>
              <span>${planarState.nearSingularity ? "Near singular" : "Healthy"}</span>
            </div>
            <div className="info-card">
              <strong>Target Error</strong>
              <span>${formatNumber(planarState.targetDistance, 4)} m</span>
            </div>
          </div>
        `
      },
      {
        id: "calibration",
        title: "Robot Calibration",
        subtitle: "Servo offsets and inversions",
        content: html`
          <div className="control-grid">
            <${SliderNumberControl} label="Joint 1 Zero Offset" value=${planarConfig.calibration.joint1OffsetDeg} min=${-180} max=${180} step=${1} displayValue=${`${formatNumber(planarConfig.calibration.joint1OffsetDeg, 1)} deg`} onChange=${(value) => updateSimulatorConfig("planar2", (previous) => ({ ...previous, calibration: { ...previous.calibration, joint1OffsetDeg: value } }))} />
            <${SliderNumberControl} label="Joint 2 Zero Offset" value=${planarConfig.calibration.joint2OffsetDeg} min=${-180} max=${180} step=${1} displayValue=${`${formatNumber(planarConfig.calibration.joint2OffsetDeg, 1)} deg`} onChange=${(value) => updateSimulatorConfig("planar2", (previous) => ({ ...previous, calibration: { ...previous.calibration, joint2OffsetDeg: value } }))} />
            <div className="toggle-group">
              <${ToggleButton} active=${!planarConfig.calibration.joint1Invert} label="Joint 1 Normal" onClick=${() => updateSimulatorConfig("planar2", (previous) => ({ ...previous, calibration: { ...previous.calibration, joint1Invert: false } }))} />
              <${ToggleButton} active=${planarConfig.calibration.joint1Invert} label="Joint 1 Inverted" onClick=${() => updateSimulatorConfig("planar2", (previous) => ({ ...previous, calibration: { ...previous.calibration, joint1Invert: true } }))} />
            </div>
            <div className="toggle-group">
              <${ToggleButton} active=${!planarConfig.calibration.joint2Invert} label="Joint 2 Normal" onClick=${() => updateSimulatorConfig("planar2", (previous) => ({ ...previous, calibration: { ...previous.calibration, joint2Invert: false } }))} />
              <${ToggleButton} active=${planarConfig.calibration.joint2Invert} label="Joint 2 Inverted" onClick=${() => updateSimulatorConfig("planar2", (previous) => ({ ...previous, calibration: { ...previous.calibration, joint2Invert: true } }))} />
            </div>
          </div>
        `
      },
      {
        id: "presets-motion",
        title: "Presets and Motion",
        subtitle: activePreset.label,
        content: html`
          <div className="control-grid">
            <div className="control-row">
              <label>
                <span>Preset</span>
                <span>${activePreset.label}</span>
              </label>
              <select value=${planarPresets.selectedId} onChange=${(event) => updateSimulator("planar2", (simulator) => { simulator.presets.selectedId = event.target.value; })}>
                ${presetOptions.map((preset) => html`<option value=${preset.id}>${preset.label}</option>`)}
              </select>
            </div>
            <div className="button-grid">
              <button type="button" className="primary" onClick=${() => handlePresetLoad("planar2")}>Load Preset</button>
              <button type="button" onClick=${() => handlePresetSave("planar2")}>Save Current</button>
              <button type="button" onClick=${() => handlePresetDelete("planar2")} disabled=${!planarPresets.selectedId.startsWith("custom-")}>Delete Custom</button>
              <button type="button" onClick=${handlePlanarReset}>Reset Arm</button>
            </div>
            <div className="button-grid">
              <button type="button" onClick=${() => updateSimulator("planar2", (simulator) => { simulator.trajectory.waypoints.push({ x: planarConfig.solverMode === "ik" ? planarConfig.targetX : planarState.fk.endEffector.x, y: planarConfig.solverMode === "ik" ? planarConfig.targetY : planarState.fk.endEffector.y }); })}>Add Waypoint</button>
              <button type="button" onClick=${() => {
                updateSimulator("planar2", (simulator) => { simulator.trajectory.waypoints = []; });
                setIsPlanarPathPlaying(false);
              }}>Clear Waypoints</button>
              <button type="button" className=${isPlanarPathPlaying ? "" : "primary"} onClick=${() => setIsPlanarPathPlaying((previous) => !previous)}>
                ${isPlanarPathPlaying ? "Stop Playback" : "Play Path"}
              </button>
              <button type="button" onClick=${() => updateSimulator("planar2", (simulator) => { simulator.trajectory.trail = []; })}>Clear Trail</button>
            </div>
            <div className="info-grid">
              <div className="info-card">
                <strong>Waypoints</strong>
                <span>${planarTrajectory.waypoints.length}</span>
              </div>
              <div className="info-card">
                <strong>Trail Samples</strong>
                <span>${planarTrajectory.trail.length}</span>
              </div>
            </div>
          </div>
        `
      }
    ];
  }

  function buildLegCards() {
    const angleUnitText = unitLabel(legConfig.angleUnit);
    const theta1Ui = angleToUi(legState.activeTheta1, legConfig.angleUnit);
    const theta2Ui = angleToUi(legState.activeTheta2, legConfig.angleUnit);
    const theta3Ui = angleToUi(legState.activeTheta3, legConfig.angleUnit);
    const limitValue = (key) => angleToUi(legConfig.jointLimits[key], legConfig.angleUnit);
    const builtIns = LEG3_BUILT_IN_PRESETS;
    const presetOptions = [...builtIns, ...legPresets.custom];
    const activePreset =
      presetOptions.find((preset) => preset.id === legPresets.selectedId) || presetOptions[0];
    const radiusExtent = Math.max(0.4, legConfig.L1 + legConfig.L2 + legConfig.L3);

    return [
      {
        id: "leg-settings",
        title: "Leg Mode Settings",
        subtitle: `${legState.stateLabel} · ${activePreset.label}`,
        content: html`
          <div className="toggle-group">
            <${ToggleButton} active=${legConfig.solverMode === "fk"} label="Direct Joint FK" onClick=${() => handleLegSolverModeChange("fk")} />
            <${ToggleButton} active=${legConfig.solverMode === "ik"} label="Foot Target IK" onClick=${() => handleLegSolverModeChange("ik")} />
          </div>
          <div className="toggle-group" style=${{ marginTop: "10px" }}>
            <${ToggleButton} active=${legConfig.kneeMode === "folded"} label="Folded Knee" onClick=${() => updateSimulatorConfig("leg3", (previous) => ({ ...previous, kneeMode: "folded" }))} />
            <${ToggleButton} active=${legConfig.kneeMode === "extended"} label="Extended Knee" onClick=${() => updateSimulatorConfig("leg3", (previous) => ({ ...previous, kneeMode: "extended" }))} />
          </div>
          <div className="button-grid" style=${{ marginTop: "10px" }}>
            <button type="button" className="primary" onClick=${() => handlePresetLoad("leg3")}>Load Preset</button>
            <button type="button" onClick=${() => handlePresetSave("leg3")}>Save Current</button>
            <button type="button" onClick=${() => handlePresetDelete("leg3")} disabled=${!legPresets.selectedId.startsWith("custom-")}>Delete Custom</button>
            <button type="button" onClick=${handleLegReset}>Reset Leg</button>
          </div>
        `
      },
      {
        id: "link-lengths",
        title: "Link Lengths",
        subtitle: "Coxa, upper leg, and lower leg",
        content: html`
          <div className="control-grid">
            <${SliderNumberControl} label="L1 Coxa" value=${legConfig.L1} min=${0.1} max=${1.5} step=${0.01} displayValue=${`${formatNumber(legConfig.L1)} m`} onChange=${(value) => updateSimulatorConfig("leg3", (previous) => ({ ...previous, L1: Math.max(0.1, sanitizeNumber(value, previous.L1)) }))} />
            <${SliderNumberControl} label="L2 Upper Leg" value=${legConfig.L2} min=${0.15} max=${1.8} step=${0.01} displayValue=${`${formatNumber(legConfig.L2)} m`} onChange=${(value) => updateSimulatorConfig("leg3", (previous) => ({ ...previous, L2: Math.max(0.15, sanitizeNumber(value, previous.L2)) }))} />
            <${SliderNumberControl} label="L3 Lower Leg" value=${legConfig.L3} min=${0.15} max=${1.8} step=${0.01} displayValue=${`${formatNumber(legConfig.L3)} m`} onChange=${(value) => updateSimulatorConfig("leg3", (previous) => ({ ...previous, L3: Math.max(0.15, sanitizeNumber(value, previous.L3)) }))} />
          </div>
        `
      },
      {
        id: "joint-angles",
        title: "Joint Angles",
        subtitle: legConfig.solverMode === "ik" ? "Solved leg pose" : "Direct leg joint control",
        content: html`
          <div className="control-grid">
            <${SliderNumberControl} label="Theta 1 Hip Yaw" value=${theta1Ui} min=${limitValue("theta1Min")} max=${limitValue("theta1Max")} step=${legConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(theta1Ui, legConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`} onChange=${(value) => updateSimulatorConfig("leg3", (previous) => ({ ...previous, theta1: angleFromUi(value, previous.angleUnit) }))} disabled=${legConfig.solverMode === "ik"} />
            <${SliderNumberControl} label="Theta 2 Hip Pitch" value=${theta2Ui} min=${limitValue("theta2Min")} max=${limitValue("theta2Max")} step=${legConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(theta2Ui, legConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`} onChange=${(value) => updateSimulatorConfig("leg3", (previous) => ({ ...previous, theta2: angleFromUi(value, previous.angleUnit) }))} disabled=${legConfig.solverMode === "ik"} />
            <${SliderNumberControl} label="Theta 3 Knee" value=${theta3Ui} min=${limitValue("theta3Min")} max=${limitValue("theta3Max")} step=${legConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(theta3Ui, legConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`} onChange=${(value) => updateSimulatorConfig("leg3", (previous) => ({ ...previous, theta3: angleFromUi(value, previous.angleUnit) }))} disabled=${legConfig.solverMode === "ik"} />
          </div>
        `
      },
      {
        id: "foot-target",
        title: "Foot Target",
        subtitle: "Optional target point for 3 DOF IK",
        content: html`
          <div className="toggle-group">
            <${ToggleButton} active=${legConfig.footTargetEnabled} label="Foot Target On" onClick=${() => updateSimulatorConfig("leg3", (previous) => ({ ...previous, footTargetEnabled: true }))} />
            <${ToggleButton} active=${!legConfig.footTargetEnabled} label="Foot Target Off" onClick=${() => updateSimulatorConfig("leg3", (previous) => ({ ...previous, footTargetEnabled: false }))} />
          </div>
          <div className="control-grid" style=${{ marginTop: "10px" }}>
            <${SliderNumberControl} label="Target X" value=${legConfig.targetX} min=${-radiusExtent} max=${radiusExtent} step=${0.01} displayValue=${`${formatNumber(legConfig.targetX)} m`} onChange=${(value) => handleLegTargetChange({ x: value, y: legConfig.targetY, z: legConfig.targetZ })} />
            <${SliderNumberControl} label="Target Y" value=${legConfig.targetY} min=${-radiusExtent} max=${radiusExtent} step=${0.01} displayValue=${`${formatNumber(legConfig.targetY)} m`} onChange=${(value) => handleLegTargetChange({ x: legConfig.targetX, y: value, z: legConfig.targetZ })} />
            <${SliderNumberControl} label="Target Z" value=${legConfig.targetZ} min=${-radiusExtent} max=${radiusExtent} step=${0.01} displayValue=${`${formatNumber(legConfig.targetZ)} m`} onChange=${(value) => handleLegTargetChange({ x: legConfig.targetX, y: legConfig.targetY, z: value })} />
          </div>
        `
      },
      {
        id: "units-display",
        title: "Units and Display",
        subtitle: "Leg dashboard display preferences",
        content: html`
          <div className="toggle-group">
            <${ToggleButton} active=${legConfig.angleUnit === "deg"} label="Degrees" onClick=${() => updateSimulatorConfig("leg3", (previous) => ({ ...previous, angleUnit: "deg" }))} />
            <${ToggleButton} active=${legConfig.angleUnit === "rad"} label="Radians" onClick=${() => updateSimulatorConfig("leg3", (previous) => ({ ...previous, angleUnit: "rad" }))} />
          </div>
          <div className="toggle-group" style=${{ marginTop: "10px" }}>
            <${ToggleButton} active=${legConfig.show3dPreview} label="3D Preview On" onClick=${() => updateSimulatorConfig("leg3", (previous) => ({ ...previous, show3dPreview: true }))} />
            <${ToggleButton} active=${!legConfig.show3dPreview} label="3D Preview Off" onClick=${() => updateSimulatorConfig("leg3", (previous) => ({ ...previous, show3dPreview: false }))} />
          </div>
        `
      },
      {
        id: "joint-limits",
        title: "Joint Limits",
        subtitle: "Leg motion envelope",
        content: html`
          <div className="control-grid">
            <${SliderNumberControl} label="Yaw Min" value=${limitValue("theta1Min")} min=${legConfig.angleUnit === "deg" ? -180 : -Math.PI} max=${limitValue("theta1Max")} step=${legConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(limitValue("theta1Min"), legConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`} onChange=${(value) => handleLegJointLimitChange("theta1Min", angleFromUi(value, legConfig.angleUnit))} />
            <${SliderNumberControl} label="Yaw Max" value=${limitValue("theta1Max")} min=${limitValue("theta1Min")} max=${legConfig.angleUnit === "deg" ? 180 : Math.PI} step=${legConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(limitValue("theta1Max"), legConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`} onChange=${(value) => handleLegJointLimitChange("theta1Max", angleFromUi(value, legConfig.angleUnit))} />
            <${SliderNumberControl} label="Hip Min" value=${limitValue("theta2Min")} min=${legConfig.angleUnit === "deg" ? -180 : -Math.PI} max=${limitValue("theta2Max")} step=${legConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(limitValue("theta2Min"), legConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`} onChange=${(value) => handleLegJointLimitChange("theta2Min", angleFromUi(value, legConfig.angleUnit))} />
            <${SliderNumberControl} label="Hip Max" value=${limitValue("theta2Max")} min=${limitValue("theta2Min")} max=${legConfig.angleUnit === "deg" ? 180 : Math.PI} step=${legConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(limitValue("theta2Max"), legConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`} onChange=${(value) => handleLegJointLimitChange("theta2Max", angleFromUi(value, legConfig.angleUnit))} />
            <${SliderNumberControl} label="Knee Min" value=${limitValue("theta3Min")} min=${legConfig.angleUnit === "deg" ? -180 : -Math.PI} max=${limitValue("theta3Max")} step=${legConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(limitValue("theta3Min"), legConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`} onChange=${(value) => handleLegJointLimitChange("theta3Min", angleFromUi(value, legConfig.angleUnit))} />
            <${SliderNumberControl} label="Knee Max" value=${limitValue("theta3Max")} min=${limitValue("theta3Min")} max=${legConfig.angleUnit === "deg" ? 180 : Math.PI} step=${legConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(limitValue("theta3Max"), legConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`} onChange=${(value) => handleLegJointLimitChange("theta3Max", angleFromUi(value, legConfig.angleUnit))} />
          </div>
        `
      },
      {
        id: "jacobian-info",
        title: "Jacobian / Singularity Info",
        subtitle: "Leg reachability diagnostics",
        content: html`
          <div className="info-grid">
            <div className="info-card">
              <strong>Determinant</strong>
              <span>${formatNumber(legState.determinant, 6)}</span>
            </div>
            <div className="info-card">
              <strong>Reachability</strong>
              <span>${legState.ik.reachable ? "Reachable" : "Warning"}</span>
            </div>
            <div className="info-card">
              <strong>Plane Reach</strong>
              <span>${formatNumber(legState.ik.planeRadius, 4)} m</span>
            </div>
            <div className="info-card">
              <strong>Target Error</strong>
              <span>${formatNumber(legState.targetDistance, 4)} m</span>
            </div>
          </div>
        `
      },
      {
        id: "calibration",
        title: "Robot Calibration",
        subtitle: "Per-joint zero offsets",
        content: html`
          <div className="control-grid">
            <${SliderNumberControl} label="Joint 1 Offset" value=${legConfig.calibration.joint1OffsetDeg} min=${-180} max=${180} step=${1} displayValue=${`${formatNumber(legConfig.calibration.joint1OffsetDeg, 1)} deg`} onChange=${(value) => updateSimulatorConfig("leg3", (previous) => ({ ...previous, calibration: { ...previous.calibration, joint1OffsetDeg: value } }))} />
            <${SliderNumberControl} label="Joint 2 Offset" value=${legConfig.calibration.joint2OffsetDeg} min=${-180} max=${180} step=${1} displayValue=${`${formatNumber(legConfig.calibration.joint2OffsetDeg, 1)} deg`} onChange=${(value) => updateSimulatorConfig("leg3", (previous) => ({ ...previous, calibration: { ...previous.calibration, joint2OffsetDeg: value } }))} />
            <${SliderNumberControl} label="Joint 3 Offset" value=${legConfig.calibration.joint3OffsetDeg} min=${-180} max=${180} step=${1} displayValue=${`${formatNumber(legConfig.calibration.joint3OffsetDeg, 1)} deg`} onChange=${(value) => updateSimulatorConfig("leg3", (previous) => ({ ...previous, calibration: { ...previous.calibration, joint3OffsetDeg: value } }))} />
            <div className="toggle-group">
              <${ToggleButton} active=${!legConfig.calibration.joint1Invert} label="Yaw Normal" onClick=${() => updateSimulatorConfig("leg3", (previous) => ({ ...previous, calibration: { ...previous.calibration, joint1Invert: false } }))} />
              <${ToggleButton} active=${legConfig.calibration.joint1Invert} label="Yaw Inverted" onClick=${() => updateSimulatorConfig("leg3", (previous) => ({ ...previous, calibration: { ...previous.calibration, joint1Invert: true } }))} />
            </div>
            <div className="toggle-group">
              <${ToggleButton} active=${!legConfig.calibration.joint2Invert} label="Hip Normal" onClick=${() => updateSimulatorConfig("leg3", (previous) => ({ ...previous, calibration: { ...previous.calibration, joint2Invert: false } }))} />
              <${ToggleButton} active=${legConfig.calibration.joint2Invert} label="Hip Inverted" onClick=${() => updateSimulatorConfig("leg3", (previous) => ({ ...previous, calibration: { ...previous.calibration, joint2Invert: true } }))} />
            </div>
            <div className="toggle-group">
              <${ToggleButton} active=${!legConfig.calibration.joint3Invert} label="Knee Normal" onClick=${() => updateSimulatorConfig("leg3", (previous) => ({ ...previous, calibration: { ...previous.calibration, joint3Invert: false } }))} />
              <${ToggleButton} active=${legConfig.calibration.joint3Invert} label="Knee Inverted" onClick=${() => updateSimulatorConfig("leg3", (previous) => ({ ...previous, calibration: { ...previous.calibration, joint3Invert: true } }))} />
            </div>
          </div>
        `
      },
      {
        id: "leg-view",
        title: "Leg View Controls",
        subtitle: "3D preview and per-joint velocity",
        content: html`
          <div className="control-grid">
            <${SliderNumberControl} label="Yaw Rate" value=${angleToUi(legConfig.theta1Dot, legConfig.angleUnit)} min=${-180} max=${180} step=${legConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(angleToUi(legConfig.theta1Dot, legConfig.angleUnit), legConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}/s`} onChange=${(value) => updateSimulatorConfig("leg3", (previous) => ({ ...previous, theta1Dot: angleFromUi(value, previous.angleUnit) }))} />
            <${SliderNumberControl} label="Hip Rate" value=${angleToUi(legConfig.theta2Dot, legConfig.angleUnit)} min=${-180} max=${180} step=${legConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(angleToUi(legConfig.theta2Dot, legConfig.angleUnit), legConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}/s`} onChange=${(value) => updateSimulatorConfig("leg3", (previous) => ({ ...previous, theta2Dot: angleFromUi(value, previous.angleUnit) }))} />
            <${SliderNumberControl} label="Knee Rate" value=${angleToUi(legConfig.theta3Dot, legConfig.angleUnit)} min=${-180} max=${180} step=${legConfig.angleUnit === "deg" ? 1 : 0.01} displayValue=${`${formatNumber(angleToUi(legConfig.theta3Dot, legConfig.angleUnit), legConfig.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}/s`} onChange=${(value) => updateSimulatorConfig("leg3", (previous) => ({ ...previous, theta3Dot: angleFromUi(value, previous.angleUnit) }))} />
            <div className="control-row">
              <label>
                <span>Preset</span>
                <span>${activePreset.label}</span>
              </label>
              <select value=${legPresets.selectedId} onChange=${(event) => updateSimulator("leg3", (simulator) => { simulator.presets.selectedId = event.target.value; })}>
                ${presetOptions.map((preset) => html`<option value=${preset.id}>${preset.label}</option>`)}
              </select>
            </div>
          </div>
        `
      }
    ];
  }

  const activeCards = activeMode === "leg3" ? buildLegCards() : buildPlanarCards();
  const activeOrder = appState.simulators[activeMode].ui.moduleOrder;
  const heroBadges =
    activeMode === "leg3"
      ? [
          { label: legConfig.solverMode === "ik" ? "Leg IK Active" : "Leg FK Active", warn: false },
          { label: legState.ik.reachable ? "Foot Reachable" : "Foot Warning", warn: !legState.ik.reachable },
          { label: legState.nearSingularity ? "Near Singularity" : "Stable Jacobian", warn: legState.nearSingularity },
          { label: legState.jointLimitStatus.every(Boolean) ? "Within Limits" : "Limit Violation", warn: !legState.jointLimitStatus.every(Boolean) }
        ]
      : [
          { label: planarConfig.solverMode === "ik" ? "IK Mode" : "FK Mode", warn: false },
          { label: planarState.ik.reachable ? "Target Reachable" : "Target Unreachable", warn: !planarState.ik.reachable },
          { label: planarState.nearSingularity ? "Near Singularity" : "Jacobian Healthy", warn: planarState.nearSingularity },
          { label: planarState.jointLimitStatus.every(Boolean) ? "Within Limits" : "Joint Limit Violation", warn: !planarState.jointLimitStatus.every(Boolean) }
        ];

  return html`
    <div className="app-shell">
      <${DraggableControlPanel}
        cards=${activeCards}
        order=${activeOrder}
        onReorder=${(nextOrder) =>
          updateSimulator(activeMode, (simulator) => {
            simulator.ui.moduleOrder = nextOrder;
          })}
      />

      <main className="workspace">
        <section className="panel hero">
          <div className="hero-copy">
            <div className="hero-kicker">Modular Robotics Simulator Dashboard</div>
            <h1>${activeMode === "leg3" ? "3 DOF Leg Simulator" : "2 DOF Planar Robot Arm Simulator"}</h1>
            <p>
              ${activeMode === "leg3"
                ? "Engineering workspace for yaw-plus-sagittal leg design with draggable controls, symbolic kinematics, top and side views, and export-ready parameters for future quadruped or hexapod integration."
                : "Educational robotics workspace with closed-form IK, Jacobian analysis, draggable control modules, path playback, and export-ready parameters for arm prototyping."}
            </p>
          </div>
          <div className="hero-tools">
            <${ModeSwitcher} activeMode=${activeMode} onChange=${setActiveMode} />
            <div className="hero-badges">
              ${heroBadges.map(
                (badge) => html`
                  <span key=${badge.label} className=${badge.warn ? "badge warn" : "badge"}>${badge.label}</span>
                `
              )}
            </div>
          </div>
        </section>

        <section className="panel page-switch">
          <div className="toggle-group">
            <button type="button" className=${appState.activeWorkspacePage === "analysis" ? "active" : ""} onClick=${() => setAppState((previous) => ({ ...previous, activeWorkspacePage: "analysis" }))}>
              Simulation Workspace
            </button>
            <button type="button" className=${appState.activeWorkspacePage === "integration" ? "active" : ""} onClick=${() => setAppState((previous) => ({ ...previous, activeWorkspacePage: "integration" }))}>
              State and Export
            </button>
          </div>
        </section>

        <section className="page-content">
          ${appState.activeWorkspacePage === "analysis"
            ? html`
                <${RobotCanvas}
                  mode=${activeMode}
                  config=${activeConfig}
                  robotState=${activeRobotState}
                  trajectory=${activeTrajectory}
                  onTargetChange=${activeMode === "leg3" ? handleLegTargetChange : handlePlanarTargetChange}
                  onCameraChange=${(key, value) =>
                    updateSimulatorConfig("leg3", (previous) => ({ ...previous, [key]: value }))}
                />
                <${EquationsPanel} mode=${activeMode} config=${activeConfig} robotState=${activeRobotState} />
              `
            : html`
                <${StatePanel} mode=${activeMode} config=${activeConfig} robotState=${activeRobotState} trajectory=${activeTrajectory} />
              `}
        </section>
      </main>
    </div>
  `;
}
