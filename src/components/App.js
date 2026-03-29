import { html, useEffect, useMemo, useRef, useState } from "../lib.js";
import { BUILT_IN_PRESETS, DEFAULT_CONFIG } from "../constants.js";
import { ControlsPanel } from "./ControlsPanel.js";
import { EquationsPanel } from "./EquationsPanel.js";
import { StatePanel } from "./StatePanel.js";
import { VisualizationPanel } from "./VisualizationPanel.js";
import { deriveRobotState } from "../utils/kinematics.js";
import { distance, lerp, sanitizeNumber } from "../utils/math.js";
import { loadPresets, loadSavedState, savePresets, saveState } from "../utils/storage.js";

function cloneConfig(config) {
  return typeof structuredClone === "function"
    ? structuredClone(config)
    : JSON.parse(JSON.stringify(config));
}

function limitTrail(nextTrail) {
  return nextTrail.slice(-1200);
}

export function App() {
  const [config, setConfig] = useState(() => loadSavedState());
  const [customPresets, setCustomPresets] = useState(() => loadPresets());
  const [selectedPresetId, setSelectedPresetId] = useState(BUILT_IN_PRESETS[0].id);
  const [activeWorkspacePage, setActiveWorkspacePage] = useState("analysis");
  const [waypoints, setWaypoints] = useState([]);
  const [trail, setTrail] = useState([]);
  const [isPlaying, setIsPlaying] = useState(false);
  const targetAnimationRef = useRef(null);

  const robotState = useMemo(() => deriveRobotState(config), [config]);

  useEffect(() => {
    saveState(config);
  }, [config]);

  useEffect(() => {
    savePresets(customPresets);
  }, [customPresets]);

  function cancelTargetAnimation() {
    if (targetAnimationRef.current) {
      cancelAnimationFrame(targetAnimationRef.current);
      targetAnimationRef.current = null;
    }
  }

  function appendTrailPoint(point) {
    setTrail((previous) => {
      const sample = {
        x: Number(point.x.toFixed(4)),
        y: Number(point.y.toFixed(4))
      };
      const last = previous[previous.length - 1];
      if (last && distance(last, sample) < 0.015) {
        return previous;
      }
      return limitTrail([...previous, sample]);
    });
  }

  useEffect(() => {
    if (config.traceEnabled) {
      appendTrailPoint(robotState.fk.endEffector);
    }
  }, [
    config.traceEnabled,
    robotState.fk.endEffector.x,
    robotState.fk.endEffector.y
  ]);

  function setTargetInstant(x, y) {
    setConfig((previous) => ({
      ...previous,
      mode: "ik",
      targetX: sanitizeNumber(x, previous.targetX),
      targetY: sanitizeNumber(y, previous.targetY)
    }));
  }

  function animateTargetTo(x, y) {
    cancelTargetAnimation();
    const start =
      config.mode === "ik"
        ? { x: config.targetX, y: config.targetY }
        : robotState.fk.endEffector;
    const end = { x, y };
    const durationMs = 420;
    const startedAt = performance.now();

    function step(now) {
      const t = Math.min(1, (now - startedAt) / durationMs);
      const eased = 1 - Math.pow(1 - t, 3);
      const nextPoint = {
        x: lerp(start.x, end.x, eased),
        y: lerp(start.y, end.y, eased)
      };

      setConfig((previous) => ({
        ...previous,
        mode: "ik",
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
  }

  function handleTargetChange(x, y, options = {}) {
    setIsPlaying(false);
    if (config.animateTargetChanges && !options.immediate) {
      animateTargetTo(x, y);
      return;
    }

    cancelTargetAnimation();
    setTargetInstant(x, y);
  }

  function handleModeChange(nextMode) {
    if (nextMode === config.mode) {
      return;
    }

    setIsPlaying(false);
    cancelTargetAnimation();
    setConfig((previous) => {
      if (nextMode === "ik") {
        return {
          ...previous,
          mode: "ik",
          theta1: robotState.activeTheta1,
          theta2: robotState.activeTheta2,
          targetX: robotState.fk.endEffector.x,
          targetY: robotState.fk.endEffector.y
        };
      }

      return {
        ...previous,
        mode: "fk",
        theta1: robotState.activeTheta1,
        theta2: robotState.activeTheta2
      };
    });
  }

  function handleLengthChange(key, value) {
    setConfig((previous) => ({
      ...previous,
      [key]: Math.max(0.2, sanitizeNumber(value, previous[key]))
    }));
  }

  function handleAngleChange(key, value) {
    setConfig((previous) => ({
      ...previous,
      [key]: sanitizeNumber(value, previous[key])
    }));
  }

  function handleVelocityChange(key, value) {
    setConfig((previous) => ({
      ...previous,
      [key]: sanitizeNumber(value, previous[key])
    }));
  }

  function handleJointLimitChange(key, value) {
    setConfig((previous) => {
      const next = {
        ...previous,
        jointLimits: {
          ...previous.jointLimits,
          [key]: sanitizeNumber(value, previous.jointLimits[key])
        }
      };

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

  function handleCalibrationChange(key, value) {
    setConfig((previous) => ({
      ...previous,
      calibration: {
        ...previous.calibration,
        [key]: value
      }
    }));
  }

  function handleToggleFlag(key, value) {
    setConfig((previous) => ({
      ...previous,
      [key]: value
    }));
  }

  function handleReset() {
    setIsPlaying(false);
    cancelTargetAnimation();
    setConfig(cloneConfig(DEFAULT_CONFIG));
    setWaypoints([]);
    setTrail([]);
    setSelectedPresetId(BUILT_IN_PRESETS[0].id);
  }

  function handlePresetLoad() {
    const preset = [...BUILT_IN_PRESETS, ...customPresets].find((entry) => entry.id === selectedPresetId);
    if (!preset) {
      return;
    }

    setIsPlaying(false);
    cancelTargetAnimation();
    setConfig(cloneConfig(preset.config));
  }

  function handlePresetSave() {
    const label = window.prompt("Preset name", `Custom Preset ${customPresets.length + 1}`);
    if (!label) {
      return;
    }

    const nextPreset = {
      id: `custom-${Date.now()}`,
      label: label.trim(),
      config: cloneConfig(config)
    };

    setCustomPresets((previous) => [...previous, nextPreset]);
    setSelectedPresetId(nextPreset.id);
  }

  function handlePresetDelete() {
    if (!selectedPresetId.startsWith("custom-")) {
      return;
    }

    setCustomPresets((previous) => previous.filter((preset) => preset.id !== selectedPresetId));
    setSelectedPresetId(BUILT_IN_PRESETS[0].id);
  }

  function handleAddWaypoint() {
    const point =
      config.mode === "ik"
        ? { x: config.targetX, y: config.targetY }
        : robotState.fk.endEffector;
    setWaypoints((previous) => [...previous, point]);
  }

  function handleClearWaypoints() {
    setWaypoints([]);
    setIsPlaying(false);
  }

  function handleClearTrail() {
    setTrail([]);
  }

  useEffect(() => {
    if (!isPlaying) {
      return undefined;
    }

    if (waypoints.length < 2) {
      setIsPlaying(false);
      return undefined;
    }

    cancelTargetAnimation();

    const segments = waypoints.slice(1).map((point, index) => ({
      start: waypoints[index],
      end: point,
      length: distance(waypoints[index], point)
    }));
    const totalLength = segments.reduce((sum, segment) => sum + segment.length, 0);
    if (totalLength <= 1e-9) {
      setIsPlaying(false);
      return undefined;
    }

    const speed = Math.max(0.3, (config.L1 + config.L2) * 0.35);
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
      const point = {
        x: lerp(activeSegment.start.x, activeSegment.end.x, t),
        y: lerp(activeSegment.start.y, activeSegment.end.y, t)
      };

      setConfig((previous) => ({
        ...previous,
        mode: "ik",
        targetX: point.x,
        targetY: point.y
      }));

      if (travelled >= totalLength) {
        setIsPlaying(false);
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
  }, [config.L1, config.L2, config.traceEnabled, isPlaying, waypoints]);

  const heroBadges = [
    {
      label: config.mode === "ik" ? "IK Mode" : "FK Mode",
      warn: false
    },
    {
      label: robotState.ik.reachable ? "Target Reachable" : "Target Unreachable",
      warn: !robotState.ik.reachable
    },
    {
      label: robotState.nearSingularity ? "Near Singularity" : "Jacobian Healthy",
      warn: robotState.nearSingularity
    },
    {
      label:
        robotState.theta1WithinLimits && robotState.theta2WithinLimits
          ? "Within Limits"
          : "Joint Limit Violation",
      warn: !(robotState.theta1WithinLimits && robotState.theta2WithinLimits)
    }
  ];

  return html`
    <div className="app-shell">
      <${ControlsPanel}
        config=${config}
        robotState=${robotState}
        customPresets=${customPresets}
        selectedPresetId=${selectedPresetId}
        onPresetSelection=${setSelectedPresetId}
        onPresetLoad=${handlePresetLoad}
        onPresetSave=${handlePresetSave}
        onPresetDelete=${handlePresetDelete}
        onModeChange=${handleModeChange}
        onAngleUnitChange=${(angleUnit) => handleToggleFlag("angleUnit", angleUnit)}
        onElbowModeChange=${(elbowMode) => handleToggleFlag("elbowMode", elbowMode)}
        onLengthChange=${handleLengthChange}
        onAngleChange=${handleAngleChange}
        onVelocityChange=${handleVelocityChange}
        onTargetChange=${handleTargetChange}
        onJointLimitChange=${handleJointLimitChange}
        onCalibrationChange=${handleCalibrationChange}
        onToggleFlag=${handleToggleFlag}
        onReset=${handleReset}
        onAddWaypoint=${handleAddWaypoint}
        onClearWaypoints=${handleClearWaypoints}
        onPlayPath=${() => setIsPlaying(true)}
        onStopPath=${() => setIsPlaying(false)}
        onClearTrail=${handleClearTrail}
        isPlaying=${isPlaying}
        waypointCount=${waypoints.length}
        trailCount=${trail.length}
      />

      <main className="workspace">
        <section className="panel hero">
          <div>
            <h1>2 DOF Planar Robot Arm Simulator</h1>
            <p>
              Educational kinematics dashboard with closed-form IK, Jacobian analysis,
              workspace inspection, path playback, and export-ready parameters for Arduino,
              ESP32, or Python control code.
            </p>
          </div>
          <div className="hero-badges">
            ${heroBadges.map(
              (badge) => html`
                <span key=${badge.label} className=${badge.warn ? "badge warn" : "badge"}>
                  ${badge.label}
                </span>
              `
            )}
          </div>
        </section>

        <section className="panel page-switch">
          <div className="toggle-group">
            <button
              type="button"
              className=${activeWorkspacePage === "analysis" ? "active" : ""}
              onClick=${() => setActiveWorkspacePage("analysis")}
            >
              Workspace and Math
            </button>
            <button
              type="button"
              className=${activeWorkspacePage === "integration" ? "active" : ""}
              onClick=${() => setActiveWorkspacePage("integration")}
            >
              Robot State and Export
            </button>
          </div>
        </section>

        <section className="page-content">
          ${activeWorkspacePage === "analysis"
            ? html`
                <${VisualizationPanel}
                  config=${config}
                  robotState=${robotState}
                  waypoints=${waypoints}
                  trail=${trail}
                  onTargetChange=${handleTargetChange}
                />
                <${EquationsPanel} config=${config} robotState=${robotState} />
              `
            : html`
                <${StatePanel}
                  config=${config}
                  robotState=${robotState}
                  waypoints=${waypoints}
                />
              `}
        </section>
      </main>
    </div>
  `;
}
