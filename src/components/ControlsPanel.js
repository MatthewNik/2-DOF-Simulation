import { html } from "../lib.js";
import { BUILT_IN_PRESETS } from "../constants.js";
import { formatNumber, radToDeg, degToRad } from "../utils/math.js";

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

export function ControlsPanel({
  config,
  robotState,
  customPresets,
  selectedPresetId,
  onPresetSelection,
  onPresetLoad,
  onPresetSave,
  onPresetDelete,
  onModeChange,
  onAngleUnitChange,
  onElbowModeChange,
  onLengthChange,
  onAngleChange,
  onVelocityChange,
  onTargetChange,
  onJointLimitChange,
  onCalibrationChange,
  onToggleFlag,
  onReset,
  onAddWaypoint,
  onClearWaypoints,
  onPlayPath,
  onStopPath,
  onClearTrail,
  isPlaying,
  waypointCount,
  trailCount
}) {
  const activePreset =
    [...BUILT_IN_PRESETS, ...customPresets].find((preset) => preset.id === selectedPresetId) ||
    BUILT_IN_PRESETS[0];
  const workspaceExtent = Math.max(0.5, (config.L1 + config.L2) * 1.15);
  const angleUnitText = unitLabel(config.angleUnit);
  const theta1Ui = angleToUi(robotState.activeTheta1, config.angleUnit);
  const theta2Ui = angleToUi(robotState.activeTheta2, config.angleUnit);
  const theta1LimitMinUi = angleToUi(config.jointLimits.theta1Min, config.angleUnit);
  const theta1LimitMaxUi = angleToUi(config.jointLimits.theta1Max, config.angleUnit);
  const theta2LimitMinUi = angleToUi(config.jointLimits.theta2Min, config.angleUnit);
  const theta2LimitMaxUi = angleToUi(config.jointLimits.theta2Max, config.angleUnit);
  const velocityRange = config.angleUnit === "deg" ? 180 : Math.PI;

  return html`
    <aside className="panel sidebar">
      <div className="section">
        <div className="section-header">
          <h2>Controls</h2>
          <small>${robotState.stateLabel}</small>
        </div>
        <div className="toggle-group">
          <${ToggleButton}
            active=${config.mode === "fk"}
            label="Forward Kinematics"
            onClick=${() => onModeChange("fk")}
          />
          <${ToggleButton}
            active=${config.mode === "ik"}
            label="Inverse Kinematics"
            onClick=${() => onModeChange("ik")}
          />
        </div>
        <div className="toggle-group" style=${{ marginTop: "10px" }}>
          <${ToggleButton}
            active=${config.angleUnit === "deg"}
            label="Degrees"
            onClick=${() => onAngleUnitChange("deg")}
          />
          <${ToggleButton}
            active=${config.angleUnit === "rad"}
            label="Radians"
            onClick=${() => onAngleUnitChange("rad")}
          />
        </div>
        <div className="toggle-group" style=${{ marginTop: "10px" }}>
          <${ToggleButton}
            active=${config.elbowMode === "down"}
            label="Elbow Down"
            onClick=${() => onElbowModeChange("down")}
          />
          <${ToggleButton}
            active=${config.elbowMode === "up"}
            label="Elbow Up"
            onClick=${() => onElbowModeChange("up")}
          />
        </div>
      </div>

      <div className="section">
        <h3>Robot Geometry</h3>
        <div className="control-grid">
          <${SliderNumberControl}
            label="Link L1"
            value=${config.L1}
            min=${0.2}
            max=${2.5}
            step=${0.01}
            displayValue=${`${formatNumber(config.L1)} m`}
            onChange=${(value) => onLengthChange("L1", value)}
          />
          <${SliderNumberControl}
            label="Link L2"
            value=${config.L2}
            min=${0.2}
            max=${2.5}
            step=${0.01}
            displayValue=${`${formatNumber(config.L2)} m`}
            onChange=${(value) => onLengthChange("L2", value)}
          />
        </div>
      </div>

      <div className="section">
        <h3>Joint Angles</h3>
        <div className="control-grid">
          <${SliderNumberControl}
            label="Theta 1"
            value=${theta1Ui}
            min=${theta1LimitMinUi}
            max=${theta1LimitMaxUi}
            step=${config.angleUnit === "deg" ? 1 : 0.01}
            displayValue=${`${formatNumber(theta1Ui, config.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`}
            onChange=${(value) => onAngleChange("theta1", angleFromUi(value, config.angleUnit))}
            disabled=${config.mode === "ik"}
          />
          <${SliderNumberControl}
            label="Theta 2"
            value=${theta2Ui}
            min=${theta2LimitMinUi}
            max=${theta2LimitMaxUi}
            step=${config.angleUnit === "deg" ? 1 : 0.01}
            displayValue=${`${formatNumber(theta2Ui, config.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`}
            onChange=${(value) => onAngleChange("theta2", angleFromUi(value, config.angleUnit))}
            disabled=${config.mode === "ik"}
          />
        </div>
        <p className="footer-note">
          In IK mode these fields display the solved pose. In FK mode they are directly editable.
        </p>
      </div>

      <div className="section">
        <h3>Target Point</h3>
        <div className="control-grid">
          <${SliderNumberControl}
            label="Target X"
            value=${config.targetX}
            min=${-workspaceExtent}
            max=${workspaceExtent}
            step=${0.01}
            displayValue=${`${formatNumber(config.targetX)} m`}
            onChange=${(value) => onTargetChange(value, config.targetY)}
          />
          <${SliderNumberControl}
            label="Target Y"
            value=${config.targetY}
            min=${-workspaceExtent}
            max=${workspaceExtent}
            step=${0.01}
            displayValue=${`${formatNumber(config.targetY)} m`}
            onChange=${(value) => onTargetChange(config.targetX, value)}
          />
        </div>
        <p className="footer-note">
          Click or drag directly in the workspace to move the IK target. Target changes automatically switch to IK mode.
        </p>
      </div>

      <div className="section">
        <h3>Joint Velocity</h3>
        <div className="control-grid">
          <${SliderNumberControl}
            label="Theta 1 Dot"
            value=${angleToUi(config.theta1Dot, config.angleUnit)}
            min=${-velocityRange}
            max=${velocityRange}
            step=${config.angleUnit === "deg" ? 1 : 0.01}
            displayValue=${`${formatNumber(angleToUi(config.theta1Dot, config.angleUnit), config.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}/s`}
            onChange=${(value) => onVelocityChange("theta1Dot", angleFromUi(value, config.angleUnit))}
          />
          <${SliderNumberControl}
            label="Theta 2 Dot"
            value=${angleToUi(config.theta2Dot, config.angleUnit)}
            min=${-velocityRange}
            max=${velocityRange}
            step=${config.angleUnit === "deg" ? 1 : 0.01}
            displayValue=${`${formatNumber(angleToUi(config.theta2Dot, config.angleUnit), config.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}/s`}
            onChange=${(value) => onVelocityChange("theta2Dot", angleFromUi(value, config.angleUnit))}
          />
        </div>
      </div>

      <div className="section">
        <h3>Joint Limits</h3>
        <div className="control-grid">
          <${SliderNumberControl}
            label="Theta 1 Min"
            value=${theta1LimitMinUi}
            min=${config.angleUnit === "deg" ? -180 : -Math.PI}
            max=${theta1LimitMaxUi}
            step=${config.angleUnit === "deg" ? 1 : 0.01}
            displayValue=${`${formatNumber(theta1LimitMinUi, config.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`}
            onChange=${(value) => onJointLimitChange("theta1Min", angleFromUi(value, config.angleUnit))}
          />
          <${SliderNumberControl}
            label="Theta 1 Max"
            value=${theta1LimitMaxUi}
            min=${theta1LimitMinUi}
            max=${config.angleUnit === "deg" ? 180 : Math.PI}
            step=${config.angleUnit === "deg" ? 1 : 0.01}
            displayValue=${`${formatNumber(theta1LimitMaxUi, config.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`}
            onChange=${(value) => onJointLimitChange("theta1Max", angleFromUi(value, config.angleUnit))}
          />
          <${SliderNumberControl}
            label="Theta 2 Min"
            value=${theta2LimitMinUi}
            min=${config.angleUnit === "deg" ? -180 : -Math.PI}
            max=${theta2LimitMaxUi}
            step=${config.angleUnit === "deg" ? 1 : 0.01}
            displayValue=${`${formatNumber(theta2LimitMinUi, config.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`}
            onChange=${(value) => onJointLimitChange("theta2Min", angleFromUi(value, config.angleUnit))}
          />
          <${SliderNumberControl}
            label="Theta 2 Max"
            value=${theta2LimitMaxUi}
            min=${theta2LimitMinUi}
            max=${config.angleUnit === "deg" ? 180 : Math.PI}
            step=${config.angleUnit === "deg" ? 1 : 0.01}
            displayValue=${`${formatNumber(theta2LimitMaxUi, config.angleUnit === "deg" ? 1 : 3)} ${angleUnitText}`}
            onChange=${(value) => onJointLimitChange("theta2Max", angleFromUi(value, config.angleUnit))}
          />
        </div>
      </div>

      <div className="section">
        <h3>Calibration</h3>
        <div className="control-grid">
          <${SliderNumberControl}
            label="Joint 1 Zero Offset"
            value=${config.calibration.joint1OffsetDeg}
            min=${-180}
            max=${180}
            step=${1}
            displayValue=${`${formatNumber(config.calibration.joint1OffsetDeg, 1)} deg`}
            onChange=${(value) => onCalibrationChange("joint1OffsetDeg", value)}
          />
          <${SliderNumberControl}
            label="Joint 2 Zero Offset"
            value=${config.calibration.joint2OffsetDeg}
            min=${-180}
            max=${180}
            step=${1}
            displayValue=${`${formatNumber(config.calibration.joint2OffsetDeg, 1)} deg`}
            onChange=${(value) => onCalibrationChange("joint2OffsetDeg", value)}
          />
          <div className="toggle-group">
            <${ToggleButton}
              active=${!config.calibration.joint1Invert}
              label="Joint 1 Normal"
              onClick=${() => onCalibrationChange("joint1Invert", false)}
            />
            <${ToggleButton}
              active=${config.calibration.joint1Invert}
              label="Joint 1 Inverted"
              onClick=${() => onCalibrationChange("joint1Invert", true)}
            />
          </div>
          <div className="toggle-group">
            <${ToggleButton}
              active=${!config.calibration.joint2Invert}
              label="Joint 2 Normal"
              onClick=${() => onCalibrationChange("joint2Invert", false)}
            />
            <${ToggleButton}
              active=${config.calibration.joint2Invert}
              label="Joint 2 Inverted"
              onClick=${() => onCalibrationChange("joint2Invert", true)}
            />
          </div>
        </div>
      </div>

      <div className="section">
        <h3>Presets and Motion</h3>
        <div className="control-grid">
          <div className="control-row">
            <label>
              <span>Preset</span>
              <span>${activePreset.label}</span>
            </label>
            <select value=${selectedPresetId} onChange=${(event) => onPresetSelection(event.target.value)}>
              ${[...BUILT_IN_PRESETS, ...customPresets].map(
                (preset) => html`<option value=${preset.id}>${preset.label}</option>`
              )}
            </select>
          </div>
          <div className="button-grid">
            <button type="button" className="primary" onClick=${onPresetLoad}>Load Preset</button>
            <button type="button" onClick=${onPresetSave}>Save Current</button>
            <button type="button" onClick=${onPresetDelete} disabled=${!selectedPresetId.startsWith("custom-")}>
              Delete Custom
            </button>
            <button type="button" onClick=${onReset}>Reset App</button>
          </div>
          <div className="button-grid">
            <button type="button" onClick=${onAddWaypoint}>Add Waypoint</button>
            <button type="button" onClick=${onClearWaypoints}>Clear Waypoints</button>
            <button type="button" className=${isPlaying ? "" : "primary"} onClick=${isPlaying ? onStopPath : onPlayPath}>
              ${isPlaying ? "Stop Playback" : "Play Path"}
            </button>
            <button type="button" onClick=${onClearTrail}>Clear Trail</button>
          </div>
          <div className="toggle-group">
            <${ToggleButton}
              active=${config.traceEnabled}
              label="Trail On"
              onClick=${() => onToggleFlag("traceEnabled", true)}
            />
            <${ToggleButton}
              active=${!config.traceEnabled}
              label="Trail Off"
              onClick=${() => onToggleFlag("traceEnabled", false)}
            />
          </div>
          <div className="toggle-group">
            <${ToggleButton}
              active=${config.animateTargetChanges}
              label="Animate Target"
              onClick=${() => onToggleFlag("animateTargetChanges", true)}
            />
            <${ToggleButton}
              active=${!config.animateTargetChanges}
              label="Instant Target"
              onClick=${() => onToggleFlag("animateTargetChanges", false)}
            />
          </div>
        </div>
        <div className="info-grid" style=${{ marginTop: "12px" }}>
          <div className="info-card">
            <strong>Waypoints</strong>
            <span>${waypointCount}</span>
          </div>
          <div className="info-card">
            <strong>Trail Samples</strong>
            <span>${trailCount}</span>
          </div>
        </div>
      </div>
    </aside>
  `;
}
