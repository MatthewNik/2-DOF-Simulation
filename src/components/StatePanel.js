import { html, useMemo, useState } from "../lib.js";
import {
  copyText,
  createExportArtifact,
  createRobotExportPayload,
  downloadTextFile,
  getExportOptionsForMode
} from "../utils/export.js";
import { formatNumber, radToDeg } from "../utils/math.js";

function ExportSelector({ selected, options, onChange }) {
  return html`
    <div className=${options.length > 2 ? "toggle-group three" : "toggle-group"}>
      ${options.map(
        (option) => html`
          <button
            key=${option.id}
            type="button"
            className=${selected === option.id ? "active" : ""}
            onClick=${() => onChange(option.id)}
          >
            ${option.label}
          </button>
        `
      )}
    </div>
  `;
}

function KinematicStateCards({ mode, config, robotState, trajectory }) {
  if (mode === "leg3") {
    return html`
      <div className="kv-grid">
        <div className="kv">
          <strong>Joint Commands</strong>
          <span>
            yaw ${formatNumber(radToDeg(robotState.activeTheta1), 2)} deg,
            hip ${formatNumber(radToDeg(robotState.activeTheta2), 2)} deg,
            knee ${formatNumber(radToDeg(robotState.activeTheta3), 2)} deg
          </span>
        </div>
        <div className="kv">
          <strong>Foot Position</strong>
          <span>
            x = ${formatNumber(robotState.fk.foot.x, 4)} m, y = ${formatNumber(robotState.fk.foot.y, 4)} m, z = ${formatNumber(robotState.fk.foot.z, 4)} m
          </span>
        </div>
        <div className="kv">
          <strong>Intermediate Joints</strong>
          <span>
            coxa (${formatNumber(robotState.fk.coxa.x, 3)}, ${formatNumber(robotState.fk.coxa.y, 3)}, ${formatNumber(robotState.fk.coxa.z, 3)}), knee (${formatNumber(robotState.fk.knee.x, 3)}, ${formatNumber(robotState.fk.knee.y, 3)}, ${formatNumber(robotState.fk.knee.z, 3)})
          </span>
        </div>
        <div className="kv">
          <strong>Target</strong>
          <span>
            x = ${formatNumber(config.targetX, 4)} m, y = ${formatNumber(config.targetY, 4)} m, z = ${formatNumber(config.targetZ, 4)} m
          </span>
        </div>
        <div className="kv">
          <strong>Servo Outputs</strong>
          <span>
            J1 ${formatNumber(robotState.servoCommands.joint1ServoDeg, 2)} deg, J2 ${formatNumber(robotState.servoCommands.joint2ServoDeg, 2)} deg, J3 ${formatNumber(robotState.servoCommands.joint3ServoDeg, 2)} deg
          </span>
        </div>
        <div className="kv">
          <strong>State</strong>
          <span>${robotState.stateLabel}</span>
        </div>
      </div>
    `;
  }

  return html`
    <div className="kv-grid">
      <div className="kv">
        <strong>Joint Commands</strong>
        <span>
          theta1 ${formatNumber(robotState.activeTheta1, 4)} rad (${formatNumber(radToDeg(robotState.activeTheta1), 2)} deg),
          theta2 ${formatNumber(robotState.activeTheta2, 4)} rad (${formatNumber(radToDeg(robotState.activeTheta2), 2)} deg)
        </span>
      </div>
      <div className="kv">
        <strong>End Effector</strong>
        <span>
          x = ${formatNumber(robotState.fk.endEffector.x, 4)} m, y = ${formatNumber(robotState.fk.endEffector.y, 4)} m
        </span>
      </div>
      <div className="kv">
        <strong>Target</strong>
        <span>
          x = ${formatNumber(config.targetX, 4)} m, y = ${formatNumber(config.targetY, 4)} m
        </span>
      </div>
      <div className="kv">
        <strong>Servo Outputs</strong>
        <span>
          J1 ${formatNumber(robotState.servoCommands.joint1ServoDeg, 2)} deg, J2 ${formatNumber(robotState.servoCommands.joint2ServoDeg, 2)} deg
        </span>
      </div>
      <div className="kv">
        <strong>Waypoints</strong>
        <span>${trajectory.waypoints.length} saved targets</span>
      </div>
      <div className="kv">
        <strong>State</strong>
        <span>${robotState.stateLabel}</span>
      </div>
    </div>
  `;
}

export function StatePanel({ mode, config, robotState, trajectory }) {
  const options = getExportOptionsForMode(mode);
  const [selectedExport, setSelectedExport] = useState(options[0].id);
  const [copyStatus, setCopyStatus] = useState("");
  const payload = useMemo(
    () => createRobotExportPayload(mode, config, robotState, trajectory),
    [mode, config, robotState, trajectory]
  );
  const exportText = useMemo(
    () => createExportArtifact(mode, selectedExport, config, robotState, trajectory),
    [mode, selectedExport, config, robotState, trajectory]
  );
  const selectedMeta = options.find((option) => option.id === selectedExport) || options[0];
  const statusItems =
    mode === "leg3"
      ? [
          {
            label: "Reachability",
            value: robotState.ik.reachable ? "Reachable" : "Unreachable",
            kind: robotState.ik.reachable ? "good" : "warn"
          },
          {
            label: "Joint limits",
            value: robotState.jointLimitStatus.every(Boolean) ? "Inside configured limits" : "Outside configured limits",
            kind: robotState.jointLimitStatus.every(Boolean) ? "good" : "warn"
          },
          {
            label: "Leg singularity",
            value: robotState.nearSingularity ? "Near singularity" : "Well-conditioned",
            kind: robotState.nearSingularity ? "warn" : "good"
          }
        ]
      : [
          {
            label: "Reachability",
            value: robotState.ik.reachable ? "Reachable" : "Unreachable",
            kind: robotState.ik.reachable ? "good" : "warn"
          },
          {
            label: "Joint limits",
            value: robotState.jointLimitStatus.every(Boolean) ? "Inside configured limits" : "Outside configured limits",
            kind: robotState.jointLimitStatus.every(Boolean) ? "good" : "warn"
          },
          {
            label: "Singularity",
            value: robotState.nearSingularity ? "Near singularity" : "Well-conditioned",
            kind: robotState.nearSingularity ? "warn" : "good"
          }
        ];

  async function handleCopy() {
    try {
      const copied = await copyText(exportText);
      setCopyStatus(copied ? "Copied to clipboard." : "Clipboard API unavailable in this browser.");
    } catch {
      setCopyStatus("Copy failed.");
    }
  }

  return html`
    <section className="panel state-panel">
      <div className="section-header">
        <h2>Robot State and Export</h2>
        <small>${mode === "leg3" ? "Leg controller scaffold and inspection" : "Move-to scaffolds for 2 DOF integrations"}</small>
      </div>

      <div className="panel-stack">
        <div className="section">
          <h3>Live State</h3>
          <${KinematicStateCards} mode=${mode} config=${config} robotState=${robotState} trajectory=${trajectory} />
        </div>

        <div className="section">
          <h3>Health Checks</h3>
          <div className="status-list">
            ${statusItems.map(
              (item) => html`
                <div key=${item.label} className=${`status-pill ${item.kind}`}>
                  <strong>${item.label}</strong>
                  <div>${item.value}</div>
                </div>
              `
            )}
          </div>
        </div>

        <div className="section">
          <div className="section-header">
            <h3>Exports</h3>
            <small>${selectedMeta.description}</small>
          </div>
          <p className="footer-note">
            ${mode === "leg3"
              ? "The generated export captures the 3 DOF leg geometry, active pose, calibration, and IK assumptions used by the simulator."
              : "The generated export includes the current link lengths, joint limits, zero offsets, inversion flags, and a move-to helper for robot integration."}
          </p>
          <${ExportSelector} selected=${selectedExport} options=${options} onChange=${setSelectedExport} />
          <textarea className="code-output" readOnly value=${exportText}></textarea>
          <div className="button-grid">
            <button type="button" className="primary" onClick=${handleCopy}>Copy Export</button>
            <button type="button" onClick=${() => downloadTextFile(`robot-${mode}-${selectedExport}.${selectedMeta.extension}`, exportText)}>
              Download Export
            </button>
          </div>
          ${copyStatus ? html`<div className="footer-note">${copyStatus}</div>` : null}
          <details className="json-preview">
            <summary>Structured snapshot</summary>
            <pre>${JSON.stringify(payload, null, 2)}</pre>
          </details>
        </div>
      </div>
    </section>
  `;
}
