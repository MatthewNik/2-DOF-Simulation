import { html, useMemo, useState } from "../lib.js";
import {
  copyText,
  createExportArtifact,
  createRobotExportPayload,
  downloadTextFile,
  EXPORT_OPTIONS
} from "../utils/export.js";
import { formatNumber, radToDeg } from "../utils/math.js";

function ExportSelector({ selected, onChange }) {
  return html`
    <div className="toggle-group three">
      ${EXPORT_OPTIONS.map(
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

export function StatePanel({ config, robotState, waypoints }) {
  const [selectedExport, setSelectedExport] = useState("json");
  const [copyStatus, setCopyStatus] = useState("");

  const payload = useMemo(
    () => createRobotExportPayload(config, robotState, waypoints),
    [config, robotState, waypoints]
  );

  const exportText = useMemo(
    () => createExportArtifact(selectedExport, config, robotState, waypoints),
    [config, robotState, selectedExport, waypoints]
  );

  const selectedExportMeta =
    EXPORT_OPTIONS.find((option) => option.id === selectedExport) || EXPORT_OPTIONS[0];

  async function handleCopy() {
    try {
      const copied = await copyText(exportText);
      setCopyStatus(copied ? "Copied to clipboard." : "Clipboard API unavailable in this browser.");
    } catch {
      setCopyStatus("Copy failed.");
    }
  }

  function handleDownload() {
    downloadTextFile(
      `planar-arm-${selectedExport}.${selectedExportMeta.extension}`,
      exportText
    );
  }

  const statusItems = [
    {
      label: "Reachability",
      value: robotState.ik.reachable ? "Reachable" : "Unreachable",
      kind: robotState.ik.reachable ? "good" : "warn"
    },
    {
      label: "Joint limits",
      value:
        robotState.theta1WithinLimits && robotState.theta2WithinLimits
          ? "Inside configured limits"
          : "Outside configured limits",
      kind:
        robotState.theta1WithinLimits && robotState.theta2WithinLimits ? "good" : "warn"
    },
    {
      label: "Singularity",
      value: robotState.nearSingularity ? "Near singularity" : "Well-conditioned",
      kind: robotState.nearSingularity ? "warn" : "good"
    }
  ];

  return html`
    <section className="panel state-panel">
      <div className="section-header">
        <h2>Robot State and Integration</h2>
        <small>Move-to scaffolds for servo and stepper workflows</small>
      </div>

      <div className="panel-stack">
        <div className="section">
          <h3>Live State</h3>
          <div className="kv-grid">
            <div className="kv">
              <strong>Joint 1 Command</strong>
              <span>
                ${formatNumber(robotState.activeTheta1, 4)} rad (${formatNumber(radToDeg(robotState.activeTheta1), 2)} deg)
              </span>
            </div>
            <div className="kv">
              <strong>Joint 2 Command</strong>
              <span>
                ${formatNumber(robotState.activeTheta2, 4)} rad (${formatNumber(radToDeg(robotState.activeTheta2), 2)} deg)
              </span>
            </div>
            <div className="kv">
              <strong>Servo Outputs</strong>
              <span>
                J1 ${formatNumber(robotState.servoCommands.joint1ServoDeg, 2)} deg, J2 ${formatNumber(robotState.servoCommands.joint2ServoDeg, 2)} deg
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
              <strong>Waypoints</strong>
              <span>${waypoints.length} saved targets</span>
            </div>
            <div className="kv">
              <strong>Robot State</strong>
              <span>${robotState.stateLabel}</span>
            </div>
            <div className="kv">
              <strong>Snapshot</strong>
              <span>${payload.state.reachable ? "Move-ready geometry" : "Target needs adjustment"}</span>
            </div>
          </div>
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
            <small>${selectedExportMeta.description}</small>
          </div>
          <p className="footer-note">
            The generated code includes the current link lengths, joint limits, zero offsets,
            inversion flags, home pose, and a direct move-to helper for real robot integration.
          </p>
          <${ExportSelector} selected=${selectedExport} onChange=${setSelectedExport} />
          <textarea className="code-output" readOnly value=${exportText}></textarea>
          <div className="button-grid">
            <button type="button" className="primary" onClick=${handleCopy}>Copy Export</button>
            <button type="button" onClick=${handleDownload}>Download Export</button>
          </div>
          ${copyStatus ? html`<div className="footer-note">${copyStatus}</div>` : null}
        </div>
      </div>
    </section>
  `;
}
