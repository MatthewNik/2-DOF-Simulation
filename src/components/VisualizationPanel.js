import { html, useMemo, useRef, useState } from "../lib.js";
import { arcPath, formatNumber } from "../utils/math.js";

function createGridLines(extent, step) {
  const lines = [];

  for (let value = -extent; value <= extent + 1e-6; value += step) {
    lines.push({
      key: `v-${value.toFixed(2)}`,
      x1: value,
      y1: -extent,
      x2: value,
      y2: extent
    });
    lines.push({
      key: `h-${value.toFixed(2)}`,
      x1: -extent,
      y1: value,
      x2: extent,
      y2: value
    });
  }

  return lines;
}

function polylinePoints(points) {
  return points.map((point) => `${point.x},${-point.y}`).join(" ");
}

function ringPath(outerRadius, innerRadius) {
  const outer = `M ${outerRadius} 0 A ${outerRadius} ${outerRadius} 0 1 0 ${-outerRadius} 0 A ${outerRadius} ${outerRadius} 0 1 0 ${outerRadius} 0`;
  const inner =
    innerRadius > 0
      ? `M ${innerRadius} 0 A ${innerRadius} ${innerRadius} 0 1 1 ${-innerRadius} 0 A ${innerRadius} ${innerRadius} 0 1 1 ${innerRadius} 0`
      : "";
  return `${outer} ${inner}`.trim();
}

function axisTickPositions(extent, step) {
  const ticks = [];
  for (let value = -extent; value <= extent + 1e-6; value += step) {
    ticks.push(Number(value.toFixed(6)));
  }
  return ticks;
}

function MetaCard({ title, value, detail }) {
  return html`
    <div className="info-card">
      <strong>${title}</strong>
      <span>${value}</span>
      ${detail ? html`<div className="subtle" style=${{ marginTop: "6px" }}>${detail}</div>` : null}
    </div>
  `;
}

export function VisualizationPanel({
  config,
  robotState,
  waypoints,
  trail,
  onTargetChange
}) {
  const svgRef = useRef(null);
  const [dragging, setDragging] = useState(false);
  const base = { x: 0, y: 0 };
  const joint1 = robotState.fk.joint1;
  const endEffector = robotState.fk.endEffector;
  const target = { x: config.targetX, y: config.targetY };
  const pointsForBounds = [target, joint1, endEffector, ...waypoints, ...trail];
  const maxFromPoints = Math.max(
    ...pointsForBounds.flatMap((point) => [Math.abs(point.x), Math.abs(point.y)]),
    0
  );
  const extent = Math.max(config.L1 + config.L2 + 0.4, maxFromPoints + 0.35);
  const gridStep = extent > 3 ? 0.5 : 0.25;
  const gridLines = useMemo(() => createGridLines(extent, gridStep), [extent, gridStep]);
  const ticks = useMemo(() => axisTickPositions(extent, gridStep), [extent, gridStep]);
  const velocityScale = Math.max(0.15, extent * 0.1);
  const velocityTip = {
    x: endEffector.x + robotState.velocity.xDot * velocityScale,
    y: endEffector.y + robotState.velocity.yDot * velocityScale
  };
  const theta1ArcPath = arcPath(0, 0, extent * 0.14, 0, robotState.activeTheta1);
  const theta2ArcPath = arcPath(
    joint1.x,
    -joint1.y,
    extent * 0.11,
    robotState.activeTheta1,
    robotState.activeTheta1 + robotState.activeTheta2
  );
  const theta1Label = {
    x: Math.cos(robotState.activeTheta1 * 0.5) * extent * 0.18,
    y: -Math.sin(robotState.activeTheta1 * 0.5) * extent * 0.18
  };
  const theta2Label = {
    x: joint1.x + Math.cos(robotState.activeTheta1 + robotState.activeTheta2 * 0.5) * extent * 0.15,
    y: -(joint1.y + Math.sin(robotState.activeTheta1 + robotState.activeTheta2 * 0.5) * extent * 0.15)
  };

  function eventToWorld(event) {
    const bounds = svgRef.current.getBoundingClientRect();
    const normalizedX = (event.clientX - bounds.left) / bounds.width;
    const normalizedY = (event.clientY - bounds.top) / bounds.height;
    const x = -extent + normalizedX * extent * 2;
    const y = extent - normalizedY * extent * 2;
    return { x, y };
  }

  function pushTargetFromEvent(event, immediate = false) {
    const point = eventToWorld(event);
    onTargetChange(point.x, point.y, { immediate });
  }

  function handlePointerDown(event) {
    event.preventDefault();
    setDragging(true);
    svgRef.current.setPointerCapture(event.pointerId);
    pushTargetFromEvent(event, false);
  }

  function handlePointerMove(event) {
    if (!dragging) {
      return;
    }

    pushTargetFromEvent(event, true);
  }

  function handlePointerUp(event) {
    if (!dragging) {
      return;
    }

    setDragging(false);
    if (svgRef.current?.hasPointerCapture(event.pointerId)) {
      svgRef.current.releasePointerCapture(event.pointerId);
    }
  }

  const targetReachClass =
    config.mode === "ik" && !robotState.ik.reachable
      ? "warn"
      : robotState.reachableWithLimits
        ? "good"
        : "";

  return html`
    <section className="panel viz-card">
      <div className="viz-main">
        <div className="viz-stage">
          <svg
            ref=${svgRef}
            viewBox=${`${-extent} ${-extent} ${extent * 2} ${extent * 2}`}
            width="100%"
            height="100%"
            onPointerDown=${handlePointerDown}
            onPointerMove=${handlePointerMove}
            onPointerUp=${handlePointerUp}
            onPointerLeave=${handlePointerUp}
            style=${{ touchAction: "none" }}
          >
            <defs>
              <marker id="velocity-arrow" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto">
                <polygon points="0 0, 7 3.5, 0 7" fill="#f57c3d"></polygon>
              </marker>
            </defs>

            <path
              d=${ringPath(robotState.ik.outerRadius, robotState.ik.innerRadius)}
              fill="rgba(15, 139, 141, 0.08)"
              fillRule="evenodd"
            />

            ${gridLines.map(
              (line) => html`
                <line
                  key=${line.key}
                  x1=${line.x1}
                  y1=${-line.y1}
                  x2=${line.x2}
                  y2=${-line.y2}
                  stroke="rgba(23, 48, 66, 0.08)"
                  strokeWidth=${0.01}
                />
              `
            )}

            <line x1=${-extent} y1=${0} x2=${extent} y2=${0} stroke="rgba(23, 48, 66, 0.22)" strokeWidth=${0.02} />
            <line x1=${0} y1=${-extent} x2=${0} y2=${extent} stroke="rgba(23, 48, 66, 0.22)" strokeWidth=${0.02} />

            ${ticks.map(
              (tick) =>
                Math.abs(tick) > 1e-6
                  ? html`
                      <g key=${`tick-${tick}`}>
                        <line x1=${tick} y1=${-0.03} x2=${tick} y2=${0.03} stroke="rgba(23, 48, 66, 0.35)" strokeWidth=${0.02} />
                        <line x1=${-0.03} y1=${-tick} x2=${0.03} y2=${-tick} stroke="rgba(23, 48, 66, 0.35)" strokeWidth=${0.02} />
                        <text x=${tick} y=${0.11} textAnchor="middle" fontSize=${0.1} fill="#527084">${formatNumber(tick, 2)}</text>
                        <text x=${0.12} y=${-tick + 0.03} fontSize=${0.1} fill="#527084">${formatNumber(tick, 2)}</text>
                      </g>
                    `
                  : null
            )}

            ${trail.length > 1
              ? html`
                  <polyline
                    points=${polylinePoints(trail)}
                    fill="none"
                    stroke="rgba(245, 124, 61, 0.72)"
                    strokeWidth=${0.035}
                    strokeLinecap="round"
                    strokeLinejoin="round"
                  />
                `
              : null}

            ${waypoints.length > 1
              ? html`
                  <polyline
                    points=${polylinePoints(waypoints)}
                    fill="none"
                    stroke="rgba(23, 48, 66, 0.28)"
                    strokeWidth=${0.03}
                    strokeDasharray="0.1 0.08"
                  />
                `
              : null}

            <path d=${theta1ArcPath} fill="none" stroke="rgba(15, 139, 141, 0.8)" strokeWidth=${0.03} />
            <path d=${theta2ArcPath} fill="none" stroke="rgba(245, 124, 61, 0.85)" strokeWidth=${0.03} />

            <line
              x1=${base.x}
              y1=${-base.y}
              x2=${joint1.x}
              y2=${-joint1.y}
              stroke=${robotState.nearSingularity ? "#d9485f" : "#0f8b8d"}
              strokeWidth=${0.07}
              strokeLinecap="round"
            />
            <line
              x1=${joint1.x}
              y1=${-joint1.y}
              x2=${endEffector.x}
              y2=${-endEffector.y}
              stroke=${robotState.nearSingularity ? "#d9485f" : "#184d74"}
              strokeWidth=${0.06}
              strokeLinecap="round"
            />

            <line
              x1=${endEffector.x}
              y1=${-endEffector.y}
              x2=${velocityTip.x}
              y2=${-velocityTip.y}
              stroke="#f57c3d"
              strokeWidth=${0.03}
              markerEnd="url(#velocity-arrow)"
            />

            ${config.mode === "ik" && !robotState.ik.reachable
              ? html`
                  <line
                    x1=${endEffector.x}
                    y1=${-endEffector.y}
                    x2=${target.x}
                    y2=${-target.y}
                    stroke="#d9485f"
                    strokeWidth=${0.03}
                    strokeDasharray="0.12 0.08"
                  />
                `
              : null}

            <circle cx=${0} cy=${0} r=${0.06} fill="#173042" />
            <circle cx=${joint1.x} cy=${-joint1.y} r=${0.05} fill="#0f8b8d" />
            <circle
              cx=${endEffector.x}
              cy=${-endEffector.y}
              r=${robotState.nearSingularity ? 0.065 : 0.055}
              fill=${robotState.nearSingularity ? "#d9485f" : "#184d74"}
            />

            ${waypoints.map(
              (point, index) => html`
                <circle
                  key=${`waypoint-${index}`}
                  cx=${point.x}
                  cy=${-point.y}
                  r=${0.032}
                  fill="#ffffff"
                  stroke="#173042"
                  strokeWidth=${0.02}
                />
              `
            )}

            ${config.mode === "ik"
              ? html`
                  <g>
                    <circle
                      cx=${target.x}
                      cy=${-target.y}
                      r=${0.07}
                      fill="none"
                      stroke=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"}
                      strokeWidth=${0.03}
                    />
                    <line x1=${target.x - 0.08} y1=${-target.y} x2=${target.x + 0.08} y2=${-target.y} stroke=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"} strokeWidth=${0.025} />
                    <line x1=${target.x} y1=${-target.y - 0.08} x2=${target.x} y2=${-target.y + 0.08} stroke=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"} strokeWidth=${0.025} />
                  </g>
                `
              : null}

            <text x=${0.08} y=${-0.08} fontSize=${0.12} fill="#173042">O</text>
            <text x=${joint1.x + 0.08} y=${-joint1.y - 0.08} fontSize=${0.12} fill="#0f8b8d">J1</text>
            <text x=${endEffector.x + 0.08} y=${-endEffector.y - 0.08} fontSize=${0.12} fill="#184d74">EE</text>
            ${config.mode === "ik"
              ? html`
                  <text
                    x=${target.x + 0.08}
                    y=${-target.y + 0.12}
                    fontSize=${0.12}
                    fill=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"}
                  >
                    T
                  </text>
                `
              : null}
            <text x=${theta1Label.x} y=${theta1Label.y} fontSize=${0.12} fill="#0f8b8d">theta1</text>
            <text x=${theta2Label.x} y=${theta2Label.y} fontSize=${0.12} fill="#f57c3d">theta2</text>
          </svg>

          <div className="viz-note">
            Drag or click anywhere in the workspace to move the target. Any target interaction switches into IK mode automatically.
          </div>
        </div>
      </div>

      <div className="viz-meta">
        <div className="section">
          <h3>Visualization Status</h3>
          <div className="info-grid">
            <${MetaCard}
              title="Reach"
              value=${robotState.ik.reachable ? "Inside Workspace" : "Outside Workspace"}
              detail=${`r = ${formatNumber(robotState.ik.radius)} m`}
            />
            <${MetaCard}
              title="Workspace"
              value=${`${formatNumber(robotState.ik.innerRadius)} to ${formatNumber(robotState.ik.outerRadius)} m`}
              detail="Annulus for a 2-link planar arm"
            />
            <${MetaCard}
              title="End Effector"
              value=${`(${formatNumber(endEffector.x)}, ${formatNumber(endEffector.y)})`}
              detail="Current simulated tool position"
            />
            <${MetaCard}
              title="Velocity"
              value=${`(${formatNumber(robotState.velocity.xDot)}, ${formatNumber(robotState.velocity.yDot)})`}
              detail="Cartesian velocity from J qdot"
            />
          </div>
        </div>

        <div className="section">
          <h3>Legend</h3>
          <div className="legend">
            <div className="legend-item"><span className="legend-line"></span> Link 1 / Link 2</div>
            <div className="legend-item"><span className="legend-swatch" style=${{ background: "#f57c3d" }}></span> Target and velocity</div>
            <div className="legend-item"><span className="legend-swatch" style=${{ background: "rgba(15, 139, 141, 0.2)", border: "1px solid rgba(15, 139, 141, 0.6)" }}></span> Reachable workspace</div>
            <div className="legend-item"><span className="legend-swatch" style=${{ background: "#d9485f" }}></span> Unreachable or singular warning</div>
          </div>
        </div>

        <div className=${`status-pill ${targetReachClass}`.trim()}>
          <strong>Current target status</strong>
          <div>
            ${config.mode === "ik"
              ? robotState.ik.reachable
                ? "The selected target is geometrically reachable."
                : "The selected target is outside the workspace. The arm is projected onto the closest feasible boundary pose."
              : "Forward-kinematics mode is active. The target remains available for future IK moves."}
          </div>
        </div>
      </div>
    </section>
  `;
}
