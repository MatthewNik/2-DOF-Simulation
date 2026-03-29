import { html, useMemo, useRef, useState } from "../lib.js";
import { arcPath, formatNumber } from "../utils/math.js";

function createGridLines(extent, step) {
  const lines = [];
  for (let value = -extent; value <= extent + 1e-6; value += step) {
    lines.push({ key: `v-${value.toFixed(2)}`, x1: value, y1: -extent, x2: value, y2: extent });
    lines.push({ key: `h-${value.toFixed(2)}`, x1: -extent, y1: value, x2: extent, y2: value });
  }
  return lines;
}

function axisTickPositions(extent, step) {
  const ticks = [];
  for (let value = -extent; value <= extent + 1e-6; value += step) {
    ticks.push(Number(value.toFixed(6)));
  }
  return ticks;
}

function polylinePoints(points, projectPoint) {
  return points.map((point) => projectPoint(point)).join(" ");
}

function ringPath(outerRadius, innerRadius) {
  const outer = `M ${outerRadius} 0 A ${outerRadius} ${outerRadius} 0 1 0 ${-outerRadius} 0 A ${outerRadius} ${outerRadius} 0 1 0 ${outerRadius} 0`;
  const inner =
    innerRadius > 0
      ? `M ${innerRadius} 0 A ${innerRadius} ${innerRadius} 0 1 1 ${-innerRadius} 0 A ${innerRadius} ${innerRadius} 0 1 1 ${innerRadius} 0`
      : "";
  return `${outer} ${inner}`.trim();
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

function project3D(point, azimuth, elevation) {
  const cosA = Math.cos(azimuth);
  const sinA = Math.sin(azimuth);
  const cosE = Math.cos(elevation);
  const sinE = Math.sin(elevation);
  const xr = point.x * cosA - point.y * sinA;
  const yr = point.x * sinA + point.y * cosA;
  const zr = point.z;
  return {
    x: xr,
    y: -(yr * sinE + zr * cosE)
  };
}

function createPlaneGrid3D(extent, step) {
  const lines = [];
  for (let value = -extent; value <= extent + 1e-6; value += step) {
    lines.push([
      { x: -extent, y: value, z: 0 },
      { x: extent, y: value, z: 0 }
    ]);
    lines.push([
      { x: value, y: -extent, z: 0 },
      { x: value, y: extent, z: 0 }
    ]);
  }
  return lines;
}

function Planar2Canvas({ config, robotState, trajectory, onTargetChange }) {
  const svgRef = useRef(null);
  const [dragging, setDragging] = useState(false);
  const base = { x: 0, y: 0 };
  const joint1 = robotState.fk.joint1;
  const endEffector = robotState.fk.endEffector;
  const target = robotState.activeTarget;
  const waypoints = trajectory.waypoints || [];
  const trail = trajectory.trail || [];
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

  function eventToWorld(event) {
    const bounds = svgRef.current.getBoundingClientRect();
    const normalizedX = (event.clientX - bounds.left) / bounds.width;
    const normalizedY = (event.clientY - bounds.top) / bounds.height;
    return {
      x: -extent + normalizedX * extent * 2,
      y: extent - normalizedY * extent * 2
    };
  }

  function pushTargetFromEvent(event, immediate = false) {
    const point = eventToWorld(event);
    onTargetChange(point, { immediate });
  }

  return html`
    <section className="panel canvas-card">
      <div className="canvas-stage-wrap">
        <div className="viz-stage">
          <svg
            ref=${svgRef}
            viewBox=${`${-extent} ${-extent} ${extent * 2} ${extent * 2}`}
            width="100%"
            height="100%"
            onPointerDown=${(event) => {
              event.preventDefault();
              setDragging(true);
              svgRef.current.setPointerCapture(event.pointerId);
              pushTargetFromEvent(event, false);
            }}
            onPointerMove=${(event) => dragging && pushTargetFromEvent(event, true)}
            onPointerUp=${(event) => {
              setDragging(false);
              if (svgRef.current?.hasPointerCapture(event.pointerId)) {
                svgRef.current.releasePointerCapture(event.pointerId);
              }
            }}
            onPointerLeave=${() => setDragging(false)}
            style=${{ touchAction: "none" }}
          >
            <defs>
              <marker id="velocity-arrow" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto">
                <polygon points="0 0, 7 3.5, 0 7" fill="#f57c3d"></polygon>
              </marker>
            </defs>

            <path d=${ringPath(robotState.ik.outerRadius, robotState.ik.innerRadius)} fill="rgba(15, 139, 141, 0.08)" fillRule="evenodd" />
            ${gridLines.map(
              (line) => html`
                <line key=${line.key} x1=${line.x1} y1=${-line.y1} x2=${line.x2} y2=${-line.y2} stroke="rgba(23, 48, 66, 0.08)" strokeWidth=${0.01} />
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
              ? html`<polyline points=${polylinePoints(trail, (point) => `${point.x},${-point.y}`)} fill="none" stroke="rgba(245, 124, 61, 0.72)" strokeWidth=${0.035} strokeLinecap="round" strokeLinejoin="round" />`
              : null}
            ${waypoints.length > 1
              ? html`<polyline points=${polylinePoints(waypoints, (point) => `${point.x},${-point.y}`)} fill="none" stroke="rgba(23, 48, 66, 0.28)" strokeWidth=${0.03} strokeDasharray="0.1 0.08" />`
              : null}
            <path d=${theta1ArcPath} fill="none" stroke="rgba(15, 139, 141, 0.8)" strokeWidth=${0.03} />
            <path d=${theta2ArcPath} fill="none" stroke="rgba(245, 124, 61, 0.85)" strokeWidth=${0.03} />
            <line x1=${base.x} y1=${-base.y} x2=${joint1.x} y2=${-joint1.y} stroke=${robotState.nearSingularity ? "#d9485f" : "#0f8b8d"} strokeWidth=${0.07} strokeLinecap="round" />
            <line x1=${joint1.x} y1=${-joint1.y} x2=${endEffector.x} y2=${-endEffector.y} stroke=${robotState.nearSingularity ? "#d9485f" : "#184d74"} strokeWidth=${0.06} strokeLinecap="round" />
            <line x1=${endEffector.x} y1=${-endEffector.y} x2=${velocityTip.x} y2=${-velocityTip.y} stroke="#f57c3d" strokeWidth=${0.03} markerEnd="url(#velocity-arrow)" />
            ${config.solverMode === "ik" && !robotState.ik.reachable
              ? html`<line x1=${endEffector.x} y1=${-endEffector.y} x2=${target.x} y2=${-target.y} stroke="#d9485f" strokeWidth=${0.03} strokeDasharray="0.12 0.08" />`
              : null}
            <circle cx=${0} cy=${0} r=${0.06} fill="#173042" />
            <circle cx=${joint1.x} cy=${-joint1.y} r=${0.05} fill="#0f8b8d" />
            <circle cx=${endEffector.x} cy=${-endEffector.y} r=${robotState.nearSingularity ? 0.065 : 0.055} fill=${robotState.nearSingularity ? "#d9485f" : "#184d74"} />
            ${waypoints.map(
              (point, index) => html`
                <circle key=${`waypoint-${index}`} cx=${point.x} cy=${-point.y} r=${0.032} fill="#ffffff" stroke="#173042" strokeWidth=${0.02} />
              `
            )}
            ${config.solverMode === "ik"
              ? html`
                  <g>
                    <circle cx=${target.x} cy=${-target.y} r=${0.07} fill="none" stroke=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"} strokeWidth=${0.03} />
                    <line x1=${target.x - 0.08} y1=${-target.y} x2=${target.x + 0.08} y2=${-target.y} stroke=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"} strokeWidth=${0.025} />
                    <line x1=${target.x} y1=${-target.y - 0.08} x2=${target.x} y2=${-target.y + 0.08} stroke=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"} strokeWidth=${0.025} />
                  </g>
                `
              : null}
          </svg>
          <div className="viz-note">Drag or click in the workspace to move the target. Any target interaction switches to IK immediately.</div>
        </div>
      </div>

      <div className="canvas-aside">
        <div className="section">
          <h3>Visualization Status</h3>
          <div className="info-grid">
            <${MetaCard} title="Reach" value=${robotState.ik.reachable ? "Inside Workspace" : "Outside Workspace"} detail=${`r = ${formatNumber(robotState.ik.radius)} m`} />
            <${MetaCard} title="Workspace" value=${`${formatNumber(robotState.ik.innerRadius)} to ${formatNumber(robotState.ik.outerRadius)} m`} detail="Annulus for a 2-link planar arm" />
            <${MetaCard} title="End Effector" value=${`(${formatNumber(endEffector.x)}, ${formatNumber(endEffector.y)})`} detail="Current tool position" />
            <${MetaCard} title="Velocity" value=${`(${formatNumber(robotState.velocity.xDot)}, ${formatNumber(robotState.velocity.yDot)})`} detail="Cartesian velocity from J qdot" />
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
        <div className=${`status-pill ${config.solverMode === "ik" && !robotState.ik.reachable ? "warn" : "good"}`.trim()}>
          <strong>Current target status</strong>
          <div>
            ${config.solverMode === "ik"
              ? robotState.ik.reachable
                ? "The selected target is geometrically reachable."
                : "The selected target is outside the workspace. The solver clamps to the nearest feasible pose."
              : "Forward kinematics mode is active. The target remains available for future IK moves."}
          </div>
        </div>
      </div>
    </section>
  `;
}

function Leg3Canvas({ config, robotState, onTargetChange, onCameraChange }) {
  const sideSvgRef = useRef(null);
  const topSvgRef = useRef(null);
  const previewSvgRef = useRef(null);
  const [draggingSide, setDraggingSide] = useState(false);
  const [draggingTop, setDraggingTop] = useState(false);
  const [draggingPreview, setDraggingPreview] = useState(false);
  const orbitRef = useRef(null);
  const sidePoints = [
    robotState.fk.sideView.base,
    robotState.fk.sideView.coxa,
    robotState.fk.sideView.knee,
    robotState.fk.sideView.foot,
    { r: Math.hypot(config.targetX, config.targetY), z: config.targetZ }
  ];
  const topPoints = [
    { x: 0, y: 0 },
    robotState.fk.coxa,
    robotState.fk.knee,
    robotState.fk.foot,
    robotState.activeTarget
  ];
  const sideExtentR = Math.max(...sidePoints.map((point) => Math.abs(point.r)), config.L1 + config.L2 + config.L3) + 0.25;
  const sideExtentZ = Math.max(...sidePoints.map((point) => Math.abs(point.z)), config.L2 + config.L3) + 0.25;
  const sideExtent = Math.max(sideExtentR, sideExtentZ, 0.8);
  const topExtent = Math.max(
    config.L1 + config.L2 + config.L3 + 0.25,
    ...topPoints.flatMap((point) => [Math.abs(point.x), Math.abs(point.y)])
  );
  const sideGridLines = useMemo(() => createGridLines(sideExtent, sideExtent > 2.5 ? 0.5 : 0.25), [sideExtent]);
  const topGridLines = useMemo(() => createGridLines(topExtent, topExtent > 2.5 ? 0.5 : 0.25), [topExtent]);
  const sideTicks = useMemo(() => axisTickPositions(sideExtent, sideExtent > 2.5 ? 0.5 : 0.25), [sideExtent]);
  const topTicks = useMemo(() => axisTickPositions(topExtent, topExtent > 2.5 ? 0.5 : 0.25), [topExtent]);
  const cameraAzimuth = (config.cameraAzimuthDeg * Math.PI) / 180;
  const cameraElevation = (config.cameraElevationDeg * Math.PI) / 180;
  const isoPoints = [
    { x: 0, y: 0, z: 0 },
    robotState.fk.coxa,
    robotState.fk.knee,
    robotState.fk.foot
  ];
  const projected3d = isoPoints.map((point) => project3D(point, cameraAzimuth, cameraElevation));
  const projectedTarget = project3D(robotState.activeTarget, cameraAzimuth, cameraElevation);
  const worldExtent = Math.max(config.L1 + config.L2 + config.L3, 0.8);
  const planeGrid = useMemo(
    () => createPlaneGrid3D(worldExtent, worldExtent > 2.5 ? 0.5 : 0.25),
    [worldExtent]
  );
  const axes = {
    origin: project3D({ x: 0, y: 0, z: 0 }, cameraAzimuth, cameraElevation),
    x: project3D({ x: worldExtent, y: 0, z: 0 }, cameraAzimuth, cameraElevation),
    y: project3D({ x: 0, y: worldExtent, z: 0 }, cameraAzimuth, cameraElevation),
    z: project3D({ x: 0, y: 0, z: worldExtent }, cameraAzimuth, cameraElevation)
  };
  const previewExtent = Math.max(
    0.8,
    ...projected3d.flatMap((point) => [Math.abs(point.x), Math.abs(point.y)]),
    ...planeGrid.flatMap((line) =>
      line.flatMap((point) => {
        const projected = project3D(point, cameraAzimuth, cameraElevation);
        return [Math.abs(projected.x), Math.abs(projected.y)];
      })
    ),
    Math.abs(projectedTarget.x),
    Math.abs(projectedTarget.y)
  ) + 0.3;

  function sideEventToWorld(event) {
    const bounds = sideSvgRef.current.getBoundingClientRect();
    const normalizedX = (event.clientX - bounds.left) / bounds.width;
    const normalizedY = (event.clientY - bounds.top) / bounds.height;
    return {
      r: normalizedX * sideExtent * 2,
      z: sideExtent - normalizedY * sideExtent * 2
    };
  }

  function topEventToWorld(event) {
    const bounds = topSvgRef.current.getBoundingClientRect();
    const normalizedX = (event.clientX - bounds.left) / bounds.width;
    const normalizedY = (event.clientY - bounds.top) / bounds.height;
    return {
      x: -topExtent + normalizedX * topExtent * 2,
      y: topExtent - normalizedY * topExtent * 2
    };
  }

  function moveTargetFromSide(event, immediate = false) {
    const point = sideEventToWorld(event);
    const yaw = Math.atan2(config.targetY, config.targetX);
    const radius = Math.max(0, point.r);
    onTargetChange(
      {
        x: radius * Math.cos(yaw),
        y: radius * Math.sin(yaw),
        z: point.z
      },
      { immediate }
    );
  }

  function moveTargetFromTop(event, immediate = false) {
    const point = topEventToWorld(event);
    onTargetChange(
      {
        x: point.x,
        y: point.y,
        z: config.targetZ
      },
      { immediate }
    );
  }

  function handlePreviewPointerDown(event) {
    event.preventDefault();
    setDraggingPreview(true);
    orbitRef.current = {
      pointerId: event.pointerId,
      lastX: event.clientX,
      lastY: event.clientY,
      azimuthDeg: config.cameraAzimuthDeg,
      elevationDeg: config.cameraElevationDeg
    };
    previewSvgRef.current?.setPointerCapture(event.pointerId);
  }

  function handlePreviewPointerMove(event) {
    if (!draggingPreview || !orbitRef.current) {
      return;
    }

    const deltaX = event.clientX - orbitRef.current.lastX;
    const deltaY = event.clientY - orbitRef.current.lastY;
    const nextAzimuth = orbitRef.current.azimuthDeg + deltaX * 0.45;
    const nextElevation = Math.max(5, Math.min(80, orbitRef.current.elevationDeg - deltaY * 0.25));

    onCameraChange("cameraAzimuthDeg", nextAzimuth);
    onCameraChange("cameraElevationDeg", nextElevation);

    orbitRef.current = {
      ...orbitRef.current,
      lastX: event.clientX,
      lastY: event.clientY,
      azimuthDeg: nextAzimuth,
      elevationDeg: nextElevation
    };
  }

  function handlePreviewPointerUp(event) {
    setDraggingPreview(false);
    orbitRef.current = null;
    if (previewSvgRef.current?.hasPointerCapture(event.pointerId)) {
      previewSvgRef.current.releasePointerCapture(event.pointerId);
    }
  }

  return html`
    <section className="panel canvas-card canvas-card-leg">
      <div className="leg-main-grid">
        <div className="section leg-side-section">
          <div className="canvas-section-header">
            <div>
              <h3>Side View</h3>
              <p>Sagittal plane of the leg after hip yaw rotation</p>
            </div>
            <span className="canvas-badge">${config.solverMode === "ik" ? "Foot Targeting" : "Direct Angles"}</span>
          </div>
          <div className="leg-stage">
            <svg
              ref=${sideSvgRef}
              viewBox=${`0 ${-sideExtent} ${sideExtent * 2} ${sideExtent * 2}`}
              width="100%"
              height="100%"
              onPointerDown=${(event) => {
                event.preventDefault();
                setDraggingSide(true);
                sideSvgRef.current.setPointerCapture(event.pointerId);
                moveTargetFromSide(event, false);
              }}
              onPointerMove=${(event) => draggingSide && moveTargetFromSide(event, true)}
              onPointerUp=${(event) => {
                setDraggingSide(false);
                if (sideSvgRef.current?.hasPointerCapture(event.pointerId)) {
                  sideSvgRef.current.releasePointerCapture(event.pointerId);
                }
              }}
              onPointerLeave=${() => setDraggingSide(false)}
              style=${{ touchAction: "none" }}
            >
              ${sideGridLines
                .filter((line) => line.x1 >= 0 && line.x2 >= 0)
                .map(
                  (line) => html`
                    <line key=${line.key} x1=${line.x1} y1=${-line.y1} x2=${line.x2} y2=${-line.y2} stroke="rgba(23, 48, 66, 0.08)" strokeWidth=${0.01} />
                  `
                )}
              <line x1=${0} y1=${0} x2=${sideExtent * 2} y2=${0} stroke="rgba(23, 48, 66, 0.22)" strokeWidth=${0.02} />
              <line x1=${0} y1=${-sideExtent} x2=${0} y2=${sideExtent} stroke="rgba(23, 48, 66, 0.22)" strokeWidth=${0.02} />
              ${sideTicks
                .filter((tick) => tick >= 0)
                .map(
                  (tick) => html`
                    <g key=${`side-tick-${tick}`}>
                      <line x1=${tick} y1=${-0.03} x2=${tick} y2=${0.03} stroke="rgba(23, 48, 66, 0.32)" strokeWidth=${0.02} />
                      ${tick > 0 ? html`<text x=${tick} y=${0.12} textAnchor="middle" fontSize=${0.1} fill="#527084">${formatNumber(tick, 2)}</text>` : null}
                    </g>
                  `
                )}
              <line x1=${robotState.fk.sideView.base.r} y1=${-robotState.fk.sideView.base.z} x2=${robotState.fk.sideView.coxa.r} y2=${-robotState.fk.sideView.coxa.z} stroke="#173042" strokeWidth=${0.065} strokeLinecap="round" />
              <line x1=${robotState.fk.sideView.coxa.r} y1=${-robotState.fk.sideView.coxa.z} x2=${robotState.fk.sideView.knee.r} y2=${-robotState.fk.sideView.knee.z} stroke="#0f8b8d" strokeWidth=${0.06} strokeLinecap="round" />
              <line x1=${robotState.fk.sideView.knee.r} y1=${-robotState.fk.sideView.knee.z} x2=${robotState.fk.sideView.foot.r} y2=${-robotState.fk.sideView.foot.z} stroke="#184d74" strokeWidth=${0.055} strokeLinecap="round" />
              <circle cx=${robotState.fk.sideView.base.r} cy=${-robotState.fk.sideView.base.z} r=${0.05} fill="#173042" />
              <circle cx=${robotState.fk.sideView.coxa.r} cy=${-robotState.fk.sideView.coxa.z} r=${0.05} fill="#173042" />
              <circle cx=${robotState.fk.sideView.knee.r} cy=${-robotState.fk.sideView.knee.z} r=${0.05} fill="#0f8b8d" />
              <circle cx=${robotState.fk.sideView.foot.r} cy=${-robotState.fk.sideView.foot.z} r=${0.055} fill=${robotState.nearSingularity ? "#d9485f" : "#184d74"} />
              ${config.footTargetEnabled
                ? html`
                    <g>
                      <circle cx=${Math.hypot(config.targetX, config.targetY)} cy=${-config.targetZ} r=${0.07} fill="none" stroke=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"} strokeWidth=${0.03} />
                      <line x1=${Math.hypot(config.targetX, config.targetY) - 0.08} y1=${-config.targetZ} x2=${Math.hypot(config.targetX, config.targetY) + 0.08} y2=${-config.targetZ} stroke=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"} strokeWidth=${0.025} />
                      <line x1=${Math.hypot(config.targetX, config.targetY)} y1=${-config.targetZ - 0.08} x2=${Math.hypot(config.targetX, config.targetY)} y2=${-config.targetZ + 0.08} stroke=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"} strokeWidth=${0.025} />
                    </g>
                  `
                : null}
            </svg>
            <div className="viz-note">Drag in side view to adjust leg reach and height while preserving the current yaw heading.</div>
          </div>
        </div>

        <div className="leg-side-panel">
          <div className="section">
            <div className="canvas-section-header">
              <div>
                <h3>Top View</h3>
                <p>Hip yaw and XY foot projection</p>
              </div>
            </div>
            <div className="mini-stage">
              <svg
                ref=${topSvgRef}
                viewBox=${`${-topExtent} ${-topExtent} ${topExtent * 2} ${topExtent * 2}`}
                width="100%"
                height="100%"
                onPointerDown=${(event) => {
                  event.preventDefault();
                  setDraggingTop(true);
                  topSvgRef.current.setPointerCapture(event.pointerId);
                  moveTargetFromTop(event, false);
                }}
                onPointerMove=${(event) => draggingTop && moveTargetFromTop(event, true)}
                onPointerUp=${(event) => {
                  setDraggingTop(false);
                  if (topSvgRef.current?.hasPointerCapture(event.pointerId)) {
                    topSvgRef.current.releasePointerCapture(event.pointerId);
                  }
                }}
                onPointerLeave=${() => setDraggingTop(false)}
                style=${{ touchAction: "none" }}
              >
                ${topGridLines.map(
                  (line) => html`
                    <line key=${line.key} x1=${line.x1} y1=${-line.y1} x2=${line.x2} y2=${-line.y2} stroke="rgba(23, 48, 66, 0.08)" strokeWidth=${0.01} />
                  `
                )}
                <line x1=${-topExtent} y1=${0} x2=${topExtent} y2=${0} stroke="rgba(23, 48, 66, 0.22)" strokeWidth=${0.02} />
                <line x1=${0} y1=${-topExtent} x2=${0} y2=${topExtent} stroke="rgba(23, 48, 66, 0.22)" strokeWidth=${0.02} />
                ${topTicks.map(
                  (tick) =>
                    Math.abs(tick) > 1e-6
                      ? html`<line key=${`top-tick-${tick}`} x1=${tick} y1=${-0.03} x2=${tick} y2=${0.03} stroke="rgba(23, 48, 66, 0.32)" strokeWidth=${0.02} />`
                      : null
                )}
                <line x1=${0} y1=${0} x2=${robotState.fk.coxa.x} y2=${-robotState.fk.coxa.y} stroke="#173042" strokeWidth=${0.06} strokeLinecap="round" />
                <line x1=${robotState.fk.coxa.x} y1=${-robotState.fk.coxa.y} x2=${robotState.fk.foot.x} y2=${-robotState.fk.foot.y} stroke="#0f8b8d" strokeWidth=${0.04} strokeDasharray="0.1 0.06" />
                <circle cx=${0} cy=${0} r=${0.05} fill="#173042" />
                <circle cx=${robotState.fk.coxa.x} cy=${-robotState.fk.coxa.y} r=${0.05} fill="#173042" />
                <circle cx=${robotState.fk.foot.x} cy=${-robotState.fk.foot.y} r=${0.055} fill="#184d74" />
                ${config.footTargetEnabled
                  ? html`
                      <g>
                        <circle cx=${config.targetX} cy=${-config.targetY} r=${0.07} fill="none" stroke=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"} strokeWidth=${0.03} />
                        <line x1=${config.targetX - 0.08} y1=${-config.targetY} x2=${config.targetX + 0.08} y2=${-config.targetY} stroke=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"} strokeWidth=${0.025} />
                        <line x1=${config.targetX} y1=${-config.targetY - 0.08} x2=${config.targetX} y2=${-config.targetY + 0.08} stroke=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"} strokeWidth=${0.025} />
                      </g>
                    `
                  : null}
              </svg>
            </div>
          </div>

          <div className="section">
            <div className="canvas-section-header">
              <div>
                <h3>3D Preview</h3>
                <p>Orbitable engineering view</p>
              </div>
            </div>
            ${config.show3dPreview
              ? html`
                  <div className=${draggingPreview ? "mini-stage orbiting" : "mini-stage"}>
                    <svg
                      ref=${previewSvgRef}
                      viewBox=${`${-previewExtent} ${-previewExtent} ${previewExtent * 2} ${previewExtent * 2}`}
                      width="100%"
                      height="100%"
                      onPointerDown=${handlePreviewPointerDown}
                      onPointerMove=${handlePreviewPointerMove}
                      onPointerUp=${handlePreviewPointerUp}
                      onPointerLeave=${() => {
                        setDraggingPreview(false);
                        orbitRef.current = null;
                      }}
                      style=${{ touchAction: "none" }}
                    >
                      ${planeGrid.map((line, index) => {
                        const start = project3D(line[0], cameraAzimuth, cameraElevation);
                        const end = project3D(line[1], cameraAzimuth, cameraElevation);
                        return html`
                          <line
                            key=${`plane-${index}`}
                            x1=${start.x}
                            y1=${start.y}
                            x2=${end.x}
                            y2=${end.y}
                            stroke="rgba(23, 48, 66, 0.1)"
                            strokeWidth=${0.018}
                          />
                        `;
                      })}

                      <line x1=${axes.origin.x} y1=${axes.origin.y} x2=${axes.x.x} y2=${axes.x.y} stroke="#f57c3d" strokeWidth=${0.028} />
                      <line x1=${axes.origin.x} y1=${axes.origin.y} x2=${axes.y.x} y2=${axes.y.y} stroke="#0f8b8d" strokeWidth=${0.028} />
                      <line x1=${axes.origin.x} y1=${axes.origin.y} x2=${axes.z.x} y2=${axes.z.y} stroke="#184d74" strokeWidth=${0.028} />
                      <text x=${axes.x.x + 0.06} y=${axes.x.y} fontSize=${0.12} fill="#f57c3d">X</text>
                      <text x=${axes.y.x + 0.06} y=${axes.y.y} fontSize=${0.12} fill="#0f8b8d">Y</text>
                      <text x=${axes.z.x + 0.06} y=${axes.z.y} fontSize=${0.12} fill="#184d74">Z</text>

                      <line x1=${projected3d[0].x} y1=${projected3d[0].y} x2=${projected3d[1].x} y2=${projected3d[1].y} stroke="#173042" strokeWidth=${0.05} strokeLinecap="round" />
                      <line x1=${projected3d[1].x} y1=${projected3d[1].y} x2=${projected3d[2].x} y2=${projected3d[2].y} stroke="#0f8b8d" strokeWidth=${0.05} strokeLinecap="round" />
                      <line x1=${projected3d[2].x} y1=${projected3d[2].y} x2=${projected3d[3].x} y2=${projected3d[3].y} stroke="#184d74" strokeWidth=${0.045} strokeLinecap="round" />
                      ${config.footTargetEnabled
                        ? html`
                            <g>
                              <circle cx=${projectedTarget.x} cy=${projectedTarget.y} r=${0.06} fill="none" stroke=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"} strokeWidth=${0.024} />
                              <line x1=${projectedTarget.x - 0.07} y1=${projectedTarget.y} x2=${projectedTarget.x + 0.07} y2=${projectedTarget.y} stroke=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"} strokeWidth=${0.02} />
                              <line x1=${projectedTarget.x} y1=${projectedTarget.y - 0.07} x2=${projectedTarget.x} y2=${projectedTarget.y + 0.07} stroke=${robotState.ik.reachable ? "#f57c3d" : "#d9485f"} strokeWidth=${0.02} />
                            </g>
                          `
                        : null}
                      ${projected3d.map(
                        (point, index) => html`
                          <circle key=${`iso-${index}`} cx=${point.x} cy=${point.y} r=${0.045} fill=${index === 3 ? "#184d74" : "#173042"} />
                        `
                      )}
                    </svg>
                    <div className="orbit-hint">
                      ${draggingPreview ? "Orbiting preview" : "Drag to orbit the preview"}
                    </div>
                  </div>
                  <div className="control-grid compact-grid">
                    <label className="compact-label">
                      <span>Camera Azimuth</span>
                      <span>${formatNumber(config.cameraAzimuthDeg, 0)} deg</span>
                    </label>
                    <input type="range" min="-180" max="180" step="1" value=${config.cameraAzimuthDeg} onInput=${(event) => onCameraChange("cameraAzimuthDeg", Number(event.target.value))} />
                    <label className="compact-label">
                      <span>Camera Elevation</span>
                      <span>${formatNumber(config.cameraElevationDeg, 0)} deg</span>
                    </label>
                    <input type="range" min="5" max="80" step="1" value=${config.cameraElevationDeg} onInput=${(event) => onCameraChange("cameraElevationDeg", Number(event.target.value))} />
                  </div>
                `
              : html`
                  <div className="preview-disabled">
                    3D preview is disabled. Turn it back on in the display settings card.
                  </div>
                `}
          </div>

          <div className="section">
            <h3>Leg Status</h3>
            <div className="info-grid">
              <${MetaCard} title="Foot" value=${`(${formatNumber(robotState.fk.foot.x)}, ${formatNumber(robotState.fk.foot.y)}, ${formatNumber(robotState.fk.foot.z)})`} detail="Current foot position" />
              <${MetaCard} title="Hip Yaw" value=${`${formatNumber(robotState.activeTheta1 * 180 / Math.PI, 1)} deg`} detail="Top-down steering joint" />
              <${MetaCard} title="Planar Reach" value=${`${formatNumber(robotState.ik.planeRadius)} m`} detail="Distance from coxa tip to foot in sagittal plane" />
              <${MetaCard} title="Velocity" value=${`(${formatNumber(robotState.velocity.xDot)}, ${formatNumber(robotState.velocity.yDot)}, ${formatNumber(robotState.velocity.zDot)})`} detail="Cartesian leg velocity" />
            </div>
          </div>
        </div>
      </div>
    </section>
  `;
}

export function RobotCanvas(props) {
  if (props.mode === "leg3") {
    return html`
      <${Leg3Canvas}
        config=${props.config}
        robotState=${props.robotState}
        onTargetChange=${props.onTargetChange}
        onCameraChange=${props.onCameraChange}
      />
    `;
  }
  return html`
    <${Planar2Canvas}
      config=${props.config}
      robotState=${props.robotState}
      trajectory=${props.trajectory}
      onTargetChange=${props.onTargetChange}
    />
  `;
}
