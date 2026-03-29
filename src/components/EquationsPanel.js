import { html, MathBlock, useDeferredValue } from "../lib.js";
import { formatNumber, radToDeg } from "../utils/math.js";

function matrixToTex(matrix) {
  return `\\begin{bmatrix}
${matrix.map((row) => row.join(" & ")).join(" \\\\ ")}
\\end{bmatrix}`;
}

function Planar2Equations({ config, robotState }) {
  const deferredConfig = useDeferredValue(config);
  const deferredState = useDeferredValue(robotState);
  const x1 = deferredState.fk.joint1.x;
  const y1 = deferredState.fk.joint1.y;
  const x = deferredState.fk.endEffector.x;
  const y = deferredState.fk.endEffector.y;
  const theta1 = deferredState.activeTheta1;
  const theta2 = deferredState.activeTheta2;
  const theta1Display =
    deferredConfig.angleUnit === "deg"
      ? `${formatNumber(radToDeg(theta1), 2)}^\\circ`
      : `${formatNumber(theta1, 4)}\\,\\text{rad}`;
  const theta2Display =
    deferredConfig.angleUnit === "deg"
      ? `${formatNumber(radToDeg(theta2), 2)}^\\circ`
      : `${formatNumber(theta2, 4)}\\,\\text{rad}`;
  const jacobianTex = matrixToTex(
    deferredState.jacobian.map((row) => row.map((entry) => formatNumber(entry, 4)))
  );

  return html`
    <section className="panel math-panel">
      <div className="section-header">
        <h2>Dynamic Math</h2>
        <small>Live symbolic and numeric evaluation</small>
      </div>

      <div className="panel-stack">
        <div className="section">
          <h3>Forward Kinematics</h3>
          <div className="math-caption">Symbolic form</div>
          <${MathBlock} tex=${`x = L_1 \\cos(\\theta_1) + L_2 \\cos(\\theta_1 + \\theta_2), \\quad
          y = L_1 \\sin(\\theta_1) + L_2 \\sin(\\theta_1 + \\theta_2)`} />
          <div className="math-caption">Current substitution</div>
          <${MathBlock} tex=${`x_1 = ${formatNumber(deferredConfig.L1, 3)}\\cos(${theta1Display}) = ${formatNumber(x1, 4)}`} />
          <${MathBlock} tex=${`y_1 = ${formatNumber(deferredConfig.L1, 3)}\\sin(${theta1Display}) = ${formatNumber(y1, 4)}`} />
          <${MathBlock} tex=${`x = ${formatNumber(deferredConfig.L1, 3)}\\cos(${theta1Display}) + ${formatNumber(deferredConfig.L2, 3)}\\cos(${theta1Display} + ${theta2Display}) = ${formatNumber(x, 4)}`} />
          <${MathBlock} tex=${`y = ${formatNumber(deferredConfig.L1, 3)}\\sin(${theta1Display}) + ${formatNumber(deferredConfig.L2, 3)}\\sin(${theta1Display} + ${theta2Display}) = ${formatNumber(y, 4)}`} />
        </div>

        <div className="section">
          <h3>Inverse Kinematics</h3>
          <div className="math-caption">Closed-form solution</div>
          <${MathBlock} tex=${`\\cos(\\theta_2) = \\frac{x^2 + y^2 - L_1^2 - L_2^2}{2L_1L_2}, \\quad
          \\theta_2 = \\operatorname{atan2}(\\pm\\sqrt{1 - \\cos^2(\\theta_2)}, \\cos(\\theta_2))`} />
          <${MathBlock} tex=${`\\theta_1 = \\operatorname{atan2}(y, x) - \\operatorname{atan2}(L_2\\sin(\\theta_2), L_1 + L_2\\cos(\\theta_2))`} />
          <div className="math-caption">Current substitution</div>
          <${MathBlock} tex=${`\\cos(\\theta_2) = \\frac{${formatNumber(deferredConfig.targetX, 4)}^2 + ${formatNumber(deferredConfig.targetY, 4)}^2 - ${formatNumber(deferredConfig.L1, 4)}^2 - ${formatNumber(deferredConfig.L2, 4)}^2}{2(${formatNumber(deferredConfig.L1, 4)})(${formatNumber(deferredConfig.L2, 4)})} = ${formatNumber(deferredState.ik.rawCosTheta2, 5)}`} />
          <${MathBlock} tex=${`\\theta_2 = ${theta2Display}, \\quad \\theta_1 = ${theta1Display}`} />
          <div className="footer-note">
            ${deferredState.ik.reachable
              ? "The selected IK branch is currently reachable."
              : "The raw cosine term lies outside [-1, 1], so the target is outside the reachable workspace and the solver clamps to the nearest valid pose."}
          </div>
        </div>

        <div className="section">
          <h3>Jacobian and Velocity</h3>
          <${MathBlock} tex=${`J = ${jacobianTex}`} />
          <${MathBlock} tex=${`\\det(J) = ${formatNumber(deferredState.determinant, 6)}`} />
          <${MathBlock} tex=${`\\begin{bmatrix}\\dot{x} \\\\ \\dot{y}\\end{bmatrix} = J \\begin{bmatrix}\\dot{\\theta}_1 \\\\ \\dot{\\theta}_2\\end{bmatrix} = \\begin{bmatrix}${formatNumber(deferredState.velocity.xDot, 4)} \\\\ ${formatNumber(deferredState.velocity.yDot, 4)}\\end{bmatrix}`} />
        </div>
      </div>
    </section>
  `;
}

function Leg3Equations({ config, robotState }) {
  const deferredConfig = useDeferredValue(config);
  const deferredState = useDeferredValue(robotState);
  const theta1 = deferredState.activeTheta1;
  const theta2 = deferredState.activeTheta2;
  const theta3 = deferredState.activeTheta3;
  const angle = (value) =>
    deferredConfig.angleUnit === "deg"
      ? `${formatNumber(radToDeg(value), 2)}^\\circ`
      : `${formatNumber(value, 4)}\\,\\text{rad}`;
  const jacobianTex = matrixToTex(
    deferredState.jacobian.map((row) => row.map((entry) => formatNumber(entry, 4)))
  );

  return html`
    <section className="panel math-panel">
      <div className="section-header">
        <h2>Leg Kinematics</h2>
        <small>Yaw plus sagittal-plane femur/tibia equations</small>
      </div>

      <div className="panel-stack">
        <div className="section">
          <h3>Forward Kinematics</h3>
          <div className="math-caption">Symbolic form</div>
          <${MathBlock} tex=${`x = \\cos(\\theta_1)\\left(L_1 + L_2\\cos(\\theta_2) + L_3\\cos(\\theta_2 + \\theta_3)\\right)`} />
          <${MathBlock} tex=${`y = \\sin(\\theta_1)\\left(L_1 + L_2\\cos(\\theta_2) + L_3\\cos(\\theta_2 + \\theta_3)\\right)`} />
          <${MathBlock} tex=${`z = L_2\\sin(\\theta_2) + L_3\\sin(\\theta_2 + \\theta_3)`} />
          <div className="math-caption">Intermediate joints</div>
          <${MathBlock} tex=${`\\mathbf{p}_{coxa} = \\begin{bmatrix}L_1\\cos(\\theta_1) \\\\ L_1\\sin(\\theta_1) \\\\ 0\\end{bmatrix} = \\begin{bmatrix}${formatNumber(deferredState.fk.coxa.x, 4)} \\\\ ${formatNumber(deferredState.fk.coxa.y, 4)} \\\\ 0\\end{bmatrix}`} />
          <${MathBlock} tex=${`\\mathbf{p}_{knee} = \\begin{bmatrix}(L_1 + L_2\\cos(\\theta_2))\\cos(\\theta_1) \\\\ (L_1 + L_2\\cos(\\theta_2))\\sin(\\theta_1) \\\\ L_2\\sin(\\theta_2)\\end{bmatrix} = \\begin{bmatrix}${formatNumber(deferredState.fk.knee.x, 4)} \\\\ ${formatNumber(deferredState.fk.knee.y, 4)} \\\\ ${formatNumber(deferredState.fk.knee.z, 4)}\\end{bmatrix}`} />
          <${MathBlock} tex=${`\\mathbf{p}_{foot} = \\begin{bmatrix}${formatNumber(deferredState.fk.foot.x, 4)} \\\\ ${formatNumber(deferredState.fk.foot.y, 4)} \\\\ ${formatNumber(deferredState.fk.foot.z, 4)}\\end{bmatrix}`} />
        </div>

        <div className="section">
          <h3>Inverse Kinematics Assumption</h3>
          <div className="footer-note">
            Hip yaw is solved from the XY target, then the remaining femur/tibia problem is solved in the sagittal plane using the coxa tip as the planar origin. The UI exposes folded and extended knee branches.
          </div>
          <${MathBlock} tex=${`\\theta_1 = \\operatorname{atan2}(y, x), \\quad r = \\sqrt{x^2 + y^2}, \\quad x_p = r - L_1`} />
          <${MathBlock} tex=${`\\cos(\\theta_3) = \\frac{x_p^2 + z^2 - L_2^2 - L_3^2}{2L_2L_3}, \\quad \\theta_2 = \\operatorname{atan2}(z, x_p) - \\operatorname{atan2}(L_3\\sin(\\theta_3), L_2 + L_3\\cos(\\theta_3))`} />
          <div className="math-caption">Current substitution</div>
          <${MathBlock} tex=${`\\theta_1 = ${angle(theta1)}, \\quad \\theta_2 = ${angle(theta2)}, \\quad \\theta_3 = ${angle(theta3)}`} />
          <${MathBlock} tex=${`x_p = ${formatNumber(deferredState.ik.planeX, 4)}, \\quad z = ${formatNumber(deferredState.ik.planeY, 4)}, \\quad \\cos(\\theta_3) = ${formatNumber(deferredState.ik.rawCosTheta3, 5)}`} />
        </div>

        <div className="section">
          <h3>Jacobian and Foot Velocity</h3>
          <${MathBlock} tex=${`J = ${jacobianTex}`} />
          <${MathBlock} tex=${`\\det(J) = ${formatNumber(deferredState.determinant, 6)}`} />
          <${MathBlock} tex=${`\\dot{\\mathbf{p}} = J\\dot{\\mathbf{q}} = \\begin{bmatrix}${formatNumber(deferredState.velocity.xDot, 4)} \\\\ ${formatNumber(deferredState.velocity.yDot, 4)} \\\\ ${formatNumber(deferredState.velocity.zDot, 4)}\\end{bmatrix}`} />
        </div>
      </div>
    </section>
  `;
}

export function EquationsPanel({ mode, config, robotState }) {
  return mode === "leg3"
    ? html`<${Leg3Equations} config=${config} robotState=${robotState} />`
    : html`<${Planar2Equations} config=${config} robotState=${robotState} />`;
}
