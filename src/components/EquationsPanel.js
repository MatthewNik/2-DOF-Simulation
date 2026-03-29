import { html, MathBlock, useDeferredValue } from "../lib.js";
import { formatNumber, radToDeg } from "../utils/math.js";

function matrixToTex(matrix) {
  return `\\begin{bmatrix}
${matrix[0][0]} & ${matrix[0][1]} \\\\
${matrix[1][0]} & ${matrix[1][1]}
\\end{bmatrix}`;
}

export function EquationsPanel({ config, robotState }) {
  const deferredConfig = useDeferredValue(config);
  const deferredRobotState = useDeferredValue(robotState);

  const x1 = deferredRobotState.fk.joint1.x;
  const y1 = deferredRobotState.fk.joint1.y;
  const x = deferredRobotState.fk.endEffector.x;
  const y = deferredRobotState.fk.endEffector.y;
  const theta1 = deferredRobotState.activeTheta1;
  const theta2 = deferredRobotState.activeTheta2;
  const theta1Display =
    deferredConfig.angleUnit === "deg"
      ? `${formatNumber(radToDeg(theta1), 2)}^\\circ`
      : `${formatNumber(theta1, 4)}\\,\\text{rad}`;
  const theta2Display =
    deferredConfig.angleUnit === "deg"
      ? `${formatNumber(radToDeg(theta2), 2)}^\\circ`
      : `${formatNumber(theta2, 4)}\\,\\text{rad}`;
  const theta1Plain =
    deferredConfig.angleUnit === "deg"
      ? `${formatNumber(radToDeg(theta1), 2)} deg`
      : `${formatNumber(theta1, 4)} rad`;
  const theta2Plain =
    deferredConfig.angleUnit === "deg"
      ? `${formatNumber(radToDeg(theta2), 2)} deg`
      : `${formatNumber(theta2, 4)} rad`;
  const cos2 = deferredRobotState.ik.cosTheta2;
  const rawCos2 = deferredRobotState.ik.rawCosTheta2;
  const jacobianTex = matrixToTex(
    deferredRobotState.jacobian.map((row) => row.map((entry) => formatNumber(entry, 4)))
  );
  const velocityTex = `\\begin{bmatrix}${formatNumber(deferredRobotState.velocity.xDot, 4)} \\\\ ${formatNumber(deferredRobotState.velocity.yDot, 4)}\\end{bmatrix}`;

  return html`
    <section className="panel math-panel">
      <div className="section-header">
        <h2>Dynamic Math</h2>
        <small>Live symbolic and numeric evaluation</small>
      </div>

      <div className="section">
        <h3>Forward Kinematics</h3>
        <div className="math-caption">Symbolic form</div>
        <${MathBlock}
          tex=${`x = L_1 \\cos(\\theta_1) + L_2 \\cos(\\theta_1 + \\theta_2), \\quad
          y = L_1 \\sin(\\theta_1) + L_2 \\sin(\\theta_1 + \\theta_2)`}
        />
        <div className="math-caption">Current substitution</div>
        <${MathBlock} tex=${`x_1 = ${formatNumber(deferredConfig.L1, 3)}\\cos(${theta1Display}) = ${formatNumber(x1, 4)}`} />
        <${MathBlock} tex=${`y_1 = ${formatNumber(deferredConfig.L1, 3)}\\sin(${theta1Display}) = ${formatNumber(y1, 4)}`} />
        <${MathBlock}
          tex=${`x = ${formatNumber(deferredConfig.L1, 3)}\\cos(${theta1Display}) + ${formatNumber(deferredConfig.L2, 3)}\\cos(${theta1Display} + ${theta2Display}) = ${formatNumber(x, 4)}`}
        />
        <${MathBlock}
          tex=${`y = ${formatNumber(deferredConfig.L1, 3)}\\sin(${theta1Display}) + ${formatNumber(deferredConfig.L2, 3)}\\sin(${theta1Display} + ${theta2Display}) = ${formatNumber(y, 4)}`}
        />
      </div>

      <div className="section">
        <h3>Inverse Kinematics</h3>
        <div className="math-caption">Closed-form solution</div>
        <${MathBlock}
          tex=${`\\cos(\\theta_2) = \\frac{x^2 + y^2 - L_1^2 - L_2^2}{2L_1L_2}, \\quad
          \\theta_2 = \\operatorname{atan2}(\\pm\\sqrt{1 - \\cos^2(\\theta_2)}, \\cos(\\theta_2))`}
        />
        <${MathBlock}
          tex=${`\\theta_1 = \\operatorname{atan2}(y, x) - \\operatorname{atan2}(L_2\\sin(\\theta_2), L_1 + L_2\\cos(\\theta_2))`}
        />
        <div className="math-caption">Current substitution</div>
        <${MathBlock}
          tex=${`\\cos(\\theta_2) = \\frac{${formatNumber(deferredConfig.targetX, 4)}^2 + ${formatNumber(deferredConfig.targetY, 4)}^2 - ${formatNumber(deferredConfig.L1, 4)}^2 - ${formatNumber(deferredConfig.L2, 4)}^2}{2(${formatNumber(deferredConfig.L1, 4)})(${formatNumber(deferredConfig.L2, 4)})}
          = ${formatNumber(rawCos2, 5)}`}
        />
        <${MathBlock}
          tex=${`\\theta_2 = \\operatorname{atan2}\\left(${deferredConfig.elbowMode === "up" ? "+" : "-"}\\sqrt{1 - (${formatNumber(cos2, 5)})^2}, ${formatNumber(cos2, 5)}\\right)
          = ${theta2Display}`}
        />
        <${MathBlock}
          tex=${`\\theta_1 = \\operatorname{atan2}(${formatNumber(deferredConfig.targetY, 4)}, ${formatNumber(deferredConfig.targetX, 4)}) -
          \\operatorname{atan2}\\left(${formatNumber(deferredConfig.L2 * deferredRobotState.ik.sinTheta2, 4)}, ${formatNumber(deferredConfig.L1 + deferredConfig.L2 * deferredRobotState.ik.cosTheta2, 4)}\\right)
          = ${theta1Display}`}
        />
        <div className="footer-note">
          ${deferredRobotState.ik.reachable
            ? "The IK branch is currently reachable."
            : "The raw cosine term lies outside [-1, 1], so the target is outside the reachable workspace and the solver clamps to the nearest valid pose."}
        </div>
      </div>

      <div className="section">
        <h3>Jacobian and Velocity</h3>
        <div className="math-caption">Jacobian</div>
        <${MathBlock}
          tex=${`J = \\begin{bmatrix}
          -L_1\\sin(\\theta_1) - L_2\\sin(\\theta_1 + \\theta_2) & -L_2\\sin(\\theta_1 + \\theta_2) \\\\
          L_1\\cos(\\theta_1) + L_2\\cos(\\theta_1 + \\theta_2) & L_2\\cos(\\theta_1 + \\theta_2)
          \\end{bmatrix}
          = ${jacobianTex}`}
        />
        <${MathBlock} tex=${`\\det(J) = ${formatNumber(deferredRobotState.determinant, 6)}`} />
        <${MathBlock}
          tex=${`\\begin{bmatrix}\\dot{x} \\\\ \\dot{y}\\end{bmatrix}
          = J \\begin{bmatrix}\\dot{\\theta}_1 \\\\ \\dot{\\theta}_2\\end{bmatrix}
          = ${velocityTex}`}
        />
        <div className="info-grid">
          <div className="info-card">
            <strong>Joint 1</strong>
            <span>${theta1Plain}</span>
          </div>
          <div className="info-card">
            <strong>Joint 2</strong>
            <span>${theta2Plain}</span>
          </div>
          <div className="info-card">
            <strong>Joint 1 Coord</strong>
            <span>(${formatNumber(x1)}, ${formatNumber(y1)})</span>
          </div>
          <div className="info-card">
            <strong>End Effector</strong>
            <span>(${formatNumber(x)}, ${formatNumber(y)})</span>
          </div>
        </div>
      </div>
    </section>
  `;
}
