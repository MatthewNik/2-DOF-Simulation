# 2 DOF Planar Robot Arm Simulator

Single-page Vite + React web app for studying and prototyping a 2-link planar manipulator with:

- Forward kinematics and inverse kinematics
- Elbow-up and elbow-down IK branches
- Jacobian, determinant, singularity warning, and end-effector velocity
- Joint limits, calibration offsets, motor direction inversion, and servo command conversion
- Workspace visualization, target dragging, target clicking, trail drawing, waypoint playback, and presets
- Separate in-app pages for `Workspace and Math` and `Robot State and Export`
- JSON / Arduino Servo / Arduino Stepper / ESP32 Stepper / Python export helpers for later real-robot integration

## Run

This project now uses a proper Node-based frontend workflow with Vite.

1. Open a terminal in this folder.
2. Install dependencies:

```powershell
npm.cmd install
```

3. Start the dev server:

```powershell
npm.cmd run dev
```

4. Open the local URL printed by Vite.

For a production build:

```powershell
npm.cmd run build
```

## Deploy on Vercel

1. Push this folder to a GitHub repository.
2. Go to [Vercel](https://vercel.com) and create a new project.
3. Import the GitHub repository.
4. Vercel should detect it as a Vite app automatically.
5. If prompted, use:

```text
Build Command: npm run build
Output Directory: dist
```

6. Click deploy.

The project includes `vercel.json` so Vercel has explicit build settings.
Vercel runs the build on Linux, so the deploy config uses `npm run build` rather than `npm.cmd run build`.

## Folder structure

```text
.
|-- index.html
|-- package.json
|-- README.md
`-- src
    |-- components
    |   |-- App.js
    |   |-- ControlsPanel.js
    |   |-- EquationsPanel.js
    |   |-- StatePanel.js
    |   `-- VisualizationPanel.js
    |-- utils
    |   |-- export.js
    |   |-- kinematics.js
    |   |-- math.js
    |   `-- storage.js
    |-- constants.js
    |-- lib.js
    |-- main.js
    `-- styles.css
```

## Implementation notes

### Forward kinematics

The app uses the standard 2-link planar model:

- `x1 = L1 cos(theta1)`
- `y1 = L1 sin(theta1)`
- `x = L1 cos(theta1) + L2 cos(theta1 + theta2)`
- `y = L1 sin(theta1) + L2 sin(theta1 + theta2)`

The intermediate joint position and end-effector position are both computed every frame.

### Inverse kinematics

IK uses the closed-form geometric solution:

- `cos(theta2) = (x^2 + y^2 - L1^2 - L2^2) / (2 L1 L2)`
- `theta2 = atan2(+-sqrt(1 - cos^2(theta2)), cos(theta2))`
- `theta1 = atan2(y, x) - atan2(L2 sin(theta2), L1 + L2 cos(theta2))`

The implementation clamps `cos(theta2)` into `[-1, 1]` for numerical stability, while separately reporting whether the original target is actually reachable. That makes boundary behavior stable and gives a sensible projected pose for unreachable targets.

### Jacobian

The Jacobian is evaluated from the active joint angles:

```text
J = [
  -L1 sin(theta1) - L2 sin(theta1 + theta2),   -L2 sin(theta1 + theta2)
   L1 cos(theta1) + L2 cos(theta1 + theta2),    L2 cos(theta1 + theta2)
]
```

The app also computes:

- `det(J)` for singularity detection
- `x_dot, y_dot = J * [theta1_dot, theta2_dot]`
- warnings when `|det(J)|` is close to zero

### Hardware-oriented structure

The code keeps simulation state, calibration, joint limits, and export generation separate so it can be adapted later to:

- Arduino servo control
- ESP32 control loops
- Python control / logging / desktop tooling

The export panel now generates move-to oriented starter code for each target. The generated snippets include:

- A closed-form IK solver
- Joint-limit checks
- Zero offsets and direction inversion
- `moveToXY` or `move_to` style helpers
- Home target values taken from the current simulator state

The stepper exports also include placeholder steps-per-radian constants that you can tune for your motor, microstepping, and gearbox configuration.

## Notes

- Math rendering now uses local KaTeX assets bundled through Vite.
- If `npm` is blocked in PowerShell, use `npm.cmd` or update your execution policy.
