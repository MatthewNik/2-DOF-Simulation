export function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

export function degToRad(value) {
  return (value * Math.PI) / 180;
}

export function radToDeg(value) {
  return (value * 180) / Math.PI;
}

export function lerp(start, end, t) {
  return start + (end - start) * t;
}

export function distance(a, b) {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  return Math.hypot(dx, dy);
}

export function nearlyEqual(a, b, tolerance = 1e-6) {
  return Math.abs(a - b) <= tolerance;
}

export function formatNumber(value, digits = 3) {
  if (!Number.isFinite(value)) {
    return "NaN";
  }

  if (Math.abs(value) >= 1000) {
    return value.toFixed(1);
  }

  return value.toFixed(digits);
}

export function formatAngleRadians(value, digits = 3) {
  return `${formatNumber(value, digits)} rad`;
}

export function formatAngleDegrees(value, digits = 2) {
  return `${formatNumber(radToDeg(value), digits)} deg`;
}

export function sanitizeNumber(value, fallback = 0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

export function arcPath(cx, cy, radius, startAngle, endAngle) {
  const start = {
    x: cx + radius * Math.cos(startAngle),
    y: cy - radius * Math.sin(startAngle)
  };
  const end = {
    x: cx + radius * Math.cos(endAngle),
    y: cy - radius * Math.sin(endAngle)
  };
  const largeArc = Math.abs(endAngle - startAngle) > Math.PI ? 1 : 0;
  const sweep = endAngle > startAngle ? 0 : 1;
  return `M ${start.x} ${start.y} A ${radius} ${radius} 0 ${largeArc} ${sweep} ${end.x} ${end.y}`;
}

export function circlePath(radius) {
  return `M ${radius} 0 A ${radius} ${radius} 0 1 0 ${-radius} 0 A ${radius} ${radius} 0 1 0 ${radius} 0`;
}
