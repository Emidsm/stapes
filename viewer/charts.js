// The two 2D views, drawn on plain canvases: the view down the drill axis, and the tip error
// over time.

const style = getComputedStyle(document.documentElement);
const colour = (name) => style.getPropertyValue(name).trim();
const INK = colour("--ink");
const TEAL = colour("--teal");
const MUTED = colour("--muted");
const RULE = colour("--rule");
const SIGNAL_RED = colour("--signal-red");
const FONT = "22px system-ui, sans-serif";

// How far the across view reaches from the line, in micrometres.
const ACROSS_RANGE_UM = 420;
const TRAIL_TICKS = 400;

export function drawAcross(canvas, trace, tick) {
  const context = canvas.getContext("2d");
  const size = canvas.width;
  const centre = size / 2;
  const scale = (centre - 24) / ACROSS_RANGE_UM;
  const { across_u_um: u, across_v_um: v } = trace.series;
  const stopped = trace.events.stop_tick !== null && tick >= trace.events.stop_tick;

  context.clearRect(0, 0, size, size);
  context.lineWidth = 2;

  // Cross hairs through the planned line.
  context.strokeStyle = RULE;
  context.beginPath();
  context.moveTo(centre, 12);
  context.lineTo(centre, size - 12);
  context.moveTo(12, centre);
  context.lineTo(size - 12, centre);
  context.stroke();

  const ring = (radiusUm, dashed, strokeStyle, label) => {
    context.strokeStyle = strokeStyle;
    context.setLineDash(dashed ? [4, 8] : []);
    context.beginPath();
    context.arc(centre, centre, radiusUm * scale, 0, 2 * Math.PI);
    context.stroke();
    context.setLineDash([]);
    context.fillStyle = strokeStyle;
    context.font = FONT;
    context.fillText(label, centre + 8, centre - radiusUm * scale - 8);
  };
  ring(trace.limits.design_target_um, true, INK, "0.2 mm");
  if (trace.limits.corridor_radius_um !== null) {
    ring(trace.limits.corridor_radius_um, false, MUTED, "0.3 mm");
  }

  // The recent path of the tip.
  const first = Math.max(0, tick - TRAIL_TICKS);
  context.strokeStyle = TEAL;
  context.globalAlpha = 0.45;
  context.beginPath();
  for (let i = first; i <= tick; i += 1) {
    const x = centre + u[i] * scale;
    const y = centre - v[i] * scale;
    if (i === first) {
      context.moveTo(x, y);
    } else {
      context.lineTo(x, y);
    }
  }
  context.stroke();
  context.globalAlpha = 1;

  // The tip now. Off the scale, it is held at the edge and drawn as an open ring.
  const distance = Math.hypot(u[tick], v[tick]);
  const shrink = distance > ACROSS_RANGE_UM ? ACROSS_RANGE_UM / distance : 1;
  const x = centre + u[tick] * shrink * scale;
  const y = centre - v[tick] * shrink * scale;
  context.beginPath();
  context.arc(x, y, 9, 0, 2 * Math.PI);
  context.fillStyle = stopped ? SIGNAL_RED : TEAL;
  context.strokeStyle = stopped ? SIGNAL_RED : TEAL;
  context.lineWidth = 3;
  if (shrink < 1) {
    context.stroke();
    context.fillStyle = INK;
    context.font = FONT;
    context.textAlign = "center";
    context.fillText(`off the scale: ${formatDistance(distance)}`, centre, size - 20);
    context.textAlign = "start";
  } else {
    context.fill();
  }
}

export function formatDistance(micrometres) {
  if (micrometres >= 1e6) {
    return `${(micrometres / 1e6).toFixed(micrometres >= 1e7 ? 0 : 1)} m`;
  }
  if (micrometres >= 1000) {
    return `${(micrometres / 1000).toFixed(micrometres >= 10000 ? 0 : 1)} mm`;
  }
  return `${micrometres.toFixed(0)} µm`;
}

// A run without the monitor reaches a metre; a monitored one stays under half a millimetre. The
// first needs a logarithmic axis to be readable at all.
function verticalAxis(trace, top, bottom) {
  const peak = Math.max(...trace.series.radial_um);
  if (peak > 1000) {
    const low = Math.log10(1);
    const high = Math.log10(peak * 1.3);
    return {
      y: (value) => bottom - ((Math.log10(Math.max(value, 1)) - low) / (high - low)) * (bottom - top),
      ticks: [10, 100, 1000, 10000, 100000, 1000000].filter((t) => t < peak * 1.3),
    };
  }
  const high = Math.max(400, peak * 1.1);
  return {
    y: (value) => bottom - (value / high) * (bottom - top),
    ticks: [100, 200, 300, 400].filter((t) => t <= high),
  };
}

export function drawSeries(canvas, trace, tick) {
  const context = canvas.getContext("2d");
  const { width, height } = canvas;
  const left = 112;
  const right = width - 16;
  const top = 16;
  const bottom = height - 44;
  const radial = trace.series.radial_um;
  const count = radial.length;
  const x = (index) => left + (index / (count - 1)) * (right - left);
  const axis = verticalAxis(trace, top, bottom);

  context.clearRect(0, 0, width, height);
  context.font = FONT;
  context.lineWidth = 1.5;

  // Grid and labels.
  context.textAlign = "right";
  for (const value of axis.ticks) {
    context.strokeStyle = RULE;
    context.beginPath();
    context.moveTo(left, axis.y(value));
    context.lineTo(right, axis.y(value));
    context.stroke();
    context.fillStyle = MUTED;
    context.fillText(formatDistance(value), left - 10, axis.y(value) + 7);
  }
  context.textAlign = "center";
  const seconds = (count * trace.period_s) | 0;
  for (let s = 0; s <= seconds; s += 1) {
    context.fillStyle = MUTED;
    context.fillText(`${s} s`, x(s / trace.period_s), height - 12);
  }
  context.textAlign = "start";

  // Limits.
  const limit = (value, dashed, strokeStyle) => {
    context.strokeStyle = strokeStyle;
    context.setLineDash(dashed ? [4, 8] : []);
    context.beginPath();
    context.moveTo(left, axis.y(value));
    context.lineTo(right, axis.y(value));
    context.stroke();
    context.setLineDash([]);
  };
  limit(trace.limits.design_target_um, true, INK);
  if (trace.limits.corridor_radius_um !== null) {
    limit(trace.limits.corridor_radius_um, false, MUTED);
  }

  // The error itself.
  context.strokeStyle = TEAL;
  context.lineWidth = 3;
  context.beginPath();
  for (let i = 0; i < count; i += 1) {
    if (i === 0) {
      context.moveTo(x(i), axis.y(radial[i]));
    } else {
      context.lineTo(x(i), axis.y(radial[i]));
    }
  }
  context.stroke();

  // Events: the only red on the chart.
  const event = (eventTick, dashed) => {
    if (eventTick === null) {
      return;
    }
    context.strokeStyle = SIGNAL_RED;
    context.lineWidth = dashed ? 2 : 3;
    context.setLineDash(dashed ? [10, 8] : []);
    context.beginPath();
    context.moveTo(x(eventTick), top);
    context.lineTo(x(eventTick), bottom);
    context.stroke();
    context.setLineDash([]);
  };
  event(trace.events.fault_start_tick, true);
  event(trace.events.stop_tick, false);

  // The playhead.
  context.strokeStyle = INK;
  context.lineWidth = 2;
  context.beginPath();
  context.moveTo(x(tick), top);
  context.lineTo(x(tick), bottom);
  context.stroke();
  context.fillStyle = INK;
  context.beginPath();
  context.arc(x(tick), axis.y(radial[tick]), 7, 0, 2 * Math.PI);
  context.fill();
}
