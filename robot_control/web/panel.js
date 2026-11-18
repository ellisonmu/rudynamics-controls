// Shared code for both panels: API calls, state polling, top bar, leg drawing.

const Panel = (() => {
  const deg = (rad) => (rad * 180) / Math.PI;
  const rad = (d) => (d * Math.PI) / 180;
  const fmt = (v, digits = 1) => (v == null ? "--" : v.toFixed(digits));

  let toastTimer = null;
  function toast(msg) {
    const el = document.getElementById("toast");
    el.textContent = msg;
    el.classList.add("show");
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => el.classList.remove("show"), 3500);
  }

  async function api(path, body) {
    try {
      const res = await fetch(path, {
        method: body === undefined ? "GET" : "POST",
        headers: { "Content-Type": "application/json" },
        body: body === undefined ? undefined : JSON.stringify(body),
      });
      const data = await res.json();
      if (!res.ok) {
        toast(data.error || `error ${res.status}`);
        return null;
      }
      return data;
    } catch (e) {
      toast("can't reach the leg server");
      return null;
    }
  }

  // ---------- top bar ----------
  function bindHeader() {
    const on = (id, fn) => document.getElementById(id).addEventListener("click", fn);
    on("btn-enable", () => api("/api/enable", {}));
    on("btn-disable", () => api("/api/disable", {}));
    on("btn-estop", () => api("/api/estop", {}));
    on("btn-calibrate", () => {
      if (confirm("Is the leg hanging straight down (all joints at zero)?\nThis pose will become the zero reference.")) {
        api("/api/calibrate", {});
      }
    });
    on("btn-clear", () => api("/api/clear_fault", {}));
    document.addEventListener("keydown", (e) => {
      if (e.code === "Space" && e.target.tagName !== "INPUT") {
        e.preventDefault();
        api("/api/estop", {});
      }
    });
  }

  function renderHeader(s) {
    const chip = document.getElementById("mode-chip");
    chip.textContent = s ? s.mode : "offline";
    chip.className = "chip " + (s ? s.mode : "offline");
    document.getElementById("status-msg").textContent = s ? s.message : "no connection to leg server";
    document.getElementById("loop-hz").textContent = s ? `${s.loop_hz} Hz` : "--";
    document.getElementById("calib").textContent = s && s.calibrated ? "calibrated" : "NOT calibrated";
    const enabled = s && (s.mode === "manual" || s.mode === "motion");
    document.getElementById("btn-enable").disabled = !s || enabled || s.mode === "fault";
    document.getElementById("btn-disable").disabled = !s || s.mode === "disabled";
    document.getElementById("btn-calibrate").disabled = !s || enabled;
    document.getElementById("btn-clear").style.display = s && s.mode === "fault" ? "" : "none";
  }

  // ---------- polling ----------
  function start({ onConfig, onState, hz = 20 }) {
    bindHeader();
    api("/api/config").then((cfg) => cfg && onConfig && onConfig(cfg));
    let busy = false;
    setInterval(async () => {
      if (busy) return;
      busy = true;
      let s = null;
      try {
        const res = await fetch("/api/state", { cache: "no-store" });
        if (res.ok) s = await res.json();
      } catch (e) {
        s = null;
      }
      busy = false;
      renderHeader(s);
      onState && onState(s);
    }, 1000 / hz);
  }

  // ---------- leg drawing ----------
  // view "side": x (forward, right) vs z;  view "front": y (left, shown on the left) vs z
  const NS = "http://www.w3.org/2000/svg";

  function makeView(svg, geom) {
    const R = geom.thigh + geom.shank;
    svg.setAttribute("viewBox", `${-0.75 * R} ${-0.2 * R} ${1.5 * R} ${1.3 * R}`);
    svg.innerHTML = "";
    const g = (tag, attrs) => {
      const el = document.createElementNS(NS, tag);
      for (const k in attrs) {
        if (k === "fill" || k === "stroke") el.style.setProperty(k, attrs[k]);
        else el.setAttribute(k, attrs[k]);
      }
      el.setAttribute("vector-effect", "non-scaling-stroke");
      svg.appendChild(el);
      return el;
    };
    // reach circle + ground reference at the default stance height
    g("circle", { cx: 0, cy: 0, r: R, fill: "none", stroke: "var(--line)", "stroke-dasharray": "3 4", "stroke-width": 1 });
    g("line", { x1: -0.75 * R, x2: 0.75 * R, y1: 0.7 * R, y2: 0.7 * R, stroke: "var(--line)", "stroke-width": 1 });
    g("rect", { x: -0.06 * R, y: -0.08 * R, width: 0.12 * R, height: 0.1 * R, rx: 0.02 * R, fill: "var(--panel)", stroke: "var(--muted)", "stroke-width": 1.5 });
    const trail = g("polyline", { fill: "none", stroke: "var(--accent)", "stroke-opacity": 0.35, "stroke-width": 2 });
    const ghost = g("polyline", { fill: "none", stroke: "var(--ghost)", "stroke-opacity": 0.55, "stroke-width": 3, "stroke-dasharray": "5 4", "stroke-linecap": "round", "stroke-linejoin": "round" });
    const leg = g("polyline", { fill: "none", stroke: "var(--accent)", "stroke-width": 7, "stroke-linecap": "round", "stroke-linejoin": "round" });
    const knee = g("circle", { r: 0.03 * R, fill: "var(--bg)", stroke: "var(--accent)", "stroke-width": 2.5 });
    const foot = g("circle", { r: 0.035 * R, fill: "var(--accent-2)" });
    g("circle", { cx: 0, cy: 0, r: 0.03 * R, fill: "var(--bg)", stroke: "var(--muted)", "stroke-width": 2.5 });
    return { trail, ghost, leg, knee, foot, points: [] };
  }

  // project a 3D point to the view's 2D coordinates (svg y = -z)
  const project = (p, view) => (view === "side" ? [p[0], -p[2]] : [-p[1], -p[2]]);
  const pts = (list) => list.map((p) => p.join(",")).join(" ");

  function drawLeg(v, view, s, { trail = false } = {}) {
    if (!s || !s.measured_pts) return;
    const k = project(s.measured_pts.knee, view);
    const f = project(s.measured_pts.foot, view);
    v.leg.setAttribute("points", pts([[0, 0], k, f]));
    v.knee.setAttribute("cx", k[0]); v.knee.setAttribute("cy", k[1]);
    v.foot.setAttribute("cx", f[0]); v.foot.setAttribute("cy", f[1]);
    if (s.cmd_pts) {
      const ck = project(s.cmd_pts.knee, view);
      const cf = project(s.cmd_pts.foot, view);
      v.ghost.setAttribute("points", pts([[0, 0], ck, cf]));
    } else {
      v.ghost.setAttribute("points", "");
    }
    if (trail) {
      v.points.push(f);
      if (v.points.length > 60) v.points.shift();
      v.trail.setAttribute("points", pts(v.points));
    }
  }

  function footText(s) {
    if (!s || !s.measured_pts) return "foot: --";
    const [x, y, z] = s.measured_pts.foot.map((m) => (m * 100).toFixed(1));
    return `foot  x ${x}  y ${y}  z ${z} cm`;
  }

  return { api, start, toast, makeView, drawLeg, footText, deg, rad, fmt };
})();
