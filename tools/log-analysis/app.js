/* WPILog analysis dashboard.
 *
 * Loads data/summary.json for the fleet view and data/match_<stem>.json on
 * demand for the match view. The UI is analyzer-agnostic: every analyzer
 * contributes columns to the fleet table and event lanes in the match view
 * purely from its summary/events structure, so new analyzers appear without
 * UI edits.
 */

const EVENT_COLORS = {
  jams: "#f85149",
  turret_flips: "#d29922",
  motivator_zero_cause: "#58a6ff",
  firing_intervals: "#56d364",
  bps: "#a371f7",
  slugs: "#3fb950",
  spindexer_events: "#db61a2",
};

const TARGET_BPS = 12.0;

const CAUSE_COLORS = {
  operator_release: "#8b949e",
  turret_flipping: "#d29922",
  speed_gate: "#f85149",
  not_achievable: "#db61a2",
  spindexer_suppressed: "#a371f7",
  spindexer_stopped: "#58a6ff",
  no_fire_zone: "#e3b341",
  settling: "#3fb950",
  unknown: "#6e7681",
};

const state = {
  summary: null,
  currentMatch: null,
  matchCache: new Map(),
  charts: {},
  mainChart: null,
  activeEventKey: null,
};

// --- Boot ---

async function boot() {
  const res = await fetch("data/summary.json");
  state.summary = await res.json();
  document.getElementById("generated-at").textContent =
    "generated " + state.summary.generated_at;

  populateMatchPicker();
  renderFleet();
  hookupNav();
}

function hookupNav() {
  document.getElementById("nav-fleet").onclick = () => showView("fleet");
  document.getElementById("nav-match").onclick = () => showView("match");
  document.getElementById("match-picker").onchange = (e) => loadMatch(e.target.value);
  document.querySelectorAll('.event-filter input[type=checkbox]').forEach((cb) => {
    cb.onchange = () => renderEventList();
  });
}

function showView(which) {
  document.getElementById("nav-fleet").classList.toggle("active", which === "fleet");
  document.getElementById("nav-match").classList.toggle("active", which === "match");
  document.getElementById("fleet-view").hidden = which !== "fleet";
  document.getElementById("match-view").hidden = which !== "match";
  if (which === "match" && state.currentMatch === null) {
    const pick = document.getElementById("match-picker");
    if (pick.value) loadMatch(pick.value);
  }
}

function populateMatchPicker() {
  const pick = document.getElementById("match-picker");
  pick.innerHTML = "";
  for (const m of state.summary.matches) {
    const opt = document.createElement("option");
    opt.value = m.stem;
    opt.textContent = m.stem;
    pick.appendChild(opt);
  }
}

// --- Fleet view ---

function renderFleet() {
  const matches = state.summary.matches;
  const analyzerOrder = state.summary.analyzer_order;

  // Headline cards
  const totalOf = (aid, key) =>
    matches.reduce((acc, m) => acc + (m.summaries[aid]?.[key] ?? 0), 0);
  const totalBalls = totalOf("bps", "balls_during_firing");
  const totalFiringS = totalOf("bps", "firing_seconds");
  const flipsFiring = totalOf("turret_flips", "flips_during_firing");
  const jamsTotal = totalOf("jams", "jam_events");
  const jamsSeconds = totalOf("jams", "jam_seconds_total");
  const unclogManual = totalOf("spindexer_events", "unclogging_events");
  const unclogAuto = totalOf("spindexer_events", "auto_unclogging_events");

  // Best slug across the fleet; median-of-medians for typical.
  let bestSlug = 0;
  const perMatchMedian = [];
  for (const m of matches) {
    const s = m.summaries.slugs ?? {};
    if ((s.best_slug_bps ?? 0) > bestSlug) bestSlug = s.best_slug_bps;
    if (s.median_slug_bps != null) perMatchMedian.push(s.median_slug_bps);
  }
  perMatchMedian.sort((a, b) => a - b);
  const medianSlug = perMatchMedian.length
    ? perMatchMedian[Math.floor(perMatchMedian.length / 2)]
    : 0;

  const firingBps = totalFiringS > 0 ? totalBalls / totalFiringS : 0;
  document.getElementById("stat-match-count").textContent = matches.length;
  document.getElementById("stat-avg-firing-bps").textContent = firingBps.toFixed(2);
  tintBps("stat-avg-firing-bps", firingBps);
  document.getElementById("stat-best-slug").textContent = bestSlug.toFixed(2);
  tintBps("stat-best-slug", bestSlug);
  document.getElementById("stat-median-slug").textContent = medianSlug.toFixed(2);
  tintBps("stat-median-slug", medianSlug);
  document.getElementById("stat-flips-firing").textContent = flipsFiring;
  document.getElementById("stat-jams").textContent = jamsTotal;
  document.getElementById("stat-jam-s").textContent = jamsSeconds.toFixed(1);
  document.getElementById("stat-unclogs").textContent = `${unclogManual} / ${unclogAuto}`;

  // Table columns: derive from union of summary keys across matches.
  const cols = deriveColumns(matches, analyzerOrder);
  const theadRow = document.querySelector("#fleet-table thead tr");
  const tbody = document.querySelector("#fleet-table tbody");
  theadRow.innerHTML = "";
  tbody.innerHTML = "";

  const addCell = (row, text, cls = "") => {
    const td = document.createElement(cls === "th" ? "th" : "td");
    td.className = cls === "th" ? "" : cls;
    td.textContent = text;
    row.appendChild(td);
  };

  addCell(theadRow, "Match", "th");
  addCell(theadRow, "Duration", "th");
  for (const [aid, key, label] of cols) addCell(theadRow, label, "th");

  for (const m of matches) {
    const tr = document.createElement("tr");
    tr.onclick = () => {
      document.getElementById("match-picker").value = m.stem;
      loadMatch(m.stem);
      showView("match");
    };
    addCell(tr, m.stem.replace(/^akit_26-04-17_/, ""));
    addCell(tr, fmtSeconds(m.duration_s), "num");
    for (const [aid, key, label, fmt] of cols) {
      const v = m.summaries[aid]?.[key];
      addCell(tr, v === undefined ? "—" : fmt(v), "num " + cellTint(aid, key, v));
    }
    tbody.appendChild(tr);
  }

  renderFleetCharts(matches);
}

function deriveColumns(matches, analyzerOrder) {
  const priority = {
    "bps.firing_bps": ["Firing BPS", (v) => v.toFixed(2)],
    "bps.target_gap_bps": ["Gap to 12", (v) => v.toFixed(2)],
    "bps.peak_2s_bps": ["Peak 2s BPS", (v) => v.toFixed(2)],
    "slugs.best_slug_bps": ["Best slug BPS", (v) => v.toFixed(2)],
    "slugs.median_slug_bps": ["Median slug BPS", (v) => v.toFixed(2)],
    "slugs.largest_slug_balls": ["Biggest slug", (v) => v],
    "slugs.slug_count": ["Slugs", (v) => v],
    "slugs.p10_gap_ms": ["P10 gap ms", (v) => v.toFixed(0)],
    "slugs.p50_gap_ms": ["P50 gap ms", (v) => v.toFixed(0)],
    "bps.balls_during_firing": ["Balls", (v) => v],
    "bps.firing_seconds": ["Firing s", (v) => v.toFixed(1)],
    "jams.jam_events": ["Jams", (v) => v],
    "jams.jam_seconds_total": ["Jam s", (v) => v.toFixed(1)],
    "turret_flips.flip_count": ["Flips", (v) => v],
    "turret_flips.flips_during_firing": ["Flips/fire", (v) => v],
    "turret_flips.flip_seconds_during_firing": ["Flip s/fire", (v) => v.toFixed(1)],
    "spindexer_events.suppressed_events": ["Hold-fire", (v) => v],
    "spindexer_events.unclogging_events": ["Manual unclog", (v) => v],
    "spindexer_events.auto_unclogging_events": ["Auto-unclog", (v) => v],
    "spindexer_events.jammed_events": ["Spindexer JAMMED", (v) => v],
    "motivator_zero_cause.total_drops": ["Mot→0", (v) => v],
    "motivator_zero_cause.cause_turret_flipping_or_stalled": ["\u00a0\u00a0↳turret flip/stall", (v) => v],
    "motivator_zero_cause.cause_unachievable": ["\u00a0\u00a0↳unachievable", (v) => v],
    "motivator_zero_cause.cause_zone_change": ["\u00a0\u00a0↳zone change", (v) => v],
    "motivator_zero_cause.cause_coord_left_firing": ["\u00a0\u00a0↳coord exited", (v) => v],
    "motivator_zero_cause.cause_unknown": ["\u00a0\u00a0↳unknown", (v) => v],
  };
  const cols = [];
  for (const [path, [label, fmt]] of Object.entries(priority)) {
    const [aid, key] = path.split(".");
    if (matches.some((m) => m.summaries[aid]?.[key] !== undefined)) {
      cols.push([aid, key, label, fmt]);
    }
  }
  // Include anything else we didn't anticipate (new analyzer → auto column).
  const seen = new Set(cols.map((c) => c[0] + "." + c[1]));
  for (const aid of analyzerOrder) {
    for (const m of matches) {
      for (const k of Object.keys(m.summaries[aid] ?? {})) {
        if (!seen.has(aid + "." + k)) {
          cols.push([aid, k, aid + "/" + k, (v) => (typeof v === "number" ? v.toFixed(2) : String(v))]);
          seen.add(aid + "." + k);
        }
      }
    }
  }
  return cols;
}

function cellTint(aid, key, v) {
  if (typeof v !== "number") return "";
  if (aid === "bps" && (key === "firing_bps" || key === "peak_2s_bps")) return bpsTintClass(v);
  if (aid === "bps" && key === "target_gap_bps") return v <= 2 ? "ok" : v >= 8 ? "bad" : "warn";
  if (aid === "slugs" && (key === "best_slug_bps" || key === "median_slug_bps")) return bpsTintClass(v);
  if (aid === "slugs" && key === "p10_gap_ms") return v <= 90 ? "ok" : v <= 150 ? "warn" : "bad";
  if (aid === "jams" && key === "jam_events") return v === 0 ? "" : v >= 20 ? "bad" : "warn";
  if (aid === "jams" && key === "jam_seconds_total") return v === 0 ? "" : v >= 20 ? "bad" : "warn";
  if (aid === "turret_flips" && key === "flips_during_firing") return v >= 10 ? "bad" : v >= 5 ? "warn" : "";
  if (aid === "spindexer_events" && (key === "unclogging_events" || key === "auto_unclogging_events"))
    return v >= 5 ? "bad" : v >= 1 ? "warn" : "";
  if (aid === "spindexer_events" && key === "jammed_events") return v > 0 ? "bad" : "";
  if (aid === "motivator_zero_cause" && key === "total_drops") return v >= 5 ? "warn" : "";
  return "";
}

function bpsTintClass(v) {
  if (v >= TARGET_BPS) return "ok";
  if (v >= TARGET_BPS * 0.5) return "warn";
  return "bad";
}

function tintBps(id, v) {
  const el = document.getElementById(id);
  if (!el) return;
  el.classList.remove("ok", "warn", "bad");
  el.classList.add(bpsTintClass(v));
}

function renderFleetCharts(matches) {
  const labels = matches.map((m) => m.stem.replace(/^akit_26-04-17_/, ""));

  destroyChart("chart-bps");
  state.charts["chart-bps"] = new Chart(document.getElementById("chart-bps"), {
    type: "bar",
    data: {
      labels,
      datasets: [
        {
          label: "Firing BPS (driver)",
          data: matches.map((m) => m.summaries.bps?.firing_bps ?? 0),
          backgroundColor: "#8b949e",
        },
        {
          label: "Best slug BPS (mechanism peak)",
          data: matches.map((m) => m.summaries.slugs?.best_slug_bps ?? 0),
          backgroundColor: "#58a6ff",
        },
        {
          label: "Target (12 BPS)",
          type: "line",
          data: matches.map(() => TARGET_BPS),
          borderColor: "#f85149",
          borderDash: [5, 4],
          pointRadius: 0,
          fill: false,
        },
      ],
    },
    options: chartOpts("BPS vs target"),
  });

  destroyChart("chart-flips");
  state.charts["chart-flips"] = new Chart(document.getElementById("chart-flips"), {
    type: "bar",
    data: {
      labels,
      datasets: [
        {
          label: "Flips during firing",
          data: matches.map((m) => m.summaries.turret_flips?.flips_during_firing ?? 0),
          backgroundColor: "#d29922",
        },
        {
          label: "Flips (all)",
          data: matches.map((m) => m.summaries.turret_flips?.flip_count ?? 0),
          backgroundColor: "#6e7681",
        },
      ],
    },
    options: chartOpts("Turret flips"),
  });

  // Spindexer non-feeding events by type — shows hold-fire, manual/auto unclog.
  const spinKinds = [
    ["suppressed_events", "Hold-fire (op.)", "#a371f7"],
    ["unclogging_events", "Manual unclog", "#d29922"],
    ["auto_unclogging_events", "Auto-unclog", "#db61a2"],
    ["jammed_events", "JAMMED", "#f85149"],
    ["reciprocating_events", "Reciprocating", "#6e7681"],
  ];
  destroyChart("chart-causes");
  state.charts["chart-causes"] = new Chart(document.getElementById("chart-causes"), {
    type: "bar",
    data: {
      labels,
      datasets: spinKinds.map(([key, label, color]) => ({
        label,
        data: matches.map((m) => m.summaries.spindexer_events?.[key] ?? 0),
        backgroundColor: color,
      })),
    },
    options: chartOpts("Spindexer non-feeding events during firing", { stacked: true }),
  });
}

function chartOpts(title, extra = {}) {
  return {
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      title: { display: true, text: title, color: "#c9d1d9" },
      legend: { labels: { color: "#c9d1d9", font: { size: 10 } } },
    },
    scales: {
      x: { stacked: !!extra.stacked, ticks: { color: "#8b949e", font: { size: 10 } } },
      y: { stacked: !!extra.stacked, ticks: { color: "#8b949e" }, grid: { color: "#30363d" } },
    },
  };
}

function destroyChart(id) {
  if (state.charts[id]) {
    state.charts[id].destroy();
    delete state.charts[id];
  }
}

// --- Match view ---

async function loadMatch(stem) {
  if (!stem) return;
  if (state.currentMatch?.stem === stem) return;
  let match = state.matchCache.get(stem);
  if (!match) {
    const res = await fetch(`data/match_${stem}.json`);
    match = await res.json();
    state.matchCache.set(stem, match);
  }
  state.currentMatch = match;
  renderMatch(match);
}

function renderMatch(match) {
  document.getElementById("match-title").textContent = match.file + "  ·  " + match.duration_s + "s";
  renderMatchCards(match);
  try {
    renderTimeline(match);
  } catch (err) {
    console.error("timeline failed:", err);
  }
  renderEventList();
  document.getElementById("inspector").textContent = "Click an event to see detail.";
}

function renderMatchCards(match) {
  const row = document.getElementById("match-summary-cards");
  const bps = match.analyzers.bps?.summary ?? {};
  const slugs = match.analyzers.slugs?.summary ?? {};
  const flips = match.analyzers.turret_flips?.summary ?? {};
  const jams = match.analyzers.jams?.summary ?? {};
  const spin = match.analyzers.spindexer_events?.summary ?? {};
  const drops = match.analyzers.motivator_zero_cause?.summary ?? {};
  row.innerHTML = "";

  const card = (title, value, sub = "", tint = "") => {
    const div = document.createElement("div");
    div.className = "card";
    div.innerHTML = `<h3>${title}</h3><div class="big ${tint}">${value}</div>${sub ? `<div class="sub">${sub}</div>` : ""}`;
    row.appendChild(div);
  };

  const firingBps = bps.firing_bps ?? 0;
  card("Firing BPS", firingBps.toFixed(2), `${bps.balls_during_firing ?? 0} balls / ${(bps.firing_seconds ?? 0).toFixed(1)}s  ·  target ${TARGET_BPS}`, bpsTintClass(firingBps));
  card("Best slug BPS", (slugs.best_slug_bps ?? 0).toFixed(2), `${slugs.largest_slug_balls ?? 0} ball peak slug`, bpsTintClass(slugs.best_slug_bps ?? 0));
  card("Median slug BPS", (slugs.median_slug_bps ?? 0).toFixed(2), `${slugs.multi_ball_slug_count ?? 0} multi-ball slugs`, bpsTintClass(slugs.median_slug_bps ?? 0));
  card("P10 gap ms", (slugs.p10_gap_ms ?? 0).toFixed(0), `${slugs.bps_at_p10_gap ? slugs.bps_at_p10_gap.toFixed(1) : "0"} BPS @ p10`);
  card("Jams", jams.jam_events ?? 0, `${(jams.jam_seconds_total ?? 0).toFixed(1)}s lost`, (jams.jam_events ?? 0) >= 20 ? "bad" : (jams.jam_events ?? 0) > 0 ? "warn" : "");
  card("Hold-fire / Unclog / Auto", `${spin.suppressed_events ?? 0} / ${spin.unclogging_events ?? 0} / ${spin.auto_unclogging_events ?? 0}`, `${(spin.total_interrupted_seconds ?? 0).toFixed(1)}s spindexer interrupted`);
  card("Flips during firing", `${flips.flips_during_firing ?? 0}/${flips.flip_count ?? 0}`, `${(flips.flip_seconds_during_firing ?? 0).toFixed(1)}s`);
  card("Motivator → 0 during firing", drops.total_drops ?? 0);
}

function renderTimeline(match) {
  const series = match.analyzers.match_summary?.series ?? {};
  const datasets = [];

  const line = (key, label, color, yAxisID = "rpm", pushToDatasets = datasets) => {
    const s = series[key];
    if (!s || !s.t_s.length) return;
    pushToDatasets.push({
      label,
      data: s.t_s.map((t, i) => ({ x: t, y: typeof s.v[i] === "number" ? s.v[i] : null })),
      borderColor: color,
      backgroundColor: color,
      borderWidth: 1.2,
      pointRadius: 0,
      tension: 0,
      spanGaps: true,
      yAxisID,
    });
  };

  line("launcher_actual_rpm", "Launcher RPM", "#58a6ff");
  line("launcher_target_rpm", "Launcher target", "#1f6feb");
  line("motivator_actual_rpm", "Motivator RPM", "#d29922");
  line("motivator_target_rpm", "Motivator target", "#bb8009");
  line("turret_angle_deg", "Turret angle", "#a371f7", "deg");
  line("turret_target_deg", "Turret target", "#6e40c9", "deg");

  // Event markers as scatter points.
  const evtPts = (aid, color, yAxisID = "events") => {
    const events = match.analyzers[aid]?.events ?? [];
    if (!events.length) return;
    datasets.push({
      label: aid,
      data: events.map((e) => ({ x: e.t_s ?? e.start_s ?? 0, y: 0 })),
      backgroundColor: color,
      borderColor: color,
      pointRadius: 4,
      pointStyle: "rectRot",
      showLine: false,
      yAxisID,
    });
  };
  // Ball impacts as small dots (from sampled series)
  const ballCount = series.ball_impact_count;
  if (ballCount && ballCount.t_s.length) {
    const pts = [];
    let prev = 0;
    for (let i = 0; i < ballCount.t_s.length; i++) {
      const c = ballCount.v[i] ?? 0;
      if (c > prev) pts.push({ x: ballCount.t_s[i], y: 0 });
      prev = c;
    }
    datasets.push({
      label: "Ball impacts",
      data: pts,
      backgroundColor: "#56d364",
      borderColor: "#56d364",
      pointRadius: 3,
      pointStyle: "circle",
      showLine: false,
      yAxisID: "events",
    });
  }
  evtPts("slugs", EVENT_COLORS.slugs);
  evtPts("jams", EVENT_COLORS.jams);
  evtPts("turret_flips", EVENT_COLORS.turret_flips);
  evtPts("spindexer_events", EVENT_COLORS.spindexer_events);
  evtPts("motivator_zero_cause", EVENT_COLORS.motivator_zero_cause);

  if (state.mainChart) state.mainChart.destroy();

  const canvas = document.getElementById("timeline-main");
  state.mainChart = new Chart(canvas, {
    type: "line",
    data: { datasets },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      interaction: { mode: "nearest", intersect: false },
      animation: false,
      parsing: false,
      plugins: {
        legend: { labels: { color: "#c9d1d9", font: { size: 10 } } },
        tooltip: { callbacks: { title: (c) => (c[0]?.parsed?.x ?? 0).toFixed(2) + "s" } },
      },
      scales: {
        x: {
          type: "linear",
          title: { display: true, text: "match time (s)", color: "#8b949e" },
          ticks: { color: "#8b949e" },
          grid: { color: "#30363d" },
        },
        rpm: {
          type: "linear",
          position: "left",
          title: { display: true, text: "RPM", color: "#8b949e" },
          ticks: { color: "#8b949e" },
          grid: { color: "#30363d" },
        },
        deg: {
          type: "linear",
          position: "right",
          title: { display: true, text: "deg", color: "#8b949e" },
          ticks: { color: "#8b949e" },
          grid: { drawOnChartArea: false },
        },
        events: {
          type: "linear",
          display: false,
          min: -1,
          max: 1,
        },
      },
    },
  });

  renderTimelineLegend();
}

function renderTimelineLegend() {
  const items = [
    { color: "#58a6ff", text: "Launcher RPM (actual/target)" },
    { color: "#d29922", text: "Motivator RPM (actual/target)" },
    { color: "#a371f7", text: "Turret angle/target" },
    { color: "#56d364", text: "● Ball impact" },
    { color: EVENT_COLORS.slugs, text: "◆ Slug" },
    { color: EVENT_COLORS.jams, text: "◆ Jam" },
    { color: EVENT_COLORS.turret_flips, text: "◆ Turret flip" },
    { color: EVENT_COLORS.spindexer_events, text: "◆ Spindexer event" },
    { color: EVENT_COLORS.motivator_zero_cause, text: "◆ Motivator → 0" },
  ];
  const el = document.getElementById("timeline-legend");
  el.innerHTML = "";
  for (const it of items) {
    const div = document.createElement("div");
    div.className = "legend-item";
    div.innerHTML = `<div class="legend-swatch" style="background:${it.color}"></div>${it.text}`;
    el.appendChild(div);
  }
}

function renderEventList() {
  const list = document.getElementById("event-list");
  if (!state.currentMatch) { list.innerHTML = ""; return; }
  const enabled = new Set(
    Array.from(document.querySelectorAll('.event-filter input:checked')).map((cb) => cb.value)
  );
  const rows = [];
  for (const aid of Object.keys(state.currentMatch.analyzers)) {
    if (!enabled.has(aid)) continue;
    const events = state.currentMatch.analyzers[aid].events ?? [];
    for (const e of events) {
      const t = e.t_s ?? e.start_s ?? 0;
      rows.push({ aid, t, e });
    }
  }
  rows.sort((a, b) => a.t - b.t);

  list.innerHTML = "";
  for (const r of rows) {
    const div = document.createElement("div");
    div.className = "event-row " + r.aid;
    const label = eventLabel(r.aid, r.e);
    div.innerHTML = `<span class="t">${r.t.toFixed(2)}s</span><span class="k">${r.aid}</span><span class="d">${label}</span><span class="tag">${eventTag(r.aid, r.e)}</span>`;
    div.onclick = () => { setInspector(r); seekTimeline(r.t); };
    list.appendChild(div);
  }
}

function eventLabel(aid, e) {
  if (aid === "slugs") return `${e.balls} balls in ${e.duration_s}s → ${e.bps} BPS  (min gap ${e.min_gap_ms}ms)`;
  if (aid === "jams") return `jam ${e.duration_s}s @ ${e.launcher_target_rpm} RPM (spindexer ${e.spindexer_state_at_start})`;
  if (aid === "turret_flips") return `${e.entry_angle}° → ${e.exit_angle}° (${e.duration_s}s)${e.during_firing ? ", during firing" : ""}`;
  if (aid === "spindexer_events") return `${e.state} for ${e.duration_s}s  (coord: ${e.coord_state_at_start})`;
  if (aid === "motivator_zero_cause") return `cause: ${e.cause}  coord: ${e.coord_before}→${e.coord_after}  ${e.transition_reason}`;
  if (aid === "firing_intervals") return `firing ${e.duration_s}s, ${e.balls_fired} balls [${e.phase}]`;
  if (aid === "bps") return `${e.balls_fired} balls in ${e.duration_s}s → ${e.bps} BPS, peak 2s=${e.peak_2s_bps}`;
  return JSON.stringify(e);
}

function eventTag(aid, e) {
  if (aid === "motivator_zero_cause") return e.cause;
  if (aid === "turret_flips") return e.during_firing ? "during firing" : "";
  if (aid === "spindexer_events") return e.state;
  if (aid === "slugs") return `${e.balls}b`;
  return "";
}

function setInspector(r) {
  const lines = [`event: ${r.aid}`, `time: ${r.t.toFixed(3)}s`, ""];
  for (const [k, v] of Object.entries(r.e)) {
    lines.push(`${k}:\t${JSON.stringify(v)}`);
  }
  document.getElementById("inspector").textContent = lines.join("\n");
}

function seekTimeline(t) {
  if (!state.mainChart) return;
  // Zoom the X scale to ±8s around the event.
  const xs = state.mainChart.options.scales.x;
  xs.min = Math.max(0, t - 8);
  xs.max = t + 8;
  state.mainChart.update();
}

// --- Utilities ---

function fmtSeconds(s) {
  if (s == null) return "—";
  const m = Math.floor(s / 60);
  const r = Math.floor(s % 60);
  return `${m}:${r.toString().padStart(2, "0")}`;
}

boot();
