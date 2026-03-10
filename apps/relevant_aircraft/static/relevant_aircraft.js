(function () {
  const sliderConfigs = [
    { inputId: "sepThreshold", valueId: "sepThresholdValue", unit: " NM" },
    { inputId: "speedDiff", valueId: "speedDiffValue", unit: " kt" },
    { inputId: "projectionTime", valueId: "projectionTimeValue", unit: " min" },
    { inputId: "gridDensity", valueId: "gridDensityValue", unit: " pts" },
    { inputId: "viewExtent", valueId: "viewExtentValue", unit: " NM" },
    { inputId: "aHeadingSlider", valueId: "aHeadingSliderValue", unit: " deg" },
    { inputId: "bHeadingSlider", valueId: "bHeadingSliderValue", unit: " deg" },
    { inputId: "aTargetHeadingSlider", valueId: "aTargetHeadingSliderValue", unit: " deg" },
    { inputId: "bTargetHeadingSlider", valueId: "bTargetHeadingSliderValue", unit: " deg" },
    { inputId: "aTurnRateSlider", valueId: "aTurnRateSliderValue", unit: " deg/s" },
    { inputId: "bTurnRateSlider", valueId: "bTurnRateSliderValue", unit: " deg/s" },
    { inputId: "turnSpeedUncertainty", valueId: "turnSpeedUncertaintyValue", unit: " kt" },
  ];

  const presetMapping = {
    model_mode: "modelMode",
    turn_preset_id: "turnPreset",
    separation_threshold_nm: "sepThreshold",
    speed_diff_kt: "speedDiff",
    projection_time_min: "projectionTime",
    grid_density: "gridDensity",
    view_half_extent_nm: "viewExtent",
    a_lat: "aLat",
    a_lon: "aLon",
    a_heading: "aHeadingSlider",
    a_target_heading_deg: "aTargetHeadingSlider",
    a_turn_rate_deg_sec: "aTurnRateSlider",
    a_speed_kt: "aSpeed",
    b_lat: "bLat",
    b_lon: "bLon",
    b_heading: "bHeadingSlider",
    b_target_heading_deg: "bTargetHeadingSlider",
    b_turn_rate_deg_sec: "bTurnRateSlider",
    b_speed_kt: "bSpeed",
    turn_speed_schedule_uncertainty_kt: "turnSpeedUncertainty",
  };

  const gridCanvas = document.getElementById("gridCanvas");
  const timeCanvas = document.getElementById("timeCanvas");
  const turnSpatialCanvas = document.getElementById("turnSpatialCanvas");
  const turnHullCanvas = document.getElementById("turnHullCanvas");
  const turnDistanceCanvas = document.getElementById("turnDistanceCanvas");
  const gridOverlay = document.getElementById("gridOverlay");
  const timeOverlay = document.getElementById("timeOverlay");
  const turnTimeOverlay = document.getElementById("turnTimeOverlay");
  const gridStatusEl = document.getElementById("gridStatus");
  const timeStatusEl = document.getElementById("timeStatus");
  const timeSlider = document.getElementById("timeSlider");
  const timeSliderValue = document.getElementById("timeSliderValue");
  const refreshGridBtn = document.getElementById("refreshGridBtn");
  const evaluateBtn = document.getElementById("evaluateBtn");
  const modelModeSelect = document.getElementById("modelMode");
  const turnPresetSelect = document.getElementById("turnPreset");
  const turnControlsSection = document.getElementById("turnControlsSection");
  const fixedTimeView = document.getElementById("fixedTimeView");
  const turnTimeView = document.getElementById("turnTimeView");
  const detailTitle = document.getElementById("detailTitle");
  const detailCaption = document.getElementById("detailCaption");
  const gridCaption = document.getElementById("gridCaption");
  const turnSpatialInfo = document.getElementById("turnSpatialInfo");

  const presetDataEl = document.getElementById("turnPresetData");
  const turnPresets = presetDataEl ? JSON.parse(presetDataEl.textContent || "[]") : [];
  const turnPresetMap = new Map(turnPresets.map((preset) => [preset.id, preset]));

  let gridDataset = null;
  let timeDataset = null;
  let currentFrameIndex = 0;
  let gridDirty = true;

  function currentMode() {
    return modelModeSelect ? modelModeSelect.value : "fixed_heading";
  }

  function isTurnMode() {
    return currentMode() === "turn_aware";
  }

  function updateSliderDisplays() {
    sliderConfigs.forEach(({ inputId, valueId, unit }) => {
      const input = document.getElementById(inputId);
      const display = document.getElementById(valueId);
      if (!input || !display) {
        return;
      }
      const update = () => {
        const step = Number(input.step || "1");
        const precision = step < 1 ? 1 : 0;
        display.textContent = `${parseFloat(input.value).toFixed(precision)}${unit}`;
      };
      update();
      if (input.dataset.sliderDisplayBound !== "1") {
        input.addEventListener("input", update);
        input.addEventListener("change", update);
        input.dataset.sliderDisplayBound = "1";
      }
    });
  }

  function readNumber(id) {
    const el = document.getElementById(id);
    return el ? parseFloat(el.value) : 0;
  }

  function readInteger(id) {
    const el = document.getElementById(id);
    return el ? parseInt(el.value, 10) : 0;
  }

  function setInputValue(id, value) {
    const el = document.getElementById(id);
    if (!el) {
      return;
    }
    if (el.type === "number") {
      el.value = Number(value).toFixed(3).replace(/\.?0+$/, "");
      return;
    }
    el.value = `${value}`;
  }

  function setStatus(target, message, isError = false) {
    const el = target === "grid" ? gridStatusEl : timeStatusEl;
    if (!el) {
      return;
    }
    el.textContent = message;
    el.classList.toggle("error", Boolean(isError));
  }

  function toggleOverlay(target, isVisible) {
    const overlay = target === "grid" ? gridOverlay : isTurnMode() ? turnTimeOverlay : timeOverlay;
    if (!overlay) {
      return;
    }
    overlay.classList.toggle("visible", Boolean(isVisible));
  }

  function canvasCSS(el) {
    const dpr = window.devicePixelRatio || 1;
    return { width: el.width / dpr, height: el.height / dpr };
  }

  function clearCanvas(el) {
    if (!el) {
      return;
    }
    const ctx = el.getContext("2d");
    if (!ctx) { return; }
    const dpr = window.devicePixelRatio || 1;
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, el.width, el.height);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.restore();
  }

  function updateTimeSliderState(frameCount) {
    if (!timeSlider) {
      return;
    }
    const disabled = frameCount <= 0;
    timeSlider.disabled = disabled;
    if (disabled) {
      timeSlider.value = "0";
      timeSlider.max = "1";
      if (timeSliderValue) {
        timeSliderValue.textContent = "0 s";
      }
    }
  }

  function clearTimeDataset(message) {
    timeDataset = null;
    currentFrameIndex = 0;
    updateTimeSliderState(0);
    clearCanvas(timeCanvas);
    clearCanvas(turnSpatialCanvas);
    clearCanvas(turnHullCanvas);
    clearCanvas(turnDistanceCanvas);
    if (turnSpatialInfo) {
      turnSpatialInfo.textContent = "";
    }
    setStatus("time", message);
  }

  function markGridDirty() {
    gridDirty = true;
    clearTimeDataset("Detail view is stale. Press Evaluate detail view.");
  }

  function updateBDisplays(lat, lon) {
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) {
      return;
    }
    setInputValue("bLat", lat);
    setInputValue("bLon", lon);
  }

  function collectCommonPayload() {
    return {
      model_mode: currentMode(),
      turn_preset_id: turnPresetSelect ? turnPresetSelect.value : "custom",
      separation_threshold_nm: readNumber("sepThreshold"),
      speed_diff_kt: readNumber("speedDiff"),
      projection_time_min: readNumber("projectionTime"),
      grid_density: readInteger("gridDensity"),
      view_half_extent_nm: readNumber("viewExtent"),
      a_lat: readNumber("aLat"),
      a_lon: readNumber("aLon"),
      a_heading: readNumber("aHeadingSlider"),
      a_target_heading_deg: readNumber("aTargetHeadingSlider"),
      a_turn_rate_deg_sec: readNumber("aTurnRateSlider"),
      a_speed_kt: readNumber("aSpeed"),
      b_lat: readNumber("bLat"),
      b_lon: readNumber("bLon"),
      b_heading: readNumber("bHeadingSlider"),
      b_target_heading_deg: readNumber("bTargetHeadingSlider"),
      b_turn_rate_deg_sec: readNumber("bTurnRateSlider"),
      b_speed_kt: readNumber("bSpeed"),
      turn_speed_schedule_uncertainty_kt: readNumber("turnSpeedUncertainty"),
    };
  }

  function updateModeUI() {
    const turnMode = isTurnMode();
    if (turnControlsSection) {
      turnControlsSection.classList.toggle("hidden", !turnMode);
      turnControlsSection.open = turnMode;
    }
    if (fixedTimeView) {
      fixedTimeView.classList.toggle("hidden", turnMode);
    }
    if (turnTimeView) {
      turnTimeView.classList.toggle("hidden", !turnMode);
    }
    if (detailTitle) {
      detailTitle.textContent = turnMode ? "Turn-aware detail" : "Fixed-heading detail";
    }
    if (detailCaption) {
      detailCaption.textContent = turnMode
        ? "Evaluate the selected pair to inspect sampled trajectories, relative hulls, and the distance curve from the turn-aware solver."
        : "Evaluate the selected pair to inspect straight-line reachable envelopes over the full projection horizon.";
    }
    if (gridCaption) {
      gridCaption.textContent = turnMode
        ? "Each grid cell runs the turn-aware certificate using the current initial headings, target headings, and turn rates."
        : "Each grid cell runs the fixed-heading relevant-aircraft test using the current A and B headings and speeds.";
    }
    resizeCanvases();
    if (gridDataset) {
      renderGrid();
    }
    if (timeDataset) {
      drawCurrentFrame();
    }
  }

  function resizeCanvasToParent(el, aspectRatio) {
    if (!el || !el.parentElement) {
      return;
    }
    const dpr = window.devicePixelRatio || 1;
    const cssWidth = Math.max(Math.min(el.parentElement.clientWidth || el.getBoundingClientRect().width || 260, 960), 260);
    const cssHeight = Math.round(cssWidth * aspectRatio);
    el.width = Math.round(cssWidth * dpr);
    el.height = Math.round(cssHeight * dpr);
    el.style.width = cssWidth + "px";
    el.style.height = cssHeight + "px";
    const ctx = el.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  function resizeCanvases() {
    resizeCanvasToParent(gridCanvas, 1);
    resizeCanvasToParent(timeCanvas, 1);
    resizeCanvasToParent(turnSpatialCanvas, 1);
    resizeCanvasToParent(turnHullCanvas, 1);
    resizeCanvasToParent(turnDistanceCanvas, 0.66);
  }

  function lonToX(bounds, lon, width) {
    const [lonMin, lonMax] = bounds;
    const range = lonMax - lonMin || 1e-6;
    return ((lon - lonMin) / range) * width;
  }

  function latToY(bounds, lat, height) {
    const [latMin, latMax] = bounds;
    const range = latMax - latMin || 1e-6;
    return height - ((lat - latMin) / range) * height;
  }

  function headingVector(deg) {
    const rad = (deg * Math.PI) / 180;
    return [Math.sin(rad), Math.cos(rad)];
  }

  function wrappedHeadingDeltaDeg(startDeg, targetDeg) {
    return ((targetDeg - startDeg + 540) % 360) - 180;
  }

  function subtract(a, b) {
    return [a[0] - b[0], a[1] - b[1]];
  }

  function dot(a, b) {
    return a[0] * b[0] + a[1] * b[1];
  }

  function norm(a) {
    return Math.sqrt(dot(a, a));
  }

  function addScaled(base, vec, scale) {
    return [base[0] + vec[0] * scale, base[1] + vec[1] * scale];
  }

  function clamp01(x) {
    return Math.min(Math.max(x, 0), 1);
  }

  function segmentsWithinDistance(a0, a1, b0, b1, threshold) {
    const u = subtract(a1, a0);
    const v = subtract(b1, b0);
    const w0 = subtract(a0, b0);
    const aa = dot(u, u);
    const bb = dot(u, v);
    const cc = dot(v, v);
    const dd = dot(u, w0);
    const ee = dot(v, w0);
    const denom = aa * cc - bb * bb;
    let sc;
    let tc;
    if (denom < 1e-12) {
      sc = 0;
      tc = cc > 1e-12 ? clamp01(ee / cc) : 0;
    } else {
      sc = clamp01((bb * ee - cc * dd) / denom);
      tc = clamp01((aa * ee - bb * dd) / denom);
    }
    const closestA = addScaled(a0, u, sc);
    const closestB = addScaled(b0, v, tc);
    return norm(subtract(closestA, closestB)) <= threshold + 1e-6;
  }

  function makeWorldToCanvas(bounds, canvas) {
    const { width, height } = canvasCSS(canvas);
    const xRange = Math.max(bounds.x_max - bounds.x_min, 1);
    const yRange = Math.max(bounds.y_max - bounds.y_min, 1);
    const scale = Math.min(width / xRange, height / yRange);
    const offsetX = (width - xRange * scale) / 2;
    const offsetY = (height - yRange * scale) / 2;
    return {
      scale,
      toCanvas(point) {
        return [
          offsetX + (point[0] - bounds.x_min) * scale,
          height - (offsetY + (point[1] - bounds.y_min) * scale),
        ];
      },
    };
  }

  function drawBackgroundGrid(ctx, canvas) {
    const { width, height } = canvasCSS(canvas);
    ctx.save();
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#e2e8f0";
    ctx.fillRect(0, 0, width, height);
    ctx.strokeStyle = "rgba(148, 163, 184, 0.25)";
    ctx.lineWidth = 1;
    const divisions = 6;
    for (let i = 1; i < divisions; i += 1) {
      const x = (width / divisions) * i;
      const y = (height / divisions) * i;
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, height);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(width, y);
      ctx.stroke();
    }
    ctx.restore();
  }

  function renderGrid() {
    if (!gridDataset || !gridCanvas) {
      return;
    }
    const ctx = gridCanvas.getContext("2d");
    const { width: gw, height: gh } = canvasCSS(gridCanvas);
    ctx.clearRect(0, 0, gw, gh);

    const rows = gridDataset.lats.length;
    const cols = gridDataset.lons.length;
    if (!rows || !cols) {
      return;
    }

    const cellWidth = gw / cols;
    const cellHeight = gh / rows;
    for (let i = 0; i < rows; i += 1) {
      for (let j = 0; j < cols; j += 1) {
        ctx.fillStyle = gridDataset.safe[i][j] ? "#15803d" : "#dc2626";
        const x = j * cellWidth;
        const y = gh - (i + 1) * cellHeight;
        ctx.fillRect(x, y, cellWidth + 1, cellHeight + 1);
      }
    }

    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(15, 23, 42, 0.24)";
    ctx.strokeRect(0, 0, gw, gh);

    const { a, b, lat_bounds: latBounds, lon_bounds: lonBounds } = gridDataset;
    const drawAircraft = (aircraft, color) => {
      const px = lonToX(lonBounds, aircraft.lon, gw);
      const py = latToY(latBounds, aircraft.lat, gh);
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(px, py, 6, 0, Math.PI * 2);
      ctx.fill();

      const [vx, vy] = headingVector(aircraft.heading);
      const arrowScale = Math.min(gw, gh) * 0.06;
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(px + vx * arrowScale, py - vy * arrowScale);
      ctx.lineWidth = 2.2;
      ctx.strokeStyle = color;
      ctx.setLineDash([]);
      ctx.stroke();

      if (gridDataset.model_mode === "turn_aware") {
        const [tx, ty] = headingVector(aircraft.target_heading_deg);
        ctx.beginPath();
        ctx.moveTo(px, py);
        ctx.lineTo(px + tx * arrowScale * 0.78, py - ty * arrowScale * 0.78);
        ctx.lineWidth = 1.6;
        ctx.strokeStyle = color;
        ctx.setLineDash([6, 5]);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    };

    drawAircraft(a, "#1d4ed8");
    drawAircraft(b, "#0f172a");
  }

  function strokeCapsule(ctx, start, end, width, color, dash = []) {
    ctx.save();
    ctx.lineCap = "round";
    ctx.lineWidth = Math.max(width, 1);
    ctx.strokeStyle = color;
    ctx.setLineDash(dash);
    ctx.beginPath();
    ctx.moveTo(start[0], start[1]);
    ctx.lineTo(end[0], end[1]);
    ctx.stroke();
    ctx.restore();
  }

  function drawFixedHeadingFrame(frameIndex) {
    if (!timeDataset || !timeCanvas || !timeDataset.frames || !timeDataset.frames.length) {
      clearCanvas(timeCanvas);
      setStatus("time", "Detail view not evaluated yet.");
      return;
    }
    const idx = Math.min(Math.max(frameIndex, 0), timeDataset.frames.length - 1);
    currentFrameIndex = idx;
    const frame = timeDataset.frames[idx];
    const ctx = timeCanvas.getContext("2d");
    drawBackgroundGrid(ctx, timeCanvas);

    const transform = makeWorldToCanvas(timeDataset.bounds_xy, timeCanvas);
    const toCanvas = transform.toCanvas;
    const sepRadius = (timeDataset.separation_threshold_m || 0) * transform.scale;

    const drawSegment = (origin, slow, fast, color, zoneColor) => {
      const originPx = toCanvas(origin);
      const slowPx = toCanvas(slow);
      const fastPx = toCanvas(fast);
      if (sepRadius > 0) {
        strokeCapsule(ctx, slowPx, fastPx, sepRadius * 2, zoneColor);
      }
      strokeCapsule(ctx, originPx, slowPx, 2, color, [8, 6]);
      strokeCapsule(ctx, slowPx, fastPx, 4, color);
      return { slowPx, fastPx };
    };

    const aSeg = drawSegment(timeDataset.origins.a, frame.a.slow, frame.a.fast, "#1d4ed8", "rgba(59, 130, 246, 0.2)");
    const bSeg = drawSegment(timeDataset.origins.b, frame.b.slow, frame.b.fast, "#0f172a", "rgba(15, 23, 42, 0.18)");

    const drawCenterCircle = (segmentKey, color) => {
      const seg = frame[segmentKey];
      const midWorld = [(seg.slow[0] + seg.fast[0]) / 2, (seg.slow[1] + seg.fast[1]) / 2];
      const midPx = toCanvas(midWorld);
      ctx.save();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = color;
      ctx.beginPath();
      ctx.arc(midPx[0], midPx[1], sepRadius, 0, Math.PI * 2);
      ctx.stroke();
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(midPx[0], midPx[1], 3, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
    };

    drawCenterCircle("a", "#1d4ed8");
    drawCenterCircle("b", "#0f172a");

    if (
      sepRadius > 0 &&
      segmentsWithinDistance(frame.a.slow, frame.a.fast, frame.b.slow, frame.b.fast, timeDataset.separation_threshold_m)
    ) {
      ctx.save();
      ctx.globalAlpha = 0.32;
      strokeCapsule(ctx, aSeg.slowPx, aSeg.fastPx, sepRadius * 2, "#f87171");
      strokeCapsule(ctx, bSeg.slowPx, bSeg.fastPx, sepRadius * 2, "#f87171");
      ctx.restore();
    }

    if (timeSliderValue) {
      timeSliderValue.textContent = `${frame.time_s.toFixed(0)} s (${frame.time_min.toFixed(2)} min)`;
    }
    const stats = timeDataset.stats;
    const overlapNow =
      timeDataset.separation_threshold_m > 0 &&
      segmentsWithinDistance(frame.a.slow, frame.a.fast, frame.b.slow, frame.b.fast, timeDataset.separation_threshold_m);
    setStatus(
      "time",
      `${stats.is_separated ? "Yes" : "No"} | Min distance: ${(stats.min_distance_nm ?? 0).toFixed(2)} NM | ` +
        `Closest approach: ${(stats.closest_time_min ?? 0).toFixed(2)} min | Current overlap: ${overlapNow ? "Yes" : "No"}`
    );
  }

  function drawPolyline(ctx, points, toCanvas, color, width, dash = [], closePath = false, fillStyle = null, alpha = 1) {
    if (!points || !points.length) {
      return;
    }
    ctx.save();
    ctx.lineWidth = width;
    ctx.strokeStyle = color;
    ctx.setLineDash(dash);
    ctx.globalAlpha = alpha;
    ctx.beginPath();
    const start = toCanvas(points[0]);
    ctx.moveTo(start[0], start[1]);
    for (let i = 1; i < points.length; i += 1) {
      const point = toCanvas(points[i]);
      ctx.lineTo(point[0], point[1]);
    }
    if (closePath) {
      ctx.closePath();
    }
    if (fillStyle) {
      ctx.fillStyle = fillStyle;
      ctx.fill();
    }
    ctx.stroke();
    ctx.restore();
  }

  function drawHeadingArrow(ctx, toCanvas, origin, headingDeg, lengthM, color, dash = []) {
    const [vx, vy] = headingVector(headingDeg);
    const end = [origin[0] + vx * lengthM, origin[1] + vy * lengthM];
    const startPx = toCanvas(origin);
    const endPx = toCanvas(end);
    ctx.save();
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 1.7;
    ctx.setLineDash(dash);
    ctx.beginPath();
    ctx.moveTo(startPx[0], startPx[1]);
    ctx.lineTo(endPx[0], endPx[1]);
    ctx.stroke();
    ctx.setLineDash([]);
    const angle = Math.atan2(endPx[1] - startPx[1], endPx[0] - startPx[0]);
    const headLen = 9;
    ctx.beginPath();
    ctx.moveTo(endPx[0], endPx[1]);
    ctx.lineTo(endPx[0] - headLen * Math.cos(angle - Math.PI / 6), endPx[1] - headLen * Math.sin(angle - Math.PI / 6));
    ctx.lineTo(endPx[0] - headLen * Math.cos(angle + Math.PI / 6), endPx[1] - headLen * Math.sin(angle + Math.PI / 6));
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  function drawTurnCueArc(ctx, toCanvas, origin, startHeadingDeg, targetHeadingDeg, radiusM, color, dash = []) {
    const deltaDeg = wrappedHeadingDeltaDeg(startHeadingDeg, targetHeadingDeg);
    if (Math.abs(deltaDeg) < 0.5 || radiusM <= 0) {
      return;
    }
    const stepCount = Math.max(8, Math.ceil(Math.abs(deltaDeg) / 10));
    const points = [];
    for (let idx = 0; idx <= stepCount; idx += 1) {
      const headingDeg = startHeadingDeg + (deltaDeg * idx) / stepCount;
      const [vx, vy] = headingVector(headingDeg);
      points.push([origin[0] + vx * radiusM, origin[1] + vy * radiusM]);
    }
    drawPolyline(ctx, points, toCanvas, color, 1.5, dash, false, null, 0.9);
  }

  function drawCapsuleMask(ctx, startPx, endPx, radiusPx, fillStyle) {
    if (!startPx || !endPx || radiusPx <= 0) {
      return;
    }
    ctx.save();
    ctx.fillStyle = fillStyle;
    ctx.strokeStyle = fillStyle;
    const dx = endPx[0] - startPx[0];
    const dy = endPx[1] - startPx[1];
    if (Math.hypot(dx, dy) <= 1e-6) {
      ctx.beginPath();
      ctx.arc(startPx[0], startPx[1], radiusPx, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
      return;
    }
    ctx.lineCap = "round";
    ctx.lineWidth = radiusPx * 2;
    ctx.beginPath();
    ctx.moveTo(startPx[0], startPx[1]);
    ctx.lineTo(endPx[0], endPx[1]);
    ctx.stroke();
    ctx.restore();
  }

  function drawCapsuleIntersection(ctx, canvas, startA, endA, startB, endB, radiusPx, alpha = 1) {
    if (!canvas || radiusPx <= 0) {
      return;
    }
    const dpr = window.devicePixelRatio || 1;
    const { width, height } = canvasCSS(canvas);
    const overlapCanvas = document.createElement("canvas");
    overlapCanvas.width = Math.round(width * dpr);
    overlapCanvas.height = Math.round(height * dpr);
    const overlapCtx = overlapCanvas.getContext("2d");
    overlapCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
    drawCapsuleMask(overlapCtx, startA, endA, radiusPx, "rgba(248, 113, 113, 1)");
    overlapCtx.globalCompositeOperation = "destination-in";
    drawCapsuleMask(overlapCtx, startB, endB, radiusPx, "rgba(0, 0, 0, 1)");
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.globalAlpha = alpha;
    ctx.drawImage(overlapCanvas, 0, 0);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.restore();
  }

  function updateTurnSpatialInfo(frame) {
    if (!turnSpatialInfo || !timeDataset) {
      return;
    }
    turnSpatialInfo.textContent =
      `A: ${timeDataset.info.a_heading_summary}\n` +
      `B: ${timeDataset.info.b_heading_summary}\n` +
      `Selected t: ${frame.time_s.toFixed(1)} s | Closest t*: ${timeDataset.summary.closest_time_s.toFixed(1)} s\n` +
      `Envelope d(t): ${frame.envelope_distance_nm.toFixed(2)} NM | Speed envelope: ±${timeDataset.info.speed_diff_kt.toFixed(0)} kt\n` +
      `Turn schedule uncertainty: ±${timeDataset.info.turn_speed_schedule_uncertainty_kt.toFixed(0)} kt`;
  }

  function drawTurnSpatialFrame(frameIndex) {
    if (!timeDataset || !turnSpatialCanvas) {
      clearCanvas(turnSpatialCanvas);
      return;
    }
    const frame = timeDataset.frames[frameIndex];
    const ctx = turnSpatialCanvas.getContext("2d");
    drawBackgroundGrid(ctx, turnSpatialCanvas);
    const transform = makeWorldToCanvas(timeDataset.bounds_xy, turnSpatialCanvas);
    const toCanvas = transform.toCanvas;
    const sepRadiusPx = (timeDataset.separation_threshold_m || 0) * transform.scale;

    const trajA = timeDataset.trajectories.a;
    const trajB = timeDataset.trajectories.b;
    drawPolyline(ctx, trajA.corridor, toCanvas, "rgba(59, 130, 246, 0.28)", 1.2, [], true, "rgba(59, 130, 246, 0.12)");
    drawPolyline(ctx, trajB.corridor, toCanvas, "rgba(15, 23, 42, 0.28)", 1.2, [], true, "rgba(15, 23, 42, 0.10)");
    drawPolyline(ctx, trajA.nominal, toCanvas, "#1d4ed8", 2.4);
    drawPolyline(ctx, trajB.nominal, toCanvas, "#0f172a", 2.4);
    drawPolyline(ctx, trajA.min, toCanvas, "#1d4ed8", 1.1, [5, 5], false, null, 0.88);
    drawPolyline(ctx, trajA.max, toCanvas, "#1d4ed8", 1.1, [5, 5], false, null, 0.88);
    drawPolyline(ctx, trajB.min, toCanvas, "#0f172a", 1.1, [5, 5], false, null, 0.88);
    drawPolyline(ctx, trajB.max, toCanvas, "#0f172a", 1.1, [5, 5], false, null, 0.88);

    const drawStart = (origin, label, color) => {
      const point = toCanvas(origin);
      ctx.save();
      ctx.fillStyle = color;
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(point[0], point[1], 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = color;
      ctx.font = "bold 13px Segoe UI";
      ctx.fillText(label, point[0] + 10, point[1] - 10);
      ctx.restore();
    };

    drawStart(timeDataset.origins.a, "A", "#1d4ed8");
    drawStart(timeDataset.origins.b, "B", "#0f172a");

    const arrowLengthM = Math.max(1800, 0.16 * (timeDataset.bounds_xy.half_span_m || 0));
    const instantaneousTurnA =
      !timeDataset.turns.a.end_position &&
      Math.abs(wrappedHeadingDeltaDeg(timeDataset.turns.a.start_heading_deg, timeDataset.turns.a.target_heading_deg)) > 0.5;
    const instantaneousTurnB =
      !timeDataset.turns.b.end_position &&
      Math.abs(wrappedHeadingDeltaDeg(timeDataset.turns.b.start_heading_deg, timeDataset.turns.b.target_heading_deg)) > 0.5;

    drawHeadingArrow(
      ctx,
      toCanvas,
      timeDataset.origins.a,
      timeDataset.turns.a.start_heading_deg,
      instantaneousTurnA ? arrowLengthM * 0.38 : arrowLengthM,
      "#1d4ed8"
    );
    drawHeadingArrow(
      ctx,
      toCanvas,
      timeDataset.origins.b,
      timeDataset.turns.b.start_heading_deg,
      instantaneousTurnB ? arrowLengthM * 0.38 : arrowLengthM,
      "#0f172a"
    );
    if (instantaneousTurnA) {
      drawTurnCueArc(
        ctx,
        toCanvas,
        timeDataset.origins.a,
        timeDataset.turns.a.start_heading_deg,
        timeDataset.turns.a.target_heading_deg,
        arrowLengthM * 0.3,
        "#1d4ed8",
        [4, 4]
      );
    }
    if (instantaneousTurnB) {
      drawTurnCueArc(
        ctx,
        toCanvas,
        timeDataset.origins.b,
        timeDataset.turns.b.start_heading_deg,
        timeDataset.turns.b.target_heading_deg,
        arrowLengthM * 0.3,
        "#0f172a",
        [4, 4]
      );
    }
    if (timeDataset.turns.a.start_heading_deg !== timeDataset.turns.a.target_heading_deg) {
      drawHeadingArrow(
        ctx,
        toCanvas,
        timeDataset.origins.a,
        timeDataset.turns.a.target_heading_deg,
        arrowLengthM * 0.76,
        "#1d4ed8",
        [6, 5]
      );
    }
    if (timeDataset.turns.b.start_heading_deg !== timeDataset.turns.b.target_heading_deg) {
      drawHeadingArrow(
        ctx,
        toCanvas,
        timeDataset.origins.b,
        timeDataset.turns.b.target_heading_deg,
        arrowLengthM * 0.76,
        "#0f172a",
        [6, 5]
      );
    }

    ["a", "b"].forEach((key) => {
      const turnInfo = timeDataset.turns[key];
      if (!turnInfo.end_position) {
        return;
      }
      const point = toCanvas(turnInfo.end_position);
      const color = key === "a" ? "#1d4ed8" : "#0f172a";
      ctx.save();
      ctx.fillStyle = color;
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(point[0], point[1], 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      ctx.restore();

      drawHeadingArrow(
        ctx,
        toCanvas,
        turnInfo.end_position,
        turnInfo.target_heading_deg,
        arrowLengthM * 0.58,
        color,
        [6, 5]
      );
    });

    const drawReachableSegment = (segment, color, shade) => {
      const minPx = toCanvas(segment.min);
      const maxPx = toCanvas(segment.max);
      if (sepRadiusPx > 0) {
        strokeCapsule(ctx, minPx, maxPx, sepRadiusPx * 2, shade);
      }
      strokeCapsule(ctx, minPx, maxPx, 6, shade);
      strokeCapsule(ctx, minPx, maxPx, 2.6, color);
      const nomPx = toCanvas(segment.nominal);
      ctx.save();
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(nomPx[0], nomPx[1], 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
      return { minPx, maxPx, nomPx };
    };

    const aReachable = drawReachableSegment(frame.a, "#1d4ed8", "rgba(59, 130, 246, 0.24)");
    const bReachable = drawReachableSegment(frame.b, "#0f172a", "rgba(15, 23, 42, 0.20)");
    const overlapNow =
      timeDataset.distance_curve &&
      frame.envelope_distance_nm < timeDataset.distance_curve.threshold_nm;
    if (overlapNow) {
      drawCapsuleIntersection(
        ctx,
        turnSpatialCanvas,
        aReachable.minPx,
        aReachable.maxPx,
        bReachable.minPx,
        bReachable.maxPx,
        sepRadiusPx,
        0.22
      );
    }
    drawPolyline(ctx, [frame.a.nominal, frame.b.nominal], toCanvas, "rgba(71, 85, 105, 0.82)", 1.2, [6, 5]);
    updateTurnSpatialInfo(frame);
  }

  function drawTurnHullFrame(frameIndex) {
    if (!timeDataset || !turnHullCanvas) {
      clearCanvas(turnHullCanvas);
      return;
    }
    const frame = timeDataset.frames[frameIndex];
    const ctx = turnHullCanvas.getContext("2d");
    drawBackgroundGrid(ctx, turnHullCanvas);
    const transform = makeWorldToCanvas(timeDataset.relative_bounds_xy, turnHullCanvas);
    const toCanvas = transform.toCanvas;
    const sepRadiusPx = (timeDataset.separation_threshold_m || 0) * transform.scale;
    const originPx = toCanvas([0, 0]);

    ctx.save();
    ctx.strokeStyle = "rgba(239, 68, 68, 0.9)";
    ctx.lineWidth = 1.6;
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    ctx.arc(originPx[0], originPx[1], sepRadiusPx, 0, Math.PI * 2);
    ctx.stroke();
    ctx.restore();

    drawPolyline(
      ctx,
      frame.relative_hull,
      toCanvas,
      frame.envelope_distance_nm >= timeDataset.distance_curve.threshold_nm ? "#15803d" : "#dc2626",
      2,
      [],
      frame.relative_hull.length >= 3,
      frame.envelope_distance_nm >= timeDataset.distance_curve.threshold_nm ? "rgba(22, 163, 74, 0.14)" : "rgba(220, 38, 38, 0.14)"
    );

    const relativeNominalPx = toCanvas(frame.relative_nominal);
    const closestPx = toCanvas(frame.relative_closest_point);
    ctx.save();
    ctx.fillStyle = "#0f172a";
    ctx.beginPath();
    ctx.arc(originPx[0], originPx[1], 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.beginPath();
    ctx.arc(relativeNominalPx[0], relativeNominalPx[1], 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#475569";
    ctx.lineWidth = 1.2;
    ctx.setLineDash([6, 5]);
    ctx.beginPath();
    ctx.moveTo(originPx[0], originPx[1]);
    ctx.lineTo(closestPx[0], closestPx[1]);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = frame.envelope_distance_nm >= timeDataset.distance_curve.threshold_nm ? "#15803d" : "#dc2626";
    ctx.beginPath();
    ctx.arc(closestPx[0], closestPx[1], 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }

  function drawAxes(ctx, width, height, padding, yLabel, xLabel) {
    ctx.save();
    ctx.strokeStyle = "#64748b";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(padding.left, padding.top);
    ctx.lineTo(padding.left, height - padding.bottom);
    ctx.lineTo(width - padding.right, height - padding.bottom);
    ctx.stroke();
    ctx.fillStyle = "#475569";
    ctx.font = "12px Segoe UI";
    ctx.textAlign = "center";
    ctx.fillText(xLabel, padding.left + (width - padding.left - padding.right) / 2, height - 6);
    ctx.save();
    ctx.translate(14, padding.top + (height - padding.top - padding.bottom) / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.textAlign = "center";
    ctx.fillText(yLabel, 0, 0);
    ctx.restore();
    ctx.restore();
  }

  function drawAxisTicks(ctx, padding, plotWidth, plotHeight, xValues, yValues, toCanvas) {
    ctx.save();
    ctx.strokeStyle = "rgba(100, 116, 139, 0.55)";
    ctx.fillStyle = "#475569";
    ctx.font = "11px Segoe UI";

    xValues.forEach((xValue) => {
      const [px, py] = toCanvas(xValue, yValues[0]);
      ctx.beginPath();
      ctx.moveTo(px, py);
      ctx.lineTo(px, py + 5);
      ctx.stroke();
      ctx.textAlign = "center";
      ctx.fillText(`${xValue.toFixed(0)}`, px, py + 17);
    });

    yValues.forEach((yValue) => {
      const [px, py] = toCanvas(xValues[0], yValue);
      ctx.beginPath();
      ctx.moveTo(px - 5, py);
      ctx.lineTo(px, py);
      ctx.stroke();
      ctx.textAlign = "right";
      ctx.fillText(`${yValue.toFixed(1)}`, px - 8, py + 4);
    });
    ctx.restore();
  }

  function drawTurnDistanceFrame(frameIndex) {
    if (!timeDataset || !turnDistanceCanvas) {
      clearCanvas(turnDistanceCanvas);
      return;
    }
    const frame = timeDataset.frames[frameIndex];
    const ctx = turnDistanceCanvas.getContext("2d");
    const canvas = turnDistanceCanvas;
    drawBackgroundGrid(ctx, canvas);

    const curve = timeDataset.distance_curve;
    const padding = { left: 58, right: 20, top: 24, bottom: 36 };
    const xStep = 60;
    const xMin = 0;
    const rawXMax = Math.max(...curve.times_s, 1);
    const xMax = Math.max(Math.ceil(rawXMax / xStep) * xStep, xStep);
    const rawYMax = Math.max(...curve.envelope_nm, ...curve.nominal_nm, curve.threshold_nm, 1);
    const rawYMin = Math.max(0, Math.min(...curve.envelope_nm, ...curve.nominal_nm, curve.threshold_nm) - 0.3);
    const yStep = 5;
    const yMin = Math.floor(rawYMin / yStep) * yStep;
    const yMax = Math.max(Math.ceil(rawYMax / yStep) * yStep, yMin + yStep);
    const { width: canvasW, height: canvasH } = canvasCSS(canvas);
    const plotWidth = canvasW - padding.left - padding.right;
    const plotHeight = canvasH - padding.top - padding.bottom;

    const toCanvas = (x, y) => {
      const px = padding.left + ((x - xMin) / (xMax - xMin || 1)) * plotWidth;
      const py = padding.top + (1 - (y - yMin) / (yMax - yMin || 1)) * plotHeight;
      return [px, py];
    };

    drawAxes(ctx, canvasW, canvasH, padding, "Distance (NM)", "Time (s)");
    const xTickCount = Math.round((xMax - xMin) / xStep) + 1;
    const xTickValues = Array.from({ length: xTickCount }, (_, idx) => xMin + idx * xStep);
    const yTickCount = Math.round((yMax - yMin) / yStep) + 1;
    const yTickValues = Array.from({ length: yTickCount }, (_, idx) => yMin + idx * yStep);
    drawAxisTicks(ctx, padding, plotWidth, plotHeight, xTickValues, yTickValues, toCanvas);

    const drawCurve = (times, values, color, width, dash = []) => {
      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.setLineDash(dash);
      ctx.beginPath();
      times.forEach((timeValue, idx) => {
        const point = toCanvas(timeValue, values[idx]);
        if (idx === 0) {
          ctx.moveTo(point[0], point[1]);
        } else {
          ctx.lineTo(point[0], point[1]);
        }
      });
      ctx.stroke();
      ctx.restore();
    };

    const envelopeColor = timeDataset.summary.is_separated ? "#15803d" : "#dc2626";
    drawCurve(curve.times_s, curve.envelope_nm, envelopeColor, 2.6);
    drawCurve(curve.times_s, curve.nominal_nm, "#334155", 1.6, [6, 5]);

    const thresholdStart = toCanvas(xMin, curve.threshold_nm);
    const thresholdEnd = toCanvas(xMax, curve.threshold_nm);
    ctx.save();
    ctx.strokeStyle = "#dc2626";
    ctx.lineWidth = 1.4;
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    ctx.moveTo(thresholdStart[0], thresholdStart[1]);
    ctx.lineTo(thresholdEnd[0], thresholdEnd[1]);
    ctx.stroke();
    ctx.fillStyle = "#dc2626";
    ctx.font = "11px Segoe UI";
    ctx.textAlign = "right";
    ctx.fillText("Threshold", thresholdEnd[0] - 6, thresholdEnd[1] - 6);
    ctx.restore();

    ["a", "b"].forEach((key) => {
      const turnEnd = timeDataset.turns[key].end_time_s;
      if (turnEnd <= 0 || turnEnd > xMax) {
        return;
      }
      const start = toCanvas(turnEnd, yMin);
      const end = toCanvas(turnEnd, yMax);
      ctx.save();
      ctx.strokeStyle = key === "a" ? "rgba(29, 78, 216, 0.55)" : "rgba(15, 23, 42, 0.55)";
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(start[0], start[1]);
      ctx.lineTo(end[0], end[1]);
      ctx.stroke();
      ctx.restore();
    });

    const selectedBottom = toCanvas(frame.time_s, yMin);
    const selectedTop = toCanvas(frame.time_s, yMax);
    const selectedPoint = toCanvas(frame.time_s, frame.envelope_distance_nm);
    ctx.save();
    ctx.strokeStyle = "#0f172a";
    ctx.lineWidth = 1.1;
    ctx.setLineDash([6, 5]);
    ctx.beginPath();
    ctx.moveTo(selectedBottom[0], selectedBottom[1]);
    ctx.lineTo(selectedTop[0], selectedTop[1]);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = frame.envelope_distance_nm >= curve.threshold_nm ? "#15803d" : "#dc2626";
    ctx.beginPath();
    ctx.arc(selectedPoint[0], selectedPoint[1], 4.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }

  function drawTurnAwareFrame(frameIndex) {
    if (!timeDataset || !timeDataset.frames || !timeDataset.frames.length) {
      clearCanvas(turnSpatialCanvas);
      clearCanvas(turnHullCanvas);
      clearCanvas(turnDistanceCanvas);
      setStatus("time", "Detail view not evaluated yet.");
      return;
    }
    const idx = Math.min(Math.max(frameIndex, 0), timeDataset.frames.length - 1);
    currentFrameIndex = idx;
    const frame = timeDataset.frames[idx];
    drawTurnSpatialFrame(idx);
    drawTurnHullFrame(idx);
    drawTurnDistanceFrame(idx);
    if (timeSliderValue) {
      timeSliderValue.textContent = `${frame.time_s.toFixed(1)} s (${frame.time_min.toFixed(2)} min)`;
    }
    setStatus(
      "time",
      `${timeDataset.summary.is_separated ? "Separated" : "Conflict / uncertified"} | ` +
        `Min distance: ${timeDataset.summary.min_distance_nm.toFixed(2)} NM | ` +
        `Closest time: ${timeDataset.summary.closest_time_min.toFixed(2)} min | ` +
        `Selected envelope d(t): ${frame.envelope_distance_nm.toFixed(2)} NM`
    );
  }

  function drawCurrentFrame() {
    if (!timeDataset) {
      return;
    }
    if (timeDataset.model_mode === "turn_aware") {
      drawTurnAwareFrame(currentFrameIndex);
      return;
    }
    drawFixedHeadingFrame(currentFrameIndex);
  }

  async function renderGridData(forceFetch = false, autoEvaluate = false) {
    const needsFetch = forceFetch || !gridDataset || gridDirty;
    if (!needsFetch) {
      resizeCanvases();
      renderGrid();
      return;
    }

    try {
      setStatus("grid", "Rendering grid...");
      toggleOverlay("grid", true);
      const data = await postJSON("/api/grid", collectCommonPayload());
      gridDataset = data;
      updateBDisplays(data.b.lat, data.b.lon);
      resizeCanvases();
      renderGrid();
      const stats = data.stats || {};
      setStatus(
        "grid",
        `Safe cells: ${(stats.safe_percentage ?? 0).toFixed(1)}% | Mean closest time: ${(stats.mean_closest_time_min ?? 0).toFixed(1)} min`
      );
      gridDirty = false;
    } catch (err) {
      console.error(err);
      setStatus("grid", `Unable to render grid: ${err instanceof Error ? err.message : String(err)}`, true);
    } finally {
      toggleOverlay("grid", false);
    }
    if (autoEvaluate) {
      await renderTimeSweepData();
    }
  }

  async function renderTimeSweepData() {
    try {
      setStatus("time", "Rendering detail view...");
      toggleOverlay("time", true);
      const data = await postJSON("/api/time_sweep", collectCommonPayload());
      timeDataset = data;
      resizeCanvases();
      if (gridDataset) {
        renderGrid();
      }
      const frameCount = data.frames.length;
      if (timeSlider) {
        timeSlider.max = `${Math.max(frameCount - 1, 0)}`;
        currentFrameIndex = 0;
        timeSlider.value = `${currentFrameIndex}`;
      }
      updateTimeSliderState(frameCount);
      drawCurrentFrame();
    } catch (err) {
      console.error(err);
      setStatus("time", `Unable to render detail view: ${err instanceof Error ? err.message : String(err)}`, true);
    } finally {
      toggleOverlay("time", false);
    }
  }

  function postJSON(url, payload) {
    return fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    }).then((response) => {
      if (!response.ok) {
        return response.text().then((text) => {
          throw new Error(text || `Request failed with status ${response.status}`);
        });
      }
      return response.json();
    });
  }

  function handleGridCanvasClick(event) {
    if (!gridDataset || !gridCanvas) {
      return;
    }
    const rect = gridCanvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    const lon = gridDataset.lon_bounds[0] + (x / rect.width) * (gridDataset.lon_bounds[1] - gridDataset.lon_bounds[0]);
    const lat =
      gridDataset.lat_bounds[1] - (y / rect.height) * (gridDataset.lat_bounds[1] - gridDataset.lat_bounds[0]);
    updateBDisplays(lat, lon);
    markGridDirty();
    renderGridData(true, true);
  }

  function applyPreset(presetId) {
    const preset = turnPresetMap.get(presetId);
    if (!preset || !preset.params) {
      return;
    }
    Object.entries(preset.params).forEach(([key, value]) => {
      const inputId = presetMapping[key];
      if (inputId) {
        setInputValue(inputId, value);
      }
    });
    updateSliderDisplays();
    updateModeUI();
    markGridDirty();
    renderGridData(true, true);
  }

  function snapHeading(id) {
    const el = document.getElementById(id);
    if (!el) {
      return;
    }
    const rounded = Math.round(readNumber(id) / 5) * 5;
    el.value = `${rounded}`;
  }

  function setPresetToCustom() {
    if (turnPresetSelect && turnPresetSelect.value !== "custom") {
      turnPresetSelect.value = "custom";
    }
  }

  function attachAutoRefresh(id, options = {}) {
    const el = document.getElementById(id);
    if (!el) {
      return;
    }
    const { onInput = false, onChange = true, customBeforeRefresh = false } = options;
    const refresh = () => {
      if (customBeforeRefresh && isTurnMode()) {
        setPresetToCustom();
      }
      markGridDirty();
      renderGridData(true);
    };
    if (onInput) {
      el.addEventListener("input", refresh);
    }
    if (onChange) {
      el.addEventListener("change", refresh);
    }
  }

  function attachEventHandlers() {
    if (gridCanvas) {
      gridCanvas.addEventListener("click", handleGridCanvasClick);
    }

    if (refreshGridBtn) {
      refreshGridBtn.addEventListener("click", () => {
        markGridDirty();
        renderGridData(true);
      });
    }

    if (evaluateBtn) {
      evaluateBtn.addEventListener("click", () => {
        renderTimeSweepData();
      });
    }

    if (timeSlider) {
      timeSlider.addEventListener("input", () => {
        currentFrameIndex = Number(timeSlider.value);
        drawCurrentFrame();
      });
    }

    if (modelModeSelect) {
      modelModeSelect.addEventListener("change", () => {
        if (!isTurnMode()) {
          setPresetToCustom();
          setInputValue("aTargetHeadingSlider", readNumber("aHeadingSlider"));
          setInputValue("bTargetHeadingSlider", readNumber("bHeadingSlider"));
          setInputValue("aTurnRateSlider", 0);
          setInputValue("bTurnRateSlider", 0);
        }
        updateSliderDisplays();
        updateModeUI();
        markGridDirty();
        renderGridData(true);
      });
    }

    if (turnPresetSelect) {
      turnPresetSelect.addEventListener("change", () => {
        if (turnPresetSelect.value === "custom") {
          markGridDirty();
          renderGridData(true);
          return;
        }
        applyPreset(turnPresetSelect.value);
      });
    }

    ["aHeadingSlider", "bHeadingSlider", "aTargetHeadingSlider", "bTargetHeadingSlider"].forEach((id) => {
      const el = document.getElementById(id);
      if (!el) {
        return;
      }
      el.addEventListener("input", () => {
        snapHeading(id);
        if (id === "aHeadingSlider" && !isTurnMode()) {
          setInputValue("aTargetHeadingSlider", readNumber("aHeadingSlider"));
        }
        if (id === "bHeadingSlider" && !isTurnMode()) {
          setInputValue("bTargetHeadingSlider", readNumber("bHeadingSlider"));
        }
      });
      el.addEventListener("change", () => {
        snapHeading(id);
        if (isTurnMode()) {
          setPresetToCustom();
        } else if (id === "aHeadingSlider" || id === "bHeadingSlider") {
          const targetId = id === "aHeadingSlider" ? "aTargetHeadingSlider" : "bTargetHeadingSlider";
          setInputValue(targetId, readNumber(id));
        }
        markGridDirty();
        renderGridData(true);
      });
    });

    ["viewExtent", "sepThreshold", "speedDiff", "projectionTime", "gridDensity", "aSpeed", "bSpeed"].forEach((id) => {
      attachAutoRefresh(id, { customBeforeRefresh: true });
    });

    ["aLat", "aLon", "bLat", "bLon", "aTurnRateSlider", "bTurnRateSlider", "turnSpeedUncertainty"].forEach((id) => {
      attachAutoRefresh(id, { customBeforeRefresh: true });
    });

    window.addEventListener("resize", () => {
      resizeCanvases();
      if (gridDataset) {
        renderGrid();
      }
      if (timeDataset) {
        drawCurrentFrame();
      }
    });
  }

  window.addEventListener("DOMContentLoaded", () => {
    updateSliderDisplays();
    updateModeUI();
    updateBDisplays(readNumber("bLat"), readNumber("bLon"));
    resizeCanvases();
    attachEventHandlers();
    updateTimeSliderState(0);
    clearTimeDataset("Detail view not evaluated yet.");
    renderGridData(true, true);
  });
})();
