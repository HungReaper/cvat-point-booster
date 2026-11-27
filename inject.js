(() => {
  if (window.__pbInstalled) return;
  window.__pbInstalled = true;

  // ---- Cấu hình (lưu trong localStorage) ----
  const KEY = 'cvatPointBooster';
  // smooth = 0: mỗi vùng giữa 2 vòng một màu; 1: chuyển màu mượt giữa các vòng
  // world: cỡ điểm tối thiểu (cm) ở view 2D Top/Side/Front, to dần khi zoom; 0 = tắt
  // ground: độ cao mặt đất z (m, hệ lidar) của đường kẻ ở view Side/Front; null = tự lấy từ calib
  // showGround: hiện đường mặt đất (nút "Đặt lại mặc định" không đổi mục này)
  // refTable: hiện bảng kích thước tham chiếu (L, W, H) khi chọn class / chọn cuboid
  const DEFAULTS = Object.freeze({ size: 2, bright: 1.5, mode: 1, smooth: 0, world: 4, ground: null, showGround: 0, refTable: 1 });
  // Giới hạn của các ô số / thanh trượt: [min, max, số chữ số sau dấu "."]
  const RANGES = Object.freeze({ size: [0.5, 8, 1], world: [0, 20, 1], bright: [0.5, 4, 1], ground: [-10, 5, 2] });
  const roundTo = (v, dec) => Math.round(v * 10 ** dec) / 10 ** dec;
  const fitRange = (k, v) => roundTo(Math.min(RANGES[k][1], Math.max(RANGES[k][0], v)), RANGES[k][2]);
  const cfg = { ...DEFAULTS };
  try {
    const saved = JSON.parse(localStorage.getItem(KEY) || '{}');
    Object.keys(DEFAULTS).forEach((k) => {
      if (typeof saved[k] !== 'number' || !Number.isFinite(saved[k])) return;
      cfg[k] = RANGES[k] ? fitRange(k, saved[k]) : saved[k];
    });
  } catch (e) {}
  const save = () => { try { localStorage.setItem(KEY, JSON.stringify(cfg)); } catch (e) {} };

  // ---- Đồng bộ với plugin cuboid-projection của host ----
  // Vòng khoảng cách: cùng key localStorage và cùng mặc định với host (Show range rings)
  const RING_KEY = 'cuboidProjection.rings';
  const RING_DEFAULTS = [
    { radius: 10, color: '#ff4d4f' },
    { radius: 20, color: '#faad14' },
    { radius: 30, color: '#fadb14' },
    { radius: 50, color: '#00e5ff' },
  ];
  const MAX_RINGS = 8;
  const FAR_COLOR = [0.5, 0.5, 0.5]; // điểm nằm ngoài vòng xa nhất
  const hexToRgb = (h) => [1, 3, 5].map((i) => parseInt(h.slice(i, i + 2), 16) / 255);

  let ringsRaw = null;
  let rings = [];
  const ringR = new Float32Array(MAX_RINGS);
  const ringC = new Float32Array(MAX_RINGS * 3);

  function loadRings() {
    let raw = null;
    try { raw = localStorage.getItem(RING_KEY); } catch (e) {}
    if (raw === ringsRaw && rings.length) return false;
    ringsRaw = raw;
    let list = null;
    try { list = JSON.parse(raw || 'null'); } catch (e) {}
    list = Array.isArray(list)
      ? list
        .map((r) => ({ radius: Number(r && r.radius), color: String((r && r.color) || '') }))
        .filter((r) => Number.isFinite(r.radius) && r.radius > 0)
        .map((r, i) => ({
          radius: Math.round(10 * Math.min(300, Math.max(1, r.radius))) / 10,
          color: /^#[0-9a-f]{6}$/i.test(r.color) ? r.color : RING_DEFAULTS[i % RING_DEFAULTS.length].color,
        }))
        .slice(0, MAX_RINGS)
      : [];
    if (!list.length) list = RING_DEFAULTS.map((r) => ({ ...r }));
    rings = list.sort((a, b) => a.radius - b.radius);
    rings.forEach((r, i) => { ringR[i] = r.radius; ringC.set(hexToRgb(r.color), i * 3); });
    return true;
  }
  loadRings();

  // Ma trận lidar -> ego (gốc = trục sau xe). Không có calib thì đo từ gốc lidar, giống host.
  const IDENTITY = new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
  let egoMat = IDENTITY;
  let hasEgo = false;
  let onCalibChange = () => {};
  let onGroundChange = () => {};

  // Độ cao mặt đất mặc định: mặt phẳng z_ego = 0 (gốc ego nằm trên mặt đất) đổi sang hệ lidar,
  // với calib hiện tại là khoảng -1.84 m. Không có calib thì đoán -1.8 m.
  const autoGround = () =>
    hasEgo && Math.abs(egoMat[10]) > 1e-6 ? roundTo(-egoMat[14] / egoMat[10], 2) : -1.8;
  const groundZ = () => (typeof cfg.ground === 'number' ? cfg.ground : autoGround());

  function setCalib(json) {
    const M = json && json.lidar_to_ego;
    const ok = Array.isArray(M) && M.length >= 3 &&
      [0, 1, 2].every((r) => Array.isArray(M[r]) && M[r].length >= 4 && M[r].slice(0, 4).every(Number.isFinite));
    if (ok) {
      const a = new Float32Array(16); // GLSL mat4 theo cột
      for (let c = 0; c < 4; c++) {
        for (let r = 0; r < 3; r++) a[c * 4 + r] = M[r][c];
        a[c * 4 + 3] = c === 3 ? 1 : 0;
      }
      egoMat = a;
    } else {
      egoMat = IDENTITY;
    }
    hasEgo = ok;
    onCalibChange();
  }

  // Bắt file calib mà host tải cho frame hiện tại (fetch `${calibBase}/<frame>.json`)
  const oFetch = window.fetch;
  if (oFetch) {
    window.fetch = function (input) {
      const p = oFetch.apply(this, arguments);
      try {
        const url = typeof input === 'string' ? input : (input && input.url) || String(input);
        let base = '/cuboid-calib';
        try { base = localStorage.getItem('cuboidProjection.calibBase') || base; } catch (e) {}
        if (url.includes(base + '/') && /\.json(\?|$)/.test(url)) {
          p.then((r) => (r.ok ? r.clone().json() : null))
            .then(setCalib, () => setCalib(null))
            .catch(() => {});
        }
      } catch (e) {}
      return p;
    };
  }

  // ---- Đoạn shader chèn thêm ----
  const VS_DECL =
    'uniform float uPB_size;\nuniform float uPB_world;\nuniform float uPB_vw;\nuniform mat4 uPB_l2e;\nvarying float vPB_dist;\n';
  // Khoảng cách mặt đất (XY) từ gốc xe ego tới điểm, cùng công thức với "Show distance to ego" của host
  const VS_CODE =
    '\n  gl_PointSize *= uPB_size;\n' +
    '  vec4 pbEgo = uPB_l2e * vec4(position.xyz, 1.0);\n' +
    '  vPB_dist = length(pbEgo.xy);\n';
  // View 2D (camera trực giao) của three.js giữ gl_PointSize cố định theo pixel nên phóng to thì điểm bé đi.
  // Ở đó cho điểm có cỡ tối thiểu theo mét thật => zoom càng lớn điểm càng to (tối đa 64 px).
  const VS_ORTHO =
    '  if (uPB_world > 0.0 && projectionMatrix[2][3] == 0.0) {\n' +
    '    float pbPxPerM = abs(projectionMatrix[0][0]) * uPB_vw * 0.5;\n' +
    '    gl_PointSize = max(gl_PointSize, min(uPB_world * pbPxPerM, 64.0));\n' +
    '  }\n';
  const FS_DECL =
    'uniform float uPB_bright;\nuniform float uPB_mode;\nuniform float uPB_smooth;\nuniform float uPB_n;\n' +
    `uniform float uPB_r[${MAX_RINGS}];\nuniform vec3 uPB_c[${MAX_RINGS}];\nuniform vec3 uPB_far;\n` +
    'varying float vPB_dist;\n' +
    'vec3 pbColor(float d) {\n' +
    '  float prevR = 0.0;\n  vec3 prevC = uPB_c[0];\n' +
    `  for (int i = 0; i < ${MAX_RINGS}; i++) {\n` +
    '    if (float(i) >= uPB_n) break;\n' +
    '    if (d <= uPB_r[i]) {\n' +
    '      if (uPB_smooth > 0.5 && i > 0)\n' +
    '        return mix(prevC, uPB_c[i], clamp((d - prevR) / max(uPB_r[i] - prevR, 0.001), 0.0, 1.0));\n' +
    '      return uPB_c[i];\n' +
    '    }\n' +
    '    prevR = uPB_r[i];\n    prevC = uPB_c[i];\n' +
    '  }\n' +
    '  return uPB_far;\n' +
    '}\n';
  // Màu trùng với màu range rings của host: vùng trong vòng 10 m = màu vòng 10 m, ...
  const FS_CODE =
    '\n  if (uPB_mode > 0.5) gl_FragColor.rgb = pbColor(vPB_dist);\n' +
    '  gl_FragColor.rgb = min(gl_FragColor.rgb * uPB_bright, vec3(1.0));\n';

  function inject(src, decl, code) {
    const re = /void\s+main\s*\(/g;
    let m, iMain = -1;
    while ((m = re.exec(src))) iMain = m.index;
    const iEnd = src.lastIndexOf('}');
    if (iMain < 0 || iEnd < iMain) return null;
    return src.slice(0, iMain) + decl + src.slice(iMain, iEnd) + code + src.slice(iEnd);
  }

  const patchedShaders = new WeakSet();
  const patchedPrograms = new WeakSet();
  const currentProgram = new WeakMap();
  const locCache = new WeakMap();
  const viewports = new WeakMap(); // gl -> [x, y, w, h]
  const matLocName = new WeakMap(); // uniform location -> 'projectionMatrix' | 'modelViewMatrix'
  const progMats = new WeakMap(); // program -> { projectionMatrix, modelViewMatrix }
  const views = new Map(); // canvas -> camera của view 2D vẽ gần nhất (dùng cho đường mặt đất)
  let expectFrag = false;

  // M = A * B (ma trận 4x4 theo cột)
  const mul4 = (A, B) => {
    const M = new Float32Array(16);
    for (let c = 0; c < 4; c++)
      for (let r = 0; r < 4; r++)
        M[c * 4 + r] = A[r] * B[c * 4] + A[4 + r] * B[c * 4 + 1] + A[8 + r] * B[c * 4 + 2] + A[12 + r] * B[c * 4 + 3];
    return M;
  };

  function patchProto(P) {
    if (!P) return;
    const oSrc = P.shaderSource, oAttach = P.attachShader, oUse = P.useProgram;
    const oDA = P.drawArrays, oDE = P.drawElements, oVP = P.viewport;
    const oGUL = P.getUniformLocation, oUM4 = P.uniformMatrix4fv;

    P.viewport = function (x, y, w, h) {
      viewports.set(this, [x, y, w, h]);
      return oVP.call(this, x, y, w, h);
    };

    // Ghi lại ma trận camera mà three.js gửi cho shader điểm, để biết z = ? nằm ở đâu trên màn hình
    P.getUniformLocation = function (pr, name) {
      const loc = oGUL.call(this, pr, name);
      if (loc && (name === 'projectionMatrix' || name === 'modelViewMatrix')) matLocName.set(loc, name);
      return loc;
    };
    P.uniformMatrix4fv = function (loc, transpose, data) {
      try {
        const name = loc && matLocName.get(loc);
        if (name && !transpose && data && data.length >= 16) {
          const pr = currentProgram.get(this);
          if (pr && patchedPrograms.has(pr)) {
            let m = progMats.get(pr);
            if (!m) progMats.set(pr, (m = {}));
            m[name] = Float32Array.from(Array.prototype.slice.call(data, 0, 16));
          }
        }
      } catch (e) {}
      return oUM4.apply(this, arguments);
    };

    P.shaderSource = function (sh, src) {
      try {
        const type = this.getShaderParameter(sh, this.SHADER_TYPE);
        if (type === this.VERTEX_SHADER) {
          expectFrag = false;
          if (src.includes('gl_PointSize') && /\bposition\b/.test(src)) {
            const code = /\bprojectionMatrix\b/.test(src) ? VS_CODE + VS_ORTHO : VS_CODE;
            const out = inject(src, VS_DECL, code);
            if (out) { src = out; patchedShaders.add(sh); expectFrag = true; }
          }
        } else if (type === this.FRAGMENT_SHADER && expectFrag) {
          expectFrag = false;
          const out = inject(src, FS_DECL, FS_CODE);
          if (out) { src = out; patchedShaders.add(sh); }
        }
      } catch (e) { console.warn('[PointBooster]', e); }
      return oSrc.call(this, sh, src);
    };

    P.attachShader = function (pr, sh) {
      if (patchedShaders.has(sh)) patchedPrograms.add(pr);
      return oAttach.call(this, pr, sh);
    };

    P.useProgram = function (pr) {
      currentProgram.set(this, pr);
      return oUse.call(this, pr);
    };

    function setUniforms(gl) {
      const pr = currentProgram.get(gl);
      if (!pr || !patchedPrograms.has(pr)) return;
      let L = locCache.get(pr);
      if (!L) {
        const u = (n) => gl.getUniformLocation(pr, n);
        L = {
          s: u('uPB_size'), w: u('uPB_world'), vw: u('uPB_vw'), e: u('uPB_l2e'), b: u('uPB_bright'), m: u('uPB_mode'), sm: u('uPB_smooth'),
          n: u('uPB_n'), r: u('uPB_r'), c: u('uPB_c'), f: u('uPB_far'),
        };
        locCache.set(pr, L);
      }
      gl.uniform1f(L.s, cfg.size);
      gl.uniform1f(L.w, cfg.world / 100);
      const vp = viewports.get(gl) || [0, 0, gl.drawingBufferWidth, gl.drawingBufferHeight];
      gl.uniform1f(L.vw, vp[2]);
      // View trực giao (Top/Side/Front): lưu camera để vẽ đường mặt đất
      const mats = progMats.get(pr);
      if (mats && mats.projectionMatrix && mats.modelViewMatrix && mats.projectionMatrix[11] === 0 &&
          typeof HTMLCanvasElement !== 'undefined' && gl.canvas instanceof HTMLCanvasElement) {
        views.set(gl.canvas, {
          M: mul4(mats.projectionMatrix, mats.modelViewMatrix), vp: vp.slice(), bufH: gl.drawingBufferHeight,
        });
        startGroundLoop();
      }
      gl.uniformMatrix4fv(L.e, false, egoMat);
      gl.uniform1f(L.b, cfg.bright);
      gl.uniform1f(L.m, cfg.mode);
      gl.uniform1f(L.sm, cfg.smooth);
      gl.uniform1f(L.n, rings.length);
      gl.uniform1fv(L.r, ringR);
      gl.uniform3fv(L.c, ringC);
      gl.uniform3fv(L.f, FAR_COLOR);
    }

    P.drawArrays = function (...a) { setUniforms(this); return oDA.apply(this, a); };
    P.drawElements = function (...a) { setUniforms(this); return oDE.apply(this, a); };
  }

  patchProto(window.WebGLRenderingContext && WebGLRenderingContext.prototype);
  patchProto(window.WebGL2RenderingContext && WebGL2RenderingContext.prototype);

  // ---- Đường mặt đất ở view Side/Front ----
  // Đường nằm ở độ cao z thật (hệ lidar) nên zoom / pan view thì vẫn bám đúng mặt đất.
  // Kéo đường lên/xuống để đổi z; ô "Mặt đất z" trong panel cập nhật theo.
  const GROUND_COLOR = '#95de64';
  const overlays = new Map(); // canvas -> { wrap, line, label }
  let groundLoopOn = false;

  // Trục dọc màn hình phải là trục z (view nhìn ngang). View Top nhìn từ trên xuống thì bỏ qua.
  const isSideLike = (M) => Math.abs(M[9]) > 1e-9 && Math.abs(M[9]) > 2 * Math.max(Math.abs(M[1]), Math.abs(M[5]));
  // z -> toạ độ y (px, theo CSS) tính từ mép trên canvas, và ngược lại
  const zToY = (st, rect, z) => {
    const ndc = st.M[9] * z + st.M[13];
    const fromBottom = st.vp[1] + (ndc + 1) / 2 * st.vp[3];
    return rect.height * (1 - fromBottom / st.bufH);
  };
  const yToZ = (st, rect, y) => {
    const fromBottom = (1 - y / rect.height) * st.bufH;
    const ndc = (fromBottom - st.vp[1]) / st.vp[3] * 2 - 1;
    return (ndc - st.M[13]) / st.M[9];
  };

  function makeOverlay(canvas) {
    const wrap = document.createElement('div');
    wrap.style.cssText = 'position:fixed;pointer-events:none;overflow:hidden;z-index:2147483000;display:none';
    const line = document.createElement('div');
    line.title = 'Đường mặt đất: kéo lên/xuống để chỉnh độ cao';
    line.style.cssText =
      'position:absolute;left:0;right:0;height:13px;margin-top:-6px;cursor:ns-resize;pointer-events:auto;touch-action:none';
    line.innerHTML =
      `<div style="position:absolute;left:0;right:0;top:5px;height:0;border-top:2px dashed ${GROUND_COLOR}"></div>` +
      `<div class="pb-gl-label" style="position:absolute;left:6px;bottom:8px;padding:1px 5px;border-radius:3px;` +
      `background:rgba(0,0,0,.7);color:${GROUND_COLOR};font:11px sans-serif;white-space:nowrap;pointer-events:none"></div>`;
    wrap.appendChild(line);
    document.body.appendChild(wrap);
    const o = { wrap, line, label: line.querySelector('.pb-gl-label') };

    let dragging = false;
    const stop = (e) => e.stopPropagation(); // không để CVAT nhận thao tác kéo này
    line.addEventListener('pointerdown', (e) => {
      if (e.button !== 0) return;
      dragging = true;
      line.setPointerCapture(e.pointerId);
      e.preventDefault();
      stop(e);
    });
    line.addEventListener('pointermove', (e) => {
      stop(e);
      const st = views.get(canvas);
      if (!dragging || !st) return;
      const rect = canvas.getBoundingClientRect();
      cfg.ground = fitRange('ground', yToZ(st, rect, e.clientY - rect.top));
      onGroundChange();
    });
    const end = (e) => { stop(e); if (dragging) { dragging = false; save(); } };
    line.addEventListener('pointerup', end);
    line.addEventListener('pointercancel', end);
    ['mousedown', 'mouseup', 'click', 'dblclick', 'contextmenu'].forEach((t) => line.addEventListener(t, stop));
    return o;
  }

  function drawGround() {
    if (!document.body) return;
    const z = groundZ();
    views.forEach((st, canvas) => {
      let o = overlays.get(canvas);
      if (!canvas.isConnected) {
        if (o) o.wrap.remove();
        overlays.delete(canvas);
        views.delete(canvas);
        return;
      }
      const rect = canvas.getBoundingClientRect();
      const y = rect.width > 0 && rect.height > 0 && isSideLike(st.M) ? zToY(st, rect, z) : NaN;
      const show = !!cfg.showGround && y >= 0 && y <= rect.height;
      if (!show) { if (o) o.wrap.style.display = 'none'; return; }
      if (!o) overlays.set(canvas, (o = makeOverlay(canvas)));
      const s = o.wrap.style;
      s.display = '';
      s.left = rect.left + 'px';
      s.top = rect.top + 'px';
      s.width = rect.width + 'px';
      s.height = rect.height + 'px';
      o.line.style.top = y + 'px';
      o.label.textContent = `Mặt đất z = ${z.toFixed(2)} m`;
    });
  }

  function startGroundLoop() {
    if (groundLoopOn) return;
    groundLoopOn = true;
    const tick = () => {
      try { drawGround(); } catch (e) { console.warn('[PointBooster]', e); }
      requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
  }

  // ---- Bảng tham chiếu kích thước theo class (Guideline 3D Cuboid, mục 4.3), đơn vị m ----
  // [loại, Lmin, Lmax, Wmin, Wmax, Hmin, Hmax]
  const SIZE_REF = {
    Car: [
      ['Sedan, hatchback', 3.8, 5.0, 1.70, 1.90, 1.40, 1.60],
      ['SUV, minivan, van chở người', 4.4, 5.3, 1.80, 2.10, 1.65, 2.10],
    ],
    Truck: [
      ['Bán tải', 5.2, 6.2, 1.85, 2.10, 1.80, 2.00],
      ['Xe tải hộp, xe tải nhẹ', 5.5, 9.0, 2.00, 2.55, 2.40, 3.50],
      ['Xe tải nặng, đầu kéo', 6.0, 12.0, 2.40, 2.60, 3.00, 4.00],
    ],
    Trailer: [
      ['Rơ-moóc sau đầu kéo', 8.0, 16.0, 2.50, 2.60, 3.50, 4.10],
      ['Rơ-moóc nhỏ (sau xe con)', 2.0, 5.0, 1.50, 2.20, 0.80, 2.50],
    ],
    Bus: [
      ['Xe buýt thường', 10.0, 12.5, 2.50, 2.60, 3.00, 3.50],
      ['Xe buýt nối toa', 17.0, 18.5, 2.50, 2.60, 3.00, 3.40],
    ],
    Construction_vehicle: [
      ['Máy xúc, cần cẩu, xe lu', 4.0, 12.0, 2.00, 3.20, 2.50, 4.00],
    ],
    Pedestrian: [
      ['Người lớn', 0.4, 0.9, 0.5, 0.9, 1.50, 1.95],
      ['Trẻ em', 0.3, 0.6, 0.3, 0.6, 0.90, 1.40],
    ],
    Two_wheels: [
      ['Xe đạp có người', 1.6, 1.9, 0.50, 0.70, 1.60, 1.90],
      ['Xe máy có người', 1.8, 2.3, 0.70, 0.95, 1.40, 1.80],
      ['Xe máy / xe đạp dựng', 1.6, 2.2, 0.50, 0.90, 1.00, 1.30],
    ],
    Barrier: [
      ['Rào bê tông, rào nước', 1.0, 3.0, 0.40, 0.70, 0.80, 1.10],
    ],
  };
  // Tên nhãn trong CVAT -> key của SIZE_REF (không phân biệt hoa thường, bỏ "_", "-", dấu cách)
  const REF_ALIAS = [
    [/^(car|vehicle\.car)$/, 'Car'],
    [/^truck/, 'Truck'],
    [/^trailer/, 'Trailer'],
    [/^bus/, 'Bus'],
    [/^construction/, 'Construction_vehicle'],
    [/^(pedestrian|person|human)/, 'Pedestrian'],
    [/^(twowheels?|bicycle|motorcycle|cyclist|motorbike)/, 'Two_wheels'],
    [/^barrier/, 'Barrier'],
  ];
  const refKey = (name) => {
    const n = String(name || '').toLowerCase().replace(/[\s_-]+/g, '');
    const hit = REF_ALIAS.find(([re]) => re.test(n));
    return hit ? hit[1] : null;
  };

  // Redux store của CVAT (dò qua cây React, cùng cách với cuboid-helper)
  let pbStore = null;
  function findStore() {
    if (pbStore) return pbStore;
    const root = document.getElementById('root');
    if (!root) return null;
    let fiber = null;
    const key = Object.keys(root).find((k) => k.startsWith('__reactContainer$'));
    if (key) fiber = root[key];
    else if (root._reactRootContainer) {
      const rc = root._reactRootContainer;
      fiber = (rc._internalRoot && rc._internalRoot.current) || rc.current;
    }
    const stack = [fiber];
    let steps = 0;
    while (stack.length && steps++ < 20000) {
      const f = stack.pop();
      if (!f) continue;
      const p = f.memoizedProps;
      const cand = p && (p.store || (p.value && p.value.store));
      if (cand && typeof cand.getState === 'function' && typeof cand.subscribe === 'function') return (pbStore = cand);
      if (f.sibling) stack.push(f.sibling);
      if (f.child) stack.push(f.child);
    }
    return null;
  }

  // Class đang chọn: nhãn của cuboid đang được chọn, nếu không có thì nhãn đang chọn để vẽ.
  // Trả về null nếu không ở trong job 3D.
  function readSelection(st) {
    const ann = st && st.annotation;
    const canvas = ann && ann.canvas && ann.canvas.instance;
    if (!canvas || typeof canvas.html !== 'function') return null;
    let html;
    try { html = canvas.html(); } catch (e) { return null; }
    if (!html || !html.perspective) return null;
    const job = ann.job || {};
    const labels = job.labels || (job.instance && job.instance.labels) || [];
    const drawId = ann.drawing && ann.drawing.activeLabelID;
    const drawLabel = labels.find((x) => x.id === drawId) || null;
    const actId = ann.annotations && ann.annotations.activatedStateID;
    const states = (ann.annotations && ann.annotations.states) || [];
    const state = actId == null ? null : states.find((x) => x.clientID === actId) || null;
    let dims = null;
    if (state && String(state.shapeType).toLowerCase() === 'cuboid' && state.points && state.points.length >= 9) {
      dims = [state.points[6], state.points[7], state.points[8]].map(Number);
      if (!dims.every(Number.isFinite)) dims = null;
    }
    return {
      drawName: drawLabel ? drawLabel.name : '',
      objName: state && state.label ? state.label.name : '',
      objId: state ? state.clientID : null,
      dims,
    };
  }

  // Khung bảng tham chiếu: tự hiện trong job 3D theo class đang chọn, kéo thả được, "×" ẩn đến khi đổi class.
  // Có cuboid đang chọn thì thêm dòng kích thước của box đó và tô xanh loại xe khớp.
  function buildRefCard(ui, saveUI) {
    const GOOD = '#73d13d', BAD = '#ffa940';
    const card = document.createElement('div');
    card.style.cssText =
      'position:fixed;left:0;top:0;z-index:2147483646;display:none;background:rgba(20,20,20,.9);color:#fff;' +
      'font:12px sans-serif;border-radius:8px;box-shadow:0 2px 8px rgba(0,0,0,.4);min-width:300px';
    card.innerHTML =
      '<div class="pb-ref-head" title="Kéo để di chuyển" style="display:flex;align-items:center;gap:6px;' +
      'padding:5px 6px 5px 10px;cursor:move;user-select:none;touch-action:none;border-bottom:1px solid #333">' +
      '<b class="pb-ref-title" style="flex:1"></b>' +
      ['pb-ref-min', 'pb-ref-close'].map((c) =>
        `<button class="${c}" type="button" style="width:22px;height:20px;padding:0;cursor:pointer;` +
        'background:transparent;color:#fff;border:1px solid #555;border-radius:4px;font:14px/16px sans-serif"></button>').join('') +
      '</div>' +
      '<div class="pb-ref-body" style="padding:6px 10px 8px"></div>';
    document.body.appendChild(card);
    const head = card.querySelector('.pb-ref-head');
    const titleEl = card.querySelector('.pb-ref-title');
    const bodyEl = card.querySelector('.pb-ref-body');
    const minBtn = card.querySelector('.pb-ref-min');
    const closeBtn = card.querySelector('.pb-ref-close');
    closeBtn.textContent = '×';
    closeBtn.title = 'Ẩn (tự hiện lại khi đổi class)';

    // Thu nhỏ: chỉ còn thanh tiêu đề (vẫn đổi tên class theo lựa chọn), nhớ trạng thái cho lần sau
    const setCollapsed = (c) => {
      ui.refCollapsed = !!c;
      bodyEl.style.display = c ? 'none' : '';
      head.style.borderBottom = c ? 'none' : '1px solid #333';
      minBtn.textContent = c ? '+' : '−';
      minBtn.title = c ? 'Mở rộng' : 'Thu nhỏ';
    };
    setCollapsed(ui.refCollapsed);
    const toggleCollapsed = () => { setCollapsed(!ui.refCollapsed); placeSaved(); saveUI(); };
    minBtn.addEventListener('click', toggleCollapsed);
    head.addEventListener('dblclick', (e) => { if (!e.target.closest('button')) toggleCollapsed(); });

    const place = (x, y) => {
      const maxX = Math.max(0, window.innerWidth - card.offsetWidth);
      const maxY = Math.max(0, window.innerHeight - head.offsetHeight);
      ui.refX = Math.round(Math.min(maxX, Math.max(0, x)));
      ui.refY = Math.round(Math.min(maxY, Math.max(0, y)));
      card.style.left = ui.refX + 'px';
      card.style.top = ui.refY + 'px';
    };
    const placeSaved = () => place(Number.isFinite(ui.refX) ? ui.refX : 60, Number.isFinite(ui.refY) ? ui.refY : 70);
    window.addEventListener('resize', () => { if (card.style.display !== 'none') placeSaved(); });

    let drag = null;
    head.addEventListener('pointerdown', (e) => {
      if (e.button !== 0 || e.target.closest('button')) return;
      drag = { dx: e.clientX - ui.refX, dy: e.clientY - ui.refY };
      head.setPointerCapture(e.pointerId);
      e.preventDefault();
    });
    head.addEventListener('pointermove', (e) => { if (drag) place(e.clientX - drag.dx, e.clientY - drag.dy); });
    const endDrag = () => { if (drag) { drag = null; saveUI(); } };
    head.addEventListener('pointerup', endDrag);
    head.addEventListener('pointercancel', endDrag);

    let dismissed = null; // class đã bấm "×"; đổi sang class khác thì hiện lại
    let lastKey = null;
    let shown = null;
    closeBtn.addEventListener('click', () => { dismissed = shown; update(true); });

    const f1 = (v) => v.toFixed(1), f2 = (v) => v.toFixed(2);
    const range = (a, b, f) => `${f(a)} – ${f(b)}`;
    const fits = (r, L, W, H) => L >= r[1] && L <= r[2] && W >= r[3] && W <= r[4] && H >= r[5] && H <= r[6];
    const td = (s, extra = '') => `<td style="padding:2px 6px;white-space:nowrap;${extra}">${s}</td>`;
    const tdR = (s, extra = '') => td(s, 'text-align:right;' + extra);

    function render(sel, name, key) {
      const rows = SIZE_REF[key];
      titleEl.textContent = `Kích thước tham chiếu: ${name}`;
      if (!rows) {
        bodyEl.innerHTML = '<div style="opacity:.8"></div>';
        bodyEl.firstChild.textContent = `Guideline chưa có bảng tham chiếu cho class "${name}".`;
        return;
      }
      const dims = sel.objName && refKey(sel.objName) === key ? sel.dims : null;
      const [L, W, H] = dims || [];
      let html =
        '<table style="border-collapse:collapse;width:100%">' +
        '<tr style="opacity:.7">' + td('Loại') + tdR('L (m)') + tdR('W (m)') + tdR('H (m)') + '</tr>';
      rows.forEach((r) => {
        const ok = dims && fits(r, L, W, H);
        html += `<tr style="${ok ? 'background:rgba(115,209,61,.22)' : ''}">` +
          td((ok ? '✓ ' : '') + r[0]) + tdR(range(r[1], r[2], f1)) + tdR(range(r[3], r[4], f2)) +
          tdR(range(r[5], r[6], f2)) + '</tr>';
      });
      if (dims) {
        // Ô màu xanh: nằm trong khoảng của ít nhất một loại xe thuộc class này
        const inAny = (i, v) => rows.some((r) => v >= r[1 + 2 * i] && v <= r[2 + 2 * i]);
        const cell = (i, v, f) => tdR(f(v), `font-weight:bold;color:${inAny(i, v) ? GOOD : BAD}`);
        html += '<tr style="border-top:1px solid #444">' + td(`Cuboid #${sel.objId}`) +
          cell(0, L, f2) + cell(1, W, f2) + cell(2, H, f2) + '</tr>';
      }
      html += '</table>';
      if (dims) {
        const anyFit = rows.some((r) => fits(r, L, W, H));
        const swapFit = !anyFit && rows.some((r) => fits(r, W, L, H));
        html += `<div style="margin-top:5px;color:${anyFit ? GOOD : BAD}">` +
          (anyFit ? '✓ Box nằm trong bảng tham chiếu'
            : swapFit ? '⚠ L và W có vẻ bị đảo: kiểm tra hướng đầu xe (mũi tên đỏ)'
              : '⚠ Box nằm ngoài bảng tham chiếu: đối chiếu lại point cloud và camera') + '</div>';
      }
      html += '<div style="margin-top:5px;opacity:.55">Guideline 3D Cuboid §4.3 · point cloud và camera là bằng chứng chính</div>';
      bodyEl.innerHTML = html;
    }

    function update(force) {
      const s = findStore();
      const sel = s && readSelection(s.getState());
      const name = sel ? sel.objName || sel.drawName : '';
      const key = refKey(name);
      const dimKey = sel && sel.dims ? sel.dims.map((v) => v.toFixed(3)).join(',') : '';
      const k = `${cfg.refTable}|${name}|${sel ? sel.objId : ''}|${dimKey}`;
      if (!force && k === lastKey) return;
      lastKey = k;
      if (name !== dismissed) dismissed = null; // đổi class => bỏ trạng thái đã ẩn
      shown = name;
      if (!cfg.refTable || !name || dismissed === name) { card.style.display = 'none'; return; }
      render(sel, name, key);
      const wasHidden = card.style.display === 'none';
      card.style.display = '';
      if (wasHidden) placeSaved();
    }

    // Chờ CVAT dựng xong store rồi theo dõi mọi thay đổi (chọn nhãn, chọn cuboid, sửa kích thước)
    const timer = setInterval(() => {
      const s = findStore();
      if (!s) return;
      clearInterval(timer);
      s.subscribe(() => { try { update(false); } catch (e) { console.warn('[PointBooster]', e); } });
      update(true);
    }, 1000);
    return { update };
  }

  // ---- Bảng điều khiển: kéo thả bằng thanh tiêu đề, "−" thu nhỏ, "×" đóng (Alt+P mở lại) ----
  const UI_KEY = 'cvatPointBooster.ui'; // vị trí + trạng thái thu nhỏ, tách riêng để nút reset không dời panel
  function buildPanel() {
    if (window.top !== window) return;
    let ui = {};
    try { ui = JSON.parse(localStorage.getItem(UI_KEY) || '{}') || {}; } catch (e) {}
    const saveUI = () => { try { localStorage.setItem(UI_KEY, JSON.stringify(ui)); } catch (e) {} };

    const d = document.createElement('div');
    d.style.cssText =
      'position:fixed;left:0;top:0;z-index:2147483647;background:rgba(20,20,20,.88);' +
      'color:#fff;font:12px sans-serif;border-radius:8px;width:230px;box-shadow:0 2px 8px rgba(0,0,0,.4)';
    const TIPS = {
      world: 'View Top/Side/Front: điểm có cỡ tối thiểu theo mét thật nên phóng to view thì điểm to theo. 0 = tắt',
      ground: 'Độ cao mặt đất (m, hệ lidar) của đường kẻ ở view Side/Front. Kéo đường trong view hoặc điền số. ' +
        '"Đặt lại mặc định" = lấy lại từ calib',
    };
    const val = (k) => (k === 'ground' ? groundZ() : cfg[k]);
    // Mỗi dòng: nhãn + ô điền số (min–max) + thanh trượt + dòng báo lỗi
    const row = (k, label) => {
      const [min, max, dec] = RANGES[k];
      return `<div style="margin-top:6px"${TIPS[k] ? ` title="${TIPS[k]}"` : ''}>` +
        '<div style="display:flex;align-items:center;gap:4px">' +
        `<span style="flex:1">${label}</span>` +
        `<input id="pb-n-${k}" type="text" inputmode="decimal" autocomplete="off" spellcheck="false" value="${val(k)}" ` +
        `title="Điền số từ ${min} đến ${max}, tối đa ${dec} chữ số sau dấu chấm. Enter để áp dụng, Esc để huỷ" ` +
        'style="width:50px;padding:1px 4px;background:#111;color:#fff;border:1px solid #555;border-radius:3px;' +
        'font:12px sans-serif;text-align:right;outline:none">' +
        '</div>' +
        `<input id="pb-${k}" type="range" min="${min}" max="${max}" step="${1 / 10 ** dec}" value="${val(k)}" style="width:100%">` +
        `<div id="pb-e-${k}" style="display:none;color:#ff7875;font-size:11px"></div></div>`;
    };
    const check = (k, label) =>
      `<label style="display:block;margin-top:6px"><input type="checkbox" id="pb-${k}" ${cfg[k] ? 'checked' : ''}> ${label}</label>`;
    const hdrBtn = (id, text, title) =>
      `<button id="${id}" type="button" title="${title}" style="width:22px;height:20px;padding:0;margin-left:4px;` +
      'cursor:pointer;background:transparent;color:#fff;border:1px solid #555;border-radius:4px;font:14px/16px sans-serif">' +
      `${text}</button>`;
    d.innerHTML =
      '<div id="pb-head" title="Kéo để di chuyển" style="display:flex;align-items:center;padding:6px 8px 6px 12px;' +
      'cursor:move;user-select:none;touch-action:none;border-bottom:1px solid #333">' +
      '<b style="flex:1">Point Booster</b>' +
      hdrBtn('pb-min', '−', 'Thu nhỏ / mở rộng') +
      hdrBtn('pb-close', '×', 'Đóng (Alt+P để mở lại)') +
      '</div>' +
      '<div id="pb-body" style="padding:2px 12px 10px">' +
      row('size', 'Cỡ điểm') +
      row('world', 'Cỡ điểm view 2D (cm)') +
      row('bright', 'Độ sáng') +
      check('mode', 'Tô màu theo vòng khoảng cách') +
      check('smooth', 'Chuyển màu mượt giữa các vòng') +
      check('refTable', 'Bảng L × W × H khi chọn class') +
      '<div id="pb-legend" style="margin-top:6px;line-height:18px"></div>' +
      '<div id="pb-origin" style="margin-top:4px;opacity:.7"></div>' +
      `<div style="margin-top:8px;padding-top:6px;border-top:1px solid #333">` +
      `<button id="pb-ground-toggle" type="button" style="width:100%;padding:4px 0;cursor:pointer;` +
      `background:#1f2a1a;color:${GROUND_COLOR};border:1px solid ${GROUND_COLOR};border-radius:4px;font:12px sans-serif"></button>` +
      row('ground', 'Mặt đất z (m)') +
      '</div>' +
      '<button id="pb-reset" type="button" style="margin-top:8px;width:100%;padding:4px 0;cursor:pointer;' +
      'background:#333;color:#fff;border:1px solid #666;border-radius:4px;font:12px sans-serif">↺ Đặt lại mặc định</button>' +
      '<div style="margin-top:6px;opacity:.5">Alt+P: ẩn/hiện</div>' +
      '</div>';
    document.body.appendChild(d);

    const head = d.querySelector('#pb-head');
    const body = d.querySelector('#pb-body');
    const minBtn = d.querySelector('#pb-min');

    // ---- Vị trí ----
    const place = (x, y) => {
      // giữ cả thanh tiêu đề trong màn hình
      const maxX = Math.max(0, window.innerWidth - d.offsetWidth);
      const maxY = Math.max(0, window.innerHeight - head.offsetHeight);
      ui.x = Math.round(Math.min(maxX, Math.max(0, x)));
      ui.y = Math.round(Math.min(maxY, Math.max(0, y)));
      d.style.left = ui.x + 'px';
      d.style.top = ui.y + 'px';
    };
    const setCollapsed = (c) => {
      ui.collapsed = !!c;
      body.style.display = c ? 'none' : '';
      head.style.borderBottom = c ? 'none' : '1px solid #333';
      minBtn.textContent = c ? '+' : '−';
      minBtn.title = c ? 'Mở rộng' : 'Thu nhỏ';
    };
    setCollapsed(ui.collapsed);
    if (Number.isFinite(ui.x) && Number.isFinite(ui.y)) place(ui.x, ui.y);
    else place(window.innerWidth - d.offsetWidth - 12, window.innerHeight - d.offsetHeight - 12); // mặc định góc phải dưới
    window.addEventListener('resize', () => place(ui.x, ui.y));

    // ---- Kéo thả ----
    let drag = null;
    head.addEventListener('pointerdown', (e) => {
      if (e.button !== 0 || e.target.closest('button')) return;
      drag = { dx: e.clientX - ui.x, dy: e.clientY - ui.y };
      head.setPointerCapture(e.pointerId);
      e.preventDefault();
    });
    head.addEventListener('pointermove', (e) => { if (drag) place(e.clientX - drag.dx, e.clientY - drag.dy); });
    const endDrag = () => { if (drag) { drag = null; saveUI(); } };
    head.addEventListener('pointerup', endDrag);
    head.addEventListener('pointercancel', endDrag);
    head.addEventListener('dblclick', (e) => { if (!e.target.closest('button')) minBtn.click(); });

    // ---- Thu nhỏ / đóng ----
    minBtn.addEventListener('click', () => { setCollapsed(!ui.collapsed); place(ui.x, ui.y); saveUI(); });
    d.querySelector('#pb-close').addEventListener('click', () => { d.style.display = 'none'; });

    // ---- Chú thích màu + gốc đo ----
    const chip = (color, text) =>
      `<span style="display:inline-block;margin-right:6px;white-space:nowrap">` +
      `<span style="display:inline-block;width:10px;height:10px;border-radius:2px;vertical-align:-1px;background:${color}"></span> ${text}</span>`;
    const renderLegend = () => {
      const far = `rgb(${FAR_COLOR.map((v) => Math.round(v * 255)).join(',')})`;
      d.querySelector('#pb-legend').innerHTML =
        rings.map((r) => chip(r.color, `≤${r.radius}`)).join('') +
        chip(far, `>${rings[rings.length - 1].radius} m`);
    };
    const renderOrigin = () => {
      const el = d.querySelector('#pb-origin');
      el.textContent = hasEgo ? 'Gốc đo: xe ego (trục sau), khoảng cách mặt đất' : 'Gốc đo: lidar (frame chưa có calib ego)';
      el.title = hasEgo
        ? 'Cùng cách đo với "Show distance to ego" của CVAT'
        : 'Không có lidar_to_ego cho frame này: đo từ gốc lidar, giống CVAT';
    };
    renderLegend();
    renderOrigin();

    // Theo dõi khi người dùng sửa vòng trong CVAT (nút bút chì cạnh "Show range rings")
    setInterval(() => { if (loadRings()) renderLegend(); }, 1000);

    // ---- Điều khiển ----
    const SLIDERS = ['size', 'world', 'bright', 'ground'];
    const CHECKS = ['mode', 'smooth', 'refTable'];
    const showError = (k, msg) => {
      const err = d.querySelector('#pb-e-' + k);
      err.textContent = msg || '';
      err.style.display = msg ? '' : 'none';
      d.querySelector('#pb-n-' + k).style.borderColor = msg ? '#ff4d4f' : '#555';
    };
    const showValue = (k) => {
      d.querySelector('#pb-' + k).value = val(k);
      d.querySelector('#pb-n-' + k).value = val(k);
      showError(k, '');
    };
    const groundBtn = d.querySelector('#pb-ground-toggle');
    const renderGroundBtn = () => {
      groundBtn.textContent = cfg.showGround ? '▬ Ẩn đường mặt đất' : '▬ Hiện đường mặt đất (view Side/Front)';
    };
    const syncUI = () => {
      SLIDERS.forEach(showValue);
      CHECKS.forEach((k) => { d.querySelector('#pb-' + k).checked = !!cfg[k]; });
      renderGroundBtn();
    };
    renderGroundBtn();
    groundBtn.addEventListener('click', () => { cfg.showGround = cfg.showGround ? 0 : 1; save(); renderGroundBtn(); });

    // Kéo đường trong view hoặc calib mới về => cập nhật ô "Mặt đất z" (trừ khi đang gõ trong ô đó)
    const refreshGround = () => { if (document.activeElement !== d.querySelector('#pb-n-ground')) showValue('ground'); };
    onGroundChange = refreshGround;
    onCalibChange = () => { renderOrigin(); refreshGround(); };

    // Chỉ giữ chữ số, một dấu "." (dấu "," đổi thành ".") và dấu "-" ở đầu nếu cho phép số âm;
    // cắt bớt chữ số thừa sau dấu "."
    const sanitize = (k, s) => {
      const [min, , dec] = RANGES[k];
      const neg = min < 0 && s.trim().startsWith('-');
      s = s.replace(/,/g, '.').replace(/[^0-9.]/g, '');
      const i = s.indexOf('.');
      if (i >= 0) s = s.slice(0, i + 1) + s.slice(i + 1).replace(/\./g, '').slice(0, dec);
      return (neg ? '-' : '') + s;
    };
    // Kiểm tra chuỗi trong ô: trả về { v } nếu hợp lệ, { err } nếu sai, {} nếu đang gõ dở ("", ".", "-", "3.")
    const validate = (k, s) => {
      const [min, max, dec] = RANGES[k];
      if (s === '' || s === '-' || s.endsWith('.')) return {};
      const v = Number(s);
      if (!Number.isFinite(v)) return { err: 'Không phải số' };
      if (v < min || v > max) return { err: `Chỉ nhận từ ${min} đến ${max}` };
      return { v: roundTo(v, dec) };
    };
    const apply = (k, v) => {
      cfg[k] = v;
      d.querySelector('#pb-' + k).value = v;
      save();
    };

    SLIDERS.forEach((k) => {
      const slider = d.querySelector('#pb-' + k);
      const box = d.querySelector('#pb-n-' + k);

      slider.addEventListener('input', () => {
        apply(k, fitRange(k, parseFloat(slider.value)));
        showValue(k);
      });

      box.addEventListener('input', () => {
        const clean = sanitize(k, box.value);
        if (clean !== box.value) {
          const pos = Math.max(0, (box.selectionStart || clean.length) - (box.value.length - clean.length));
          box.value = clean;
          box.setSelectionRange(pos, pos);
        }
        const r = validate(k, clean);
        showError(k, r.err);
        if (r.v !== undefined) apply(k, r.v); // hợp lệ thì áp dụng ngay khi gõ
      });

      const commit = () => {
        const r = validate(k, box.value);
        if (r.err || r.v === undefined) {
          // sai hoặc bỏ trống: quay về giá trị đang dùng
          showValue(k);
          return false;
        }
        apply(k, r.v);
        showValue(k);
        return true;
      };
      box.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') {
          const r = validate(k, box.value);
          if (r.err) { box.select(); return; } // giữ lỗi để người dùng sửa
          commit();
          box.blur();
        } else if (e.key === 'Escape') {
          showValue(k);
          box.blur();
        }
      });
      box.addEventListener('blur', commit);
      box.addEventListener('focus', () => box.select());
    });
    CHECKS.forEach((k) => {
      d.querySelector('#pb-' + k).addEventListener('change', (e) => {
        cfg[k] = e.target.checked ? 1 : 0;
        save();
        if (k === 'refTable') ref.update(true);
      });
    });
    d.querySelector('#pb-reset').addEventListener('click', () => {
      const showGround = cfg.showGround; // giữ nguyên trạng thái ẩn/hiện đường mặt đất
      Object.assign(cfg, DEFAULTS, { showGround });
      save();
      syncUI();
      ref.update(true);
    });

    const ref = buildRefCard(ui, saveUI);

    d.addEventListener('keydown', (e) => e.stopPropagation()); // tránh phím tắt CVAT
    window.addEventListener('keydown', (e) => {
      if (e.altKey && e.code === 'KeyP') {
        d.style.display = d.style.display === 'none' ? '' : 'none';
        if (d.style.display === '') place(ui.x, ui.y);
      }
    });
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', buildPanel);
  else buildPanel();
})();
