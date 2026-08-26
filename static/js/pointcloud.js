/*
 * Interactive 3D point-cloud viewer for the real-world experiments.
 *
 * Each scene ships three binary PLY files:
 *   <name>_init_da.ply  - the observed initial scene (RGB-D lifted to 3D)
 *   <name>_goal.ply     - the imagined goal scene lifted to 3D
 *   <name>_matching.ply - correspondence line segments between the two
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const DATA_ROOT = './static/data/real_world/';

function sceneUrls(name) {
  return {
    init: `${DATA_ROOT}${name}/${name}_init_da.ply`,
    goal: `${DATA_ROOT}${name}/${name}_goal.ply`,
    match: `${DATA_ROOT}${name}/${name}_matching.ply`,
  };
}

/* ------------------------------------------------------------------ */
/* Binary PLY parsing                                                  */
/* ------------------------------------------------------------------ */

const TYPE_SIZES = {
  char: 1, uchar: 1, int8: 1, uint8: 1,
  short: 2, ushort: 2, int16: 2, uint16: 2,
  int: 4, uint: 4, int32: 4, uint32: 4,
  float: 4, float32: 4,
  double: 8, float64: 8,
};

const TYPE_READERS = {
  char: (v, o, le) => v.getInt8(o), int8: (v, o, le) => v.getInt8(o),
  uchar: (v, o, le) => v.getUint8(o), uint8: (v, o, le) => v.getUint8(o),
  short: (v, o, le) => v.getInt16(o, le), int16: (v, o, le) => v.getInt16(o, le),
  ushort: (v, o, le) => v.getUint16(o, le), uint16: (v, o, le) => v.getUint16(o, le),
  int: (v, o, le) => v.getInt32(o, le), int32: (v, o, le) => v.getInt32(o, le),
  uint: (v, o, le) => v.getUint32(o, le), uint32: (v, o, le) => v.getUint32(o, le),
  float: (v, o, le) => v.getFloat32(o, le), float32: (v, o, le) => v.getFloat32(o, le),
  double: (v, o, le) => v.getFloat64(o, le), float64: (v, o, le) => v.getFloat64(o, le),
};

// sRGB -> linear lookup, so the stored photo colours survive three.js colour management.
const SRGB_TO_LINEAR = new Float32Array(256);
for (let i = 0; i < 256; i++) {
  const c = i / 255;
  SRGB_TO_LINEAR[i] = c < 0.04045 ? c * 0.0773993808 : Math.pow(c * 0.9478672986 + 0.0521327014, 2.4);
}

function parseHeader(buffer) {
  // The header is pure ASCII; windows-1252 keeps one char per byte so string
  // indices are also byte offsets.
  const probe = new TextDecoder('windows-1252')
    .decode(new Uint8Array(buffer, 0, Math.min(buffer.byteLength, 65536)));
  const marker = probe.indexOf('end_header');
  if (marker < 0) throw new Error('not a PLY file (no end_header)');
  const dataStart = probe.indexOf('\n', marker) + 1;

  const lines = probe.slice(0, marker).split('\n').map((l) => l.trim()).filter(Boolean);
  let format = null;
  const elements = [];
  for (const line of lines) {
    const parts = line.split(/\s+/);
    if (parts[0] === 'format') {
      format = parts[1];
    } else if (parts[0] === 'element') {
      elements.push({ name: parts[1], count: parseInt(parts[2], 10), props: [] });
    } else if (parts[0] === 'property' && elements.length) {
      const el = elements[elements.length - 1];
      if (parts[1] === 'list') {
        el.props.push({ name: parts[4], list: true, countType: parts[2], itemType: parts[3] });
      } else {
        el.props.push({ name: parts[2], list: false, type: parts[1] });
      }
    }
  }
  return { format, elements, dataStart };
}

/**
 * Parse a binary PLY into flat arrays.
 * @returns {{positions: Float32Array, colors: Float32Array|null, edges: Uint32Array|null}}
 */
function parsePLY(buffer) {
  const { format, elements, dataStart } = parseHeader(buffer);
  if (format !== 'binary_little_endian' && format !== 'binary_big_endian') {
    throw new Error(`unsupported PLY format "${format}" (binary only)`);
  }
  const le = format === 'binary_little_endian';
  const view = new DataView(buffer);
  let offset = dataStart;

  let positions = null;
  let colors = null;
  let edges = null;

  for (const el of elements) {
    const stride = el.props.reduce((s, p) => s + (p.list ? 0 : TYPE_SIZES[p.type]), 0);
    const hasList = el.props.some((p) => p.list);

    if (el.name === 'vertex' && !hasList) {
      positions = new Float32Array(el.count * 3);
      const idx = {};
      let o = 0;
      for (const p of el.props) { idx[p.name] = { off: o, type: p.type }; o += TYPE_SIZES[p.type]; }
      const hasColor = idx.red && idx.green && idx.blue;
      if (hasColor) colors = new Float32Array(el.count * 3);
      const rx = TYPE_READERS[idx.x.type], ry = TYPE_READERS[idx.y.type], rz = TYPE_READERS[idx.z.type];
      for (let i = 0; i < el.count; i++) {
        const base = offset + i * stride;
        positions[i * 3] = rx(view, base + idx.x.off, le);
        positions[i * 3 + 1] = ry(view, base + idx.y.off, le);
        positions[i * 3 + 2] = rz(view, base + idx.z.off, le);
        if (hasColor) {
          colors[i * 3] = SRGB_TO_LINEAR[view.getUint8(base + idx.red.off)];
          colors[i * 3 + 1] = SRGB_TO_LINEAR[view.getUint8(base + idx.green.off)];
          colors[i * 3 + 2] = SRGB_TO_LINEAR[view.getUint8(base + idx.blue.off)];
        }
      }
      offset += el.count * stride;
    } else if (el.name === 'edge' && !hasList) {
      const idx = {};
      let o = 0;
      for (const p of el.props) { idx[p.name] = { off: o, type: p.type }; o += TYPE_SIZES[p.type]; }
      const a = idx.vertex1 || idx.vertex_index1;
      const b = idx.vertex2 || idx.vertex_index2;
      if (a && b) {
        edges = new Uint32Array(el.count * 2);
        const ra = TYPE_READERS[a.type], rb = TYPE_READERS[b.type];
        for (let i = 0; i < el.count; i++) {
          const base = offset + i * stride;
          edges[i * 2] = ra(view, base + a.off, le);
          edges[i * 2 + 1] = rb(view, base + b.off, le);
        }
      }
      offset += el.count * stride;
    } else {
      // Skip elements we do not care about (faces etc.).
      if (hasList) {
        for (let i = 0; i < el.count; i++) {
          for (const p of el.props) {
            if (p.list) {
              const n = TYPE_READERS[p.countType](view, offset, le);
              offset += TYPE_SIZES[p.countType] + n * TYPE_SIZES[p.itemType];
            } else {
              offset += TYPE_SIZES[p.type];
            }
          }
        }
      } else {
        offset += el.count * stride;
      }
    }
  }

  if (!positions) throw new Error('PLY contains no vertex element');
  return { positions, colors, edges };
}

/* ------------------------------------------------------------------ */
/* Fetching with progress                                              */
/* ------------------------------------------------------------------ */

async function fetchWithProgress(url, onProgress) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`${res.status} ${res.statusText} for ${url}`);
  const total = Number(res.headers.get('content-length')) || 0;
  if (!res.body || !total) return res.arrayBuffer();

  const reader = res.body.getReader();
  const chunks = [];
  let received = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    chunks.push(value);
    received += value.length;
    if (onProgress) onProgress(received / total);
  }
  const out = new Uint8Array(received);
  let pos = 0;
  for (const c of chunks) { out.set(c, pos); pos += c.length; }
  return out.buffer;
}

/* ------------------------------------------------------------------ */
/* Viewer                                                              */
/* ------------------------------------------------------------------ */

/* ---- Look & feel: these are the knobs worth tuning --------------- */

// Scene point clouds (init + goal).
const POINT_SIZE = 0.006;       // world units (metres). viser used 0.003; a hair
                                // larger closes the gaps between neighbours.
const POINT_OPACITY = 1.0;      // 1 = solid. ~0.85 lets the far surface show through.

// Correspondence lines. Kept translucent so they read as an overlay on the
// clouds instead of a solid cage in front of them.
const MATCH_OPACITY = 0.5;
const MATCH_DENSITY = 0.6;      // draw half the correspondences, as viser does
// Thickness as the prism's RADIUS in world units (metres). WebGL ignores
// LineBasicMaterial.linewidth, so any real thickness has to be geometry; that
// is what buildThickLines does. Scene objects are only a few cm across, so keep
// this small -- 0.0004 reads as a crisp ~1px line, 0.001 is already chunky.
// Set to 0 to fall back to plain 1px GL lines.
const MATCH_WIDTH = 0.0006;

const FRAME_FILL = 0.92;        // share of the panel the cloud spans
const HINT_IDLE_MS = 11000;     // re-show the drag hint after this much idle time
const HINT_DURATION_MS = 3600;

// Idle "breathing": a slow orbital drift that starts after the viewer has been
// left alone, so the panel does not read as a flat photo. Any interaction stops
// it instantly and it eases back in later.
const IDLE_SPIN_ENABLED = true;
const IDLE_SPIN_DELAY_MS = 2500;   // stillness required before drifting resumes
const IDLE_SPIN_SPEED = 0.2;      // radians / second at full strength
const IDLE_SPIN_AMPLITUDE = 0.09;  // radians; how far it wanders from home
const IDLE_SPIN_FADE_MS = 1400;    // ease-in so it never starts with a jerk

// While the drag hint is on screen the cloud follows the animated hand instead
// of drifting, so the gesture visibly does something. HINT_CYCLE_MS must match
// the pc-drag animation duration in index.css.
const HINT_CYCLE_MS = 1800;
const HINT_FOLLOW_AMPLITUDE = 0.13;  // radians the scene swings with the hand

// Deterministic PRNG so every reload subsamples the same correspondences.
function mulberry32(a) {
  return function () {
    a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export class PointCloudViewer {
  constructor(root) {
    this.root = root;
    this.canvas = root.querySelector('canvas');
    this.statusEl = root.querySelector('[data-pc-status]');
    this.statusTextEl = root.querySelector('[data-pc-status-text]');
    this.hintEl = root.querySelector('[data-pc-hint]');

    this.cache = new Map();     // url -> parsed PLY
    this.groups = {};           // layer name -> THREE.Object3D
    this.visibility = { init: true, goal: true, match: true };
    this.sceneName = null;
    this.loadToken = 0;
    this.active = false;
    this.running = false;
    this.homeCamera = null;

    this._initThree();
    this._initHint();

    this.resizeObserver = new ResizeObserver(() => this._resize());
    this.resizeObserver.observe(this.canvas.parentElement);
  }

  _initThree() {
    this.renderer = new THREE.WebGLRenderer({
      canvas: this.canvas, antialias: true, alpha: false,
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setClearColor(0xffffff, 1);

    this.scene = new THREE.Scene();

    // The PLYs use the capture camera's convention: +x right, +y DOWN, +z forward.
    // three.js expects +y up / -z forward, so rotating the whole cloud a half turn
    // about x maps one into the other. Doing it here (instead of pointing the
    // camera's up vector at -y) keeps orbit drag directions feeling natural.
    this.world = new THREE.Group();
    this.world.rotation.x = Math.PI;
    this.scene.add(this.world);

    this.camera = new THREE.PerspectiveCamera(55, 4 / 3, 0.02, 100);
    // Same viewpoint as viser: sit at the capture camera's optical centre and
    // look straight down the depth axis, which is -z after the rotation above.
    this.camera.up.set(0, 1, 0);
    this.camera.position.set(0, 0, 0);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.rotateSpeed = 0.6;
    this.controls.zoomSpeed = 0.8;
    this.controls.panSpeed = 0.6;
    this.controls.minDistance = 0.05;
    this.controls.maxDistance = 8;
    this.controls.target.set(0, 0, -1);
    this.controls.addEventListener('start', () => this._noteInteraction());
    this.canvas.addEventListener('wheel', () => this._noteInteraction(), { passive: true });
  }

  /* --- lifecycle ------------------------------------------------- */

  setActive(active) {
    this.active = active;
    if (active) {
      this._resize();
      this._start();
      this._scheduleHint(1200);
    } else {
      this._stop();
      this._hideHint();
      clearTimeout(this.hintTimer);
    }
  }

  _start() {
    if (this.running) return;
    this.running = true;
    let last = performance.now();
    const tick = () => {
      if (!this.running) return;
      this.rafId = requestAnimationFrame(tick);
      const now = performance.now();
      const dt = Math.min((now - last) / 1000, 0.05);   // clamp after a tab stall
      last = now;
      this._idleDrift(now, dt);
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
    };
    tick();
  }

  /**
   * Slow orbital drift while the viewer is untouched, so the panel breathes
   * instead of reading as a still photo. While the drag hint is on screen the
   * same mechanism instead tracks the animated hand, so the cloud visibly
   * responds to the gesture the hint is demonstrating.
   */
  _idleDrift(now, dt) {
    if (!this.homeCamera) return;
    if (this.reducedMotion && this.reducedMotion.matches) return;

    const hinting = this.hintEl
      && this.hintEl.classList.contains('is-visible')
      && this.hintStart != null;

    let wanted;
    if (hinting) {
      wanted = this._hintFollowAngle(now);
    } else if (IDLE_SPIN_ENABLED) {
      const idleFor = now - (this.lastInteraction || 0);
      if (idleFor < IDLE_SPIN_DELAY_MS) { this.driftStrength = 0; return; }
      // Ease in over the fade window so motion never appears with a jerk.
      const f = Math.min((idleFor - IDLE_SPIN_DELAY_MS) / IDLE_SPIN_FADE_MS, 1);
      this.driftStrength = f * f * (3 - 2 * f);                  // smoothstep
      this.driftPhase = (this.driftPhase || 0) + dt * IDLE_SPIN_SPEED;
      wanted = Math.sin(this.driftPhase) * IDLE_SPIN_AMPLITUDE * this.driftStrength;
    } else {
      return;
    }

    // Swapping between the two sources must not teleport the camera: rebase so
    // this frame contributes no delta, and let later frames move relative to it.
    if (this.driftMode !== (hinting ? 'hint' : 'idle')) {
      this.driftMode = hinting ? 'hint' : 'idle';
      this.driftApplied = wanted;
      return;
    }

    // Apply only the change since last frame, so the drift composes with
    // wherever the user left the camera instead of fighting it.
    const delta = wanted - (this.driftApplied || 0);
    this.driftApplied = wanted;
    if (!delta) return;

    const offset = this.camera.position.clone().sub(this.controls.target);
    offset.applyAxisAngle(this.camera.up, delta);
    this.camera.position.copy(this.controls.target).add(offset);
  }

  /**
   * Replays the hand's CSS keyframes (pc-drag, 1.8s) in JS and maps its
   * horizontal travel to an orbit angle. The camera swings opposite the hand so
   * that the *scene* tracks it: the cloud slides the way the hand pulls.
   */
  _hintFollowAngle(now) {
    const t = ((now - this.hintStart) % HINT_CYCLE_MS) / HINT_CYCLE_MS;
    const ease = (u) => u * u * (3 - 2 * u);        // approximates ease-in-out
    let hand;                                       // -1 = left, +1 = right
    if (t < 0.15) hand = -1;                        // pressed, still at left
    else if (t < 0.50) hand = -1 + 2 * ease((t - 0.15) / 0.35);   // drag right
    else if (t < 0.70) hand = 1;                    // held, then released
    else hand = 1 - 2 * ease((t - 0.70) / 0.30);    // travel back to the left
    return -hand * HINT_FOLLOW_AMPLITUDE;
  }

  _stop() {
    this.running = false;
    if (this.rafId) cancelAnimationFrame(this.rafId);
  }

  _resize() {
    const box = this.canvas.parentElement.getBoundingClientRect();
    if (!box.width || !box.height) return;
    this.renderer.setSize(box.width, box.height, false);
    this.camera.aspect = box.width / box.height;
    this._applyFraming();
  }

  /* --- scene loading --------------------------------------------- */

  async setScene(name) {
    if (this.sceneName === name) return;
    this.sceneName = name;
    const token = ++this.loadToken;
    const urls = sceneUrls(name);

    this._clearGroups();

    // Opened straight off disk, the browser blocks both the ES-module import and
    // fetch() of the PLYs, so nothing can load and the canvas stays blank. Say so
    // instead of spinning forever on a request that can never succeed.
    if (location.protocol === 'file:') {
      this._showStatus('The 3D view needs a local web server (open the page over http://, not file://).');
      if (this.statusEl) this.statusEl.querySelector('.pc-spinner').style.display = 'none';
      return;
    }

    this._showStatus('Loading point cloud…');

    try {
      const entries = Object.entries(urls);
      const progress = new Array(entries.length).fill(0);
      const parsed = await Promise.all(entries.map(async ([layer, url], i) => {
        const data = await this._load(url, (p) => {
          progress[i] = p;
          if (token === this.loadToken) {
            const pct = Math.round((progress.reduce((a, b) => a + b, 0) / entries.length) * 100);
            this._showStatus(`Loading point cloud… ${pct}%`);
          }
        });
        return [layer, data];
      }));
      if (token !== this.loadToken) return;

      for (const [layer, data] of parsed) {
        this.groups[layer] = layer === 'match' ? buildMatching(data) : buildCloud(data);
        this.groups[layer].visible = this.visibility[layer];
        this.world.add(this.groups[layer]);
      }

      this._frameScene();
      this._hideStatus();
      this._scheduleHint(700);
    } catch (err) {
      if (token !== this.loadToken) return;
      console.error('[point cloud]', err);
      this._showStatus('Could not load the point cloud for this scene.');
    }
  }

  async _load(url, onProgress) {
    if (this.cache.has(url)) {
      if (onProgress) onProgress(1);
      return this.cache.get(url);
    }
    const buffer = await fetchWithProgress(url, onProgress);
    const data = parsePLY(buffer);
    this.cache.set(url, data);
    return data;
  }

  _clearGroups() {
    for (const obj of Object.values(this.groups)) {
      this.world.remove(obj);
      obj.traverse((child) => {
        if (child.geometry) child.geometry.dispose();
        if (child.material) child.material.dispose();
      });
    }
    this.groups = {};
  }

  /**
   * The clouds are one camera frustum back-projected from the origin, so viser's
   * view is the apex view: stay at (0, 0, 0) and look down the depth axis. Only the field of
   * view is fitted, which keeps the point-size-to-spacing ratio identical to
   * viser while letting the cloud fill the panel instead of a third of it.
   */
  _frameScene() {
    const cloud = this.groups.init || this.groups.goal;
    if (!cloud) return;
    const pos = cloud.geometry.getAttribute('position');
    let uxMin = Infinity, uxMax = -Infinity, uyMin = Infinity, uyMax = -Infinity;
    let zSum = 0, zCount = 0;
    for (let i = 0; i < pos.count; i++) {
      const z = pos.getZ(i);
      if (!(z > 1e-4)) continue;
      zSum += z; zCount++;
      const ux = pos.getX(i) / z;
      const uy = pos.getY(i) / z;
      if (ux < uxMin) uxMin = ux;
      if (ux > uxMax) uxMax = ux;
      if (uy < uyMin) uyMin = uy;
      if (uy > uyMax) uyMax = uy;
    }
    if (!Number.isFinite(uxMin)) return;

    this.framing = {
      // Tangent-space bounds of the ray bundle in the camera's own axes. The
      // half turn maps PLY (x, y, z) to world (x, -y, -z), so vertical flips.
      left: uxMin,
      right: uxMax,
      bottom: -uyMax,
      top: -uyMin,
      depth: zCount ? zSum / zCount : 1,
    };
    this.camera.position.set(0, 0, 0);
    this.camera.up.set(0, 1, 0);
    // Look straight down the optical axis. The bundle is strongly asymmetric in
    // x, so aiming at the cloud's centre would tilt the camera ~10 degrees and
    // keystone the rectangular frame into a trapezoid. _applyFraming shifts the
    // frustum sideways instead, which recentres the cloud without tilting --
    // the same trick as a shift lens.
    const d = this.framing.depth;
    this.controls.target.set(0, 0, -d);
    this._applyFraming();
    this.controls.update();
    this.homeCamera = {
      position: this.camera.position.clone(),
      target: this.controls.target.clone(),
      up: this.camera.up.clone(),
    };
  }

  /**
   * Fit the frustum to the bundle's tangent-space bounds. Those bounds are not
   * centred on the optical axis, so a plain symmetric FOV would either crop one
   * side or leave a wide empty margin on the other. setViewOffset renders a
   * sub-window of a larger virtual frame, which shifts the frustum sideways
   * while every ray stays parallel to -z: the frame projects as a true
   * rectangle, and orbiting still behaves normally.
   */
  _applyFraming() {
    if (this.framing) {
      const { left, right, bottom, top } = this.framing;
      const aspect = this.camera.aspect;

      // Grow the window to the panel's aspect so nothing is stretched, then add
      // the margin. Padding is applied about the window's own centre.
      let halfW = (right - left) / 2;
      let halfH = (top - bottom) / 2;
      if (halfW / halfH > aspect) halfH = halfW / aspect;
      else halfW = halfH * aspect;
      halfW /= FRAME_FILL;
      halfH /= FRAME_FILL;

      const midX = (left + right) / 2;
      const midY = (bottom + top) / 2;

      // A symmetric frustum large enough to contain the shifted window, then
      // crop back down to the window itself.
      const symHalfH = Math.max(Math.abs(midY - halfH), Math.abs(midY + halfH));
      const symHalfW = Math.max(Math.abs(midX - halfW), Math.abs(midX + halfW));
      const halfHeight = Math.max(symHalfH, symHalfW / aspect);
      const halfWidth = halfHeight * aspect;

      this.camera.fov = THREE.MathUtils.radToDeg(2 * Math.atan(halfHeight));
      this.camera.setViewOffset(
        2 * halfWidth, 2 * halfHeight,
        midX - halfW + halfWidth,   // window's left edge within the full frame
        halfHeight - (midY + halfH), // full frame's top edge is +halfHeight
        2 * halfW, 2 * halfH,
      );
    }
    this.camera.updateProjectionMatrix();
  }

  resetView() {
    if (!this.homeCamera) return;
    this.camera.position.copy(this.homeCamera.position);
    this.camera.up.copy(this.homeCamera.up);
    this.controls.target.copy(this.homeCamera.target);
    this.controls.update();
  }

  setLayerVisible(layer, visible) {
    this.visibility[layer] = visible;
    if (this.groups[layer]) this.groups[layer].visible = visible;
  }

  /* --- status + drag hint ---------------------------------------- */

  _showStatus(text) {
    if (!this.statusEl) return;
    this.statusTextEl.textContent = text;
    this.statusEl.style.display = 'flex';
  }

  _hideStatus() {
    if (this.statusEl) this.statusEl.style.display = 'none';
  }

  _initHint() {
    this.hintTimer = null;
    this.hintHideTimer = null;
    this.hintStart = null;
    // The CSS already disables the hand animation for this preference; the
    // JS-driven camera motion has to respect it too.
    this.reducedMotion = window.matchMedia
      ? window.matchMedia('(prefers-reduced-motion: reduce)') : null;
    const stop = () => this._noteInteraction();
    this.canvas.addEventListener('pointerdown', stop);
    this.canvas.addEventListener('touchstart', stop, { passive: true });
  }

  // Any interaction dismisses the hint; it drifts back after a while of idling.
  _noteInteraction() {
    // Stop the idle drift and forget the offset it had accumulated, so the next
    // drift starts fresh from wherever the user leaves the camera.
    this.lastInteraction = performance.now();
    this.driftPhase = 0;
    this.driftApplied = 0;
    this.driftStrength = 0;
    this._hideHint();
    this._scheduleHint(HINT_IDLE_MS);
  }

  _scheduleHint(delay) {
    clearTimeout(this.hintTimer);
    if (!this.hintEl) return;
    this.hintTimer = setTimeout(() => {
      if (!this.active || document.hidden) {
        this._scheduleHint(HINT_IDLE_MS);
        return;
      }
      this._showHint();
    }, delay);
  }

  _showHint() {
    if (!this.hintEl) return;
    this.hintEl.classList.add('is-visible');
    // Anchor the follow animation to the moment the CSS animation restarts.
    this.hintStart = performance.now();
    clearTimeout(this.hintHideTimer);
    this.hintHideTimer = setTimeout(() => {
      this._hideHint();
      this._scheduleHint(HINT_IDLE_MS);
    }, HINT_DURATION_MS);
  }

  _hideHint() {
    clearTimeout(this.hintHideTimer);
    this.hintStart = null;
    if (this.hintEl) this.hintEl.classList.remove('is-visible');
  }
}

/* ------------------------------------------------------------------ */
/* Geometry builders                                                   */
/* ------------------------------------------------------------------ */

function buildCloud(data) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(data.positions, 3));
  if (data.colors) geometry.setAttribute('color', new THREE.BufferAttribute(data.colors, 3));
  const material = new THREE.PointsMaterial({
    size: POINT_SIZE,
    sizeAttenuation: true,
    vertexColors: Boolean(data.colors),
    color: data.colors ? 0xffffff : 0x888888,
    opacity: POINT_OPACITY,
    // Only pay for blending when it is actually asked for; a fully opaque cloud
    // renders more predictably without it.
    transparent: POINT_OPACITY < 1,
    depthWrite: POINT_OPACITY >= 1,
  });
  return new THREE.Points(geometry, material);
}

/**
 * Correspondence segments. viser draws plain 1px lines at 50% density; anything
 * denser turns the matched region into a solid slab. We additionally fade them
 * (MATCH_OPACITY) so the clouds stay readable underneath.
 */
function buildMatching(data) {
  const total = data.edges ? data.edges.length / 2 : Math.floor(data.positions.length / 6);
  const order = new Uint32Array(total);
  for (let i = 0; i < total; i++) order[i] = i;
  const rand = mulberry32(0);
  for (let i = total - 1; i > 0; i--) {          // seeded shuffle -> stable subset
    const j = Math.floor(rand() * (i + 1));
    const t = order[i]; order[i] = order[j]; order[j] = t;
  }

  const keep = Math.max(1, Math.round(total * MATCH_DENSITY));
  const index = new Uint32Array(keep * 2);
  for (let k = 0; k < keep; k++) {
    const seg = order[k];
    index[k * 2] = data.edges ? data.edges[seg * 2] : seg * 2;
    index[k * 2 + 1] = data.edges ? data.edges[seg * 2 + 1] : seg * 2 + 1;
  }

  if (MATCH_WIDTH > 0) return buildThickLines(data, index);

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(data.positions, 3));
  if (data.colors) geometry.setAttribute('color', new THREE.BufferAttribute(data.colors, 3));
  geometry.setIndex(new THREE.BufferAttribute(index, 1));
  return new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({
    vertexColors: Boolean(data.colors),
    color: data.colors ? 0xffffff : 0xff5533,
    opacity: MATCH_OPACITY,
    transparent: MATCH_OPACITY < 1,
    // Translucent lines should not stamp the depth buffer, otherwise whichever
    // segment draws first hides the ones crossing behind it.
    depthWrite: MATCH_OPACITY >= 1,
  }));
}

/**
 * GL line width is capped at 1px on essentially every WebGL implementation, so
 * a controllable thickness has to be real geometry. Each correspondence becomes
 * a thin triangular prism (three quads around the segment axis) -- cheap, and
 * unlike a screen-space quad it stays solid from any orbit angle.
 */
function buildThickLines(data, index) {
  const segs = index.length / 2;
  const src = data.positions;
  const srcColor = data.colors;
  const RING = 3;                       // sides of the prism

  const verts = new Float32Array(segs * RING * 2 * 3);
  const cols = srcColor ? new Float32Array(segs * RING * 2 * 3) : null;
  const tris = new Uint32Array(segs * RING * 6);

  const ax = new THREE.Vector3(), bx = new THREE.Vector3();
  const dir = new THREE.Vector3(), n1 = new THREE.Vector3(), n2 = new THREE.Vector3();
  const tmp = new THREE.Vector3();
  let v = 0, t = 0;

  for (let s = 0; s < segs; s++) {
    const ia = index[s * 2], ib = index[s * 2 + 1];
    ax.fromArray(src, ia * 3);
    bx.fromArray(src, ib * 3);
    dir.subVectors(bx, ax);
    const len = dir.length();
    if (!(len > 1e-9)) continue;
    dir.divideScalar(len);

    // Any vector not parallel to dir gives a usable perpendicular frame.
    tmp.set(Math.abs(dir.x) < 0.9 ? 1 : 0, Math.abs(dir.x) < 0.9 ? 0 : 1, 0);
    n1.crossVectors(dir, tmp).normalize();
    n2.crossVectors(dir, n1).normalize();

    const base = v / 3;
    for (let r = 0; r < RING; r++) {
      const a = (r / RING) * Math.PI * 2;
      const ox = n1.x * Math.cos(a) * MATCH_WIDTH + n2.x * Math.sin(a) * MATCH_WIDTH;
      const oy = n1.y * Math.cos(a) * MATCH_WIDTH + n2.y * Math.sin(a) * MATCH_WIDTH;
      const oz = n1.z * Math.cos(a) * MATCH_WIDTH + n2.z * Math.sin(a) * MATCH_WIDTH;
      verts[v] = ax.x + ox; verts[v + 1] = ax.y + oy; verts[v + 2] = ax.z + oz; v += 3;
      verts[v] = bx.x + ox; verts[v + 1] = bx.y + oy; verts[v + 2] = bx.z + oz; v += 3;
    }
    if (cols) {
      for (let r = 0; r < RING; r++) {
        const o = (base + r * 2) * 3;
        cols[o] = srcColor[ia * 3]; cols[o + 1] = srcColor[ia * 3 + 1]; cols[o + 2] = srcColor[ia * 3 + 2];
        cols[o + 3] = srcColor[ib * 3]; cols[o + 4] = srcColor[ib * 3 + 1]; cols[o + 5] = srcColor[ib * 3 + 2];
      }
    }
    for (let r = 0; r < RING; r++) {
      const a0 = base + r * 2, b0 = base + ((r + 1) % RING) * 2;
      tris[t++] = a0; tris[t++] = a0 + 1; tris[t++] = b0;
      tris[t++] = b0; tris[t++] = a0 + 1; tris[t++] = b0 + 1;
    }
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(verts.subarray(0, v), 3));
  if (cols) geometry.setAttribute('color', new THREE.BufferAttribute(cols.subarray(0, v), 3));
  geometry.setIndex(new THREE.BufferAttribute(tris.subarray(0, t), 1));
  return new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({
    vertexColors: Boolean(cols),
    color: cols ? 0xffffff : 0xff5533,
    opacity: MATCH_OPACITY,
    transparent: MATCH_OPACITY < 1,
    depthWrite: MATCH_OPACITY >= 1,
    side: THREE.DoubleSide,
  }));
}
