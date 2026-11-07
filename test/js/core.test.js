// Unit tests for web/js/core.js (run with: node --test test/js)
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');
const c = require(path.join(__dirname, '..', '..', 'web', 'js', 'core.js'));

const close = (a, b, eps = 1e-9) => assert.ok(Math.abs(a - b) < eps, `${a} != ${b}`);

// ---------------------------------------------------------------------------
// Mock roslib
// ---------------------------------------------------------------------------

function makeMockRoslib() {
  const instances = [];
  class Ros {
    constructor() {
      this.handlers = {};
      this.closed = false;
      this.sent = [];
      instances.push(this);
    }
    on(ev, fn) { (this.handlers[ev] = this.handlers[ev] || []).push(fn); }
    emit(ev) { (this.handlers[ev] || []).forEach((fn) => fn()); }
    connect(url) { this.url = url; }
    close() { this.closed = true; }
  }
  class Topic {
    constructor(opts) { Object.assign(this, opts); }
    publish(msg) { this.ros.sent.push({ topic: this.name, msg }); }
  }
  return { Ros, Topic, instances };
}

function makeFakeTimers() {
  const pending = [];
  return {
    pending,
    setTimeout: (fn, ms) => { const h = { fn, ms }; pending.push(h); return h; },
    clearTimeout: (h) => { const i = pending.indexOf(h); if (i >= 0) pending.splice(i, 1); },
    fireNext() { const h = pending.shift(); h.fn(); return h.ms; }
  };
}

function makeConnection() {
  const roslib = makeMockRoslib();
  const timers = makeFakeTimers();
  const states = [];
  const conn = new c.RosConnection({
    url: 'ws://localhost:9090',
    roslib,
    topics: { '/webxr/right/pose': 'geometry_msgs/PoseStamped' },
    onStatus: (s) => states.push(s),
    setTimeout: timers.setTimeout,
    clearTimeout: timers.clearTimeout
  });
  return { conn, roslib, timers, states };
}

// ---------------------------------------------------------------------------
// Reconnection
// ---------------------------------------------------------------------------

test('publishes only while connected and drops messages otherwise', () => {
  const { conn, roslib } = makeConnection();
  conn.start();
  assert.equal(conn.publish('/webxr/right/pose', { a: 1 }), false);
  assert.equal(conn.droppedCount, 1);

  roslib.instances[0].emit('connection');
  assert.equal(conn.publish('/webxr/right/pose', { a: 2 }), true);
  assert.deepEqual(roslib.instances[0].sent, [{ topic: '/webxr/right/pose', msg: { a: 2 } }]);
  assert.equal(conn.publish('/unknown', {}), false);
});

test('reconnects with exponential backoff capped at 5 s', () => {
  const { conn, roslib, timers } = makeConnection();
  conn.start();
  const delays = [];
  for (let i = 0; i < 7; i++) {
    roslib.instances[roslib.instances.length - 1].emit('close');
    assert.equal(timers.pending.length, 1);
    delays.push(timers.fireNext());
  }
  assert.deepEqual(delays, [250, 500, 1000, 2000, 4000, 5000, 5000]);
  assert.equal(roslib.instances.length, 8);
  assert.ok(roslib.instances.slice(0, 7).every((r) => r.closed));
});

test('backoff resets after a successful connection', () => {
  const { conn, roslib, timers } = makeConnection();
  conn.start();
  roslib.instances[0].emit('close');
  timers.fireNext();
  roslib.instances[1].emit('close');
  assert.equal(timers.fireNext(), 500);
  roslib.instances[2].emit('connection');
  roslib.instances[2].emit('close');
  assert.equal(timers.fireNext(), 250);
});

test('error followed by close schedules only one reconnect', () => {
  const { conn, roslib, timers } = makeConnection();
  conn.start();
  roslib.instances[0].emit('error');
  roslib.instances[0].emit('close');
  assert.equal(timers.pending.length, 1);
});

test('new topics are created per connection and old sockets are ignored', () => {
  const { conn, roslib, timers } = makeConnection();
  conn.start();
  roslib.instances[0].emit('connection');
  roslib.instances[0].emit('close');
  timers.fireNext();
  // Late event from the old socket must not mark the connection as up.
  roslib.instances[0].emit('connection');
  assert.equal(conn.connected, false);
  roslib.instances[1].emit('connection');
  conn.publish('/webxr/right/pose', { b: 1 });
  assert.equal(roslib.instances[0].sent.length, 0);
  assert.equal(roslib.instances[1].sent.length, 1);
});

test('stop cancels pending reconnects', () => {
  const { conn, roslib, timers, states } = makeConnection();
  conn.start();
  roslib.instances[0].emit('close');
  conn.stop();
  assert.equal(timers.pending.length, 0);
  assert.equal(states[states.length - 1], 'stopped');
});

// ---------------------------------------------------------------------------
// Message formatting
// ---------------------------------------------------------------------------

test('PoseStamped and Twist structure', () => {
  const msg = c.buildPoseStamped('webxr', 1500.25, { x: 1, y: 2, z: 3 }, { x: 0, y: 0, z: 0, w: 1 });
  assert.deepEqual(msg.header, { stamp: { sec: 1, nanosec: 500250000 }, frame_id: 'webxr' });
  assert.deepEqual(msg.pose.position, { x: 1, y: 2, z: 3 });
  const tw = c.buildTwist({ x: 1, y: 0, z: 0 }, { x: 0, y: 0, z: 2 });
  assert.deepEqual(tw, { linear: { x: 1, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 2 } });
});

test('Joy layout: 4 axis slots followed by button values', () => {
  const gamepad = {
    axes: [0, 0, 0.5, -0.25],
    buttons: [
      { value: 0.7, pressed: true }, { value: 0.2, pressed: false }, { value: 0, pressed: false },
      { value: 0, pressed: false }, { value: 1, pressed: true }, { value: 0, pressed: false }
    ]
  };
  const joy = c.buildJoy('webxr', 0, gamepad);
  assert.deepEqual(joy.buttons, [1, 0, 0, 0, 1, 0]);
  assert.deepEqual(joy.axes, [0, 0, 0.5, -0.25, 0.7, 0.2, 0, 0, 1, 0]);
});

test('two-axis gamepads are normalized into thumbstick slots', () => {
  assert.deepEqual(c.normalizeAxes([0.1, -0.2]), [0, 0, 0.1, -0.2]);
  assert.deepEqual(c.normalizeAxes([1, 2, 3, 4, 5]), [1, 2, 3, 4]);
  assert.deepEqual(c.normalizeAxes(null), [0, 0, 0, 0]);
});

test('URL query overrides rosbridge address', () => {
  assert.equal(c.rosbridgeUrlFromLocation({ search: '' }), 'ws://localhost:9090');
  assert.equal(c.rosbridgeUrlFromLocation({ search: '?host=192.168.1.5&port=9091' }), 'ws://192.168.1.5:9091');
  assert.equal(c.rosbridgeUrlFromLocation({ search: '?host=pc.local&wss=1' }), 'wss://pc.local:9090');
});

// ---------------------------------------------------------------------------
// Frame conversion and velocity estimation
// ---------------------------------------------------------------------------

test('frame conversion: forward/right/up', () => {
  assert.deepEqual(c.webxrToRosVector({ x: 0, y: 0, z: -1 }), { x: 1, y: -0, z: 0 });
  assert.deepEqual(c.webxrToRosVector({ x: 1, y: 0, z: 0 }), { x: -0, y: -1, z: 0 });
  assert.deepEqual(c.webxrToRosVector({ x: 0, y: 1, z: 0 }), { x: -0, y: -0, z: 1 });
});

test('angular velocity from quaternions', () => {
  const rate = 1.2; // rad/s about +Y
  const dt = 1 / 72;
  const q = (a) => ({ x: 0, y: Math.sin(a / 2), z: 0, w: Math.cos(a / 2) });
  const w = c.angularVelocityFromQuats(q(0.3), q(0.3 + rate * dt), dt);
  close(w.x, 0);
  close(w.y, rate, 1e-9);
  close(w.z, 0);
});

test('velocity estimator converges to constant velocity and resets on gaps', () => {
  const est = new c.VelocityEstimator(0.05);
  const v = { x: 0.3, y: 0, z: -0.1 };
  const q = { x: 0, y: 0, z: 0, w: 1 };
  let out;
  for (let i = 0; i < 200; i++) {
    const t = i / 72;
    out = est.update(t, { x: v.x * t, y: 0, z: v.z * t }, q, null, null);
  }
  close(out.linear.x, 0.3, 1e-6);
  close(out.linear.z, -0.1, 1e-6);
  // A 1 s gap (tracking loss) must not produce a velocity spike.
  out = est.update(200 / 72 + 1.0, { x: 10, y: 0, z: 0 }, q, null, null);
  assert.deepEqual(out.linear, { x: 0, y: 0, z: 0 });
});

test('velocity estimator prefers native XRPose velocities', () => {
  const est = new c.VelocityEstimator(0.05);
  const q = { x: 0, y: 0, z: 0, w: 1 };
  const out = est.update(0, { x: 0, y: 0, z: 0 }, q, { x: 1, y: 2, z: 3 }, { x: 0, y: 0.5, z: 0 });
  assert.deepEqual(out.linear, { x: 1, y: 2, z: 3 });
  assert.deepEqual(out.angular, { x: 0, y: 0.5, z: 0 });
});

// ---------------------------------------------------------------------------
// Regression: the vendored roslib must publish through RosConnection.
// (roslib 1.4.1 throws on publish when Topic is created with
// reconnect_on_close: false; the mock above cannot catch that.)
// ---------------------------------------------------------------------------

test('vendored roslib advertises and publishes over a WebSocket', () => {
  const fs = require('node:fs');
  const vm = require('node:vm');
  const sockets = [];
  class FakeWebSocket {
    constructor(url) {
      this.url = url;
      this.sent = [];
      this.readyState = 0;
      sockets.push(this);
    }
    send(data) { this.sent.push(JSON.parse(data)); }
    close() { this.readyState = 3; if (this.onclose) this.onclose({}); }
    open() { this.readyState = 1; if (this.onopen) this.onopen({}); }
  }
  FakeWebSocket.OPEN = 1;
  const sandbox = { WebSocket: FakeWebSocket, console, setTimeout, clearTimeout };
  sandbox.window = sandbox;
  vm.createContext(sandbox);
  const src = fs.readFileSync(path.join(__dirname, '..', '..', 'web', 'vendor', 'roslib.min.js'), 'utf8');
  vm.runInContext(src, sandbox);

  const conn = new c.RosConnection({
    url: 'ws://localhost:9090',
    roslib: sandbox.ROSLIB,
    topics: { '/webxr/right/pose': 'geometry_msgs/PoseStamped' }
  });
  conn.start();
  sockets[0].open();
  assert.equal(conn.connected, true);
  assert.equal(conn.publish('/webxr/right/pose', { header: {} }), true);
  const ops = sockets[0].sent.map((m) => m.op);
  assert.deepEqual(ops, ['advertise', 'publish']);
  assert.equal(sockets[0].sent[0].type, 'geometry_msgs/PoseStamped');
  conn.stop();
});
