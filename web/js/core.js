/*
 * Headset-independent logic for the WebXR to ROS2 bridge page.
 *
 * This file is a classic script (no ES modules, no build step) so that it runs
 * in older Quest browsers and can also be loaded by Node.js for unit tests.
 * In the browser it exposes everything under `window.QWB`.
 */
(function (root, factory) {
  'use strict';
  if (typeof module === 'object' && module.exports) {
    module.exports = factory();
  } else {
    root.QWB = factory();
  }
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  // ---------------------------------------------------------------------------
  // Coordinate frame conversion (the only place where it is implemented).
  //
  // WebXR: +X right, +Y up, -Z forward (right-handed).
  // ROS:   +X forward, +Y left, +Z up (right-handed).
  // R_C_W = [[0,0,-1],[-1,0,0],[0,1,0]], i.e. (x, y, z)_R = (-z, -x, y)_W.
  // Because R_C_W is a proper rotation, a quaternion is converted by remapping
  // its vector part like a position and keeping w.
  // ---------------------------------------------------------------------------

  function webxrToRosVector(v) {
    return { x: -v.z, y: -v.x, z: v.y };
  }

  function webxrToRosQuaternion(q) {
    return { x: -q.z, y: -q.x, z: q.y, w: q.w };
  }

  // ---------------------------------------------------------------------------
  // Small quaternion helpers (x, y, z, w objects).
  // ---------------------------------------------------------------------------

  function quatMultiply(a, b) {
    return {
      x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    };
  }

  function quatConjugate(q) {
    return { x: -q.x, y: -q.y, z: -q.z, w: q.w };
  }

  // Angular velocity (expressed in the reference frame) that rotates qPrev
  // into qCurr over dt seconds: dq = qCurr * qPrev^-1, omega = axis * angle / dt.
  function angularVelocityFromQuats(qPrev, qCurr, dt) {
    var dq = quatMultiply(qCurr, quatConjugate(qPrev));
    if (dq.w < 0) {
      // Take the shortest path.
      dq = { x: -dq.x, y: -dq.y, z: -dq.z, w: -dq.w };
    }
    var s = Math.sqrt(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z);
    if (s < 1e-9 || dt <= 0) {
      return { x: 0, y: 0, z: 0 };
    }
    var angle = 2 * Math.atan2(s, dq.w);
    var k = angle / (s * dt);
    return { x: dq.x * k, y: dq.y * k, z: dq.z * k };
  }

  // ---------------------------------------------------------------------------
  // Velocity estimation with a first-order low-pass filter.
  // Works in the WebXR frame; the result is converted to ROS by the caller.
  // ---------------------------------------------------------------------------

  function VelocityEstimator(timeConstantS) {
    this.tau = typeof timeConstantS === 'number' ? timeConstantS : 0.05;
    this.reset();
  }

  VelocityEstimator.prototype.reset = function () {
    this.prevT = null;
    this.prevP = null;
    this.prevQ = null;
    this.linear = { x: 0, y: 0, z: 0 };
    this.angular = { x: 0, y: 0, z: 0 };
  };

  VelocityEstimator.prototype._filter = function (state, raw, alpha) {
    return {
      x: state.x + alpha * (raw.x - state.x),
      y: state.y + alpha * (raw.y - state.y),
      z: state.z + alpha * (raw.z - state.z)
    };
  };

  /**
   * Update with a new sample.
   * @param {number} tS - timestamp in seconds
   * @param {{x,y,z}} p - position (WebXR frame)
   * @param {{x,y,z,w}} q - orientation (WebXR frame)
   * @param {{x,y,z}|null} measuredLinear - XRPose.linearVelocity if available
   * @param {{x,y,z}|null} measuredAngular - XRPose.angularVelocity if available
   * @returns {{linear: {x,y,z}, angular: {x,y,z}}} velocities in the WebXR frame
   */
  VelocityEstimator.prototype.update = function (tS, p, q, measuredLinear, measuredAngular) {
    var dt = this.prevT === null ? 0 : tS - this.prevT;
    // Discard estimates across large gaps (tracking loss, paused session).
    var valid = this.prevT !== null && dt > 1e-4 && dt < 0.25;
    var alpha = valid ? dt / (this.tau + dt) : 1;

    if (measuredLinear) {
      this.linear = { x: measuredLinear.x, y: measuredLinear.y, z: measuredLinear.z };
    } else if (valid) {
      var rawLin = {
        x: (p.x - this.prevP.x) / dt,
        y: (p.y - this.prevP.y) / dt,
        z: (p.z - this.prevP.z) / dt
      };
      this.linear = this._filter(this.linear, rawLin, alpha);
    } else {
      this.linear = { x: 0, y: 0, z: 0 };
    }

    if (measuredAngular) {
      this.angular = { x: measuredAngular.x, y: measuredAngular.y, z: measuredAngular.z };
    } else if (valid) {
      this.angular = this._filter(this.angular, angularVelocityFromQuats(this.prevQ, q, dt), alpha);
    } else {
      this.angular = { x: 0, y: 0, z: 0 };
    }

    this.prevT = tS;
    this.prevP = { x: p.x, y: p.y, z: p.z };
    this.prevQ = { x: q.x, y: q.y, z: q.z, w: q.w };
    return { linear: this.linear, angular: this.angular };
  };

  // ---------------------------------------------------------------------------
  // Message builders (plain objects in rosbridge JSON form).
  // ---------------------------------------------------------------------------

  // Number of axis slots reserved at the start of Joy.axes (xr-standard:
  // touchpad x, touchpad y, thumbstick x, thumbstick y). Button analog values
  // follow at JOY_AXES_SLOTS + button index.
  var JOY_AXES_SLOTS = 4;

  function stampFromMs(ms) {
    var sec = Math.floor(ms / 1000);
    return { sec: sec, nanosec: Math.round((ms - sec * 1000) * 1e6) };
  }

  function header(frameId, ms) {
    return { stamp: stampFromMs(ms), frame_id: frameId };
  }

  function buildPoseStamped(frameId, ms, pRos, qRos) {
    return {
      header: header(frameId, ms),
      pose: {
        position: { x: pRos.x, y: pRos.y, z: pRos.z },
        orientation: { x: qRos.x, y: qRos.y, z: qRos.z, w: qRos.w }
      }
    };
  }

  function buildTwist(linRos, angRos) {
    return {
      linear: { x: linRos.x, y: linRos.y, z: linRos.z },
      angular: { x: angRos.x, y: angRos.y, z: angRos.z }
    };
  }

  /**
   * Normalize gamepad axes to the xr-standard 4-slot layout.
   * Some older browsers report only the thumbstick ([x, y]); those values are
   * placed into slots 2 and 3 so that the slot meaning stays fixed.
   */
  function normalizeAxes(rawAxes) {
    var axes = [0, 0, 0, 0];
    var a = rawAxes || [];
    if (a.length === 2) {
      axes[2] = a[0];
      axes[3] = a[1];
    } else {
      for (var i = 0; i < JOY_AXES_SLOTS && i < a.length; i++) {
        axes[i] = a[i];
      }
    }
    return axes;
  }

  /**
   * Build sensor_msgs/Joy from a Gamepad-like object.
   * buttons: 1 if pressed else 0, in xr-standard order.
   * axes: [4 normalized axis slots] + [analog value of each button].
   */
  function buildJoy(frameId, ms, gamepad) {
    var buttons = [];
    var axes = normalizeAxes(gamepad ? gamepad.axes : null);
    var gb = gamepad && gamepad.buttons ? gamepad.buttons : [];
    for (var i = 0; i < gb.length; i++) {
      var b = gb[i];
      var pressed = typeof b === 'object' ? !!b.pressed : b > 0.5;
      var value = typeof b === 'object' ? Number(b.value) || 0 : Number(b) || 0;
      buttons.push(pressed ? 1 : 0);
      axes.push(value);
    }
    return { header: header(frameId, ms), axes: axes, buttons: buttons };
  }

  // ---------------------------------------------------------------------------
  // Reconnecting rosbridge connection.
  //
  // A fresh ROSLIB.Ros (and fresh topics) is created for every attempt, and
  // messages are only sent while connected. roslib queues messages sent while
  // disconnected and flushes them on reconnect, which would replay stale poses
  // to the robot, so publish() drops messages instead.
  // ---------------------------------------------------------------------------

  /**
   * @param {object} opts
   * @param {string} opts.url - ws:// or wss:// URL
   * @param {object} opts.roslib - the ROSLIB namespace (injected for tests)
   * @param {object} opts.topics - {name: messageType}
   * @param {function} [opts.onStatus] - called with (state, detail)
   * @param {function} [opts.setTimeout] - injected timer (tests)
   * @param {function} [opts.clearTimeout] - injected timer (tests)
   * @param {number} [opts.initialDelayMs=250]
   * @param {number} [opts.maxDelayMs=5000]
   */
  function RosConnection(opts) {
    this.url = opts.url;
    this.roslib = opts.roslib;
    this.topicTypes = opts.topics;
    this.onStatus = opts.onStatus || function () {};
    this._setTimeout = opts.setTimeout || function (f, ms) { return setTimeout(f, ms); };
    this._clearTimeout = opts.clearTimeout || function (h) { clearTimeout(h); };
    this.initialDelayMs = opts.initialDelayMs || 250;
    this.maxDelayMs = opts.maxDelayMs || 5000;

    this.connected = false;
    this.ros = null;
    this.topics = {};
    this.delayMs = this.initialDelayMs;
    this.timer = null;
    this.stopped = true;
    this.attempts = 0;
    this.sentCount = 0;
    this.droppedCount = 0;
  }

  RosConnection.prototype.start = function () {
    this.stopped = false;
    this._connect();
  };

  RosConnection.prototype.stop = function () {
    this.stopped = true;
    if (this.timer !== null) {
      this._clearTimeout(this.timer);
      this.timer = null;
    }
    this._teardown();
    this.onStatus('stopped', '');
  };

  RosConnection.prototype._teardown = function () {
    var ros = this.ros;
    this.ros = null;
    this.topics = {};
    this.connected = false;
    if (ros) {
      try {
        ros.close();
      } catch (e) {
        // Ignore errors from closing an already failed socket.
      }
    }
  };

  RosConnection.prototype._connect = function () {
    var self = this;
    this.timer = null;
    this.attempts += 1;
    this.onStatus('connecting', this.url + ' (attempt ' + this.attempts + ')');
    var ros = new this.roslib.Ros();
    this.ros = ros;

    ros.on('connection', function () {
      if (self.ros !== ros) return;
      self.connected = true;
      self.delayMs = self.initialDelayMs;
      self.topics = {};
      // reconnect_on_close is left at its default: roslib 1.4.1 throws on the
      // first publish when it is false (callOnConnection is assigned unbound).
      // Re-advertising is harmless here because each Ros instance is discarded
      // after it closes and never reconnects.
      Object.keys(self.topicTypes).forEach(function (name) {
        self.topics[name] = new self.roslib.Topic({
          ros: ros,
          name: name,
          messageType: self.topicTypes[name],
          queue_size: 1
        });
      });
      self.onStatus('connected', self.url);
    });

    var onDown = function (kind) {
      return function () {
        if (self.ros !== ros) return;
        self._teardown();
        self.onStatus('disconnected', kind);
        self._scheduleReconnect();
      };
    };
    ros.on('close', onDown('closed'));
    ros.on('error', onDown('error'));

    try {
      ros.connect(this.url);
    } catch (e) {
      onDown('exception: ' + e.message)();
    }
  };

  RosConnection.prototype._scheduleReconnect = function () {
    if (this.stopped || this.timer !== null) return;
    var self = this;
    var delay = this.delayMs;
    this.delayMs = Math.min(this.delayMs * 2, this.maxDelayMs);
    this.onStatus('waiting', 'reconnect in ' + delay + ' ms');
    this.timer = this._setTimeout(function () {
      self._connect();
    }, delay);
  };

  /** Publish if connected. Returns true if the message was sent. */
  RosConnection.prototype.publish = function (topicName, msg) {
    var topic = this.connected ? this.topics[topicName] : null;
    if (!topic) {
      this.droppedCount += 1;
      return false;
    }
    topic.publish(msg);
    this.sentCount += 1;
    return true;
  };

  // ---------------------------------------------------------------------------
  // Misc helpers.
  // ---------------------------------------------------------------------------

  /** Build the rosbridge URL from a location-like object (?host=, ?port=, ?wss=1). */
  function rosbridgeUrlFromLocation(loc) {
    var params = parseQuery(loc && loc.search ? loc.search : '');
    var host = params.host || 'localhost';
    var port = params.port || '9090';
    var scheme = params.wss === '1' || params.wss === 'true' ? 'wss' : 'ws';
    return scheme + '://' + host + ':' + port;
  }

  function parseQuery(search) {
    var out = {};
    var s = search.charAt(0) === '?' ? search.substring(1) : search;
    if (!s) return out;
    s.split('&').forEach(function (kv) {
      if (!kv) return;
      var i = kv.indexOf('=');
      var k = decodeURIComponent(i < 0 ? kv : kv.substring(0, i));
      var v = i < 0 ? '' : decodeURIComponent(kv.substring(i + 1).replace(/\+/g, ' '));
      out[k] = v;
    });
    return out;
  }

  /** Exponential moving average of an event rate in Hz. */
  function RateMeter(windowS) {
    this.windowS = windowS || 1.0;
    this.count = 0;
    this.t0 = null;
    this.hz = 0;
  }

  RateMeter.prototype.tick = function (tS) {
    if (this.t0 === null) this.t0 = tS;
    this.count += 1;
    var span = tS - this.t0;
    if (span >= this.windowS) {
      this.hz = this.count / span;
      this.count = 0;
      this.t0 = tS;
    }
  };

  return {
    webxrToRosVector: webxrToRosVector,
    webxrToRosQuaternion: webxrToRosQuaternion,
    quatMultiply: quatMultiply,
    quatConjugate: quatConjugate,
    angularVelocityFromQuats: angularVelocityFromQuats,
    VelocityEstimator: VelocityEstimator,
    JOY_AXES_SLOTS: JOY_AXES_SLOTS,
    stampFromMs: stampFromMs,
    buildPoseStamped: buildPoseStamped,
    buildTwist: buildTwist,
    normalizeAxes: normalizeAxes,
    buildJoy: buildJoy,
    RosConnection: RosConnection,
    rosbridgeUrlFromLocation: rosbridgeUrlFromLocation,
    parseQuery: parseQuery,
    RateMeter: RateMeter
  };
});
