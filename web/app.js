/*
 * WebXR session handling: reads Touch controller poses and buttons and
 * publishes them to rosbridge as standard ROS messages on /webxr/*.
 *
 * URL query parameters:
 *   host, port  rosbridge address (default localhost:9090)
 *   wss=1       use wss:// (optional HTTPS over Wi-Fi mode)
 *   rate        max publish rate per hand in Hz (default 0 = every XR frame)
 *   space       "grip" (default) or "target" (use targetRaySpace instead)
 */
(function () {
  'use strict';

  var QWB = window.QWB;
  var params = QWB.parseQuery(window.location.search);
  var ROSBRIDGE_URL = QWB.rosbridgeUrlFromLocation(window.location);
  var MAX_RATE_HZ = parseFloat(params.rate || '0') || 0;
  var POSE_SPACE = params.space === 'target' ? 'target' : 'grip';
  var FRAME_ID = 'webxr';
  var SIDES = ['left', 'right'];

  var TOPICS = {};
  SIDES.forEach(function (side) {
    TOPICS['/webxr/' + side + '/pose'] = 'geometry_msgs/PoseStamped';
    TOPICS['/webxr/' + side + '/twist'] = 'geometry_msgs/Twist';
    TOPICS['/webxr/' + side + '/joy'] = 'sensor_msgs/Joy';
  });
  TOPICS['/webxr/diagnostics'] = 'std_msgs/String';

  var el = function (id) { return document.getElementById(id); };

  // ---------------------------------------------------------------------------
  // Logging and status helpers
  // ---------------------------------------------------------------------------

  var logLines = [];
  function log(msg) {
    var line = new Date().toISOString().substring(11, 23) + ' ' + msg;
    logLines.push(line);
    if (logLines.length > 30) logLines.shift();
    el('log').textContent = logLines.join('\n');
    if (window.console) console.log('[qwb] ' + msg);
  }

  function setStatus(id, text, cls) {
    var node = el(id);
    node.textContent = text;
    node.className = cls || '';
  }

  el('ua').textContent = navigator.userAgent;

  // ---------------------------------------------------------------------------
  // rosbridge connection
  // ---------------------------------------------------------------------------

  var rosState = 'connecting';
  var ros = new QWB.RosConnection({
    url: ROSBRIDGE_URL,
    roslib: window.ROSLIB,
    topics: TOPICS,
    onStatus: function (state, detail) {
      rosState = state;
      var cls = state === 'connected' ? 'ok' : (state === 'connecting' ? 'warn' : 'bad');
      setStatus('ros-status', state.toUpperCase() + (detail ? ' - ' + detail : ''), cls);
      if (state !== 'connecting') log('rosbridge ' + state + (detail ? ': ' + detail : ''));
    }
  });
  ros.start();

  // ---------------------------------------------------------------------------
  // Per-hand state and diagnostics
  // ---------------------------------------------------------------------------

  function makeHandState() {
    return {
      estimator: new QWB.VelocityEstimator(0.05),
      lastPublishT: -Infinity,
      pubRate: new QWB.RateMeter(1.0),
      diag: {
        present: false,
        hasGripSpace: false,
        hasGamepad: false,
        mapping: '',
        poseValid: false,
        emulatedPosition: false,
        nativeVelocity: false,
        axes: [],
        buttons: [],
        pressedEver: {}
      }
    };
  }

  var hands = { left: makeHandState(), right: makeHandState() };

  function resetHands() {
    SIDES.forEach(function (side) { hands[side] = makeHandState(); });
  }

  function fmt(v) { return (v >= 0 ? ' ' : '') + v.toFixed(3); }

  function renderHandDiag(side) {
    var d = hands[side].diag;
    var lines = [
      'present:          ' + d.present,
      'gripSpace:        ' + d.hasGripSpace,
      'gamepad:          ' + d.hasGamepad + (d.mapping ? ' (mapping "' + d.mapping + '")' : ''),
      'pose valid:       ' + d.poseValid + (d.emulatedPosition ? ' (emulated position)' : ''),
      'native velocity:  ' + d.nativeVelocity,
      'publish rate:     ' + hands[side].pubRate.hz.toFixed(1) + ' Hz',
      'axes:             [' + d.axes.map(fmt).join(',') + ']',
      'buttons (value/pressed/touched):'
    ];
    d.buttons.forEach(function (b, i) {
      lines.push('  [' + i + '] ' + b.value.toFixed(2) + ' ' + (b.pressed ? 'P' : '-') + (b.touched ? 'T' : '-'));
    });
    var ever = Object.keys(d.pressedEver);
    lines.push('pressed this session: ' + (ever.length ? ever.join(', ') : 'none'));
    el('diag-' + side).textContent = lines.join('\n');
  }

  var lastSourcesText = '-';
  function describeSources(session) {
    if (!session) return lastSourcesText;
    var out = ['count: ' + session.inputSources.length];
    for (var i = 0; i < session.inputSources.length; i++) {
      var s = session.inputSources[i];
      out.push('[' + i + '] handedness=' + s.handedness +
        ' targetRayMode=' + s.targetRayMode +
        ' gripSpace=' + !!s.gripSpace +
        ' gamepad=' + !!s.gamepad +
        (s.gamepad ? ' (buttons=' + s.gamepad.buttons.length + ', axes=' + s.gamepad.axes.length + ')' : '') +
        ' profiles=' + JSON.stringify(s.profiles || []));
    }
    lastSourcesText = out.join('\n');
    return lastSourcesText;
  }

  function diagnosticsSnapshot(session) {
    return {
      user_agent: navigator.userAgent,
      session_active: !!session,
      reference_space: refSpaceType,
      pose_space: POSE_SPACE,
      frame_rate_hz: Math.round(frameRate.hz * 10) / 10,
      input_sources: describeSources(session),
      left: hands.left.diag,
      right: hands.right.diag
    };
  }

  // ---------------------------------------------------------------------------
  // WebXR session
  // ---------------------------------------------------------------------------

  var xrSession = null;
  var refSpace = null;
  var refSpaceType = '-';
  var gl = null;
  var frameRate = new QWB.RateMeter(1.0);
  var lastUiUpdate = 0;
  var lastDiagPublish = 0;

  function checkSupport() {
    if (!window.isSecureContext) {
      setStatus('xr-status', 'NOT A SECURE CONTEXT - open via http://localhost (adb reverse) or HTTPS', 'bad');
      return;
    }
    if (!navigator.xr) {
      setStatus('xr-status', 'navigator.xr not available', 'bad');
      return;
    }
    navigator.xr.isSessionSupported('immersive-vr').then(function (supported) {
      if (supported) {
        setStatus('xr-status', 'OK (supported)', 'ok');
        el('enter-vr').disabled = false;
      } else {
        setStatus('xr-status', 'immersive-vr not supported', 'bad');
      }
    }).catch(function (e) {
      setStatus('xr-status', 'isSessionSupported failed: ' + e.message, 'bad');
    });
  }

  function requestRefSpace(session) {
    return session.requestReferenceSpace('local-floor').then(function (space) {
      refSpaceType = 'local-floor';
      return space;
    }).catch(function () {
      log('local-floor unavailable, falling back to local');
      return session.requestReferenceSpace('local').then(function (space) {
        refSpaceType = 'local';
        return space;
      });
    });
  }

  function startSession() {
    el('enter-vr').disabled = true;
    navigator.xr.requestSession('immersive-vr', { optionalFeatures: ['local-floor'] }).then(function (session) {
      xrSession = session;
      resetHands();
      setStatus('session-status', 'starting', 'warn');

      var canvas = document.createElement('canvas');
      gl = canvas.getContext('webgl', { xrCompatible: true, alpha: false, antialias: false });
      var layerPromise = gl.makeXRCompatible ? gl.makeXRCompatible() : Promise.resolve();

      session.addEventListener('end', onSessionEnd);
      session.addEventListener('inputsourceschange', function () {
        log('input sources changed: ' + session.inputSources.length);
        el('diag-sources').textContent = describeSources(session);
      });

      return layerPromise.then(function () {
        session.updateRenderState({ baseLayer: new XRWebGLLayer(session, gl) });
        return requestRefSpace(session);
      }).then(function (space) {
        refSpace = space;
        el('ref-space').textContent = refSpaceType;
        setStatus('session-status', 'ACTIVE (pose space: ' + POSE_SPACE + ')', 'ok');
        log('XR session started, reference space ' + refSpaceType);
        session.requestAnimationFrame(onXRFrame);
      });
    }).catch(function (e) {
      log('failed to start XR session: ' + e.message);
      setStatus('session-status', 'failed: ' + e.message, 'bad');
      if (xrSession) xrSession.end();
      el('enter-vr').disabled = false;
    });
  }

  function onSessionEnd() {
    log('XR session ended');
    xrSession = null;
    refSpace = null;
    gl = null;
    setStatus('session-status', 'ended (not publishing)', 'warn');
    setStatus('pub-rate', 'not publishing', '');
    SIDES.forEach(renderHandDiag);
    // Publish a final diagnostics snapshot so it is visible on the ROS side.
    ros.publish('/webxr/diagnostics', { data: JSON.stringify(diagnosticsSnapshot(null)) });
    el('enter-vr').disabled = false;
  }

  function readGamepad(diag, gamepad) {
    diag.hasGamepad = !!gamepad;
    if (!gamepad) return;
    diag.mapping = gamepad.mapping || '';
    diag.axes = Array.prototype.slice.call(gamepad.axes);
    diag.buttons = [];
    for (var i = 0; i < gamepad.buttons.length; i++) {
      var b = gamepad.buttons[i];
      diag.buttons.push({ value: b.value, pressed: b.pressed, touched: b.touched });
      if (b.pressed) diag.pressedEver[i] = true;
    }
  }

  function processHand(side, source, frame, tS, nowMs) {
    var hand = hands[side];
    var diag = hand.diag;
    diag.present = true;
    diag.hasGripSpace = !!source.gripSpace;
    readGamepad(diag, source.gamepad);

    var space = POSE_SPACE === 'grip' ? source.gripSpace : source.targetRaySpace;
    var pose = space ? frame.getPose(space, refSpace) : null;
    diag.poseValid = !!pose;
    if (!pose) {
      hand.estimator.reset();
      return false;
    }
    diag.emulatedPosition = !!pose.emulatedPosition;
    diag.nativeVelocity = !!pose.linearVelocity;

    var p = pose.transform.position;
    var q = pose.transform.orientation;
    var vel = hand.estimator.update(tS, p, q, pose.linearVelocity || null, pose.angularVelocity || null);

    if (MAX_RATE_HZ > 0 && tS - hand.lastPublishT < 1.0 / MAX_RATE_HZ - 1e-3) {
      return true;
    }
    hand.lastPublishT = tS;

    var prefix = '/webxr/' + side + '/';
    ros.publish(prefix + 'pose', QWB.buildPoseStamped(FRAME_ID, nowMs,
      QWB.webxrToRosVector(p), QWB.webxrToRosQuaternion(q)));
    ros.publish(prefix + 'twist', QWB.buildTwist(
      QWB.webxrToRosVector(vel.linear), QWB.webxrToRosVector(vel.angular)));
    if (source.gamepad) {
      ros.publish(prefix + 'joy', QWB.buildJoy(FRAME_ID, nowMs, source.gamepad));
    }
    if (ros.connected) hand.pubRate.tick(tS);
    return true;
  }

  function onXRFrame(t, frame) {
    var session = frame.session;
    session.requestAnimationFrame(onXRFrame);
    var tS = t / 1000;
    var nowMs = Date.now();
    frameRate.tick(tS);

    var seen = { left: false, right: false };
    var anyPose = false;
    for (var i = 0; i < session.inputSources.length; i++) {
      var source = session.inputSources[i];
      if (source.handedness !== 'left' && source.handedness !== 'right') continue;
      seen[source.handedness] = true;
      if (processHand(source.handedness, source, frame, tS, nowMs)) anyPose = true;
    }
    SIDES.forEach(function (side) {
      if (!seen[side]) {
        hands[side].diag.present = false;
        hands[side].diag.poseValid = false;
        hands[side].estimator.reset();
      }
    });

    // Minimal rendering: clear to a color that encodes the state.
    var layer = session.renderState.baseLayer;
    gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
    if (!ros.connected) {
      gl.clearColor(0.25, 0.04, 0.04, 1.0);
    } else if (!anyPose) {
      gl.clearColor(0.22, 0.18, 0.03, 1.0);
    } else {
      gl.clearColor(0.03, 0.18, 0.07, 1.0);
    }
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    if (t - lastUiUpdate > 250) {
      lastUiUpdate = t;
      el('frame-rate').textContent = frameRate.hz.toFixed(1) + ' Hz';
      el('pub-rate').textContent = 'left ' + hands.left.pubRate.hz.toFixed(1) +
        ' Hz, right ' + hands.right.pubRate.hz.toFixed(1) + ' Hz (sent ' + ros.sentCount +
        ', dropped while disconnected ' + ros.droppedCount + ')';
      SIDES.forEach(renderHandDiag);
      el('diag-sources').textContent = describeSources(session);
    }
    if (t - lastDiagPublish > 1000) {
      lastDiagPublish = t;
      ros.publish('/webxr/diagnostics', { data: JSON.stringify(diagnosticsSnapshot(session)) });
    }
  }

  el('enter-vr').addEventListener('click', startSession);
  log('rosbridge URL: ' + ROSBRIDGE_URL + ', pose space: ' + POSE_SPACE +
    (MAX_RATE_HZ > 0 ? ', max rate ' + MAX_RATE_HZ + ' Hz' : ''));
  checkSupport();
})();
