/*
  ====================================================
  ESP32 WiFi 4WD Robot — Dual Joystick WebSocket Controller
  ====================================================
  
  WIRING:
  Motor 1 FL -> pins 4, 13, spd 32
  Motor 2 FR -> pins 16, 17, spd 33
  Motor 3 BL -> pins 18, 19, spd 25
  Motor 4 BR -> pins 21, 22, spd 26

  UI: Landscape dual-joystick
    Left stick  -> throttle (Y axis only, forward/back)
    Right stick -> steering (X axis only, left/right)
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

const char* ssid     = "NF-WIFI";
const char* password = "MasterKN34048)";

const int m1_pin1 = 4,  m1_pin2 = 13, m1_sp = 32; 
const int m2_pin1 = 16, m2_pin2 = 17, m2_sp = 33; 
const int m3_pin1 = 18, m3_pin2 = 19, m3_sp = 25; 
const int m4_pin1 = 21, m4_pin2 = 22, m4_sp = 26; 

WebServer httpServer(80);
WebSocketsServer wsServer(81);

unsigned long lastSignalTime = 0;
const unsigned long TIMEOUT_MS = 1000;

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>ROVER-1</title>
<style>
  :root { --bg: #080b10; --accent: #00e5ff; --panel: #0d1520; --border: #1a2535; }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg);
    color: #8aaccc;
    font-family: 'Courier New', monospace;
    height: 100vh;
    overflow: hidden;
    display: flex;
    flex-direction: column;
  }
  header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 8px 20px;
    border-bottom: 1px solid var(--border);
    flex-shrink: 0;
    font-size: 0.7rem;
    letter-spacing: 0.1em;
  }
  .brand { color: var(--accent); font-size: 0.85rem; letter-spacing: 0.2em; }
  .status-row { display: flex; align-items: center; gap: 8px; }
  .dot { width: 8px; height: 8px; border-radius: 50%; background: #ff3366; flex-shrink: 0; }
  .dot.on { background: #22ff88; box-shadow: 0 0 8px #22ff88; }

  .main {
    flex: 1;
    display: flex;
    align-items: center;
    justify-content: space-around;
    padding: 10px 30px;
    gap: 20px;
  }
  .stick-panel { display: flex; flex-direction: column; align-items: center; gap: 10px; }
  .stick-label { font-size: 0.65rem; letter-spacing: 0.2em; color: #445566; }
  .stick-outer {
    width: 200px; height: 200px;
    border-radius: 50%;
    background: var(--panel);
    border: 1.5px solid var(--border);
    position: relative;
    touch-action: none;
    cursor: crosshair;
  }
  .stick-outer::before {
    content: ''; position: absolute; inset: 20px;
    border-radius: 50%; border: 1px solid #111f30;
  }
  .crosshair-h {
    position: absolute; left: 15%; right: 15%; top: 50%;
    height: 1px; background: #111f30; transform: translateY(-50%);
    pointer-events: none;
  }
  .crosshair-v {
    position: absolute; top: 15%; bottom: 15%; left: 50%;
    width: 1px; background: #111f30; transform: translateX(-50%);
    pointer-events: none;
  }
  .knob {
    width: 70px; height: 70px; border-radius: 50%;
    background: radial-gradient(circle at 35% 35%, #1a3a4a, #050c14);
    border: 1.5px solid var(--accent);
    position: absolute; top: 50%; left: 50%;
    transform: translate(-50%, -50%);
    pointer-events: none;
    transition: box-shadow 0.1s;
  }
  .knob.active { box-shadow: 0 0 18px rgba(0,229,255,0.35); }

  .center-panel { display: flex; flex-direction: column; align-items: center; gap: 10px; flex-shrink: 0; }
  .data-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; }
  .data-box {
    background: var(--panel); border: 1px solid var(--border);
    border-radius: 4px; padding: 6px 10px; text-align: center; min-width: 60px;
  }
  .data-box .lbl { font-size: 0.55rem; letter-spacing: 0.1em; color: #445; display: block; }
  .data-box .val { font-size: 0.9rem; color: var(--accent); }
  .ip-display { font-size: 0.6rem; color: #2a3f55; letter-spacing: 0.05em; margin-top: 4px; }
  .ws-port { font-size: 0.55rem; color: #1a2a3a; margin-top: 2px; }

  .axis-labels { position: absolute; inset: 0; pointer-events: none; }
  .axis-labels span { position: absolute; font-size: 0.55rem; color: #1e3040; letter-spacing: 0.05em; }
  .lbl-fwd { top: 6px; left: 50%; transform: translateX(-50%); }
  .lbl-bwd { bottom: 6px; left: 50%; transform: translateX(-50%); }
  .lbl-left { left: 6px; top: 50%; transform: translateY(-50%); }
  .lbl-right { right: 6px; top: 50%; transform: translateY(-50%); }
</style>
</head>
<body>
<header>
  <div class="brand">ROVER-1</div>
  <div class="status-row">
    <div>SYSTEM: <span id="ws-status">OFFLINE</span></div>
    <div class="dot" id="ws-dot"></div>
  </div>
  <div>IP: <span id="ip-addr">---</span></div>
</header>

<div class="main">
  <div class="stick-panel">
    <div class="stick-label">THROTTLE</div>
    <div class="stick-outer" id="zone-l">
      <div class="crosshair-h"></div>
      <div class="crosshair-v"></div>
      <div class="axis-labels">
        <span class="lbl-fwd">FWD</span>
        <span class="lbl-bwd">BWD</span>
      </div>
      <div class="knob" id="knob-l"></div>
    </div>
  </div>

  <div class="center-panel">
    <div class="data-grid">
      <div class="data-box"><span class="lbl">THROTTLE</span><span class="val" id="val-y">0</span></div>
      <div class="data-box"><span class="lbl">STEERING</span><span class="val" id="val-x">0</span></div>
      <div class="data-box"><span class="lbl">LEFT SPD</span><span class="val" id="val-ls">0</span></div>
      <div class="data-box"><span class="lbl">RIGHT SPD</span><span class="val" id="val-rs">0</span></div>
    </div>
    <div class="ip-display" id="ip-display">WS: ---:81</div>
    <div class="ws-port">PORT 81</div>
  </div>

  <div class="stick-panel">
    <div class="stick-label">STEERING</div>
    <div class="stick-outer" id="zone-r">
      <div class="crosshair-h"></div>
      <div class="crosshair-v"></div>
      <div class="axis-labels">
        <span class="lbl-left">L</span>
        <span class="lbl-right">R</span>
      </div>
      <div class="knob" id="knob-r"></div>
    </div>
  </div>
</div>

<script>
const host = window.location.hostname;
document.getElementById('ip-addr').innerText = host;
document.getElementById('ip-display').innerText = 'WS: ' + host + ':81';

let ws, wsActive = false;
function initWS() {
  ws = new WebSocket('ws://' + host + ':81/');
  ws.onopen = () => {
    wsActive = true;
    document.getElementById('ws-status').innerText = 'LIVE';
    document.getElementById('ws-dot').classList.add('on');
  };
  ws.onclose = () => {
    wsActive = false;
    document.getElementById('ws-status').innerText = 'RECONNECTING';
    document.getElementById('ws-dot').classList.remove('on');
    setTimeout(initWS, 2000);
  };
}

let throttleY = 0, steerX = 0;

function sendCmd() {
  const x = Math.round(steerX);
  const y = Math.round(throttleY);
  document.getElementById('val-y').innerText = y;
  document.getElementById('val-x').innerText = x;
  const ls = Math.round(Math.max(-100, Math.min(100, y - x)));
  const rs = Math.round(Math.max(-100, Math.min(100, y + x)));
  document.getElementById('val-ls').innerText = ls;
  document.getElementById('val-rs').innerText = rs;
  if (wsActive) ws.send(x + ',' + y);
}

function makeStick(zoneId, knobId, axis) {
  const zone = document.getElementById(zoneId);
  const knob = document.getElementById(knobId);
  let active = false, touchId = null;

  function getOffset(clientX, clientY) {
    const r = zone.getBoundingClientRect();
    const cx = r.left + r.width / 2;
    const cy = r.top + r.height / 2;
    const limit = r.width / 2 - 38;
    let dx = clientX - cx;
    let dy = clientY - cy;
    const dist = Math.sqrt(dx*dx + dy*dy);
    if (dist > limit) { dx *= limit/dist; dy *= limit/dist; }
    return { dx, dy, limit };
  }

  function applyMove(clientX, clientY) {
    const { dx, dy, limit } = getOffset(clientX, clientY);
    if (axis === 'y') {
      knob.style.transform = 'translate(-50%, calc(-50% + ' + dy + 'px))';
      throttleY = Math.round((-dy / limit) * 100);
    } else {
      knob.style.transform = 'translate(calc(-50% + ' + dx + 'px), -50%)';
      steerX = Math.round((dx / limit) * 100);
    }
    knob.classList.add('active');
    sendCmd();
  }

  function release() {
    active = false; touchId = null;
    knob.style.transform = 'translate(-50%, -50%)';
    knob.classList.remove('active');
    if (axis === 'y') throttleY = 0;
    else steerX = 0;
    sendCmd();
  }

  zone.addEventListener('mousedown', e => { active = true; applyMove(e.clientX, e.clientY); });
  window.addEventListener('mousemove', e => { if (active) applyMove(e.clientX, e.clientY); });
  window.addEventListener('mouseup', () => { if (active) release(); });

  zone.addEventListener('touchstart', e => {
    e.preventDefault();
    touchId = e.changedTouches[0].identifier;
    active = true;
    applyMove(e.changedTouches[0].clientX, e.changedTouches[0].clientY);
  }, { passive: false });
  window.addEventListener('touchmove', e => {
    if (!active) return;
    for (const t of e.changedTouches) {
      if (t.identifier === touchId) { applyMove(t.clientX, t.clientY); break; }
    }
  }, { passive: false });
  window.addEventListener('touchend', e => {
    for (const t of e.changedTouches) {
      if (t.identifier === touchId) { release(); break; }
    }
  });
}

makeStick('zone-l', 'knob-l', 'y');
makeStick('zone-r', 'knob-r', 'x');

initWS();
</script>
</body>
</html>
)rawliteral";

// --- Motor Logic ---
void motorWrite(int spPin, int p1, int p2, int spd) {
  spd = constrain(spd, -255, 255);
  if (spd == 0) {
    digitalWrite(p1, LOW); digitalWrite(p2, LOW);
    ledcWrite(spPin, 0);
  } else if (spd > 0) {
    digitalWrite(p1, LOW); digitalWrite(p2, HIGH);
    ledcWrite(spPin, spd);
  } else {
    digitalWrite(p1, HIGH); digitalWrite(p2, LOW);
    ledcWrite(spPin, -spd);
  }
}

float applyDeadband(float v, float db = 0.08f) {
  if (fabs(v) < db) return 0.0f;
  if (v > 0) return (v - db) / (1.0f - db);
  return (v + db) / (1.0f - db);
}

void drive(int x, int y) {
  float steer = applyDeadband(x / 100.0f);     
  float throttle = applyDeadband(y / 100.0f);  

  float ls = 0.0f;
  float rs = 0.0f;

  // Pivot mode when nearly stopped
  const float pivotThreshold = 0.15f;

  if (fabs(throttle) < pivotThreshold) {
    // spin / pivot in place
    ls = -steer;
    rs =  steer;
  } else {
    // smooth car-like turning while moving
    float turn = steer * 0.7f;   // turning strength
    ls = throttle - turn;
    rs = throttle + turn;
  }

  ls = constrain(ls, -1.0f, 1.0f);
  rs = constrain(rs, -1.0f, 1.0f);

  motorWrite(m1_sp, m1_pin1, m1_pin2, (int)(ls * 255));
  motorWrite(m3_sp, m3_pin1, m3_pin2, (int)(ls * 255));
  motorWrite(m2_sp, m2_pin1, m2_pin2, (int)(rs * 255));
  motorWrite(m4_sp, m4_pin1, m4_pin2, (int)(rs * 255));
}

void onWebSocketEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    int comma = msg.indexOf(',');
    if (comma > 0) {
      int x = msg.substring(0, comma).toInt();
      int y = msg.substring(comma + 1).toInt();
      drive(x, y);
      lastSignalTime = millis(); 
    }
  } else if (type == WStype_DISCONNECTED) {
    drive(0, 0);
  }
}

void setup() {
  Serial.begin(115200);

  int pins[] = {m1_pin1, m1_pin2, m2_pin1, m2_pin2, m3_pin1, m3_pin2, m4_pin1, m4_pin2};
  for(int p : pins) pinMode(p, OUTPUT);

  ledcAttach(m1_sp, 1000, 8);
  ledcAttach(m2_sp, 1000, 8);
  ledcAttach(m3_sp, 1000, 8);
  ledcAttach(m4_sp, 1000, 8);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.print("\nConnected! IP: ");
  Serial.println(WiFi.localIP());

  httpServer.on("/", []() {
    httpServer.send_P(200, "text/html", INDEX_HTML);
  });
  httpServer.begin();

  wsServer.begin();
  wsServer.onEvent(onWebSocketEvent);
}

void loop() {
  httpServer.handleClient();
  wsServer.loop();

  if (millis() - lastSignalTime > TIMEOUT_MS) {
    drive(0, 0);
  }
}