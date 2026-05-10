#include <Arduino.h>
#include <NativeEthernet.h>
#define WEBSOCKETS_NETWORK_TYPE NETWORK_W5100
#include <WebSocketsServer.h>
#include <FlexCAN_T4.h>

#include <array>
#include <cstring>

struct ParamDef {
  uint8_t id;
  const char* name;
  const char* range;
};

static const ParamDef kParams[] = {
    {0, "TAG", "Non-Modified Content"},
    {1, "Motor Poles", "Even Number of 2-32"},
    {2, "Rated Speed", "80-3500"},
    {3, "Max Current", "10-500"},
    {4, "Encoder PPR", "1000-50000"},
    {5, "Current Kp", "0.001-2"},
    {6, "Current Ki", "0.001-2"},
    {7, "Speed Kp", "0.001-2"},
    {8, "Speed Ki", "0.001-2"},
    {9, "Position Kp", "0.001-2"},
    {10, "Position Ki", "0.001-2"},
    {11, "Position Kd", "0.001-2"},
    {12, "Position Kc", "0.001-2"},
    {13, "Acceleration Time", "1-200; 0.1s-20s"},
    {14, "Position", "Slow Down in Advance 20-100"},
    {15, "Magnetic", "0.001-0.999"},
    {18, "System", "CAN-ID (decimalism)"},
    {19, "Control Ways", "1-Analog 2-CAN 3-Serial Port 4-RC 5-CANOPEN"},
    {20, "Control Mode", "1-speed 2-Torque 3-Pos 4-Pos"},
    {21, "BPS", "CAN BPS 1-125K 2-250K 3-500K 4-1M"},
    {22, "Position", "1-Encoder 2-Hall 3-Magnetic"},
    {23, "Over Voltage", "Over Voltage Setting"},
    {24, "Less Voltage", "Less Voltage Setting"},
    {25, "Motor Temp", "Motor Temp Protection Setting"},
    {26, "Direction", "1-Motor 1 Reverse; 2-Motor 2"},
    {27, "Brake Time", "1-30; 0.1s-3s"},
    {28, "Over", "1-20; 1s-20s"},
    {29, "Hall status", "Hall reverse"},
    {30, "Deceleration time", "1-200; 0.1s-20s"},
    {31, "Spare", "Spare"},
    {32, "Spare", "Spare"},
};

constexpr uint32_t KEYA_CONFIG_ID = 0x06000591;
constexpr uint32_t KEYA_RESPONSE_ID = 0x181;
constexpr uint32_t KEYA_DRIVE_ID = 0x06000001;
constexpr uint32_t KEYA_HEARTBEAT_ID = 0x07000001;

const uint8_t CMD_ENTER_CONFIG[] = {0xFA, 0xFA, 0x00, 0x00};
const uint8_t CMD_STORE_EEPROM[] = {0xFA, 0xFA, 0x00, 0x08};
const uint8_t CMD_EXIT_CONFIG[] = {0xFA, 0xFA, 0x00, 0xAA};
const uint8_t CMD_TEST_DISABLE[] = {0x23, 0x0C, 0x20, 0x01};
const uint8_t CMD_TEST_ENABLE[] = {0x23, 0x0D, 0x20, 0x01, 0x01, 0x90, 0x00, 0x00};

IPAddress kIp(192, 168, 1, 126);
IPAddress kSubnet(255, 255, 255, 0);
IPAddress kGateway(0, 0, 0, 0);
IPAddress kDns(0, 0, 0, 0);

byte kMac[6] = {0x04, 0xE9, 0xE5, 0x41, 0x00, 0x42};

EthernetServer httpServer(80);
WebSocketsServer wsServer(81);
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

uint32_t currentCanBitrate = 250000;

std::array<bool, 256> dirty{};
std::array<uint8_t, 256> dirtyValue{};
std::array<uint8_t, 256> readKindMask{};

struct WriteCommand {
  uint8_t id;
  uint8_t value;
};

constexpr size_t WRITE_QUEUE_SIZE = 64;
std::array<WriteCommand, WRITE_QUEUE_SIZE> writeQueue{};
size_t writeHead = 0;
size_t writeTail = 0;
bool writeSequenceActive = false;
uint32_t nextWriteAt = 0;
enum class StoreSequenceStage : uint8_t {
  None,
  WriteChanged,
  SendStore,
  SendExit,
  Complete,
};
StoreSequenceStage storeStage = StoreSequenceStage::None;
size_t storeChangedCount = 0;
bool readPollActive = false;
uint32_t readPollStartedAt = 0;
uint32_t readPollLastRxAt = 0;
uint32_t readPollLastStatusAt = 0;
uint16_t readPollFrameCount = 0;
bool debugCanRxLogs = false;
bool inConfigMode = false;
bool heartbeatSeenRecently = false;
uint32_t lastHeartbeatAt = 0;

void updateReadPollFromResponse(uint8_t paramId, uint8_t readKind);
void serviceReadPoll();

const char INDEX_HTML[] = R"HTML(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>Keya Config</title>
  <style>
    :root {
      --bg: #f2f6f4;
      --panel: #ffffff;
      --line: #d3ddd8;
      --text: #17352b;
      --muted: #597569;
      --accent: #12805f;
      --warn: #a94724;
    }
    body {
      margin: 0;
      font-family: "Segoe UI", Tahoma, sans-serif;
      background: radial-gradient(circle at top left, #ecfff5, #f2f6f4 40%), var(--bg);
      color: var(--text);
      overflow-x: hidden;
    }
    .wrap { padding: 14px; max-width: 1200px; margin: 0 auto; }
    .bar { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; margin-bottom: 10px; }
    .tabs { display: flex; gap: 8px; margin-bottom: 12px; }
    .tab {
      border: 1px solid var(--line);
      background: var(--panel);
      color: var(--muted);
      border-radius: 10px;
      padding: 8px 14px;
      cursor: pointer;
      font-weight: 700;
    }
    .tab.active { color: #fff; background: var(--accent); border-color: var(--accent); }
    .panel {
      border: 1px solid var(--line);
      border-radius: 12px;
      background: var(--panel);
      box-shadow: 0 6px 20px rgba(12, 46, 35, .06);
      padding: 12px;
      overflow: hidden;
    }
    .hidden { display: none; }
    .table-wrap {
      overflow: auto;
      max-height: 70vh;
      -webkit-overflow-scrolling: touch;
    }
    table { width: 100%; min-width: 760px; border-collapse: collapse; font-size: 13px; }
    th, td { border-bottom: 1px solid #ecf1ee; padding: 6px; text-align: left; }
    th { color: var(--muted); font-weight: 700; }
    input, select, button {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 7px 10px;
      background: #fff;
      color: var(--text);
      font-size: 16px;
      min-height: 40px;
      touch-action: manipulation;
    }
    input.small { width: 88px; }
    button { cursor: pointer; font-weight: 700; }
    button:disabled { cursor: not-allowed; opacity: .45; }
    button.primary { background: var(--accent); color: #fff; border-color: var(--accent); }
    button.warn { background: var(--warn); color: #fff; border-color: var(--warn); }
    .status { font-size: 12px; color: var(--muted); min-height: 16px; }
    .mode-pill {
      display: inline-block;
      font-size: 12px;
      font-weight: 700;
      padding: 4px 10px;
      border-radius: 999px;
      border: 1px solid var(--line);
      background: #f3faf7;
      color: var(--muted);
    }
    .mode-pill.in {
      background: #e4f8ef;
      border-color: #8fd5b8;
      color: #0e6e4f;
    }
    .mode-pill.out {
      background: #f7f9f8;
      border-color: #cdd9d3;
      color: #4f6d61;
    }
    .hb-btn {
      border-radius: 999px;
      border: 1px solid transparent;
      color: #fff;
      font-weight: 700;
      min-height: 32px;
      padding: 4px 12px;
    }
    .hb-btn:disabled { opacity: 1; cursor: default; }
    .hb-btn.bad { background: #b93a3a; border-color: #9d2d2d; }
    .hb-btn.ok { background: #178a56; border-color: #136f45; }
    .grid2 { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .log {
      font-family: Consolas, monospace;
      background: #f5fbf8;
      border: 1px solid var(--line);
      border-radius: 8px;
      height: 220px;
      overflow: auto;
      padding: 8px;
      white-space: pre-wrap;
      font-size: 12px;
    }
    #manPayload { min-width: 320px; }
    @media (max-width: 900px) {
      .grid2 { grid-template-columns: 1fr; }
      table { font-size: 12px; }
    }
    @media (max-width: 820px) {
      .wrap { padding: 8px; }
      .tabs { display: grid; grid-template-columns: 1fr 1fr; }
      .tab { width: 100%; }
      .bar > * { flex: 1 1 100%; min-width: 0; }
      .mode-pill { flex: 1 1 100%; text-align: center; }
      .bar label { font-size: 12px; }
      #manId, #manPayload, #manExt, #canSpeed { width: 100%; min-width: 0; }
      .table-wrap { max-height: none; overflow: visible; }
      .param-table { min-width: 0; border-collapse: separate; border-spacing: 0 10px; }
      .param-table thead { display: none; }
      .param-table tbody, .param-table tr, .param-table td { display: block; width: 100%; }
      .param-table tr {
        border: 1px solid var(--line);
        border-radius: 12px;
        background: #fff;
        box-shadow: 0 4px 14px rgba(12, 46, 35, .05);
        padding: 8px;
      }
      .param-table td {
        border-bottom: 1px dashed #e3ece7;
        padding: 6px 0;
        word-break: break-word;
      }
      .param-table td:last-child { border-bottom: 0; }
      .param-table td::before {
        content: attr(data-label);
        display: block;
        font-size: 11px;
        font-weight: 700;
        color: var(--muted);
        margin-bottom: 3px;
        text-transform: uppercase;
        letter-spacing: .04em;
      }
      .param-table input,
      .param-table button { width: 100%; }
      .status { word-break: break-word; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="tabs">
      <button class="tab active" id="tabSetup">Setup</button>
      <button class="tab" id="tabTest">Test</button>
    </div>

    <div class="panel" id="panelSetup">
      <div class="bar">
        <span class="mode-pill out" id="cfgMode">Config Mode: OUT</span>
        <button id="hbState" class="hb-btn bad" type="button" disabled>Heartbeat</button>
        <button id="btnEnter">Enter Config / Poll</button>
        <button id="btnExit">Exit Config</button>
        <button class="warn" id="btnStore">Store EEPROM</button>
        <label>CAN bitrate:</label>
        <select id="canSpeed">
          <option value="125000">125k</option>
          <option value="250000" selected>250k</option>
          <option value="500000">500k</option>
          <option value="1000000">1M</option>
        </select>
      </div>
      <div class="status" id="status"></div>
      <div class="table-wrap">
        <table class="param-table">
          <thead>
            <tr>
              <th>Name</th>
              <th>ID</th>
              <th>Param</th>
              <th>RAM</th>
              <th>ROM</th>
              <th>New</th>
              <th>Write</th>
              <th>Range</th>
            </tr>
          </thead>
          <tbody id="rows"></tbody>
        </table>
      </div>
    </div>

    <div class="panel hidden" id="panelTest">
      <div class="grid2">
        <div>
          <h3>Wheel Control</h3>
          <div class="bar">
            <label>Speed</label>
            <input id="wheelSpeed" type="number" min="0" max="1000" step="10" value="500" />
            <button class="primary" id="btnWheelLeft">Left</button>
            <button class="primary" id="btnWheelRight">Right</button>
            <button class="warn" id="btnWheelDisable">Disable</button>
          </div>
          <p style="font-size:12px;color:#597569">Command payload mapping to be finalized; buttons are wired and ready.</p>

          <h3>Manual Command</h3>
          <div class="bar">
            <label>CAN ID (hex)</label>
            <input id="manId" value="06000591" />
            <label>Extended</label>
            <select id="manExt"><option value="1" selected>Yes</option><option value="0">No</option></select>
          </div>
          <div class="bar">
            <label>Payload bytes (hex, space separated)</label>
            <input id="manPayload" value="" />
            <button class="primary" id="btnManualSend">Send</button>
          </div>
          <p style="font-size:12px;color:#597569">Testing page only. No automatic safety or deadman behavior is enforced.</p>
        </div>
        <div>
          <h3>Traffic Log</h3>
          <div class="log" id="log"></div>
        </div>
      </div>
    </div>
  </div>

<script>
const params = [
  {id:0,name:'TAG',range:'Non-Modified Content'},
  {id:1,name:'Motor Poles',range:'Even Number of 2-32'},
  {id:2,name:'Rated Speed',range:'80-3500'},
  {id:3,name:'Max Current',range:'10-500'},
  {id:4,name:'Encoder PPR',range:'1000-50000'},
  {id:5,name:'Current Kp',range:'0.001-2'},
  {id:6,name:'Current Ki',range:'0.001-2'},
  {id:7,name:'Speed Kp',range:'0.001-2'},
  {id:8,name:'Speed Ki',range:'0.001-2'},
  {id:9,name:'Position Kp',range:'0.001-2'},
  {id:10,name:'Position Ki',range:'0.001-2'},
  {id:11,name:'Position Kd',range:'0.001-2'},
  {id:12,name:'Position Kc',range:'0.001-2'},
  {id:13,name:'Acceleration Time',range:'1-200; 0.1s-20s'},
  {id:14,name:'Position',range:'Slow Down in Advance 20-100'},
  {id:15,name:'Magnetic',range:'0.001-0.999'},
  {id:18,name:'System',range:'CAN-ID (decimalism)'},
  {id:19,name:'Control Ways',range:'1-Analog 2-CAN 3-Serial Port 4-RC 5-CANOPEN'},
  {id:20,name:'Control Mode',range:'1-speed 2-Torque 3-Pos 4-Pos'},
  {id:21,name:'BPS',range:'CAN BPS 1-125K 2-250K 3-500K 4-1M'},
  {id:22,name:'Position',range:'1-Encoder 2-Hall 3-Magnetic'},
  {id:23,name:'Over Voltage',range:'Over Voltage Setting'},
  {id:24,name:'Less Voltage',range:'Less Voltage Setting'},
  {id:25,name:'Motor Temp',range:'Motor Temp Protection Setting'},
  {id:26,name:'Direction',range:'1-Motor 1 Reverse; 2-Motor 2'},
  {id:27,name:'Brake Time',range:'1-30; 0.1s-3s'},
  {id:28,name:'Over',range:'1-20; 1s-20s'},
  {id:29,name:'Hall status',range:'Hall reverse'},
  {id:30,name:'Deceleration time',range:'1-200; 0.1s-20s'},
  {id:31,name:'Spare',range:'Spare'},
  {id:32,name:'Spare',range:'Spare'}
];

const rows = document.getElementById('rows');
const statusEl = document.getElementById('status');
const logEl = document.getElementById('log');
const cfgModeEl = document.getElementById('cfgMode');
const hbStateEl = document.getElementById('hbState');
let isInConfigMode = false;

function addLog(line) {
  const t = new Date().toLocaleTimeString();
  logEl.textContent += `[${t}] ${line}\n`;
  logEl.scrollTop = logEl.scrollHeight;
}

function rowHtml(p) {
  return `<tr id="r${p.id}">
    <td data-label="Name">${p.name}</td>
    <td data-label="ID">${String(p.id).padStart(4,'0')}</td>
    <td data-label="Param" id="p${p.id}_0"></td>
    <td data-label="RAM" id="p${p.id}_1"></td>
    <td data-label="ROM" id="p${p.id}_2"></td>
    <td data-label="New"><input class="small" id="n${p.id}"/></td>
    <td data-label="Write"><button class="write-now-btn" onclick="queueWrite(${p.id})" disabled>Write Now</button></td>
    <td data-label="Range">${p.range}</td>
  </tr>`;
}

rows.innerHTML = params.map(rowHtml).join('');

function setStatus(txt) { statusEl.textContent = txt; }

function applyWriteButtonsState() {
  document.querySelectorAll('.write-now-btn').forEach((btn) => {
    btn.disabled = !isInConfigMode;
    btn.title = isInConfigMode ? '' : 'Enter config mode to enable writes';
  });
}

function setMode(mode) {
  const v = String(mode || '').trim().toUpperCase();
  const inMode = (v === 'IN' || v === 'CONFIG' || v === '1' || v === 'TRUE');
  isInConfigMode = inMode;
  cfgModeEl.textContent = `Mode: ${inMode ? 'Config' : 'NoConfig'}`;
  cfgModeEl.classList.toggle('in', inMode);
  cfgModeEl.classList.toggle('out', !inMode);
  applyWriteButtonsState();
}

function setHeartbeat(isAlive, ageMs) {
  const age = Number.isFinite(ageMs) && ageMs >= 0 ? Math.round(ageMs) : -1;
  hbStateEl.classList.toggle('ok', !!isAlive);
  hbStateEl.classList.toggle('bad', !isAlive);
  if (age >= 0) {
    hbStateEl.textContent = isAlive ? `Heartbeat: OK (${age} ms)` : `Heartbeat: LOST (${age} ms)`;
  } else {
    hbStateEl.textContent = isAlive ? 'Heartbeat: OK' : 'Heartbeat: LOST';
  }
  hbStateEl.title = age >= 0 ? `Last seen ${age} ms ago` : 'No heartbeat seen yet';
}

setMode('OUT');
setHeartbeat(false, -1);
applyWriteButtonsState();

function showTab(which) {
  const setup = document.getElementById('panelSetup');
  const test = document.getElementById('panelTest');
  const tabSetup = document.getElementById('tabSetup');
  const tabTest = document.getElementById('tabTest');
  const onSetup = which === 'setup';
  setup.classList.toggle('hidden', !onSetup);
  test.classList.toggle('hidden', onSetup);
  tabSetup.classList.toggle('active', onSetup);
  tabTest.classList.toggle('active', !onSetup);
}

document.getElementById('tabSetup').onclick = () => showTab('setup');
document.getElementById('tabTest').onclick = () => showTab('test');

const ws = new WebSocket(`ws://${location.hostname}:81`);
ws.onopen = () => { setStatus('Connected'); addLog('WebSocket connected'); applyWriteButtonsState(); };
ws.onclose = () => { setStatus('Disconnected'); addLog('WebSocket disconnected'); setMode('OUT'); setHeartbeat(false, -1); applyWriteButtonsState(); };
ws.onerror = () => setStatus('Socket error');

ws.onmessage = (ev) => {
  const m = String(ev.data || '');
  if (m.startsWith('PARAM:')) {
    const s = m.split(':');
    const id = Number(s[1]);
    const kind = Number(s[2]);
    const value = s[3] || '';
    const el = document.getElementById(`p${id}_${kind}`);
    if (el) el.textContent = value;
    return;
  }
  if (m.startsWith('STATUS:')) {
    const txt = m.slice(7);
    setStatus(txt);
    addLog(txt);
    return;
  }
  if (m.startsWith('MODE:')) {
    setMode(m.slice(5).trim());
    return;
  }
  if (m.startsWith('HEARTBEAT:')) {
    const payload = m.slice(10).trim();
    const parts = payload.split(':');
    const statePart = (parts[0] || '').trim();
    const agePart = (parts[1] || '').trim();
    const alive = statePart === '1' || statePart.toUpperCase() === 'OK' || statePart.toUpperCase() === 'TRUE';
    const ageMs = Number(agePart);
    setHeartbeat(alive, Number.isFinite(ageMs) ? ageMs : -1);
    return;
  }
  if (m.startsWith('RX:')) {
    addLog(m);
    return;
  }
  if (m.startsWith('TX:')) {
    addLog(m);
    return;
  }
  addLog(m);
};

function send(cmd) {
  if (ws.readyState === WebSocket.OPEN) ws.send(cmd);
}

function queueWrite(id) {
  if (!isInConfigMode) {
    setStatus('Write disabled: enter config mode first.');
    return;
  }

  const input = document.getElementById(`n${id}`);
  const raw = String(input.value || '').trim();
  if (!raw.length) {
    setStatus(`Set New value for ${id} before Write Now.`);
    return;
  }

  const value = Number(raw);
  if (!Number.isFinite(value) || value < 0 || value > 255) {
    setStatus(`Invalid value for ${id}. Use 0..255.`);
    return;
  }
  send(`WRITE_RAM:${id}:${value}`);
}

document.getElementById('btnEnter').onclick = () => send('ENTER_CONFIG');
document.getElementById('btnExit').onclick = () => send('EXIT_CONFIG');
document.getElementById('btnStore').onclick = () => send('STORE_EEPROM');
document.getElementById('btnWheelLeft').onclick = () => {
  const speed = String(document.getElementById('wheelSpeed').value || '').trim();
  const speedVal = Number(speed);
  if (!speed.length || !Number.isInteger(speedVal) || speedVal < 0 || speedVal > 1000) {
    setStatus('Speed must be an integer in range 0..1000.');
    return;
  }
  send(`TEST_LEFT:${speedVal}`);
};
document.getElementById('btnWheelRight').onclick = () => {
  const speed = String(document.getElementById('wheelSpeed').value || '').trim();
  const speedVal = Number(speed);
  if (!speed.length || !Number.isInteger(speedVal) || speedVal < 0 || speedVal > 1000) {
    setStatus('Speed must be an integer in range 0..1000.');
    return;
  }
  send(`TEST_RIGHT:${speedVal}`);
};
document.getElementById('btnWheelDisable').onclick = () => send('TEST_DISABLE');
document.getElementById('btnManualSend').onclick = () => {
  const id = document.getElementById('manId').value.trim();
  const payload = document.getElementById('manPayload').value.trim();
  const ext = document.getElementById('manExt').value;
  send(`MANUAL:${id}:${ext}:${payload}`);
};
document.getElementById('canSpeed').onchange = (e) => send(`CAN_BPS:${e.target.value}`);
</script>
</body>
</html>
)HTML";

bool queueFull() {
  return ((writeTail + 1) % WRITE_QUEUE_SIZE) == writeHead;
}

bool queueEmpty() {
  return writeHead == writeTail;
}

bool enqueueWrite(uint8_t id, uint8_t value) {
  if (queueFull()) {
    return false;
  }
  writeQueue[writeTail] = {id, value};
  writeTail = (writeTail + 1) % WRITE_QUEUE_SIZE;
  return true;
}

bool dequeueWrite(WriteCommand& cmd) {
  if (queueEmpty()) {
    return false;
  }
  cmd = writeQueue[writeHead];
  writeHead = (writeHead + 1) % WRITE_QUEUE_SIZE;
  return true;
}

void wsStatus(const String& s) {
  wsServer.broadcastTXT("STATUS:" + s);
}

void wsConfigMode() {
  wsServer.broadcastTXT(String("MODE:") + (inConfigMode ? "IN" : "OUT"));
}

void wsHeartbeatState() {
  long ageMs = heartbeatSeenRecently ? static_cast<long>(millis() - lastHeartbeatAt) : -1;
  wsServer.broadcastTXT(String("HEARTBEAT:") + (heartbeatSeenRecently ? "1" : "0") + ":" + String(ageMs));
}

bool shouldLogCanId(uint32_t id) {
  if (id == 0x70000001) {
    return false;
  }
  return id == KEYA_RESPONSE_ID || id == KEYA_CONFIG_ID || id == KEYA_DRIVE_ID;
}

void sendCanFrame(uint32_t id, bool extended, const uint8_t* data, uint8_t len) {
  CAN_message_t msg;
  msg.id = id;
  msg.flags.extended = extended;
  msg.len = len;
  memset(msg.buf, 0, sizeof(msg.buf));
  memcpy(msg.buf, data, len);
  bool writeOk = can3.write(msg);

  if (shouldLogCanId(msg.id)) {
    String tx = "TX:";
    tx += String(msg.id, HEX);
    tx += " ok=";
    tx += String(writeOk ? 1 : 0);
    tx += " ext=";
    tx += String(msg.flags.extended ? 1 : 0);
    tx += " len=";
    tx += String(msg.len);
    tx += " data=";
    for (uint8_t i = 0; i < msg.len; ++i) {
      if (msg.buf[i] < 16) {
        tx += "0";
      }
      tx += String(msg.buf[i], HEX);
      if (i + 1 < msg.len) {
        tx += " ";
      }
    }
    wsServer.broadcastTXT(tx);
  }
}

void sendConfigFrame(const uint8_t* data, uint8_t len) {
  sendCanFrame(KEYA_CONFIG_ID, true, data, len);
}

void applyCanBitrate(uint32_t bitrate) {
  currentCanBitrate = bitrate;
  can3.setBaudRate(currentCanBitrate);
  wsStatus("CAN bitrate set to " + String(currentCanBitrate));
}

void processWriteSequence() {
  if (!writeSequenceActive) {
    return;
  }
  if (millis() < nextWriteAt) {
    return;
  }

  if (storeStage == StoreSequenceStage::WriteChanged) {
    WriteCommand cmd;
    if (dequeueWrite(cmd)) {
      uint8_t payload[8] = {0xBB, 0xBB, 0x00, 0x00, 0x00, cmd.id, 0x00, cmd.value};
      sendConfigFrame(payload, sizeof(payload));
      wsStatus("EEPROM prep write ID " + String(cmd.id) + " = " + String(cmd.value));
      nextWriteAt = millis() + 1000;
      return;
    }

    storeStage = StoreSequenceStage::SendStore;
  }

  if (storeStage == StoreSequenceStage::SendStore) {
    sendConfigFrame(CMD_STORE_EEPROM, sizeof(CMD_STORE_EEPROM));
    wsStatus("Store EEPROM command sent for " + String(storeChangedCount) + " changed params");
    storeStage = StoreSequenceStage::SendExit;
    nextWriteAt = millis() + 1000;
    return;
  }

  if (storeStage == StoreSequenceStage::SendExit) {
    if (inConfigMode) {
      sendConfigFrame(CMD_EXIT_CONFIG, sizeof(CMD_EXIT_CONFIG));
      inConfigMode = false;
      wsConfigMode();
    }
    storeStage = StoreSequenceStage::Complete;
    nextWriteAt = millis() + 250;
    return;
  }

  if (storeStage == StoreSequenceStage::Complete) {
    for (size_t i = 0; i < dirty.size(); ++i) {
      dirty[i] = false;
    }
    storeChangedCount = 0;
    storeStage = StoreSequenceStage::None;
    writeSequenceActive = false;
    wsStatus("EEPROM store complete");
  }
}

void dumpCanFrameToSerial(const CAN_message_t& msg) {
  if (!debugCanRxLogs) {
    return;
  }

  if (!shouldLogCanId(msg.id)) {
    return;
  }

  Serial.print("CAN RAW id=0x");
  Serial.print(msg.id, HEX);
  Serial.print(" ext=");
  Serial.print(msg.flags.extended ? 1 : 0);
  Serial.print(" len=");
  Serial.print(msg.len);
  Serial.print(" data=");

  for (uint8_t i = 0; i < msg.len; ++i) {
    if (msg.buf[i] < 16) {
      Serial.print('0');
    }
    Serial.print(msg.buf[i], HEX);
    if (i + 1 < msg.len) {
      Serial.print(' ');
    }
  }
  Serial.println();
}

void parseCanResponse(const CAN_message_t& msg) {
  if (msg.id != KEYA_RESPONSE_ID || msg.len < 8) {
    return;
  }
  if (msg.buf[0] != 0xAA || msg.buf[1] != 0xAA) {
    return;
  }

  uint8_t paramId = msg.buf[3];
  uint8_t readKind = msg.buf[5];
  uint8_t value = msg.buf[7];

  if (debugCanRxLogs) {
    Serial.print("READ RX id=");
    Serial.print(paramId);
    Serial.print(" kind=");
    Serial.print(readKind);
    Serial.print(" val=");
    Serial.println(value);
  }

  updateReadPollFromResponse(paramId, readKind);

  String line = "PARAM:" + String(paramId) + ":" + String(readKind) + ":" + String(value);
  wsServer.broadcastTXT(line);

  String rx = "RX:181 ";
  for (uint8_t i = 0; i < msg.len; ++i) {
    if (msg.buf[i] < 16) {
      rx += "0";
    }
    rx += String(msg.buf[i], HEX);
    if (i + 1 < msg.len) {
      rx += " ";
    }
  }
  wsServer.broadcastTXT(rx);
}

void processCanRx() {
  CAN_message_t msg;
  while (can3.read(msg)) {
    if (msg.id == KEYA_HEARTBEAT_ID) {
      lastHeartbeatAt = millis();
      if (!heartbeatSeenRecently) {
        heartbeatSeenRecently = true;
        wsHeartbeatState();
      }
    }

    dumpCanFrameToSerial(msg);
    parseCanResponse(msg);
  }
}

void serviceHeartbeatState() {
  if (!heartbeatSeenRecently) {
    return;
  }

  if (millis() - lastHeartbeatAt > 1000) {
    heartbeatSeenRecently = false;
    wsHeartbeatState();
  }
}

size_t paramCountWithReadValue() {
  size_t count = 0;
  for (size_t i = 0; i < readKindMask.size(); ++i) {
    if (readKindMask[i] & (1u << 2)) {
      ++count;
    }
  }
  return count;
}

void finishReadPoll(const String& reason) {
  readPollActive = false;
  if (inConfigMode) {
    sendConfigFrame(CMD_EXIT_CONFIG, sizeof(CMD_EXIT_CONFIG));
    inConfigMode = false;
    wsConfigMode();
  }
  wsStatus("Read poll complete: " + reason + " (frames=" + String(readPollFrameCount) + ", read-values=" + String(paramCountWithReadValue()) + ")");
}

void updateReadPollFromResponse(uint8_t paramId, uint8_t readKind) {
  if (!readPollActive) {
    return;
  }

  if (readKind <= 2) {
    readKindMask[paramId] |= static_cast<uint8_t>(1u << readKind);
  }

  readPollFrameCount++;
  readPollLastRxAt = millis();

  if (millis() - readPollLastStatusAt >= 1000) {
    readPollLastStatusAt = millis();
    wsStatus("Read poll running: frames=" + String(readPollFrameCount) + ", read-values=" + String(paramCountWithReadValue()));
  }
}

void serviceReadPoll() {
  if (!readPollActive) {
    return;
  }

  uint32_t now = millis();
  if (now - readPollStartedAt > 15000) {
    finishReadPoll("timeout");
    return;
  }

  if (readPollFrameCount > 0 && now - readPollLastRxAt > 1500) {
    finishReadPoll("idle");
  }
}

void parseManualCommand(const String& payload) {
  String tmp = payload;
  tmp.trim();
  if (tmp.length() == 0) {
    wsStatus("Empty manual command");
    return;
  }

  int p1 = tmp.indexOf(':');
  int p2 = tmp.indexOf(':', p1 + 1);
  if (p1 < 0 || p2 < 0) {
    wsStatus("Manual format: MANUAL:<id_hex>:<0|1>:<hex bytes>");
    return;
  }

  String idHex = tmp.substring(0, p1);
  String extStr = tmp.substring(p1 + 1, p2);
  String bytesStr = tmp.substring(p2 + 1);

  uint32_t id = strtoul(idHex.c_str(), nullptr, 16);
  bool ext = extStr == "1";

  uint8_t data[8] = {0};
  uint8_t len = 0;

  char copy[96];
  bytesStr.toCharArray(copy, sizeof(copy));
  char* tok = strtok(copy, " ");
  while (tok && len < 8) {
    data[len++] = static_cast<uint8_t>(strtoul(tok, nullptr, 16));
    tok = strtok(nullptr, " ");
  }

  if (len == 0) {
    wsStatus("Manual payload empty");
    return;
  }

  sendCanFrame(id, ext, data, len);
  wsStatus("Manual frame sent");
}

bool parseUint16Strict(const String& s, uint16_t& outValue) {
  String t = s;
  t.trim();
  if (t.length() == 0) {
    return false;
  }

  for (size_t i = 0; i < static_cast<size_t>(t.length()); ++i) {
    char c = t.charAt(i);
    if (c < '0' || c > '9') {
      return false;
    }
  }

  unsigned long v = strtoul(t.c_str(), nullptr, 10);
  if (v > 65535UL) {
    return false;
  }

  outValue = static_cast<uint16_t>(v);
  return true;
}

void sendTestDisable() {
  sendCanFrame(KEYA_DRIVE_ID, true, CMD_TEST_DISABLE, sizeof(CMD_TEST_DISABLE));
}

void sendTestSteer(int16_t steerSpeed) {
  uint8_t cmd[8] = {0x23, 0x00, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
  uint16_t encodedSpeed = static_cast<uint16_t>(steerSpeed);
  cmd[4] = highByte(encodedSpeed);
  cmd[5] = lowByte(encodedSpeed);

  // Match reference control logic: left uses 0xFFFF, right uses 0x0000.
  bool isLeft = steerSpeed < 0;
  cmd[6] = isLeft ? 0xFF : 0x00;
  cmd[7] = isLeft ? 0xFF : 0x00;
    
  sendCanFrame(KEYA_DRIVE_ID, true, cmd, sizeof(cmd));
  sendCanFrame(KEYA_DRIVE_ID, true, CMD_TEST_ENABLE, sizeof(CMD_TEST_ENABLE));
}

void beginReadPoll() {
  for (size_t i = 0; i < readKindMask.size(); ++i) {
    readKindMask[i] = 0;
  }

  readPollActive = true;
  readPollStartedAt = millis();
  readPollLastRxAt = readPollStartedAt;
  readPollLastStatusAt = 0;
  readPollFrameCount = 0;

  if (!inConfigMode) {
    sendConfigFrame(CMD_ENTER_CONFIG, sizeof(CMD_ENTER_CONFIG));
    inConfigMode = true;
    wsConfigMode();
    wsStatus("Read poll started (entered config mode)");
  } else {
    wsStatus("Read poll started (already in config mode)");
  }
}

void queueChangedForRam() {
  size_t changed = 0;

  sendConfigFrame(CMD_ENTER_CONFIG, sizeof(CMD_ENTER_CONFIG));

  for (size_t i = 0; i < dirty.size(); ++i) {
    if (!dirty[i]) {
      continue;
    }
    if (enqueueWrite(static_cast<uint8_t>(i), dirtyValue[i])) {
      dirty[i] = false;
      ++changed;
    }
  }

  if (changed == 0) {
    wsStatus("No changed params to write");
    return;
  }

  writeSequenceActive = true;
  nextWriteAt = millis() + 1000;
  wsStatus("Queued " + String(changed) + " changed params for RAM write");
}

void handleCommand(const String& text) {
  if (text == "READ_POLL") {
    beginReadPoll();
    return;
  }
  if (text == "ENTER_CONFIG") {
    if (inConfigMode) {
      wsStatus("Already in config mode");
      wsConfigMode();
      return;
    }
    sendConfigFrame(CMD_ENTER_CONFIG, sizeof(CMD_ENTER_CONFIG));
    inConfigMode = true;
    wsConfigMode();
    wsStatus("Entered config mode");
    return;
  }
  if (text == "EXIT_CONFIG") {
    if (!inConfigMode) {
      wsStatus("Already out of config mode");
      wsConfigMode();
      return;
    }
    sendConfigFrame(CMD_EXIT_CONFIG, sizeof(CMD_EXIT_CONFIG));
    inConfigMode = false;
    wsConfigMode();
    wsStatus("Exited config mode");
    return;
  }
  if (text == "STORE_EEPROM") {
    if (writeSequenceActive) {
      wsStatus("Store already in progress");
      return;
    }

    writeHead = 0;
    writeTail = 0;
    storeChangedCount = 0;

    for (size_t i = 0; i < dirty.size(); ++i) {
      if (!dirty[i]) {
        continue;
      }
      if (enqueueWrite(static_cast<uint8_t>(i), dirtyValue[i])) {
        ++storeChangedCount;
      }
    }

    if (storeChangedCount == 0) {
      wsStatus("No changed params to store");
      return;
    }

    if (!inConfigMode) {
      sendConfigFrame(CMD_ENTER_CONFIG, sizeof(CMD_ENTER_CONFIG));
      inConfigMode = true;
      wsConfigMode();
    }

    storeStage = StoreSequenceStage::WriteChanged;
    writeSequenceActive = true;
    nextWriteAt = millis() + 1000;
    wsStatus("Storing " + String(storeChangedCount) + " changed params to EEPROM");
    return;
  }

  if (text.startsWith("WRITE_RAM:")) {
    int p1 = text.indexOf(':', 10);
    if (p1 < 0) {
      wsStatus("WRITE_RAM format: WRITE_RAM:<id>:<value>");
      return;
    }
    uint8_t id = static_cast<uint8_t>(text.substring(10, p1).toInt());
    int valueInt = text.substring(p1 + 1).toInt();
    if (valueInt < 0 || valueInt > 255) {
      wsStatus("WRITE_RAM value must be 0..255");
      return;
    }

    uint8_t payload[8] = {0xBB, 0xBB, 0x00, 0x00, 0x00, id, 0x00, static_cast<uint8_t>(valueInt)};
    sendConfigFrame(payload, sizeof(payload));
    dirty[id] = true;
    dirtyValue[id] = static_cast<uint8_t>(valueInt);
    wsStatus("Wrote RAM ID " + String(id) + " = " + String(valueInt));
    return;
  }

  if (text.startsWith("CAN_BPS:")) {
    uint32_t bps = static_cast<uint32_t>(text.substring(8).toInt());
    if (bps == 125000 || bps == 250000 || bps == 500000 || bps == 1000000) {
      applyCanBitrate(bps);
    } else {
      wsStatus("Unsupported CAN bitrate");
    }
    return;
  }

  if (text.startsWith("MANUAL:")) {
    parseManualCommand(text.substring(7));
    return;
  }

  if (text.startsWith("TEST_LEFT:")) {
    uint16_t speed = 0;
    if (!parseUint16Strict(text.substring(10), speed)) {
      wsStatus("TEST_LEFT speed must be 0..1000");
      return;
    }
    if (speed > 1000) {
      wsStatus("TEST_LEFT speed must be 0..1000");
      return;
    }
    int16_t steerSpeed = static_cast<int16_t>(-static_cast<int32_t>(speed));
    sendTestSteer(steerSpeed);
    wsStatus("Wheel LEFT sent (speed=" + String(steerSpeed) + ")");
    return;
  }

  if (text.startsWith("TEST_RIGHT:")) {
    uint16_t speed = 0;
    if (!parseUint16Strict(text.substring(11), speed)) {
      wsStatus("TEST_RIGHT speed must be 0..1000");
      return;
    }
    if (speed > 1000) {
      wsStatus("TEST_RIGHT speed must be 0..1000");
      return;
    }
    int16_t steerSpeed = static_cast<int16_t>(speed);
    sendTestSteer(steerSpeed);
    wsStatus("Wheel RIGHT sent (speed=" + String(steerSpeed) + ")");
    return;
  }

  if (text == "TEST_DISABLE") {
    sendTestDisable();
    wsStatus("Wheel DISABLE sent");
    return;
  }

  wsStatus("Unknown command");
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    wsStatus("Client connected #" + String(num));
    wsServer.sendTXT(num, String("MODE:") + (inConfigMode ? "IN" : "OUT"));
    long ageMs = heartbeatSeenRecently ? static_cast<long>(millis() - lastHeartbeatAt) : -1;
    wsServer.sendTXT(num, String("HEARTBEAT:") + (heartbeatSeenRecently ? "1" : "0") + ":" + String(ageMs));
    return;
  }

  if (type == WStype_TEXT) {
    String text;
    text.reserve(length);
    for (size_t i = 0; i < length; ++i) {
      text += static_cast<char>(payload[i]);
    }
    handleCommand(text);
  }
}

bool writeAll(EthernetClient& client, const uint8_t* data, size_t len, uint32_t timeoutMs, size_t& sentOut) {
  size_t sent = 0;
  uint32_t start = millis();

  while (sent < len && client.connected()) {
    if (millis() - start > timeoutMs) {
      break;
    }

    size_t remaining = len - sent;
    size_t chunk = remaining > 256 ? 256 : remaining;
    size_t n = client.write(data + sent, chunk);
    if (n > 0) {
      sent += n;
      start = millis();
      continue;
    }

    delay(1);
  }

  sentOut = sent;
  return sent == len;
}

void handleHttpClient() {
  EthernetClient client = httpServer.accept();
  if (!client) {
    return;
  }

  String requestLine;
  uint32_t start = millis();
  bool gotFirstLine = false;
  bool gotHeaderEnd = false;
  uint8_t newlineCount = 0;

  while (client.connected() && millis() - start < 1500) {
    while (client.available()) {
      char c = static_cast<char>(client.read());

      if (!gotFirstLine) {
        if (c == '\n') {
          gotFirstLine = true;
        } else if (c != '\r') {
          requestLine += c;
        }
      }

      if (c == '\n') {
        newlineCount++;
        if (newlineCount >= 2) {
          gotHeaderEnd = true;
          break;
        }
      } else if (c != '\r') {
        newlineCount = 0;
      }
    }

    if (gotHeaderEnd) {
      break;
    }
  }

  bool ok = requestLine.startsWith("GET /") || requestLine.startsWith("HEAD /");
  size_t htmlLen = strlen(INDEX_HTML);

  Serial.print("HTTP req: ");
  Serial.println(requestLine.length() ? requestLine : "<empty>");

  if (ok) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html; charset=utf-8");
    client.print("Content-Length: ");
    client.println(htmlLen);
    client.println("Cache-Control: no-store");
    client.println("Connection: close");
    client.println();
    if (requestLine.startsWith("GET /")) {
      size_t sentBytes = 0;
      bool fullBody = writeAll(client, reinterpret_cast<const uint8_t*>(INDEX_HTML), htmlLen, 3000, sentBytes);
      Serial.print("HTTP body sent: ");
      Serial.print(fullBody ? "yes" : "partial");
      Serial.print(" (");
      Serial.print(sentBytes);
      Serial.print("/");
      Serial.print(htmlLen);
      Serial.println(" bytes)");
    }
  } else {
    client.println("HTTP/1.1 404 Not Found");
    client.println("Content-Type: text/plain; charset=utf-8");
    client.println("Content-Length: 9");
    client.println("Connection: close");
    client.println();
    client.print("Not Found");
  }

  client.flush();
  delay(2);
  client.stop();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Ethernet.begin(kMac, kIp, kDns, kGateway, kSubnet);
  httpServer.begin();

  wsServer.begin();
  wsServer.onEvent(onWebSocketEvent);

  can3.begin();
  can3.setBaudRate(currentCanBitrate);

  Serial.print("Web UI: http://");
  Serial.println(Ethernet.localIP());
  Serial.println("WebSocket: ws://<device-ip>:81");
}

void loop() {
  handleHttpClient();
  wsServer.loop();
  // Service FlexCAN internal event queue so RX frames are moved into readable buffers.
  can3.events();
  processCanRx();
  serviceHeartbeatState();
  serviceReadPoll();
  processWriteSequence();
}
