#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include "esp_camera.h"
#include "img_converters.h"
#include <HardwareSerial.h>
#include <esp_arduino_version.h>

// ================================================
// LAB SECURITY MONITORING SYSTEM
// ESP32-CAM + GC2145 RGB565 + Arduino Uno
// ================================================

// Wi-Fi credentials
const char* WIFI_SSID = "Deevena";
const char* WIFI_PASSWORD = "123456789";

// UART2: ESP32 RX = GPIO13, TX = GPIO14
HardwareSerial UnoSerial(2);

// Local dashboard web server
WebServer server(80);

// ================================================
// AI-THINKER ESP32-CAM PIN CONFIGURATION
// ================================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ================================================
// REFERENCE IMAGE SETTINGS
// ================================================

const int REF_WIDTH = 96;
const int REF_HEIGHT = 96;
const size_t REF_SIZE = REF_WIDTH * REF_HEIGHT * 2;

const int REFERENCE_COUNT = 3;
const float ACCESS_THRESHOLD = 69.0f; // Access is allowed only when best similarity is strictly above 69%
const char* refPaths[REFERENCE_COUNT] = {
  "/ref1.raw",
  "/ref2.raw",
  "/ref3.raw"
};

uint8_t* referenceImages[REFERENCE_COUNT] = {nullptr, nullptr, nullptr};
bool referenceReady[REFERENCE_COUNT] = {false, false, false};
int referenceCount = 0;

bool cameraReady = false;
bool filesystemReady = false;

unsigned long lastStatusTime = 0;
const unsigned long STATUS_INTERVAL = 2000;

// Latest sensor status received from Uno
int tamperStatus = 0;
int vibrationStatus = 0;
int unoState = 0;

// ================================================
// LOAD REFERENCE IMAGES FROM LITTLEFS
// ================================================

void loadReferenceImages() {
  referenceCount = 0;

  for (int i = 0; i < REFERENCE_COUNT; i++) {
    File file = LittleFS.open(refPaths[i], "r");

    if (!file) {
      Serial.printf("Missing reference: %s\n", refPaths[i]);
      continue;
    }

    if (file.size() != REF_SIZE) {
      Serial.printf("Incorrect size: %s (%u bytes)\n",
                    refPaths[i], (unsigned int)file.size());
      file.close();
      referenceReady[i] = false;
      referenceImages[i] = nullptr;
      continue;
    }

    uint8_t* buffer = (uint8_t*)malloc(REF_SIZE);

    if (!buffer) {
      Serial.println("Not enough memory for reference image");
      file.close();
      continue;
    }

    size_t bytesRead = file.read(buffer, REF_SIZE);
    file.close();

    if (bytesRead == REF_SIZE) {
      referenceImages[i] = buffer;
      referenceReady[i] = true;
      referenceCount++;
      Serial.printf("Loaded %s\n", refPaths[i]);
    } else {
      free(buffer);
      referenceReady[i] = false;
      referenceImages[i] = nullptr;
      Serial.printf("Failed to read %s\n", refPaths[i]);
    }
  }

  Serial.printf("References loaded: %d/%d\n", referenceCount, REFERENCE_COUNT);
}

// ================================================
// RGB565 IMAGE COMPARISON
// Compares general pixel patterns, not identities
// ================================================

uint16_t getReferencePixel(const uint8_t* image, int index) {
  return (uint16_t)image[index * 2] |
         ((uint16_t)image[index * 2 + 1] << 8);
}

float compareImage(camera_fb_t* frame, const uint8_t* reference) {
  if (!frame || !reference || frame->format != PIXFORMAT_RGB565 ||
      !frame->buf || frame->width <= 0 || frame->height <= 0 ||
      frame->len < (size_t)frame->width * (size_t)frame->height * 2) {
    return 0.0f;
  }

  float totalDifference = 0.0f;
  const int totalPixels = REF_WIDTH * REF_HEIGHT;

  for (int y = 0; y < REF_HEIGHT; y++) {
    int sourceY = y * frame->height / REF_HEIGHT;

    for (int x = 0; x < REF_WIDTH; x++) {
      int sourceX = x * frame->width / REF_WIDTH;

      int sourceIndex = (sourceY * frame->width + sourceX) * 2;

      uint16_t currentPixel =
        (uint16_t)frame->buf[sourceIndex] |
        ((uint16_t)frame->buf[sourceIndex + 1] << 8);

      int refIndex = y * REF_WIDTH + x;
      uint16_t referencePixel = getReferencePixel(reference, refIndex);

      int r1 = (currentPixel >> 11) & 0x1F;
      int g1 = (currentPixel >> 5) & 0x3F;
      int b1 = currentPixel & 0x1F;

      int r2 = (referencePixel >> 11) & 0x1F;
      int g2 = (referencePixel >> 5) & 0x3F;
      int b2 = referencePixel & 0x1F;

      float redDiff = abs(r1 - r2) / 31.0f;
      float greenDiff = abs(g1 - g2) / 63.0f;
      float blueDiff = abs(b1 - b2) / 31.0f;

      totalDifference +=
        (redDiff + greenDiff + blueDiff) / 3.0f;
    }
  }

  float averageDifference = totalDifference / totalPixels;
  float similarity = (1.0f - averageDifference) * 100.0f;

  if (similarity < 0) similarity = 0;
  if (similarity > 100) similarity = 100;

  return similarity;
}

// ================================================
// CAMERA INITIALIZATION
// GC2145 uses RGB565; JPEG is generated in software
// ================================================

bool initializeCamera() {
  camera_config_t config = {};

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
#else
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
#endif

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;

  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  esp_err_t result = esp_camera_init(&config);

  if (result != ESP_OK) {
    Serial.printf("Camera initialization failed: 0x%x\n", result);
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();

  if (sensor) {
    sensor->set_framesize(sensor, FRAMESIZE_QQVGA);
  }

  Serial.println("Camera initialized in RGB565 mode");
  return true;
}

// ================================================
// DASHBOARD HTML (BRIGHT PROFESSIONAL THEME WITH ANIMATIONS)
// ================================================

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Lab Security Monitor</title>
<style>
  :root {
    --bg-main: #f8fafc;
    --card-bg: #ffffff;
    --text-primary: #0f172a;
    --text-secondary: #475569;
    --text-muted: #64748b;
    --accent: #2563eb;
    --accent-hover: #1d4ed8;
    --accent-light: #eff6ff;
    --border-color: #e2e8f0;
    --success: #10b981;
    --shadow-sm: 0 1px 3px rgba(0, 0, 0, 0.05);
    --shadow-md: 0 4px 12px rgba(0, 0, 0, 0.05);
    --shadow-hover: 0 10px 25px -5px rgba(0, 0, 0, 0.08);
  }

  * { box-sizing: border-box; }

  body {
    margin: 0;
    padding: 24px 16px;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    background: var(--bg-main);
    color: var(--text-primary);
    line-height: 1.5;
  }

  .container {
    max-width: 800px;
    margin: 0 auto;
  }

  header {
    text-align: center;
    margin-bottom: 24px;
    animation: fadeInDown 0.6s cubic-bezier(0.16, 1, 0.3, 1);
  }

  h1 {
    font-size: 24px;
    font-weight: 700;
    color: var(--text-primary);
    margin: 0 0 6px 0;
    letter-spacing: -0.02em;
  }

  .subtitle {
    font-size: 14px;
    color: var(--text-secondary);
    margin: 0;
  }

  .panel {
    background: var(--card-bg);
    border: 1px solid var(--border-color);
    border-radius: 12px;
    padding: 20px;
    margin-bottom: 20px;
    box-shadow: var(--shadow-sm);
    transition: transform 0.3s ease, box-shadow 0.3s ease;
    animation: fadeInUp 0.6s cubic-bezier(0.16, 1, 0.3, 1) both;
  }

  .panel:hover {
    transform: translateY(-2px);
    box-shadow: var(--shadow-hover);
  }

  .panel:nth-child(1) { animation-delay: 0.1s; }
  .panel:nth-child(2) { animation-delay: 0.2s; }
  .panel:nth-child(3) { animation-delay: 0.3s; }

  h2 {
    font-size: 16px;
    font-weight: 600;
    color: var(--text-primary);
    margin: 0 0 16px 0;
    display: flex;
    align-items: center;
    gap: 8px;
  }

  .live-indicator {
    width: 8px;
    height: 8px;
    background-color: var(--success);
    border-radius: 50%;
    display: inline-block;
    position: relative;
  }

  .live-indicator::after {
    content: '';
    position: absolute;
    width: 100%;
    height: 100%;
    top: 0;
    left: 0;
    background-color: var(--success);
    border-radius: 50%;
    animation: pulse 1.8s infinite ease-in-out;
  }

  .img-container {
    position: relative;
    background: #f1f5f9;
    border: 1px solid var(--border-color);
    border-radius: 8px;
    overflow: hidden;
    max-width: 480px;
    margin: 0 auto 16px auto;
  }

  img#live {
    width: 100%;
    display: block;
    object-fit: cover;
    transition: opacity 0.2s ease;
  }

  .button-group {
    display: flex;
    gap: 10px;
    justify-content: center;
    flex-wrap: wrap;
  }

  button, a.download {
    display: inline-flex;
    align-items: center;
    justify-content: center;
    padding: 9px 16px;
    font-size: 13px;
    font-weight: 600;
    border-radius: 6px;
    border: 1px solid transparent;
    background: var(--accent);
    color: #ffffff;
    text-decoration: none;
    cursor: pointer;
    transition: all 0.2s ease;
    box-shadow: 0 1px 2px rgba(37, 99, 235, 0.2);
  }

  button:hover, a.download:hover {
    background: var(--accent-hover);
    transform: translateY(-1px);
    box-shadow: 0 4px 8px rgba(37, 99, 235, 0.25);
  }

  button:active, a.download:active {
    transform: translateY(0);
    box-shadow: 0 1px 2px rgba(37, 99, 235, 0.2);
  }

  a.download {
    background: #ffffff;
    color: var(--text-secondary);
    border: 1px solid var(--border-color);
    box-shadow: var(--shadow-sm);
  }

  a.download:hover {
    background: var(--accent-light);
    color: var(--accent);
    border-color: var(--accent);
  }

  .status-box {
    background: var(--bg-main);
    border: 1px solid var(--border-color);
    border-radius: 8px;
    padding: 12px 16px;
    font-size: 13px;
    color: var(--text-secondary);
    line-height: 1.6;
    margin-bottom: 12px;
  }

  .status-badge {
    display: inline-block;
    padding: 2px 8px;
    border-radius: 4px;
    font-weight: 600;
    font-size: 12px;
  }

  .badge-online { background: #dcfce7; color: #15803d; }
  .badge-offline { background: #fee2e2; color: #b91c1c; }

  .refs {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 12px;
  }

  @media (min-width: 600px) {
    .refs { grid-template-columns: repeat(3, minmax(0, 1fr)); }
  }

  .ref-card {
    background: var(--bg-main);
    border: 1px solid var(--border-color);
    border-radius: 8px;
    padding: 10px;
    text-align: center;
    transition: border-color 0.2s ease;
  }

  .ref-card:hover {
    border-color: var(--accent);
  }

  canvas {
    width: 100%;
    aspect-ratio: 1;
    image-rendering: pixelated;
    background: #000;
    border-radius: 4px;
    margin-bottom: 8px;
    border: 1px solid var(--border-color);
  }

  .ref-card p {
    font-size: 12px;
    font-weight: 600;
    margin: 0 0 8px 0;
    color: var(--text-primary);
  }

  .small {
    font-size: 12px;
    color: var(--text-muted);
    margin-top: 10px;
    text-align: center;
  }

  /* Animations */
  @keyframes fadeInDown {
    from { opacity: 0; transform: translateY(-12px); }
    to { opacity: 1; transform: translateY(0); }
  }

  @keyframes fadeInUp {
    from { opacity: 0; transform: translateY(12px); }
    to { opacity: 1; transform: translateY(0); }
  }

  @keyframes pulse {
    0% { transform: scale(1); opacity: 0.8; }
    70% { transform: scale(2.5); opacity: 0; }
    100% { transform: scale(1); opacity: 0; }
  }
</style>
</head>
<body>

<div class="container">
  <header>
    <h1>LAB SECURITY MONITOR</h1>
    <p class="subtitle">ESP32-CAM Security Node & Hardware Diagnostics</p>
  </header>

  <div class="panel">
    <h2><span class="live-indicator"></span> Live Camera Feed</h2>
    <div class="img-container">
      <img id="live" alt="Camera feed snapshot">
    </div>
    <div class="button-group">
      <button onclick="refreshImage()">Refresh Camera</button>
      <button onclick="compareImages('manual')">Compare Image</button>
      <button id="autoButton" onclick="toggleAutoCompare()">Auto Compare: ON</button>
    </div>
    <p id="faceStatus" class="small">Face detector is loading. Automatic comparison will start when a face is detected.</p>
    <p class="small">Automatic mode compares once when a face first appears. Manual comparison remains available.</p>
  </div>

  <div class="panel">
    <h2>System & Sensor Status</h2>
    <div class="status-box" id="status">Connecting to node...</div>
    <div id="result" class="status-box" style="display:none;"></div>
  </div>

  <div class="panel">
    <h2>Reference Signatures</h2>
    <div class="refs">
      <div class="ref-card">
        <canvas id="ref0" width="96" height="96"></canvas>
        <p>Reference 1</p>
        <a class="download" href="/ref1.raw">RAW File</a>
      </div>
      <div class="ref-card">
        <canvas id="ref1" width="96" height="96"></canvas>
        <p>Reference 2</p>
        <a class="download" href="/ref2.raw">RAW File</a>
      </div>
      <div class="ref-card">
        <canvas id="ref2" width="96" height="96"></canvas>
        <p>Reference 3</p>
        <a class="download" href="/ref3.raw">RAW File</a>
      </div>
    </div>
    <p class="small">Face detection only triggers comparison. Scores compare whole-frame RGB565 pixel patterns; they are not face recognition or identity verification. Lighting and camera position can change scores.</p>
  </div>
</div>

<script>
let cvReady = false;
let faceCascade = null;
let autoCompareEnabled = true;
let faceWasPresent = false;
let faceCheckBusy = false;
let compareBusy = false;
let lastAutoCompareAt = 0;
const AUTO_COMPARE_COOLDOWN_MS = 10000;

function setFaceStatus(message) {
  const el = document.getElementById('faceStatus');
  if (el) el.textContent = message;
}

function toggleAutoCompare() {
  autoCompareEnabled = !autoCompareEnabled;
  const button = document.getElementById('autoButton');
  button.textContent = 'Auto Compare: ' + (autoCompareEnabled ? 'ON' : 'OFF');
  button.style.opacity = autoCompareEnabled ? '1' : '0.65';
  setFaceStatus(autoCompareEnabled
    ? 'Automatic comparison enabled. Waiting for a face...'
    : 'Automatic comparison disabled. Manual Compare Image still works.');
  if (!autoCompareEnabled) faceWasPresent = false;
}

async function initializeFaceDetector() {
  try {
    setFaceStatus('OpenCV loaded. Downloading face-detection model...');
    const modelUrl = 'https://raw.githubusercontent.com/opencv/opencv/4.x/data/haarcascades/haarcascade_frontalface_default.xml';
    const response = await fetch(modelUrl, { cache: 'force-cache' });
    if (!response.ok) throw new Error('Could not download Haar cascade model');
    const modelData = new Uint8Array(await response.arrayBuffer());
    cv.FS_createDataFile('/', 'haarcascade_frontalface_default.xml', modelData, true, false, false);
    faceCascade = new cv.CascadeClassifier();
    if (!faceCascade.load('haarcascade_frontalface_default.xml')) {
      faceCascade.delete();
      faceCascade = null;
      throw new Error('Haar cascade model could not be loaded');
    }
    cvReady = true;
    setFaceStatus('Face detection ready. Automatic comparison is ' + (autoCompareEnabled ? 'ON' : 'OFF') + '.');
  } catch (error) {
    cvReady = false;
    setFaceStatus('Face detection unavailable: check dashboard-device internet access, then reload this page. Manual comparison still works.');
    console.error('Face detector initialization failed:', error);
  }
}

function onOpenCvReady() {
  // OpenCV.js calls this after its WebAssembly runtime is initialized.
  initializeFaceDetector();
}

function processFaceDetection(img) {
  if (!cvReady || !faceCascade || faceCheckBusy || !img.complete || !img.naturalWidth) return;
  faceCheckBusy = true;
  let source = null, gray = null, faces = null;
  try {
    const canvas = document.createElement('canvas');
    canvas.width = img.naturalWidth;
    canvas.height = img.naturalHeight;
    canvas.getContext('2d', { willReadFrequently: true }).drawImage(img, 0, 0);

    source = cv.imread(canvas);
    gray = new cv.Mat();
    faces = new cv.RectVector();
    cv.cvtColor(source, gray, cv.COLOR_RGBA2GRAY);
    cv.equalizeHist(gray, gray);
    faceCascade.detectMultiScale(gray, faces, 1.1, 4, 0, new cv.Size(24, 24), new cv.Size());

    const faceFound = faces.size() > 0;
    if (faceFound) {
      setFaceStatus('FACE DETECTED (' + faces.size() + '). ' + (autoCompareEnabled ? 'Checking references...' : 'Auto comparison is OFF.'));
      if (autoCompareEnabled && !faceWasPresent && !compareBusy &&
          Date.now() - lastAutoCompareAt >= AUTO_COMPARE_COOLDOWN_MS) {
        lastAutoCompareAt = Date.now();
        faceWasPresent = true;
        compareImages('auto');
      } else {
        faceWasPresent = true;
      }
    } else {
      faceWasPresent = false;
      setFaceStatus('No face detected. ' + (autoCompareEnabled ? 'Waiting for a face...' : 'Auto comparison is OFF.'));
    }
  } catch (error) {
    console.warn('Face detection frame failed:', error);
  } finally {
    if (faces) faces.delete();
    if (gray) gray.delete();
    if (source) source.delete();
    faceCheckBusy = false;
  }
}

function refreshImage() {
  const img = document.getElementById('live');
  img.onload = function() { processFaceDetection(img); };
  img.src = '/jpg?t=' + Date.now();
}

async function loadReferences() {
  for (let i = 1; i <= 3; i++) {
    try {
      const response = await fetch('/ref' + i + '.raw');
      if (!response.ok) continue;

      const bytes = new Uint8Array(await response.arrayBuffer());
      if (bytes.length !== 96 * 96 * 2) continue;

      const canvas = document.getElementById('ref' + (i - 1));
      const ctx = canvas.getContext('2d');
      const image = ctx.createImageData(96, 96);

      for (let p = 0; p < 96 * 96; p++) {
        const pixel = bytes[p * 2] | (bytes[p * 2 + 1] << 8);
        image.data[p * 4] = Math.round(((pixel >> 11) & 31) * 255 / 31);
        image.data[p * 4 + 1] = Math.round(((pixel >> 5) & 63) * 255 / 63);
        image.data[p * 4 + 2] = Math.round((pixel & 31) * 255 / 31);
        image.data[p * 4 + 3] = 255;
      }
      ctx.putImageData(image, 0, 0);
    } catch (e) {
      console.log('Reference unavailable:', i);
    }
  }
}

async function compareImages(source = 'manual') {
  const result = document.getElementById('result');
  if (compareBusy) return;
  compareBusy = true;
  result.style.display = 'block';
  result.innerHTML = (source === 'auto' ? 'Face detected. Automatically comparing references...' : 'Manual comparison in progress...');

  try {
    const response = await fetch('/compare?source=' + encodeURIComponent(source) + '&t=' + Date.now());
    const data = await response.json();

    if (!response.ok) {
      result.textContent = data.error || 'Comparison failed.';
      return;
    }

    let text = '<strong>' + (source === 'auto' ? 'Automatic face-triggered comparison' : 'Manual comparison') + '</strong><br><strong>Pixel similarity breakdown:</strong><br>';
    for (let i = 0; i < data.scores.length; i++) {
      text += 'Reference ' + (i + 1) + ': ' + data.scores[i].toFixed(1) + '%<br>';
    }
    const bestMatch = data.scores[data.best];
    const accessAllowed = data.access === 'ALLOWED';
    text += '<div style="margin-top:6px; font-weight:600;">Highest match: Reference ' +
      (data.best + 1) + ' (' + bestMatch.toFixed(1) + '%)</div>';
    text += '<div style="margin-top:10px;padding:12px;border-radius:8px;font-weight:800;color:white;background:' +
      (accessAllowed ? '#16803c' : '#b42318') + ';">' +
      (accessAllowed ? 'ACCESS ALLOWED' : 'ACCESS DENIED') +
      ' — ' + bestMatch.toFixed(1) + '% (required: above 69%)</div>';
    result.innerHTML = text;
  } catch (e) {
    result.textContent = 'Network timeout during image comparison.';
  } finally {
    compareBusy = false;
  }
}

async function updateStatus() {
  try {
    const response = await fetch('/status?t=' + Date.now());
    const data = await response.json();

    const camBadge = data.camera 
      ? '<span class="status-badge badge-online">ONLINE</span>' 
      : '<span class="status-badge badge-offline">OFFLINE</span>';

    const fsBadge = data.fs 
      ? '<span class="status-badge badge-online">READY</span>' 
      : '<span class="status-badge badge-offline">ERROR</span>';

    document.getElementById('status').innerHTML =
      'Camera: ' + camBadge + ' &bull; ' +
      'FileSystem: ' + fsBadge + '<br>' +
      'Loaded References: <strong>' + data.refs + '/3</strong> &bull; '+
      'Tamper: <strong>' + data.tamper + '</strong> &bull; ' +
      'Vibration: <strong>' + data.vibration + '</strong> &bull; ' +
      'Uno State: <strong>' + data.state + '</strong>';
  } catch (e) {
    document.getElementById('status').textContent = 'Status unavailable.';
  }
}

refreshImage();
loadReferences();
updateStatus();

setInterval(refreshImage, 1500);
setInterval(updateStatus, 2000);
</script>
<script>
// OpenCV.js performs actual face detection in the dashboard browser, not on the ESP32-CAM.
window.Module = { onRuntimeInitialized: onOpenCvReady };
</script>
<script async src="https://docs.opencv.org/4.x/opencv.js"></script>
</body>
</html>
)rawliteral";

// ================================================
// CAMERA JPEG ENDPOINT
// RGB565 -> JPEG software conversion
// ================================================

void handleJpeg() {
  if (!cameraReady) {
    server.send(503, "text/plain", "Camera unavailable");
    return;
  }

  camera_fb_t* frame = esp_camera_fb_get();

  if (!frame) {
    server.send(500, "text/plain", "Frame capture failed");
    return;
  }

  uint8_t* jpgBuffer = nullptr;
  size_t jpgLength = 0;

  bool converted = frame2jpg(frame, 75, &jpgBuffer, &jpgLength);

  esp_camera_fb_return(frame);

  if (!converted || !jpgBuffer || jpgLength == 0) {
    if (jpgBuffer) free(jpgBuffer);
    server.send(500, "text/plain", "JPEG conversion failed");
    return;
  }

  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(jpgLength);
  server.send(200, "image/jpeg", "");
  server.sendContent((const char*)jpgBuffer, jpgLength);

  free(jpgBuffer);
}

// ================================================
// SERVE RAW REFERENCE FILES
// ================================================

void handleReference(int index) {
  if (index < 0 || index >= REFERENCE_COUNT || !filesystemReady) {
    server.send(404, "text/plain", "Reference unavailable");
    return;
  }

  File file = LittleFS.open(refPaths[index], "r");

  if (!file || file.size() != REF_SIZE) {
    if (file) file.close();
    server.send(404, "text/plain", "Reference file missing");
    return;
  }

  server.sendHeader("Cache-Control", "no-store");
  server.streamFile(file, "application/octet-stream");
  file.close();
}

// ================================================
// COMPARE CURRENT CAMERA IMAGE
// ================================================

void handleCompare() {
  if (!cameraReady) {
    UnoSerial.println("A,DENIED,0,0"); // Fail closed if camera is unavailable
    server.send(503, "application/json",
                "{\"error\":\"Camera unavailable\"}");
    return;
  }

  if (referenceCount == 0) {
    UnoSerial.println("A,DENIED,0,0"); // No reference data means no access
    server.send(503, "application/json",
                "{\"error\":\"No reference images loaded\"}");
    return;
  }

  camera_fb_t* frame = esp_camera_fb_get();

  if (!frame) {
    UnoSerial.println("A,DENIED,0,0"); // Fail closed if image capture fails
    server.send(500, "application/json",
                "{\"error\":\"Capture failed\"}");
    return;
  }

  float scores[REFERENCE_COUNT] = {0, 0, 0};
  int bestIndex = -1;
  float bestScore = -1;

  for (int i = 0; i < REFERENCE_COUNT; i++) {
    if (!referenceReady[i]) continue;

    scores[i] = compareImage(frame, referenceImages[i]);

    if (scores[i] > bestScore) {
      bestScore = scores[i];
      bestIndex = i;
    }
  }

  esp_camera_fb_return(frame);

  bool accessAllowed = (bestIndex >= 0 && bestScore > ACCESS_THRESHOLD);
  const char* accessText = accessAllowed ? "ALLOWED" : "DENIED";

  // Notify the Arduino Uno so it can drive the physical LEDs, buzzer and OLED.
  // Protocol: A,ALLOWED|DENIED,score,referenceNumber\n
  UnoSerial.print("A,");
  UnoSerial.print(accessText);
  UnoSerial.print(",");
  UnoSerial.print(bestScore < 0 ? 0.0f : bestScore, 2);
  UnoSerial.print(",");
  UnoSerial.println(bestIndex >= 0 ? bestIndex + 1 : 0);

  char json[240];
  snprintf(json, sizeof(json),
    "{\"scores\":[%.2f,%.2f,%.2f],\"best\":%d,\"match\":%.2f,\"access\":\"%s\",\"threshold\":69}",
    scores[0], scores[1], scores[2],
    bestIndex >= 0 ? bestIndex : 0,
    bestScore < 0 ? 0.0f : bestScore,
    accessText);

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

// ================================================
// DASHBOARD STATUS
// ================================================

void handleStatus() {
  char json[200];

  snprintf(json, sizeof(json),
    "{\"camera\":%s,\"fs\":%s,\"refs\":%d,"
    "\"tamper\":%d,\"vibration\":%d,\"state\":%d}",
    cameraReady ? "true" : "false",
    filesystemReady ? "true" : "false",
    referenceCount,
    tamperStatus,
    vibrationStatus,
    unoState);

  server.send(200, "application/json", json);
}

// ================================================
// RECEIVE UNO DATA
// Format: S,TAMPER,VIBRATION,STATE
// ================================================

void readUnoSerial() {
  static char line[80];
  static byte index = 0;

  while (UnoSerial.available()) {
    char c = UnoSerial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      line[index] = '\0';
      index = 0;

      if (strncmp(line, "S,", 2) == 0) {
        char* token = strtok(line, ",");

        token = strtok(NULL, ",");
        if (token) tamperStatus = atoi(token);

        token = strtok(NULL, ",");
        if (token) vibrationStatus = atoi(token);

        token = strtok(NULL, ",");
        if (token) unoState = atoi(token);
      }
    }
    else if (index < sizeof(line) - 1) {
      line[index++] = c;
    }
    else {
      index = 0;
    }
  }
}

// ================================================
// SEND STATUS TO UNO
// Format: C,CAMERA_OK,FS_OK,REFS,IP
// ================================================

void sendStatusToUno() {
  if (millis() - lastStatusTime < STATUS_INTERVAL) return;

  lastStatusTime = millis();

  UnoSerial.print("C,");
  UnoSerial.print(cameraReady ? "CAMERA_OK" : "CAMERA_ERROR");
  UnoSerial.print(",");
  UnoSerial.print(filesystemReady ? "FS_OK" : "FS_ERROR");
  UnoSerial.print(",");
  UnoSerial.print(referenceCount);
  UnoSerial.print(",");
  UnoSerial.println(WiFi.localIP().toString());
}

// ================================================
// SETUP
// ================================================

void setup() {
  Serial.begin(115200);

  // Communication with Arduino Uno at 9600 baud
  UnoSerial.begin(9600, SERIAL_8N1, 13, 14);

  Serial.println();
  Serial.println("LAB SECURITY MONITOR");
  Serial.println("Starting camera...");

  cameraReady = initializeCamera();

  // Do not format flash if mounting fails.
  filesystemReady = LittleFS.begin(false);

  if (filesystemReady) {
    Serial.println("LittleFS mounted");
    loadReferenceImages();
  } else {
    Serial.println("LittleFS mount failed; flash was not formatted");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");

  unsigned long startTime = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - startTime < 20000) {
    delay(300);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Dashboard URL: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wi-Fi connection failed; dashboard is not reachable until Wi-Fi connects");
  }

  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/jpg", HTTP_GET, handleJpeg);
  server.on("/compare", HTTP_GET, handleCompare);
  server.on("/status", HTTP_GET, handleStatus);

  server.on("/ref1.raw", HTTP_GET, []() { handleReference(0); });
  server.on("/ref2.raw", HTTP_GET, []() { handleReference(1); });
  server.on("/ref3.raw", HTTP_GET, []() { handleReference(2); });

  server.begin();
  Serial.println("Web server started");
}

// ================================================
// MAIN LOOP
// ================================================

void loop() {
  server.handleClient();

  readUnoSerial();
  sendStatusToUno();

  delay(2);
}