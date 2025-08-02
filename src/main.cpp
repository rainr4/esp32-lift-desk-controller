#include <WiFi.h>
#include <WebServer.h>
#include <Ticker.h>
#include <Preferences.h>
#include <ArduinoOTA.h>

// Wi-Fi Credentials
const char* ssid = "YOUR_SSID_HERE"; // Replace with your Wi-Fi SSID
const char* password = "YOUR_PASSWORD_HERE"; // Replace with your Wi-Fi password

// Motor Control Pins
#define L_RPWM 25
#define L_LPWM 27
#define R_RPWM 4
#define R_LPWM 2

// Hall Sensor Pins
#define L_HALL1 36
#define L_HALL2 39
#define R_HALL1 16
#define R_HALL2 17

// Hall sensor counts
volatile int leftPulseCount = 0;
volatile int rightPulseCount = 0;

// Motor state
bool motorMovingUp = true;
bool isMoving = false;

// Debounce settings
unsigned long lastL1Time = 0;
unsigned long lastL2Time = 0;
unsigned long lastR1Time = 0;
unsigned long lastR2Time = 0;
unsigned long DEBOUNCE_TIME = 30;   // milliseconds

// PID parameters
float Kp = 2.0;    // Double the proportional gain (was 1.0)
float Ki = 0.2;    // Increase integral gain (was 0.1)
float Kd = 0.1;    // Increase derivative gain (was 0.05)
float integral = 0;
float lastError = 0;
unsigned long lastBalanceTime = 0;

// Speed control (adjustable)
int currentSpeed = 140;           // default speed
const int minPWM   = 60;  
const int maxPWM   = 225;

// Balance check interval
int balanceCheckInterval = 50;    // milliseconds

// Utilities
WebServer server(80);
Ticker balanceTicker;
Preferences preferences;
unsigned long lastHallPrintTime = 0;
int lastLeftCount = 0;
int lastRightCount = 0;

// Forward declarations
void moveBoth(bool up);
void moveLeft(bool up);
void moveRight(bool up);
void stopMotorAll();
void balanceMotors();
void applyMotorBrake();
void monitorHallSensors();
void handleRoot();

// Hall sensor interrupt handlers
void IRAM_ATTR leftHall1Interrupt() {
  if (!isMoving) return;
  unsigned long now = millis();
  if (now - lastL1Time > DEBOUNCE_TIME) {
    leftPulseCount += (motorMovingUp ? 1 : -1);
    lastL1Time = now;
  }
}
void IRAM_ATTR leftHall2Interrupt() {
  if (!isMoving) return;
  unsigned long now = millis();
  if (now - lastL2Time > DEBOUNCE_TIME) {
    leftPulseCount += (motorMovingUp ? 1 : -1);
    lastL2Time = now;
  }
}
void IRAM_ATTR rightHall1Interrupt() {
  if (!isMoving) return;
  unsigned long now = millis();
  if (now - lastR1Time > DEBOUNCE_TIME) {
    rightPulseCount += (motorMovingUp ? 1 : -1);
    lastR1Time = now;
  }
}
void IRAM_ATTR rightHall2Interrupt() {
  if (!isMoving) return;
  unsigned long now = millis();
  if (now - lastR2Time > DEBOUNCE_TIME) {
    rightPulseCount += (motorMovingUp ? 1 : -1);
    lastR2Time = now;
  }
}

// Move both motors with PID-based balancing
void moveBoth(bool up) {
  motorMovingUp = up;
  isMoving = true;
  balanceTicker.detach();

  integral = 0;
  lastError = 0;
  lastBalanceTime = millis();

  // Start with adjustable speed
  if (up) {
    ledcWrite(0, 0);           ledcWrite(1, currentSpeed);
    ledcWrite(2, 0);           ledcWrite(3, currentSpeed);
  } else {
    ledcWrite(0, currentSpeed); ledcWrite(1, 0);
    ledcWrite(2, currentSpeed); ledcWrite(3, 0);
  }
  balanceTicker.attach_ms(balanceCheckInterval, balanceMotors);
  Serial.println("[BALANCE] PID monitor started");
}

// Manual single motor control (left)
void moveLeft(bool up) {
  balanceTicker.detach();
  isMoving = true;
  motorMovingUp = up;
  if (up) {
    ledcWrite(0, 0);           ledcWrite(1, currentSpeed);
  } else {
    ledcWrite(0, currentSpeed); ledcWrite(1, 0);
  }
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

// Manual single motor control (right)
void moveRight(bool up) {
  balanceTicker.detach();
  isMoving = true;
  motorMovingUp = up;
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  if (up) {
    ledcWrite(2, 0);           ledcWrite(3, currentSpeed);
  } else {
    ledcWrite(2, currentSpeed); ledcWrite(3, 0);
  }
}

// Apply electrical braking then stop
void applyMotorBrake() {
  if (motorMovingUp) {
    ledcWrite(0, minPWM);      ledcWrite(1, 0);
    ledcWrite(2, minPWM);      ledcWrite(3, 0);
  } else {
    ledcWrite(0, 0);           ledcWrite(1, minPWM);
    ledcWrite(2, 0);           ledcWrite(3, minPWM);
  }
  delay(50);
  ledcWrite(0, 0); ledcWrite(1, 0);
  ledcWrite(2, 0); ledcWrite(3, 0);
}

// Stop both motors
void stopMotorAll() {
  applyMotorBrake();
  balanceTicker.detach();
  isMoving = false;
  Serial.println("[DEBUG] Motors stopped");
}

// PID-based balancing logic
void balanceMotors() {
  if (!isMoving) return;
  unsigned long now = millis();
  float dt = (now - lastBalanceTime) / 1000.0;
  float error = leftPulseCount - rightPulseCount;
  integral += error * dt;
  float derivative = dt > 0 ? (error - lastError) / dt : 0;
  float corr = Kp * error + Ki * integral + Kd * derivative;

  int pwmL = constrain((int)(currentSpeed - corr), minPWM, maxPWM);
  int pwmR = constrain((int)(currentSpeed + corr), minPWM, maxPWM);

  if (motorMovingUp) {
    ledcWrite(0, 0);    ledcWrite(1, pwmL);
    ledcWrite(2, 0);    ledcWrite(3, pwmR);
  } else {
    ledcWrite(0, pwmL); ledcWrite(1, 0);
    ledcWrite(2, pwmR); ledcWrite(3, 0);
  }
  lastError = error;
  lastBalanceTime = now;
}

// Print hall sensor data
void monitorHallSensors() {
  if (!isMoving) return;
  unsigned long now = millis();
  if (now - lastHallPrintTime > 100) {
    int dL = leftPulseCount - lastLeftCount;
    int dR = rightPulseCount - lastRightCount;
    Serial.printf("HALL: L=%d(+%d) R=%d(+%d) Diff=%d\n",
                  leftPulseCount, dL,
                  rightPulseCount, dR,
                  leftPulseCount - rightPulseCount);
    lastLeftCount = leftPulseCount;
    lastRightCount = rightPulseCount;
    lastHallPrintTime = now;
  }
}

// Web UI
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Lift Desk Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body {
      background-color: #121212;
      color: #e0e0e0;
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      margin: 0;
      padding: 20px;
      display: flex;
      flex-direction: column;
      align-items: center;
    }
    h1 {
      color: #bb86fc;
      margin-bottom: 25px;
      text-align: center;
    }
    .container {
      width: 100%;
      max-width: 500px;
    }
    .control-panel {
      background-color: #1e1e1e;
      border-radius: 10px;
      padding: 20px;
      margin-bottom: 20px;
      box-shadow: 0 4px 8px rgba(0,0,0,0.3);
    }
    .button-row {
      display: flex;
      justify-content: space-between;
      margin-bottom: 15px;
    }
    .btn {
      padding: 12px 10px;
      margin: 5px;
      width: calc(50% - 10px);
      border: none;
      border-radius: 8px;
      font-weight: bold;
      cursor: pointer;
      transition: all 0.2s ease;
      color: white;
      display: flex;
      flex-direction: column;
      align-items: center;
    }
    .btn:active {
      transform: scale(0.95);
    }
    .btn-left-up { background-color: #03a9f4; }
    .btn-left-down { background-color: #0288d1; }
    .btn-right-up { background-color: #00bcd4; }
    .btn-right-down { background-color: #0097a7; }
    .btn-both-up { background-color: #4caf50; width: 100%; }
    .btn-both-down { background-color: #388e3c; width: 100%; }
    .btn-stop { background-color: #f44336; width: 100%; font-size: 1.2em; }
    .status-panel {
      background-color: #1e1e1e;
      border-radius: 10px;
      padding: 20px;
      margin-bottom: 20px;
      box-shadow: 0 4px 8px rgba(0,0,0,0.3);
    }
    .status-item {
      display: flex;
      justify-content: space-between;
      margin-bottom: 8px;
      font-size: 1.1em;
    }
    .status-value {
      font-weight: bold;
      color: #bb86fc;
    }
    .arrow {
      font-size: 1.5em;
      margin-bottom: 5px;
    }
    .slider-container {
      width: 100%;
      margin: 25px 0 10px 0;
    }
    .slider {
      -webkit-appearance: none;
      width: 100%;
      height: 8px;
      border-radius: 5px;
      background: #555;
      outline: none;
    }
    .slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 20px;
      height: 20px;
      border-radius: 50%;
      background: #bb86fc;
      cursor: pointer;
    }
    .slider::-moz-range-thumb {
      width: 20px;
      height: 20px;
      border-radius: 50%;
      background: #bb86fc;
      cursor: pointer;
    }
    .speed-value {
      text-align: center;
      font-size: 1.2em;
      margin-top: 10px;
      color: #bb86fc;
    }
    .speed-label {
      display: flex;
      justify-content: space-between;
      margin-bottom: 5px;
      color: #888;
    }
  </style>
</head>
<body>
  <h1>Lift Desk Control</h1>
  <div class="container">
    <div class="control-panel">
      <div class="button-row">
        <form action="/up-left" method="post" style="width: 48%;">
          <button class="btn btn-left-up">
            <span class="arrow">↑</span>L-UP
          </button>
        </form>
        <form action="/up-right" method="post" style="width: 48%;">
          <button class="btn btn-right-up">
            <span class="arrow">↑</span>R-UP
          </button>
        </form>
      </div>
      <div class="button-row">
        <form action="/down-left" method="post" style="width: 48%;">
          <button class="btn btn-left-down">
            <span class="arrow">↓</span>L-DOWN
          </button>
        </form>
        <form action="/down-right" method="post" style="width: 48%;">
          <button class="btn btn-right-down">
            <span class="arrow">↓</span>R-DOWN
          </button>
        </form>
      </div>
      <div class="button-row">
        <form action="/up-both" method="post" style="width: 100%;">
          <button class="btn btn-both-up">
            <span class="arrow">↑↑</span>BOTH UP
          </button>
        </form>
      </div>
      <div class="button-row">
        <form action="/down-both" method="post" style="width: 100%;">
          <button class="btn btn-both-down">
            <span class="arrow">↓↓</span>BOTH DOWN
          </button>
        </form>
      </div>
      <div class="button-row">
        <form action="/stop" method="post" style="width: 100%;">
          <button class="btn btn-stop">STOP</button>
        </form>
      </div>
    </div>

    <div class="status-panel">
      <div class="status-item">
        <span>Leg Difference:</span>
        <span class="status-value">)rawliteral";
  html += String(leftPulseCount - rightPulseCount);
  html += R"rawliteral( pulses</span>
      </div>
    </div>

    <div class="control-panel">
      <form action="/set-speed" method="post">
        <h3 style="margin-top: 0; color: #bb86fc;">Speed Control</h3>
        <div class="speed-label">
          <span>Slow</span>
          <span>Fast</span>
        </div>
        <div class="slider-container">
          <input type="range" min="60" max="225" value=")rawliteral";
  html += String(currentSpeed);
  html += R"rawliteral(" name="speed" class="slider" id="speedSlider">
          <div class="speed-value" id="speedValue">)rawliteral";
  html += String(currentSpeed);
  html += R"rawliteral(</div>
        </div>
        <button type="submit" class="btn" style="background-color: #8e24aa; width: 100%; margin-top: 10px;">Apply Speed</button>
      </form>
    </div>
  </div>

  <script>
    const slider = document.getElementById("speedSlider");
    const output = document.getElementById("speedValue");
    slider.oninput = function() {
      output.innerHTML = this.value;
    }
  </script>
</body>
</html>)rawliteral";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Starting Up...");

  // Hall sensor setup
  pinMode(L_HALL1, INPUT_PULLUP);
  pinMode(L_HALL2, INPUT_PULLUP);
  pinMode(R_HALL1, INPUT_PULLUP);
  pinMode(R_HALL2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_HALL1), leftHall1Interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(L_HALL2), leftHall2Interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(R_HALL1), rightHall1Interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(R_HALL2), rightHall2Interrupt, RISING);

  // PWM channels
  ledcSetup(0, 25000, 8); ledcAttachPin(L_RPWM, 0);
  ledcSetup(1, 25000, 8); ledcAttachPin(L_LPWM, 1);
  ledcSetup(2, 25000, 8); ledcAttachPin(R_RPWM, 2);
  ledcSetup(3, 25000, 8); ledcAttachPin(R_LPWM, 3);

  // Stop initially
  stopMotorAll();

  // Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());

  // Setup OTA
  ArduinoOTA.setHostname("liftdesk-esp32");
  ArduinoOTA.setPassword("123Password"); // Change this to your own password
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else { // U_SPIFFS
        type = "filesystem";
      }
      Serial.println("OTA Start updating " + type);
      // Stop all motor operations during OTA
      stopMotorAll();
    })
    .onEnd([]() {
      Serial.println("\nOTA End");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("OTA Error[%u]: ", error);
      if      (error == OTA_AUTH_ERROR)    Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR)     Serial.println("End Failed");
    });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");

  // Load settings
  preferences.begin("deskConfig", false);
  balanceCheckInterval = preferences.getInt("balCheckInt", 50);
  DEBOUNCE_TIME = preferences.getULong("debounceTime", 30);
  currentSpeed = preferences.getInt("speed", currentSpeed);

  // HTTP endpoints
  server.on("/",          HTTP_GET,  handleRoot);
  server.on("/up-both",   HTTP_POST, [](){ moveBoth(true);  server.sendHeader("Location","/"); server.send(303); });
  server.on("/down-both", HTTP_POST, [](){ moveBoth(false); server.sendHeader("Location","/"); server.send(303); });
  server.on("/up-left",   HTTP_POST, [](){ moveLeft(true);  server.sendHeader("Location","/"); server.send(303); });
  server.on("/down-left", HTTP_POST, [](){ moveLeft(false); server.sendHeader("Location","/"); server.send(303); });
  server.on("/up-right",  HTTP_POST, [](){ moveRight(true); server.sendHeader("Location","/"); server.send(303); });
  server.on("/down-right",HTTP_POST, [](){ moveRight(false);server.sendHeader("Location","/"); server.send(303); });
  server.on("/stop",      HTTP_POST, [](){ stopMotorAll();  server.sendHeader("Location","/"); server.send(303); });
  server.on("/set-speed", HTTP_POST, [](){
    currentSpeed = server.arg("speed").toInt();
    preferences.putInt("speed", currentSpeed);
    server.sendHeader("Location","/"); server.send(303);
  });

  server.begin();
  Serial.println("HTTP server started");
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Attempting to reconnect...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      Serial.print(".");
      attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi reconnected!");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("\nFailed to reconnect. Will try again later.");
    }
  }
}
void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  monitorHallSensors();
  checkWiFiConnection();
}
