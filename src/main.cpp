#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// ============================================================================
// 1. PINES DE LOS SENSORES (5 Canales de Línea + 2 Auxiliares BFD-1000)
// ============================================================================
const int S1 = 15; // Extremo izquierdo
const int S2 = 2;  // Izquierdo
const int S3 = 4;  // Centro
const int S4 = 16; // Derecho
const int S5 = 17; // Extremo derecho

// Sensores auxiliares BFD-1000 (Seguridad y Colisión)
const int PIN_NEAR = 19; // Sensor infrarrojo de obstáculo frontal (Cable café) -> Conectar a D19
const int PIN_CLP  = 21; // Microswitch de choque frontal (Cable rojo) -> Conectar a D21

const float sensorWeights[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

// ============================================================================
// 2. PINES DEL DRIVER TB6612FNG
// ============================================================================
const int PWMA = 18; // Pin seguro (cambiado de 12 a 18 para evitar conflicto de boot)
const int AIN1 = 14;
const int AIN2 = 27;

const int PWMB = 13;
const int BIN1 = 26;
const int BIN2 = 25;

const int STBY_PIN = -1; // -1 si está puenteado a VCC/3.3V

// Sentido de giro de los motores (true si gira invertido)
const bool INVERT_MOTOR_A = false;
const bool INVERT_MOTOR_B = false; // Ajustado a false para que avance hacia el frente con Motor A

// ============================================================================
// 3. CONFIGURACIÓN PWM
// ============================================================================
const int canalPWM_A = 0;
const int canalPWM_B = 1;
const int frecuencia = 5000;
const int resolucion = 8; // 0 - 255

// ============================================================================
// 4. PARÁMETROS PID Y ESTADO DEL ROBOT
// ============================================================================
float Kp = 18.0f;
float Ki = 0.0f;
float Kd = 12.0f;

float error = 0.0f;
float lastError = 0.0f;
float lastKnownError = 0.0f;
float P = 0.0f, I = 0.0f, D = 0.0f;
float PID_value = 0.0f;

int baseSpeed = 35; // Velocidad base reducida para máxima estabilidad y control en curvas
int sensorValues[5] = {0, 0, 0, 0, 0};
bool enLinea = true;
bool robotActivo = false; // Comienza detenido por seguridad hasta recibir orden del celular

// Estado de sensores auxiliares BFD-1000
bool obstaculoDetectado = false;
bool choqueDetectado = false;

// Diagnóstico
bool invertirSensores = false;   // false para BFD-1000 normal (0 sobre blanco, 1 sobre cinta negra)
bool invertirGiro = true;        // Fijado en true por defecto para corregir siempre hacia adentro de la línea
int testMotor = 0;               // 0 = ninguno, 1 = motor A, 2 = motor B, 3 = ambos
unsigned long testHasta = 0;     // Fin de la prueba de motor (millis)
const int TEST_SPEED = 50;
const unsigned long TEST_DURATION_MS = 1500;

const float MAX_I = 150.0f;
const unsigned long CONTROL_INTERVAL_US = 5000; // 5 ms = 200 Hz
unsigned long lastControlTime = 0;

// ============================================================================
// 5. SERVIDORES RED (WI-FI, HTTP Y WEBSOCKET)
// ============================================================================
const char* ssid = "Robot_Competencia";
const char* password = "";

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
unsigned long lastSend = 0;

// ============================================================================
// PÁGINA WEB EMBEBIDA EN MEMORIA FLASH
// ============================================================================
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>Control Robot Seguidor</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    body { background-color: #0f172a; color: #f8fafc; padding: 14px; display: flex; flex-direction: column; align-items: center; }
    .container { width: 100%; max-width: 480px; display: flex; flex-direction: column; gap: 14px; }
    .card { background: #1e293b; border-radius: 12px; padding: 16px; border: 1px solid #334155; }
    .header { text-align: center; }
    .header h1 { font-size: 1.25rem; font-weight: 700; color: #38bdf8; }
    .status-badge { display: inline-flex; align-items: center; gap: 6px; padding: 4px 12px; border-radius: 9999px; font-size: 0.8rem; font-weight: 600; margin-top: 6px; }
    .status-connected { background: #064e3b; color: #34d399; }
    .status-disconnected { background: #7f1d1d; color: #f87171; }
    .dot { width: 8px; height: 8px; border-radius: 50%; }
    .dot-connected { background: #10b981; }
    .dot-disconnected { background: #ef4444; }

    /* Botón de Marcha / Paro */
    .btn-run { width: 100%; padding: 16px; border: none; border-radius: 10px; font-size: 1.1rem; font-weight: 800; cursor: pointer; transition: all 0.2s; }
    .btn-start { background: #10b981; color: #022c22; }
    .btn-stop { background: #ef4444; color: #450a0a; }

    /* Sensores */
    .sensors-container { display: flex; justify-content: space-between; gap: 8px; margin: 10px 0; }
    .sensor-box { flex: 1; height: 46px; border-radius: 8px; border: 2px solid #475569; display: flex; align-items: center; justify-content: center; font-size: 0.85rem; font-weight: 700; background: #0f172a; color: #64748b; }
    .sensor-box.active { background: #38bdf8; color: #0f172a; border-color: #7dd3fc; box-shadow: 0 0 10px #38bdf8; }

    /* Sensores Auxiliares / Seguridad */
    .aux-container { display: flex; gap: 8px; margin-top: 8px; }
    .aux-box { flex: 1; padding: 10px; border-radius: 8px; border: 2px solid #475569; display: flex; flex-direction: column; align-items: center; justify-content: center; font-size: 0.8rem; font-weight: 700; background: #0f172a; color: #94a3b8; transition: all 0.2s; }
    .aux-box.warning { background: #78350f; color: #fde68a; border-color: #f59e0b; box-shadow: 0 0 10px #f59e0b; }
    .aux-box.danger  { background: #7f1d1d; color: #fecaca; border-color: #ef4444; box-shadow: 0 0 10px #ef4444; }

    .metrics-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-top: 8px; }
    .metric-card { background: #0f172a; padding: 8px; border-radius: 8px; text-align: center; border: 1px solid #334155; }
    .metric-label { font-size: 0.7rem; color: #94a3b8; }
    .metric-value { font-size: 1.15rem; font-weight: 700; color: #f1f5f9; }

    /* Sliders */
    .control-group { margin-bottom: 12px; }
    .control-label { display: flex; justify-content: space-between; font-size: 0.85rem; font-weight: 600; margin-bottom: 4px; }
    .control-label span:last-child { color: #38bdf8; font-family: monospace; }
    input[type=range] { width: 100%; height: 6px; border-radius: 3px; background: #334155; outline: none; -webkit-appearance: none; }
    input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; width: 22px; height: 22px; border-radius: 50%; background: #38bdf8; cursor: pointer; }
    .btn-save { width: 100%; background: #0284c7; color: white; border: none; padding: 12px; border-radius: 8px; font-weight: 700; font-size: 0.95rem; cursor: pointer; margin-top: 4px; }
  </style>
</head>
<body>
  <div class="container">
    <div class="card header">
      <h1>Robot Seguidor de Línea</h1>
      <div id="status" class="status-badge status-disconnected">
        <span class="dot dot-disconnected" id="dot"></span>
        <span id="status-text">Conectando WebSocket...</span>
      </div>
    </div>

    <!-- Botón Principal: Iniciar / Detener Motores -->
    <button id="btn-toggle" class="btn-run btn-start" onclick="toggleRobot()">INICIAR ROBOT (ARRANCAR)</button>

    <!-- Sensores en vivo -->
    <div class="card">
      <div style="font-size: 0.8rem; color: #94a3b8; font-weight: 600;">ESTADO DE SENSORES</div>
      <div class="sensors-container">
        <div class="sensor-box" id="s0">S1</div>
        <div class="sensor-box" id="s1">S2</div>
        <div class="sensor-box" id="s2">S3</div>
        <div class="sensor-box" id="s3">S4</div>
        <div class="sensor-box" id="s4">S5</div>
      </div>
      <div class="metrics-grid">
        <div class="metric-card">
          <div class="metric-label">Error Calculado</div>
          <div class="metric-value" id="val-error">0.00</div>
        </div>
        <div class="metric-card">
          <div class="metric-label">Corrección PID</div>
          <div class="metric-value" id="val-pid">0.00</div>
        </div>
      </div>
    </div>

    <!-- Sensores de Seguridad BFD-1000 (NEAR y CLP) -->
    <div class="card">
      <div style="font-size: 0.8rem; color: #94a3b8; font-weight: 600;">SEGURIDAD FRONTAL (BFD-1000)</div>
      <div class="aux-container">
        <div class="aux-box" id="badge-near">
          <span style="font-size:0.7rem; color:#64748b;">OBSTÁCULO (NEAR - D19)</span>
          <span id="txt-near" style="font-size:0.95rem; margin-top:3px;">DESPEJADO</span>
        </div>
        <div class="aux-box" id="badge-clp">
          <span style="font-size:0.7rem; color:#64748b;">CHOQUE (CLP - D21)</span>
          <span id="txt-clp" style="font-size:0.95rem; margin-top:3px;">LIBRE</span>
        </div>
      </div>
    </div>

    <!-- Sliders de Calibración -->
    <div class="card">
      <div style="font-size: 0.8rem; color: #94a3b8; font-weight: 600; margin-bottom: 10px;">AJUSTE PID EN VIVO</div>
      
      <div class="control-group">
        <div class="control-label"><span>Proporcional (Kp)</span><span id="lbl-kp">18.0</span></div>
        <input type="range" id="input-kp" min="0" max="100" step="0.5" value="18.0" oninput="document.getElementById('lbl-kp').innerText = this.value; enviarParametros();">
      </div>

      <div class="control-group">
        <div class="control-label"><span>Integral (Ki)</span><span id="lbl-ki">0.0</span></div>
        <input type="range" id="input-ki" min="0" max="5" step="0.05" value="0.0" oninput="document.getElementById('lbl-ki').innerText = this.value; enviarParametros();">
      </div>

      <div class="control-group">
        <div class="control-label"><span>Derivativo (Kd)</span><span id="lbl-kd">12.0</span></div>
        <input type="range" id="input-kd" min="0" max="50" step="0.5" value="12.0" oninput="document.getElementById('lbl-kd').innerText = this.value; enviarParametros();">
      </div>

      <div class="control-group">
        <div class="control-label"><span>Velocidad Base (PWM)</span><span id="lbl-speed">35</span></div>
        <input type="range" id="input-speed" min="0" max="255" step="1" value="35" oninput="document.getElementById('lbl-speed').innerText = this.value; enviarParametros();">
      </div>

      <button class="btn-save" onclick="enviarParametros()">APLICAR PARÁMETROS</button>
    </div>

    <!-- Diagnóstico -->
    <div class="card">
      <div style="font-size: 0.8rem; color: #94a3b8; font-weight: 600; margin-bottom: 10px;">DIAGNÓSTICO</div>
      <div style="display:grid; grid-template-columns:1fr 1fr 1fr; gap:8px;">
        <button class="btn-save" style="background:#7c3aed" onclick="probarMotor(1)">MOTOR A</button>
        <button class="btn-save" style="background:#7c3aed" onclick="probarMotor(2)">MOTOR B</button>
        <button class="btn-save" style="background:#7c3aed" onclick="probarMotor(3)">AMBOS</button>
      </div>
      <label style="display:flex; align-items:center; gap:10px; margin-top:14px; font-size:0.85rem;">
        <input type="checkbox" id="chk-invertir" style="width:22px; height:22px;" onchange="enviarInvertir(this.checked)">
        Invertir sensores (marcar solo si tu pista es blanca sobre fondo negro)
      </label>
      <label style="display:flex; align-items:center; gap:10px; margin-top:14px; font-size:0.88rem; font-weight:700; color:#38bdf8;">
        <input type="checkbox" id="chk-inv-giro" checked style="width:22px; height:22px;" onchange="enviarInvertirGiro(this.checked)">
        🔄 Invertir Dirección de Giro (marcar si huye de la línea)
      </label>
    </div>
  </div>

  <script>
    let socket = null;
    let robotCorriendo = false;
    const wsUrl = "ws://" + location.hostname + ":81/";

    function conectarWebSocket() {
      const statusBadge = document.getElementById("status");
      const statusText = document.getElementById("status-text");
      const dot = document.getElementById("dot");

      socket = new WebSocket(wsUrl);

      socket.onopen = () => {
        statusBadge.className = "status-badge status-connected";
        dot.className = "dot dot-connected";
        statusText.innerText = "Robot Enlazado";
      };

      socket.onclose = () => {
        statusBadge.className = "status-badge status-disconnected";
        dot.className = "dot dot-disconnected";
        statusText.innerText = "Reconectando...";
        setTimeout(conectarWebSocket, 1500);
      };

      socket.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (data.sensores && Array.isArray(data.sensores)) {
            for (let i = 0; i < 5; i++) {
              const el = document.getElementById('s' + i);
              if (data.sensores[i] === 1) el.classList.add('active');
              else el.classList.remove('active');
            }
          }
          if (data.error !== undefined) document.getElementById('val-error').innerText = Number(data.error).toFixed(2);
          if (data.PID !== undefined) document.getElementById('val-pid').innerText = Number(data.PID).toFixed(1);
          if (data.invertido !== undefined) document.getElementById('chk-invertir').checked = data.invertido;
          if (data.invGiro !== undefined) document.getElementById('chk-inv-giro').checked = data.invGiro;
          if (data.activo !== undefined) {
            robotCorriendo = data.activo;
            actualizarBotonEstado();
          }
          if (data.obstaculo !== undefined) {
            const elNear = document.getElementById('badge-near');
            const txtNear = document.getElementById('txt-near');
            if (data.obstaculo) {
              elNear.className = 'aux-box warning';
              txtNear.innerText = '¡OBSTÁCULO!';
            } else {
              elNear.className = 'aux-box';
              txtNear.innerText = 'DESPEJADO';
            }
          }
          if (data.choque !== undefined) {
            const elClp = document.getElementById('badge-clp');
            const txtClp = document.getElementById('txt-clp');
            if (data.choque) {
              elClp.className = 'aux-box danger';
              txtClp.innerText = '¡IMPACTO!';
            } else {
              elClp.className = 'aux-box';
              txtClp.innerText = 'LIBRE';
            }
          }
        } catch(e) {}
      };
    }

    function toggleRobot() {
      if (!socket || socket.readyState !== WebSocket.OPEN) return;
      robotCorriendo = !robotCorriendo;
      actualizarBotonEstado();
      socket.send(JSON.stringify({ activo: robotCorriendo }));
    }

    function actualizarBotonEstado() {
      const btn = document.getElementById("btn-toggle");
      if (robotCorriendo) {
        btn.className = "btn-run btn-stop";
        btn.innerText = "DETENER ROBOT (PARADA)";
      } else {
        btn.className = "btn-run btn-start";
        btn.innerText = "INICIAR ROBOT (ARRANCAR)";
      }
    }

    function probarMotor(n) {
      if (!socket || socket.readyState !== WebSocket.OPEN) { alert("Sin conexión al robot"); return; }
      socket.send(JSON.stringify({ test: n }));
    }

    function enviarInvertir(v) {
      if (!socket || socket.readyState !== WebSocket.OPEN) { alert("Sin conexión al robot"); return; }
      socket.send(JSON.stringify({ invertir: v }));
    }

    function enviarInvertirGiro(v) {
      if (!socket || socket.readyState !== WebSocket.OPEN) { alert("Sin conexión al robot"); return; }
      socket.send(JSON.stringify({ invGiro: v }));
    }

    function enviarParametros() {
      if (!socket || socket.readyState !== WebSocket.OPEN) {
        alert("Sin conexión al robot");
        return;
      }
      socket.send(JSON.stringify({
        Kp: parseFloat(document.getElementById('input-kp').value),
        Ki: parseFloat(document.getElementById('input-ki').value),
        Kd: parseFloat(document.getElementById('input-kd').value),
        baseSpeed: parseInt(document.getElementById('input-speed').value)
      }));
    }

    window.addEventListener("load", conectarWebSocket);
  </script>
</body>
</html>
)rawliteral";

// Declaración de prototipos
void leerSensores();
void calcularPID(float dt);
void controlMotores();
void setMotor(int canalPWM, int in1, int in2, int speed, bool reverse = false);
void enviarTelemetria();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

void setup() {
  Serial.begin(115200);

  // Configuración de pines de sensores
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  pinMode(PIN_NEAR, INPUT_PULLDOWN);
  pinMode(PIN_CLP, INPUT_PULLDOWN);

  // Configuración del driver TB6612FNG
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  if (STBY_PIN >= 0) {
    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, HIGH);
  }

  // Configuración PWM
  ledcSetup(canalPWM_A, frecuencia, resolucion);
  ledcAttachPin(PWMA, canalPWM_A);

  ledcSetup(canalPWM_B, frecuencia, resolucion);
  ledcAttachPin(PWMB, canalPWM_B);

  // Detener motores al inicio por seguridad
  setMotor(canalPWM_A, AIN1, AIN2, 0, INVERT_MOTOR_A);
  setMotor(canalPWM_B, BIN1, BIN2, 0, INVERT_MOTOR_B);

  // Iniciar Red Wi-Fi
  WiFi.softAP(ssid, password);
  Serial.println("Punto de Acceso iniciado");

  // Servidor Web HTTP (Puerto 80)
  server.on("/", []() {
    server.send(200, "text/html", PAGE_HTML);
  });
  server.begin();
  Serial.println("Servidor Web HTTP iniciado en puerto 80");

  // Servidor WebSockets (Puerto 81)
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("Servidor WebSocket iniciado en puerto 81");

  lastControlTime = micros();
}

void loop() {
  server.handleClient();
  webSocket.loop();

  unsigned long currentMicros = micros();
  if (currentMicros - lastControlTime >= CONTROL_INTERVAL_US) {
    float dt = (currentMicros - lastControlTime) / 1000000.0f;
    lastControlTime = currentMicros;

    leerSensores();

    // Verificación de Choque (Parada de Emergencia Absoluta)
    if (choqueDetectado && robotActivo) {
      robotActivo = false;
      setMotor(canalPWM_A, AIN1, AIN2, 0, INVERT_MOTOR_A);
      setMotor(canalPWM_B, BIN1, BIN2, 0, INVERT_MOTOR_B);
      Serial.println("¡EMERGENCIA: COLISIÓN DETECTADA POR CLP!");
    }

    if (testMotor != 0 && millis() < testHasta) {
      // Prueba de motores desde el celular
      setMotor(canalPWM_A, AIN1, AIN2, (testMotor & 1) ? baseSpeed : 0, INVERT_MOTOR_A);
      setMotor(canalPWM_B, BIN1, BIN2, (testMotor & 2) ? baseSpeed : 0, INVERT_MOTOR_B);
    } else if (robotActivo) {
      testMotor = 0;
      if (obstaculoDetectado) {
        // Pausa preventiva si el sensor frontal NEAR detecta un obstáculo
        setMotor(canalPWM_A, AIN1, AIN2, 0, INVERT_MOTOR_A);
        setMotor(canalPWM_B, BIN1, BIN2, 0, INVERT_MOTOR_B);
      } else {
        calcularPID(dt);
        controlMotores();
      }
    } else {
      // Si está detenido desde el celular o por choque, apagar motores
      setMotor(canalPWM_A, AIN1, AIN2, 0, INVERT_MOTOR_A);
      setMotor(canalPWM_B, BIN1, BIN2, 0, INVERT_MOTOR_B);
    }
  }

  if (millis() - lastSend > 100) {
    enviarTelemetria();
    lastSend = millis();
  }
}

void leerSensores() {
  sensorValues[0] = digitalRead(S1);
  sensorValues[1] = digitalRead(S2);
  sensorValues[2] = digitalRead(S3);
  sensorValues[3] = digitalRead(S4);
  sensorValues[4] = digitalRead(S5);

  // Sensores de seguridad BFD-1000: en reposo están en LOW (0V).
  // Se activan en nivel HIGH (cuando el microswitch se presiona o detecta obstáculo frontal)
  obstaculoDetectado = (digitalRead(PIN_NEAR) == HIGH);
  choqueDetectado    = (digitalRead(PIN_CLP) == HIGH);

  if (invertirSensores) {
    for (int i = 0; i < 5; i++) sensorValues[i] = !sensorValues[i];
  }

  float sumaPesos = 0.0f;
  int sumaActivos = 0;

  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] == 1) {
      sumaPesos += sensorWeights[i];
      sumaActivos++;
    }
  }

  if (sumaActivos > 0 && sumaActivos < 5) {
    error = sumaPesos / (float)sumaActivos;
    lastKnownError = error;
    enLinea = true;
  } else {
    enLinea = false;
  }
}

void calcularPID(float dt) {
  if (dt <= 0.0f) return;

  P = error;
  I += (error * dt);
  I = constrain(I, -MAX_I, MAX_I);

  // Reiniciar integral al cruzar el centro (anti-windup)
  if ((error >= 0.0f && lastError < 0.0f) || (error <= 0.0f && lastError > 0.0f) || abs(error) < 0.1f) {
    I = 0.0f;
  }

  // Derivada discreta directa: NO dividimos entre dt (0.005s) para evitar
  // que el ruido y los saltos de sensores digitales disparen el PID a valores gigantes
  D = error - lastError;
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  lastError = error;
}

void controlMotores() {
  // Si la velocidad base es 0, detener completamente los motores
  if (baseSpeed <= 0) {
    setMotor(canalPWM_A, AIN1, AIN2, 0, INVERT_MOTOR_A);
    setMotor(canalPWM_B, BIN1, BIN2, 0, INVERT_MOTOR_B);
    return;
  }

  int speedRight = 0;
  int speedLeft  = 0;

  if (!enLinea) {
    // ========================================================================
    // Si pierde la línea, avanza RECTO hacia adelante buscando la pista.
    // ¡CERO círculos ni trompos! Ambos motores avanzan a la misma velocidad.
    // ========================================================================
    speedRight = baseSpeed;
    speedLeft  = baseSpeed;
  } else {
    // Modo seguimiento normal con control PID
    speedRight = baseSpeed - (int)PID_value;
    speedLeft  = baseSpeed + (int)PID_value;

    int maxPermitido = constrain(baseSpeed + 50, 0, 255);
    speedRight = constrain(speedRight, 0, maxPermitido);
    speedLeft  = constrain(speedLeft, 0, maxPermitido);
  }

  // Si se activa invertir giro desde el celular, intercambia ruedas de inmediato
  int finalLeft  = invertirGiro ? speedRight : speedLeft;
  int finalRight = invertirGiro ? speedLeft  : speedRight;

  // Motor A = Rueda Izquierda (finalLeft)
  // Motor B = Rueda Derecha   (finalRight)
  setMotor(canalPWM_A, AIN1, AIN2, finalLeft, INVERT_MOTOR_A);
  setMotor(canalPWM_B, BIN1, BIN2, finalRight, INVERT_MOTOR_B);
}

void setMotor(int canalPWM, int in1, int in2, int speed, bool reverse) {
  int effSpeed = reverse ? -speed : speed;

  if (effSpeed > 0) {
    // Sentido de avance hacia el frente
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(canalPWM, effSpeed);
  } else if (effSpeed < 0) {
    // Sentido de retroceso
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(canalPWM, -effSpeed);
  } else {
    // Detenido / Freno
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(canalPWM, 0);
  }
}

void enviarTelemetria() {
  StaticJsonDocument<384> doc;
  JsonArray sensores = doc.createNestedArray("sensores");
  for (int i = 0; i < 5; i++) {
    sensores.add(sensorValues[i]);
  }
  doc["error"] = error;
  doc["PID"] = PID_value;
  doc["enLinea"] = enLinea;
  doc["activo"] = robotActivo;
  doc["invertido"] = invertirSensores;
  doc["invGiro"] = invertirGiro;
  doc["obstaculo"] = obstaculoDetectado;
  doc["choque"] = choqueDetectado;

  String jsonString;
  serializeJson(doc, jsonString);
  webSocket.broadcastTXT(jsonString);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    // 1. Imprimir el mensaje crudo recibido para depuración
    Serial.printf("Mensaje recibido: %s\n", payload);

    // 2. Crear un documento JSON temporal en la memoria
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload);

    // 3. Verificar si el JSON es válido
    if (error) {
      Serial.print(F("Error al leer JSON: "));
      Serial.println(error.f_str());
      return;
    }

    // 4. Botón INICIAR / DETENER del panel
    if (doc.containsKey("activo")) {
      robotActivo = doc["activo"].as<bool>();
      if (!robotActivo) { I = 0.0f; lastError = 0.0f; }
      Serial.printf("Estado del robot: %s\n", robotActivo ? "ACTIVO" : "DETENIDO");
    }

    // 5. Diagnóstico: invertir lógica de sensores, invertir giro y probar motores
    if (doc.containsKey("invertir")) {
      invertirSensores = doc["invertir"].as<bool>();
      Serial.printf("Sensores invertidos: %s\n", invertirSensores ? "SI" : "NO");
    }
    if (doc.containsKey("invGiro")) {
      invertirGiro = doc["invGiro"].as<bool>();
      Serial.printf("Giro invertido: %s\n", invertirGiro ? "SI" : "NO");
    }
    if (doc.containsKey("test")) {
      testMotor = doc["test"].as<int>();
      testHasta = millis() + TEST_DURATION_MS;
      Serial.printf("Prueba de motor: %d\n", testMotor);
    }

    // 6. Actualizar las variables globales del PID (acepta "Kp" del panel o "kp")
    if (doc.containsKey("Kp")) Kp = doc["Kp"].as<float>();
    if (doc.containsKey("Ki")) Ki = doc["Ki"].as<float>();
    if (doc.containsKey("Kd")) Kd = doc["Kd"].as<float>();
    if (doc.containsKey("kp")) Kp = doc["kp"].as<float>();
    if (doc.containsKey("ki")) Ki = doc["ki"].as<float>();
    if (doc.containsKey("kd")) Kd = doc["kd"].as<float>();

    // Velocidad base enviada por el panel
    if (doc.containsKey("baseSpeed")) baseSpeed = doc["baseSpeed"].as<int>();
    if (doc.containsKey("velBase")) baseSpeed = doc["velBase"].as<int>();

    // 7. Imprimir los nuevos valores para confirmar la actualización
    Serial.printf("Kp: %.2f | Ki: %.2f | Kd: %.2f | baseSpeed: %d\n", Kp, Ki, Kd, baseSpeed);
  }
}