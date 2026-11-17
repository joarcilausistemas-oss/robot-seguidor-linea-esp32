# 🏎️ Robot Seguidor de Línea de Competencia (ESP32)

Proyecto de robótica móvil diferencial basado en **ESP32**, driver **TB6612FNG**, módulo de 5 sensores ópticos **BFD-1000** con detección de obstáculos infrarroja frontal (`NEAR`) y microswitch de colisión (`CLP`).

Incluye un bucle de control **PID en tiempo real (200 Hz)**, servidor web embebido y telemetría por **WebSockets** para calibración inalámbrica desde cualquier smartphone o computadora sin cables.

---

## 📋 Tabla de Conexiones (Pinout)

### 1. Driver de Motores TB6612FNG
| Pin Driver | Pin ESP32 (Expansion Board) | Función |
| :--- | :--- | :--- |
| **PWMA** | `D18` (GPIO 18) | Velocidad Motor Derecho (PWM 5 kHz, 8 bits) |
| **AIN1** | `D14` (GPIO 14) | Dirección Motor Derecho |
| **AIN2** | `D27` (GPIO 27) | Dirección Motor Derecho |
| **PWMB** | `D13` (GPIO 13) | Velocidad Motor Izquierdo (PWM 5 kHz, 8 bits) |
| **BIN1** | `D26` (GPIO 26) | Dirección Motor Izquierdo |
| **BIN2** | `D25` (GPIO 25) | Dirección Motor Izquierdo |
| **STBY** | `3.3V` / VCC | Habilitación permanente del puente H |
| **VM** | `(+) Batería 7.4V` | Alimentación de potencia de motores (a través de interruptor) |
| **VCC** | `3.3V` | Alimentación lógica del driver |
| **GND** | `GND` | Tierra común obligatoria (Batería, ESP32, Driver, Sensores) |

### 2. Módulo de Sensores BFD-1000
| Pin Sensor | Pin ESP32 | Descripción |
| :--- | :--- | :--- |
| **S1** | `D15` (GPIO 15) | Sensor extremo izquierdo |
| **S2** | `D2`  (GPIO 2)  | Sensor izquierdo |
| **S3** | `D4`  (GPIO 4)  | Sensor centro (línea guía) |
| **S4** | `D16` (GPIO 16) | Sensor derecho |
| **S5** | `D17` (GPIO 17) | Sensor extremo derecho |
| **NEAR** | `D19` (GPIO 19) | Detección infrarroja de obstáculo frontal (Pausa preventiva) |
| **CLP** | `D21` (GPIO 21) | Microswitch parachoques de choque (Parada de emergencia) |
| **VCC** | `3.3V` o `5V` | Alimentación de sensores |
| **GND** | `GND` | Tierra común |

---

## 🚀 Cómo Empezar y Calibrar

### 1. Abrir en VS Code / PlatformIO
1. Clona este repositorio:
   ```bash
   git clone https://github.com/joarcilausistemas-oss/robot_competencia.git
   ```
2. Abre la carpeta `robot_competencia` en **Visual Studio Code**.
3. Asegúrate de tener instalada la extensión **PlatformIO IDE**.
4. Conecta la placa ESP32 por cable USB directamente al puerto nativo del microcontrolador.
5. Haz clic en el botón de **Upload ($\rightarrow$)** en la barra inferior azul de PlatformIO.

---

### 2. Calibración en Vivo por Wi-Fi
1. Enciende la batería del robot.
2. Desde tu smartphone o laptop, conéctate a la red Wi-Fi:
   * **SSID:** `Robot_Competencia`
   * **Contraseña:** *(abierta, sin contraseña)*
3. Abre el navegador web e ingresa a:
   ```
   http://192.168.4.1
   ```
4. **Funcionalidades del Panel Web:**
   * **Estado de Sensores en Vivo:** Muestra gráficamente en tiempo real qué sensores (`S1..S5`) están sobre la línea negra.
   * **Seguridad Frontal:** Muestra el estado del sensor `NEAR` (Obstáculo) y `CLP` (Choque).
   * **Sliders PID:** Permite ajustar `Kp`, `Ki`, `Kd` y la `Velocidad Base` en caliente sin necesidad de reprogramar el microcontrolador.
   * **Prueba de Motores:** Botones de diagnóstico para probar giro independiente de Motor A, Motor B o Ambos.
   * **Invertir Sensores:** Casilla para cambiar entre pista negra sobre fondo blanco o viceversa.

---

## 🛠️ Guía Rápida para el Ajuste PID

1. **Ajuste inicial:** Coloca `Ki = 0.0` y `Kd = 0.0`.
2. **Ajustar Kp (Proporcional):** Aumenta `Kp` gradualmente (ej. 15.0 a 25.0) hasta que el robot siga la línea en curvas suaves con un ligero bamboleo.
3. **Ajustar Kd (Derivativo):** Aumenta `Kd` (ej. 8.0 a 16.0) para amortiguar el bamboleo y estabilizar el robot a la salida de las curvas rápidas.
4. **Velocidad Base:** Ve aumentando la velocidad base de 5 en 5 desde el slider una vez que el trazado sea estable.
