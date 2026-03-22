# Archery Simulation with MMA8451

Real-time archery target simulation using MMA8451 accelerometer data.

## Components

1. **Arduino Code** (`arduino/archery_sensor.ino`) - Sensor code with piezo buzzer
2. **Web Simulation** (`index.html`) - Browser-based archery target visualization

## Hardware Setup

| MMA8451 | Arduino |
|---------|---------|
| VCC | 3.3V |
| GND | GND |
| SDA | A4 |
| SCL | A5 |

| Component | Pin |
|-----------|-----|
| Passive Piezo Buzzer | 8 |

## Arduino Libraries Required

- Adafruit MMA8451
- Adafruit Sensor

## Usage

### 1. Arduino Setup

1. Open `arduino/archery_sensor.ino` in Arduino IDE
2. Install libraries via Library Manager
3. Upload to your Arduino

### 2. Web Interface

1. Open `index.html` in Chrome or Edge (Web Serial API required)
2. Select baud rate (115200 recommended)
3. Click **Connect** to connect via serial
4. Or click **Demo** to simulate without hardware

### 3. Calibration

- Hold bow steady for 10 seconds
- System auto-calibrates (triple beep confirms)
- Crosshair turns yellow when on target

## Audio Feedback

- **Single ding** - On target (deviation < 5°)
- **Continuous tone** - Off target (deviation > 15°)
- **Triple beep** - New calibration set

## Serial Data Format (JSON)

```json
{"P":12.34,"R":-5.67,"PD":12.34,"RD":-5.67,"DEV":13.67,"ONTARGET":0,"CAL":1}
```

| Field | Description |
|-------|-------------|
| P | Pitch angle |
| R | Roll angle |
| PD | Pitch deviation from reference |
| RD | Roll deviation from reference |
| DEV | Total deviation (combined) |
| ONTARGET | 1 if on target, 0 otherwise |
| CAL | 1 if calibrated, 0 otherwise |

## Controls

- **Shoot button** or **Spacebar** - Record a shot
- **Reset** - Clear all shot data
- **Debug** - View serial data log
- **Demo** - Mouse-based simulation mode
