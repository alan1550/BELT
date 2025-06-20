# BELT

## Project Overview

C++ and Python code for Accelerometer used in belt design project.

## Structure

- `cpp/`: C++/Arduino code for ESP32 (BNO055 sensor, BLE)
- `python/`: Python scripts for BLE communication, data processing, etc.

## Getting Started

### Clone the repository

```bash
git clone https://github.com/alan1550/BELT.git
cd BELT
```

---

### C++ Code (ESP32 + BNO055 + BLE)

**Location:** `cpp/`

**Typical board:** ESP32

**Example:** `cpp/main.cpp`

Edit or add more files as needed!

#### Compile & Upload

Open in Arduino IDE or PlatformIO. Install required libraries:
- Adafruit BNO055
- Adafruit Unified Sensor
- BLE (ESP32)

---

### Python Code

**Location:** `python/`

- `ble_receive.py` – Receives and prints BLE data from ESP32.
- Add your other Python scripts here (e.g., data processing, visualization).

**Install dependencies:**
```bash
pip install bleak
```

**Run BLE receiver:**
```bash
python python/ble_receive.py
```

---

## Example Code

### C++ (cpp/main.cpp)

```cpp
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// === BNO055 Setup ===
#define SDA_PIN 8
#define SCL_PIN 9
#define BNO055_ADDR 0x28  // or 0x29 if address jumper is closed
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDR, &Wire);

// === BLE UART Setup ===
BLECharacteristic* pTxCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("✅ BLE client connected");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("🔌 BLE client disconnected");
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("🔧 Starting BLE UART + BNO055...");

  // === I2C + BNO055 ===
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!bno.begin()) {
    Serial.println("❌ BNO055 not detected. Check wiring or I2C address.");
    while (1);
  }

  delay(1000);  // Wait for sensor to stabilize
  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_NDOF);

  // === BLE Setup ===
  BLEDevice::init("ESP32-BNO055-BLE");
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

  pTxCharacteristic = pService->createCharacteristic(
    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pTxCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("📡 BLE advertising started as 'ESP32-BNO055-BLE'");
}

void loop() {
  if (!deviceConnected) return;

  // Read fused sensor data
  sensors_event_t event;
  bno.getEvent(&event);

  String payload = String(event.acceleration.x, 2) + "," +
                   String(event.acceleration.y, 2) + "," +
                   String(event.acceleration.z, 2) + "," +
                   String(event.gyro.x, 2) + "," +
                   String(event.gyro.y, 2) + "," +
                   String(event.gyro.z, 2) + "," +
                   String(event.magnetic.x, 2) + "," +
                   String(event.magnetic.y, 2) + "," +
                   String(event.magnetic.z, 2) + "," +
                   String(event.orientation.x, 2) + "," +
                   String(event.orientation.y, 2) + "," +
                   String(event.orientation.z, 2);

  pTxCharacteristic->setValue(payload.c_str());
  pTxCharacteristic->notify();

  Serial.println("BLE sent: " + payload);
  delay(1000 / 30);  // 30 Hz sampling
}
```

---

### Python (python/ble_receive.py)

```python
import asyncio
from bleak import BleakScanner, BleakClient

# UUIDs for Nordic UART Service (NUS)
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # ESP32 → PC

def handle_rx(_, data):
    print("Received:", data.decode().strip())

async def main():
    print("🔍 Scanning for BLE devices...")
    devices = await BleakScanner.discover(timeout=5.0)
    
    target = None
    for d in devices:
        if d.name and "ESP32-BNO055-BLE" in d.name:
            target = d
            break

    if not target:
        print("❌ ESP32 not found. Make sure it's powered and advertising.")
        return

    print(f"✅ Found {target.name} at {target.address}")
    async with BleakClient(target.address) as client:
        print("🔗 Connected. Subscribing to notifications...")
        await client.start_notify(UART_RX_CHAR_UUID, handle_rx)

        print("📡 Listening for data (press Ctrl+C to stop)...")
        while True:
            await asyncio.sleep(1)

asyncio.run(main())
```

---

## Add More Python Code

Put your additional Python files/scripts in the `python/` folder.  
For example:

- `python/data_processing.py`
- `python/visualization.py`

Document each script at the top with a short description!

---

## .gitignore

```gitignore
# C++ build files
*.o
*.out
*.exe

# Python
__pycache__/
*.pyc

# VSCode or JetBrains settings
.vscode/
.idea/

# General
.DS_Store
```

---

## Contributing

Pull requests are welcome. For major changes, please open an issue first.

## License

[MIT](LICENSE)

---