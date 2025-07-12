# DPilot Flight Telemetry System - Complete Aircraft Integration
## IO List and System Integration Guide

## System Architecture Overview

**Aircraft Platform**: VolantexRC Ranger 1600 V757-7 (1600mm wingspan) or Ranger 2000 wings  
**Mission Computer**: Raspberry Pi Zero 2W with Sony IMX500 AI Camera for visual odometry  
**Telemetry System**: ESP32 LoRa modules with automatic role detection and custom 3D printed housings  
- **Drone ESP32**: Custom 2-layer PCB with integrated MPU6050, SD card, and LoRa (aircraft-mounted)
- **Base Station ESP32**: Commercial Heltec module with TFT display (ground station, USB powered)

---

## Aircraft Platform - VolantexRC Ranger 1600/2000 Integration

### **Aircraft Specifications**
| Parameter | Value | Notes |
|-----------|-------|-------|
| Wingspan | 1600mm (Ranger 1600) / 2000mm (Ranger 2000) | EPO foam construction |
| Length | 1075mm | Pusher configuration |
| Wing Area | 26.5dm² (Ranger 1600) / 32.8dm² (Ranger 2000) | High aspect ratio |
| Flying Weight | 1200-1500g | With telemetry system |
| Power System | Electric pusher | Rear-mounted motor |

### **Propulsion System**
| Component | Specification | Connection |
|-----------|---------------|------------|
| **Motor** | D3536 Brushless 1450KV | 3.5mm bullet connectors |
| **Propeller** | Master Airscrew 8x6 3-blade | Pusher/CW rotation |
| **ESC** | 60A Brushless with 4A UBEC | XT60 power, servo connectors |
| **Battery** | LiPo 3S-4S (user supplied) | XT60 connector |

### **Aircraft Power Distribution**
```
LiPo Battery (11.1V-14.8V)
├─ XT60 Connector
├─ ESC (60A) ──────→ D3536 Motor (Custom 3D Mount)
│   └─ UBEC (4A) ──→ 5V Bus
│       ├─ Servo power (if flight controller used)
│       └─ Telemetry system power input
└─ Power Monitor (optional)
```

### **Aircraft Mounting Layout (Custom 3D Printed Integration)**
```
VolantexRC Ranger 1600/2000 - Optimized Visual Odometry Layout:

Nose Section:
├─ Pi AI Camera (downward + 15° forward)
│   ├─ Visual odometry capability with Sony IMX500
│   ├─ 12MP high-resolution imaging
│   └─ AI processing for object detection/tracking
├─ GPS Antenna (clear sky view)
└─ Custom 3D printed camera mount

Center Fuselage:
├─ Raspberry Pi Zero 2W (custom 3D printed bucket)
├─ Custom PCB ESP32 Module (custom 3D printed bucket)
├─ Power distribution board
├─ LM2596 Buck converter (single unit)
└─ Battery bay (LiPo pack)

Wing Configuration Options:
├─ Ranger 1600 wings (standard speed)
├─ Ranger 2000 wings (slow flight + flaps)
│   ├─ Enhanced slow speed capabilities
│   ├─ Flap controls for approach/landing
│   └─ Improved stall characteristics
└─ ADC sensor integration (wing loading)

Pusher Motor Section:
├─ D3536 Motor (custom 3D printed mount)
├─ ESC mounting (optimized airflow)
├─ Vibration isolation (3D printed dampeners)
└─ Propeller clearance optimization

Tail Section:
├─ LoRa antenna (915MHz)
├─ WiFi antenna (Pi Zero 2W)
└─ Servo connections (elevator, rudder, flaps)
```

---

## Custom Drone PCB Design (JLCPCB 2-Layer)

### **PCB Specifications**
| Parameter | Value | Notes |
|-----------|-------|-------|
| **Manufacturer** | JLCPCB | Professional PCB fabrication |
| **Layer Count** | 2-layer | Cost-effective design |
| **Board Size** | ~50x40mm | Fits in 3D printed bucket |
| **Thickness** | 1.6mm | Standard thickness |
| **Surface Finish** | HASL/ENIG | Lead-free, RoHS compliant |

### **PCB Component Integration**
| Component | Placement | Interface | Notes |
|-----------|-----------|-----------|-------|
| **ESP32-S3** | Center | Main controller | WiFi + Bluetooth capability |
| **SX1262 LoRa** | Edge placement | SPI interface | 915MHz transceiver |
| **MPU6050** | Vibration isolated | I2C interface | 6-axis IMU |
| **SD Card Module** | Corner | SPI interface | MicroSD storage |
| **Power Management** | Distributed | Buck converters | 5V→3.3V regulation |
| **Connectors** | Edge | JST/Pin headers | External connections |

### **SD Card Module Integration**
**Part**: 10Pcs Micro Mini SD Storage Expansion Board (AliExpress)  
**Features**:
- MicroSD TF card socket with pins
- SPI interface for Arduino compatibility
- 3.3V/5V compatible operation
- Compact form factor for PCB integration

### **Custom PCB Pinout**
```
ESP32-S3 Custom PCB Connections:

Power:
├─ VIN: 5V input from LM2596 buck converter
├─ 3V3: 3.3V regulated output
├─ GND: Common ground plane
└─ EN: Enable pin with pull-up

LoRa SX1262 (SPI):
├─ SCK: GPIO18 (SPI clock)
├─ MISO: GPIO19 (SPI data in)
├─ MOSI: GPIO23 (SPI data out)
├─ CS: GPIO5 (Chip select)
├─ RST: GPIO14 (Reset)
├─ DIO1: GPIO26 (IRQ)
└─ ANT: SMA connector for 915MHz antenna

MPU6050 (I2C):
├─ SDA: GPIO21 (I2C data)
├─ SCL: GPIO22 (I2C clock)
├─ INT: GPIO27 (Interrupt, optional)
└─ VCC: 3.3V from onboard regulator

SD Card Module (SPI):
├─ SCK: GPIO18 (Shared with LoRa)
├─ MISO: GPIO19 (Shared with LoRa)
├─ MOSI: GPIO23 (Shared with LoRa)
├─ CS: GPIO4 (Dedicated chip select)
└─ VCC: 3.3V from onboard regulator

External Connections:
├─ GPS UART: GPIO16 (RX), GPIO17 (TX)
├─ ADC Inputs: GPIO32-GPIO39 (7 channels)
├─ Status LED: GPIO2 (Built-in LED)
├─ User Button: GPIO0 (Boot button for logging)
└─ Debug UART: GPIO1 (TX), GPIO3 (RX)

External Connectors:
├─ Pi Connection: USB-C connector (data + power)
├─ GPS Module: 4-pin JST connector
├─ ADC Sensors: 8-pin terminal block
├─ Power Input: 2-pin JST connector (5V from buck)
└─ Programming: USB-C (built-in ESP32-S3)
```

### **PCB Design Considerations**
```
Layer 1 (Top):
├─ Component placement and routing
├─ Power traces (thick copper for 5V/3.3V)
├─ High-speed signals (SPI, I2C)
└─ RF considerations for LoRa

Layer 2 (Bottom):
├─ Ground plane (continuous)
├─ Power plane sections
├─ Return paths for high-frequency signals
└─ Via stitching for EMI reduction

Design Rules:
├─ Minimum trace width: 0.1mm (4mil)
├─ Minimum via size: 0.2mm (8mil)
├─ Impedance control for RF traces
├─ Keep digital and analog sections separate
└─ Proper decoupling capacitor placement
```

---

## Mission Computer - Raspberry Pi Zero 2 W Integration

### **Raspberry Pi Zero 2 W Specifications**
| Parameter | Value | Notes |
|-----------|-------|-------|
| **CPU** | ARM Cortex-A53 1GHz quad-core | Sufficient for 50Hz processing |
| **RAM** | 512MB LPDDR2 | Adequate for flight operations |
| **Storage** | MicroSD 32GB+ Class 10 | High-speed for video recording |
| **Power** | 5V @ 1A max | Custom micro USB power cable |
| **WiFi** | 802.11n 2.4GHz | Built-in antenna |
| **Bluetooth** | 4.2 BLE | Available for expansion |

### **Pi AI Camera Module Integration (Visual Odometry)**
| Parameter | Value | Connection | Notes |
|-----------|-------|------------|-------|
| **Sensor** | Sony IMX500 | 15-pin CSI ribbon | AI-enabled vision sensor |
| **Resolution** | 12MP (4056x3040) | CSI-2 interface | High resolution for precision |
| **AI Processing** | On-sensor | Built-in neural network | Real-time object detection |
| **Frame Rate** | 60fps @ 2028x1520 | Variable modes | Optimized for visual odometry |
| **Mounting** | Downward + 15° forward | Custom 3D printed mount | Visual odometry orientation |
| **Focus** | Auto-focus | Motorized lens | 10cm to infinity |
| **Field of View** | 66° diagonal | Wide angle coverage | Ground feature tracking |

**AI Camera Capabilities:**
- **Visual Odometry**: Real-time position tracking from ground features
- **Object Detection**: AI-powered obstacle and landmark recognition  
- **High-Speed Imaging**: 60fps for smooth motion tracking
- **On-Sensor Processing**: Reduces Pi CPU load for other tasks
- **Neural Network**: Pre-trained models for aviation applications

### **Custom Cable Configuration**
| Cable Type | Connector | Purpose | Length |
|------------|-----------|---------|--------|
| **Pi Power** | Micro USB Male (custom) | 5V power input | 150mm |
| **ESP32 Drone** | USB-C Male (custom) | Unified state data | 200mm |
| **ESP32 Base** | USB-C Male (custom) | Command relay | 200mm |
| **Pi Camera** | CSI ribbon | Video recording | 100mm |

---

## Drone ESP32 Module (Custom PCB - Aircraft-Mounted)

### **Sensor Integration (Aircraft-Specific)**
```
IMU Integration (On-PCB MPU6050):
├─ Aircraft attitude/acceleration measurement
├─ Vibration isolation via PCB mounting in 3D bucket
├─ Axis alignment: X: forward, Y: right wing, Z: down
└─ Factory calibration with field offset correction

GPS Module (External):
├─ Serial UART interface
├─ Antenna placement: Top of fuselage, clear sky view
├─ Update rate: 5-10Hz navigation grade
└─ Accuracy: <3m typical, <1m with SBAS

Analog Sensors (Aircraft Systems - No Pitot):
├─ ADC Ch 0: Battery voltage (scaled divider)
├─ ADC Ch 1: Battery current (hall sensor)  
├─ ADC Ch 2: Wing loading sensor L (strain gauge)
├─ ADC Ch 3: Wing loading sensor R (strain gauge)
├─ ADC Ch 4: Environmental pressure (altitude)
├─ ADC Ch 5: Motor current/temperature
└─ ADC Ch 6: System temperature (external)

Temperature Sources:
├─ MPU6050 internal temperature sensor
├─ External temperature sensor (ADC Ch 6)
├─ Motor temperature monitoring (ADC Ch 5)
└─ Environmental monitoring for calibration
```

### **Aircraft-Specific Mounting (3D Printed Integration)**
| Component | Location | Mounting Method | 3D Print Features |
|-----------|----------|-----------------|-------------------|
| **Custom PCB** | Center fuselage | Custom 3D bucket with foam | Vibration isolation, cable management |
| **MPU6050** | On PCB | Rigid PCB mount | Critical vibration isolation |
| **Pi Zero 2W** | Center fuselage | Custom 3D bucket | Ventilation, access ports |
| **AI Camera** | Nose/belly | Custom 3D mount | 15° forward + downward angle |
| **GPS Antenna** | Fuselage top | 3D printed bracket | Clear sky view, no metal obstruction |
| **LoRa Antenna** | Wing tip/tail | 3D printed mount | Maximum range, clear of carbon fiber |
| **Motor Mount** | Pusher location | Custom 3D mount | D3536 motor, vibration dampening |

---

## Base Station ESP32 (Ground Station)

### **Ground Station Configuration**
| Component | Purpose | Mounting |
|-----------|---------|----------|
| **Heltec LoRa V3** | Command relay + telemetry display | Desktop enclosure |
| **2" TFT Display** | Real-time G-G diagram | Integrated with ESP32 |
| **LoRa Antenna** | 915MHz communication | Desktop stand, clear LOS |
| **USB Connection** | Pi Zero 2W interface | Standard USB-C cable |

### **TFT Display Module Specifications**
| Parameter | Value | Interface |
|-----------|-------|-----------|
| **Size** | 2.0 inch diagonal | SPI connection |
| **Resolution** | 240x320 RGB | ST7789V driver IC |
| **Colors** | 65K (16-bit) | Full color display |
| **Viewing Angle** | 160° | IPS technology |
| **Backlight** | LED | Adjustable brightness |

### **Ground Station Data Display**
```
TFT Display Layout (240x320):

Header (0-30px):
├─ "dpilot" title
├─ System status
└─ Connection indicator

Flight Data (30-180px):
├─ Speed: XXX km/h (large font)
├─ Altitude: XXX m
├─ GPS status indicator
└─ Battery/system status

G-G Diagram (180-300px):
├─ Real-time acceleration plot
├─ Color-coded intensity
├─ Trail visualization
└─ Acceleration values (Ax, Ay)

Status Bar (300-320px):
├─ LoRa RSSI
├─ Packet success rate
└─ Max G loading
```

---

## Power System Integration

### **Aircraft Power Architecture (Single Buck Design)**
```
LiPo Battery (3S-4S: 11.1V-14.8V)
│
├─ Main ESC (60A) ──→ D3536 Brushless Motor (Custom 3D Mount)
│   │
│   └─ UBEC (4A @ 5V) ──→ Aircraft 5V Bus
│       │
│       └─ LM2596 Buck (5V → 5V regulated, 3A)
│           ├─ Raspberry Pi Zero 2W (micro USB, 3D bucket)
│           ├─ Custom PCB ESP32 (USB-C custom cable, 3D bucket)
│           ├─ MPU6050 sensor (3.3V from ESP32 PCB)
│           ├─ GPS module (3.3V from ESP32 PCB)
│           ├─ Analog sensors (various voltages)
│           └─ Future expansion
│
└─ Ground Station:
    └─ ESP32 Base Station (USB power from Pi/Computer)
```

### **Power Budget Analysis (Optimized Single Buck)**
| Component | Voltage | Current | Power | Notes |
|-----------|---------|---------|-------|-------|
| **Pi Zero 2W** | 5V | 800mA | 4W | Peak with AI camera |
| **Custom PCB ESP32** | 5V | 400mA | 2W | With LoRa TX |
| **ESP32 Base** | 5V | 0mA | 0W | USB powered from ground |
| **Pi AI Camera** | 3.3V | 350mA | 1.2W | From Pi supply with AI |
| **MPU6050** | 3.3V | 4mA | 0.01W | From ESP32 PCB supply |
| **GPS Module** | 3.3V | 25mA | 0.08W | From ESP32 PCB supply |
| **Analog Sensors** | 3.3V/5V | 50mA | 0.2W | Mixed voltage levels |
| **Total Aircraft** | Mixed | ~1.6A | ~7.5W | Plus conversion losses |

### **Power Distribution Board (Custom)**
```
PCB Layout Requirements:
├─ Input: XT60 connector (battery)
├─ ESC connection: High-current traces
├─ UBEC input: 5V @ 4A capability
├─ Buck converter mounting
├─ Fusing/protection circuitry
├─ LED indicators (power, fault)
├─ Connector for telemetry harness
└─ Mechanical: Fits Ranger 1600/2000 fuselage
```

---

## Communication System Integration

### **LoRa Network (915MHz)**
```
Ground Station ESP32 ←→ Aircraft Custom PCB ESP32
├─ Frequency: 915MHz ISM band
├─ Power: 22dBm (158mW) maximum
├─ Range: 2-5km line of sight
├─ Encryption: AES-128 for security
├─ Protocol: Custom binary packets
└─ Antennas: Dipole/whip configuration
```

### **Data Flow Architecture (Complete System)**
```
Aircraft Sensors → Custom PCB ESP32 → LoRa → Ground ESP32 → USB → Pi Zero 2W
     ↓                    ↓                                      ↓
     ├─ IMU (50Hz) ──────→ Filtered data ──────────────────────→ Raw data
     ├─ GPS (5Hz) ───────→ Position/velocity ─────────────────→ Navigation
     ├─ ADC (50Hz) ──────→ Aircraft systems ──────────────────→ Health monitoring  
     └─ Status ──────────→ Flight status ────────────────────→ Safety monitoring
                          
Pi Zero 2W Processing:
├─ Video recording (H.264) with AI camera
├─ Data logging (CSV) at 50Hz
├─ Web interface (port 5000)
├─ WiFi hotspot (field access)
├─ Visual odometry processing
└─ Command generation (mission planning)

Ground Station Display:
├─ Real-time telemetry
├─ G-G acceleration diagram
├─ Flight status monitoring
└─ Range/signal quality
```

### **Antenna Placement Strategy**
| Antenna | Location | Considerations |
|---------|----------|----------------|
| **LoRa (Aircraft)** | Wing tip or vertical stabilizer | Clear of carbon fiber, maximum range |
| **WiFi (Pi)** | Fuselage interior | Adequate for hotspot range |
| **GPS** | Fuselage top | Unobstructed sky view |
| **LoRa (Ground)** | Elevated mast | Line of sight to aircraft |

---

## Custom 3D Printed Component Integration

### **3D Printed Component Integration**
| Component | Material | Purpose | Features |
|-----------|----------|---------|----------|
| **Pi Zero 2W Bucket** | PETG/ABS | Environmental protection | Ventilation slots, cable management |
| **Custom PCB Bucket** | PETG/ABS | Vibration isolation | Foam padding, secure mounting |
| **Camera Mount** | PETG | Visual odometry angle | 15° forward tilt, downward facing |
| **Motor Mount** | PETG/ABS | D3536 pusher installation | Vibration dampening, precise alignment |
| **Antenna Mounts** | PETG | LoRa/WiFi positioning | Optimal RF positioning, SMA connectors |
| **Cable Management** | TPU/PETG | Harness organization | Strain relief, connector protection |

### **Wing Configuration Options**
| Wing Set | Characteristics | Use Case | Flap Control |
|----------|----------------|----------|--------------|
| **Ranger 1600** | Standard speed flight | Normal operations | None |
| **Ranger 2000** | Slow flight capability | Visual odometry, landing | Servo-controlled flaps |
| **Flap Benefits** | Enhanced lift at low speed | Improved stall characteristics | ADC feedback |

### **Visual Odometry System Integration**
```
Pi AI Camera (Sony IMX500) → Visual Odometry Pipeline:
├─ Ground feature detection (60fps high resolution)
├─ Motion estimation from consecutive frames  
├─ Position drift correction vs GPS
├─ Velocity estimation for flight control
├─ Landmark recognition for navigation
└─ Emergency landing site detection

Mounting Strategy:
├─ Downward facing primary (ground tracking)
├─ 15° forward tilt (horizon visibility)
├─ Custom 3D mount (vibration isolated)
├─ Clear view (no propeller interference)
└─ Cable management (CSI ribbon protection)
```

---

## Custom Cable Harness Design

### **Primary Harness (Aircraft Internal)**
```
Main Telemetry Harness:
├─ Power Distribution:
│   ├─ 5V Bus (red) ──→ Pi, Custom PCB ESP32
│   ├─ 3.3V Bus (orange) ──→ Sensors (from PCB regulators)
│   └─ Ground Bus (black) ──→ Common ground
│
├─ Data Connections:
│   ├─ Pi ←→ Custom PCB ESP32 (USB-C custom)
│   ├─ Custom PCB ←→ MPU6050 (on-board I2C)
│   ├─ Custom PCB ←→ GPS (external UART)
│   └─ Custom PCB ←→ Analog sensors (7-wire)
│
└─ Camera Connection:
    └─ Pi ←→ AI Camera (CSI ribbon)
```

### **Custom USB Cable Specifications**
| Cable | Length | Gauge | Connectors | Purpose |
|-------|--------|-------|------------|---------|
| **Pi Power** | 150mm | 20AWG | Micro USB Male → Terminal | 5V power feed |
| **Pi↔Custom PCB** | 200mm | 28AWG | USB-C ↔ USB-C | Data @ 115200 baud |
| **Pi↔ESP32 Base** | 200mm | 28AWG | USB-C ↔ USB-C | Commands/telemetry |

### **Connector Strategy**
```
Aircraft Removable Connections:
├─ Wing separation: Quick-disconnect for transport
├─ Fuselage access: Hinged for battery/maintenance
├─ Camera: Detachable for different orientations
└─ Antennas: SMA connectors for replacement
```

---

## System Integration Testing

### **Ground Test Sequence**
1. **Power System Test**: Verify all voltages, current draw
2. **Communication Test**: LoRa link quality, range test
3. **Sensor Calibration**: IMU alignment, GPS acquisition
4. **Data Logging Test**: 50Hz unified state verification
5. **Video System Test**: AI camera, recording, visual odometry
6. **Web Interface Test**: WiFi hotspot, telemetry display
7. **Flight Control Integration**: Commands, responses, safety

### **Flight Test Progression**
1. **Taxi Tests**: Ground movement, sensor data quality
2. **Range Tests**: LoRa performance at distance
3. **Short Flights**: Basic telemetry, video recording
4. **Extended Flights**: Full mission profile testing
5. **Autonomous Tests**: Command following, waypoint navigation
6. **Visual Odometry Tests**: Ground feature tracking validation

### **Data Quality Metrics**
```
Telemetry Performance Requirements:
├─ Unified State Rate: 50Hz ±1Hz
├─ LoRa Success Rate: >95% at operational range
├─ GPS Accuracy: <3m horizontal, <5m vertical
├─ IMU Data Quality: <3% duplicate rate
├─ Video Sync: Frame-perfect CSV alignment
├─ Visual Odometry: <1% position drift over 1km
└─ Power Efficiency: >2 hour flight endurance
```

---

## Complete System Integration Benefits

### **Research-Grade Visual Odometry Platform:**
- Sony IMX500 AI camera for advanced computer vision
- 60fps high-resolution imaging for precise motion tracking
- On-sensor AI processing reduces Pi computational load
- Custom mounting optimizes visual odometry performance

### **Professional PCB Integration:**
- Custom 2-layer PCB from JLCPCB for reliable integration
- Integrated MPU6050, SD card, and LoRa on single board
- Professional surface mount assembly for durability
- Optimized RF design for maximum LoRa range

### **Simplified Power Architecture:**
- Single LM2596 buck converter reduces weight and complexity
- Base station powered from ground (no aircraft power required)
- Optimized for 2+ hour flight endurance

### **Enhanced Flight Capabilities:**
- Ranger 2000 wing option for slow flight and approach control
- Servo-controlled flaps for enhanced landing performance
- Custom 3D printed motor mount optimizes pusher configuration

### **Robust Environmental Protection:**
- All electronics in custom 3D printed enclosures
- Vibration isolation for sensitive IMU and camera systems
- Cable management and strain relief throughout

### **Advanced Temperature Monitoring:**
- MPU6050 internal temperature sensor for calibration
- External temperature sensors for environmental monitoring
- Motor thermal management for extended operations

### **Modular Research Platform:**
- Easy component access via 3D printed buckets
- Scalable sensor integration via 7-channel ADC
- Wing loading sensors for aerodynamic research
- Perfect for autonomous flight research and development

This setup transforms the Ranger 1600/2000 into a sophisticated research platform capable of advanced visual odometry, comprehensive flight data collection, and autonomous systems development with professional-grade environmental protection and reliability.

---

## Manufacturing and Assembly

### **PCB Manufacturing (JLCPCB)**
```
Order Specifications:
├─ Quantity: 5-10 PCBs (minimum order)
├─ Layer Count: 2 layers
├─ Thickness: 1.6mm
├─ Surface Finish: HASL or ENIG
├─ Solder Mask: Green (standard)
├─ Silkscreen: White
├─ Via Process: Tenting vias
└─ Electrical Test: Yes

Assembly Options:
├─ SMT Assembly: Available for complex components
├─ Through-hole: Manual assembly
├─ Component Sourcing: JLCPCB parts library
└─ Cost Optimization: Standard components preferred
```

### **Component Sourcing**
- **ESP32-S3**: JLCPCB parts library or external
- **SX1262 LoRa**: External sourcing (specialized RF)
- **MPU6050**: JLCPCB standard parts
- **SD Card Socket**: From AliExpress module integration
- **Connectors**: JST, USB-C from JLCPCB or external

### **Assembly Workflow**
1. **PCB Fabrication**: Order from JLCPCB (1-2 weeks)
2. **Component Assembly**: SMT + through-hole soldering
3. **Programming**: ESP32-S3 firmware upload
4. **Testing**: Functional verification
5. **Integration**: Install in 3D printed bucket
6. **System Test**: Full aircraft integration testing

This complete system provides a professional-grade research platform optimized for visual odometry research, autonomous flight development, and comprehensive flight data collection with industrial-grade reliability and environmental protection.