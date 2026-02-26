# hardware/

CAD files, PCBs, and mechanical specs for Ruby.
Full 3D print files and gerbers are in the companion repo:
    https://github.com/openclaw/ruby-hardware

## FlexBone-X Joint Map

```
neck_yaw    neck_pitch
     \         /
      [HEAD UNIT]
           |
  shoulder_l  shoulder_r
       |              |
   elbow_l         elbow_r
       |              |
   wrist_l          wrist_r

           |
     [SPINE 4-DOF]
      spine_0
      spine_1
      spine_2
      spine_3
           |
   hip_l       hip_r
     |               |
 knee_l           knee_r

           |
       [TAIL]
     tail_base_yaw
     tail_base_pitch
     tail_mid
     [TailSense touch at tip]

Total active DOF: 19
Servo type: MG996R (x12 body) + MG90S (x3 tail)
Driver: PCA9685 I2C PWM controller (addr 0x40)
```

## SenseFur v2 Wiring

```
RPi 3.3V  ---- ATtiny84 VCC
RPi GND   ---- ATtiny84 GND
RPi SDA   ---- ATtiny84 SDA  (I2C addr 0x48)
RPi SCL   ---- ATtiny84 SCL

ATtiny84 PA0..PA7  -- 8x fiber mux select
ATtiny84 PB0..PB3  -- 4x channel enable

Fiber array: 16 rows x 8 cols = 128 capacitive nodes
Scan time: ~5ms for full frame at 100Hz
```

## Skin Mold

Silicone: Smooth-On Ecoflex 00-30, Shore A ~00-30
Colorant: Silc Pig pigments
Thickness: 3-5mm over body, 1-2mm over fiber array zones
Cure time: 4 hours at room temperature
