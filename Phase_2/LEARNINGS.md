# LEARNINGS.md

## 1. All GPIO pins on ESP-cam are not free

When camera of the ESP-cam is in use it still uses external GPIO pins, meaning those pins cannot be connected to anything else

## 2. Finding apogee using only a gyroscope (quaternion orientation tracking)

If you only have a gyroscope (no barometer), you can still find apogee. You keep adding up how much the rocket has rotated every loop, using something called a **quaternion** (a way to store rotation with 4 numbers).Once the nose tips past a certain angle from straight up, that means the rocket has gone over the top, so apogee has happened.

## 3. RX/TX not working between ESP32 and ESP-cam

**Fix:** Adding a resistor in series on the wire