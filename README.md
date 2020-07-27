[![image](https://img.shields.io/pypi/v/anyleaf.svg)](https://python.org/pypi/anyleaf)

# Anyleaf

## For use with the AnyLeaf pH sensor in Python
[Homepage](https://anyleaf.org)

## Quickstart
To get started as quickly as possible, run these commands from a terminal:
- `pip3 install anyleaf`
- `pip3 install filterpy`
- `git clone https://github.com/anyleaf/ph-python.git`
- `cd ph-python/examples`
- `python3 ex.py`

pH readings will display in your terminal.


To install the `anyleaf` Python package, run `pip3 install anyleaf`, or 
`pip install anyleaf`, depending on how `pip` is set up on your operating system.

### Example use, for Raspberry Pi, and CircuitPython boards:
```python
import time

import board
import busio
from anyleaf import PhSensor, CalPt, CalSlot, OnBoard, OffBoard


def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    delay = 1  # Time between measurements, in seconds
    ph_sensor = PhSensor(i2c, delay)

    # 2 or 3 pt calibration both give acceptable results.
    # Calibrate with known values. (voltage, pH, temp in °C).
    # You can find voltage and temperature with `ph_sensor.read_voltage()` and 
    # `ph_sensor.read_temp()` respectively.
    # For 3 pt calibration, pass a third argument to `calibrate_all`.
    ph_sensor.calibrate_all(
        CalPt(0., 7., 25.), CalPt(0.17, 4., 25.)
    )

    # Or, call these with the sensor in the appropriate buffer solution.
    # This will automatically use voltage and temperature.
    # Voltage and Temp are returned, but calibration occurs
    # without using the return values.
    # V, T = ph_sensor.calibrate(CalSlot.ONE, 7., Offboard(40.))
    # ph_sensor.calibrate(CalSlot.TWO, 4., OnBoard())

    # Store the calibration parameters somewhere, so they persist
    # between program runs.

    while True:
        pH = ph_sensor.read(OnBoard())
        # To use an offboard temperature measurement: `ph_sensor.read(OffBoard(30.))`
        print(f"pH: {pH}")

        time.sleep(delay)


if __name__ == "__main__":
    main()
```