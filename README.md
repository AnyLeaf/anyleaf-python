[![image](https://img.shields.io/pypi/v/anyleaf.svg)](https://python.org/pypi/anyleaf)

# Anyleaf

## For use with the AnyLeaf pH/ORP, conductivity, and RTD sensors in Python
[Homepage](https://anyleaf.org)

## Quickstart
To get started as quickly as possible on Raspberry Pi:
- Activate I2C (pH and ORP module): Run `sudo raspi-config`. Select `Interfacing Options` → `I2C` → `Yes`.
- Activate SPI (RTD temp module): Same as above, but select `SPI` instead of `I2C`.
- Activate UART (Conductivity module): Same as above, but select `Serial` instead of `I2C`.
- Reboot

Then run these commands from a terminal:
- `sudo apt install python3-scipy`
- `pip3 install anyleaf`
- `git clone https://github.com/anyleaf/anyleaf-python.git`
- `cd anyleaf-python/examples`
- `python3 ph_orp.py`, (or run `conductivity.py` or `rtd.py` as appropriate.)

Readings will display in your terminal.


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

## Conductivity example:
```python
import time
from anyleaf import EcSensor


def main():
    delay = 1  # Time between measurements, in seconds
    ec_sensor = EcSensor(K=1.0)

    while True:
        print(f"conductivity: {ec_sensor.read()}")
        print(f"Air temperature: {ec_sensor.read_temp()}")

        time.sleep(delay)


if __name__ == "__main__":
    main()
```

See the [examples folder on Github](https://github.com/AnyLeaf/anyleaf-python/tree/master/examples)
for more code examples.