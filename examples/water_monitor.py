# This example demonstrates how to take readings from the AnyLeaf Water Monitor
# connected to a PC over USB, using Python.
# Note that calibration is handled directly using the Water Monitor's controls,
# not through Python.

import time

from anyleaf import WaterMonitor


def main():
    water_monitor = WaterMonitor()
    delay = 1  # Time between measurements, in seconds

    while True:
        # Take all readings at once:
        readings = water_monitor.read_all()

        print(f"Readings: {readings}")
        print(f"pH: {readings.pH}")  # Use this pattern to extract other readings as well.

        # Or take individual readings:
        ph = water_monitor.read_ph()
        orp = water_monitor.read_orp()
        ec = water_monitor.read_ec()
        temp = water_monitor.read_temp()

        time.sleep(delay)


if __name__ == "__main__":
    main()
