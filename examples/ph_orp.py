# This file demonstrates how to take measurements using the AnyLeaf pH/ORP
# module in Python, on Raspberry Pi. It includes basic operations, and some example
# code for storing calibration data in a CSV.


import time

import board
import busio
import csv
from anyleaf import PhSensor, CalPt, CalSlot, OnBoard, OffBoard
from anyleaf import OrpSensor, CalPtOrp


CFG_FILENAME = "ph_cal_data.csv"


def calibrate():
    """Save calibration data to a file"""
    with open(CFG_FILENAME, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(1, 2, 3)


def main():
    i2c = busio.I2C(board.SCL, board.SDA)
    delay = 1  # Time between measurements, in seconds

    ph_sensor = PhSensor(i2c, delay)

    # If you connect multiple AnyLeaf modules on the same I²C bus, set one's
    # jumper to the `0x49` position, and specify this as below:
    # ph_sensor = PhSensor(i2c, delay, address=0x49)

    # If you're using an ORP sensor:
    # orp_sensor = OrpSensor(i2c, delay)

    # 2 or 3 pt calibration both give acceptable results.
    # Calibrate with known values. (voltage, pH, temp in °C).
    # You can find voltage and temperature with `ph_sensor.read_voltage()` and
    # `ph_sensor.read_temp()` respectively. Or skip this to use default calibration.
    # For 3 pt calibration, pass a third argument to `calibrate_all`.

    with open(CFG_FILENAME, newline='') as f:
        reader = csv.reader(f)
        pt1 = CalPt(reader[0][0], reader[0][1], reader[0][2])
        pt2 = CalPt(reader[0][0], reader[0][1], reader[0][2])

        ph_sensor.calibrate_all(pt1, pt2)

    # `calibrate_all` stores 2 or 3 calibration values, from known previous values.
    # The first value is measured voltage; the second is nominal pH; the third is temperature
    # at which the calibration was performed.
    ph_sensor.calibrate_all(
        CalPt(0., 7., 25.), CalPt(0.18, 4., 25.)
    )

    # Or, call `calibrate` with the sensor in the appropriate buffer solution.
    # This will automatically use voltage and temperature.
    # Voltage and Temp are returned, but calibration occurs
    # without using the return values:

    # V1, T1 = ph_sensor.calibrate(CalSlot.ONE, 7., OnBoard(())
    # V2, T2 = ph_sensor.calibrate(CalSlot.TWO, 4., OffBoard(23.)

    # ORP setup is simpler: There's only 1 calibration point, and no
    # temperature compensation. Use these as equivalents to the above:
    # orp_sensor.calibrate_all(CalPtOrp(0.4, 400.0))
    # V = orp_sensor.calibrate(400.0)

    # Ideally, store the calibration parameters somewhere, so they persist
    # between program runs. You could use the example above os storing to and
    # reading from a CSV file, or just replace the `ph_sensor.calibrate_all`(
    # line above with the new calibration values. (V1, T1, V2, and T2).

    while True:
        pH = ph_sensor.read(OnBoard())
        # ORP = orp_sensor.read()

        # To use an offboard temperature measurement `ph_sensor.read(OffBoard(30.))`
        print(f"pH: {pH}")

        # print(f"ORP: {ORP}")

        time.sleep(delay)


if __name__ == "__main__":
    main()
