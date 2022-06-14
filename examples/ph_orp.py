# This file demonstrates how to take measurements using the AnyLeaf pH/ORP
# module in Python, on Raspberry Pi. It includes basic operations, and some example
# code for storing calibration data in a CSV.


import time

import board
import busio
import csv
from anyleaf import PhSensor, CalPt, CalSlot, OnBoard, OffBoard
from anyleaf import OrpSensor, CalPtOrp
import os

# Change this file name (if storing calibration to CSV), and/or these in-file calibration-points as required:
CFG_FILENAME = "ph_cal_data.csv"
CAL_1 = CalPt(0., 7., 25.),
CAL_2 = CalPt(0.17, 4., 25.)
# CAL_3 = CalPt(-0.17, 10., 25.)  # For 3-pt calibration


def save_calibration_data(point1: CalPt, point2: CalPt):
    """Save 2-pt calibration data to a file. Creates the file if it doesn't already exist. You may modify this
     by adding a third calibration point and row if desired, for 3-pt calibration."""
    with open(CFG_FILENAME, 'w+', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(point1.V, point1.pH, point1.T)
        writer.writerow(point2.V, point2.pH, point2.T)


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

    # `calibrate_all` stores 2 or 3 calibration values, from known previous values.
    # The first value is measured voltage; the second is nominal pH; the third is temperature
    # at which the calibration was performed. This is how you apply calibration data. You can apply calibration
    # data directly like this, by storing the cal data in the Python file your program uses.
    ph_sensor.calibrate_all(CAL1, CAL2)

    # Example below loading 2-point calibration data from a file. Each row is a calibration point. The columns are
    # (left to right) voltage measured, nominal pH, temperature the measurement was taken at.
    if not os.exists(CFG_FILENAME):  # Create the file and populate with default values if it doesn't exist.
        save_calibration_data(CAL1, CAL2)

    with open(CFG_FILENAME, newline='') as f:
        reader = list(csv.reader(f))
        pt1 = CalPt(reader[0][0], reader[0][1], reader[0][2])
        pt2 = CalPt(reader[1][0], reader[1][1], reader[1][2])

        ph_sensor.calibrate_all(pt1, pt2)

    # Or, call `calibrate` with the sensor in the appropriate buffer solution.
    # This will automatically use voltage and temperature.
    # Voltage and Temp are returned, but calibration occurs
    # without using the return values:

    # V1, T1 = ph_sensor.calibrate(CalSlot.ONE, 7., OnBoard(())
    # V2, T2 = ph_sensor.calibrate(CalSlot.TWO, 4., OffBoard(23.)

    # Optionally, save this calibration data to a file:
    # save_calibration_data(CalPt(V1, 7., T1), CalPt(V2, 4., T2))

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
