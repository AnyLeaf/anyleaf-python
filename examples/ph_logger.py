# This file demonstrates how to take measurements, log them to a CSV file,
# and plot them. For details on calibration, see `ph_orp.py`. It's a simple
# implementation, and will likely need to be expanded or customized, depending
# on user need.
# Requires matplotlib: `pip install matplotlib`.

import csv
from datetime import datetime
import os
import time

import board
import busio
import matplotlib.pyplot as plt

from anyleaf import PhSensor, CalPt, OnBoard


LOG_FILENAME = "ph_log.csv"
CAL_1 = CalPt(0., 7., 25.)
CAL_2 = CalPt(0.17, 4., 25.)


def log_reading(ph_measurement):
    """Store a single reading in our file, with a timestamp"""
    with open(LOG_FILENAME, 'a+', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([ph_measurement, str(datetime.now())])


def plot_log():
    """Call this function to plot all readings in our log file."""
    with open(LOG_FILENAME) as f:
        data = list(csv.reader(f))

    dates = [row[0] for row in data]
    values = [row[1] for row in data]

    plt.plot(dates, values)
    plt.show()


def measure_and_log():
    i2c = busio.I2C(board.SCL, board.SDA)
    delay = 10  # Time between measurements, in seconds

    ph_sensor = PhSensor(i2c, delay)

    ph_sensor.calibrate_all(CAL_1, CAL_2)

    while True:
        pH = ph_sensor.read(OnBoard())

        print(f"Logged pH: {pH}")

        log_reading(pH)

        time.sleep(delay)


if __name__ == "__main__":
    measure_and_log()
