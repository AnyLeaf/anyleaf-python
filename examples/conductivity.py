import time
from anyleaf import EcSensor


def main():
    delay = 1  # Time between measurements, in seconds
    ec_sensor = EcSensor(K=1.0)

    # Examples of different configs:
    # ec_sensor = EcSensor(K=0.1, excitation_mode=ExcMode.ALWAYS_ON)

    while True:
        print(f"conductivity: {ec_sensor.read()}")
        print(f"Air temperature: {ec_sensor.read_temp()}")

        time.sleep(delay)


if __name__ == "__main__":
    main()