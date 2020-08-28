import time
import board
import busio
import digitalio
from anyleaf import Rtd, RtdType, RtdWires


def main():
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    # `cs` is the pin connected to the `CS` pin of the module. It can be any
    # GPIO pin.
    cs = digitalio.DigitalInOut(board.D5)
    sensor = Rtd(spi, cs, RtdType.PT100, RtdWires.THREE)  # 3-wire pt100
    # sensor = Rtd(spi, cs, RtdType.PT1000, RtdWires.TWO)  # 2-wire pt1000

    while True:
        print(f"Temp: {sensor.read()} °C")

        # To display the measured resistance:
        print(f"Resistance: {sensor.read_resistance()} Ω")

        time.sleep(1)


if __name__ == "__main__":
    main()
