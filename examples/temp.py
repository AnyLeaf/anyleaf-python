import time
import board
import busio
import digitalio
from anyleaf import Rtd, RtdType, RtdWires


def main():
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    # `cs` can be any GPIO pin.R
    cs = digitalio.DigitalInOut(board.D5)
    rtd = Rtd(spi, cs, RtdType.PT100, RtdWires.THREE)  # 3-wire pt100
    # rtd = Rtd(spi, cs, RtdType.Pt1000, RtdWires.TWO)  # 2-wire pt1000

    while True:
        print(f"Temp: {rtd.read()} Celsius")

        # To display the measured resistance:
        print(f"Resistance: {rtd.read_resistance()}Ω")

        time.sleep(1)


if __name__ == "__main__":
    main()