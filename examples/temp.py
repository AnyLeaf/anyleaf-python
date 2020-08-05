import time
import board
import busio
import digitalio
from anyleaf import Rtd

def main():
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    # `cs` can be any GPIO pin.
    cs = digitalio.DigitalInOut(board.D5)
    rtd = Rtd(spi, cs)  # 3-wire pt100
    # rtd = Rtd(spi, cs, wires=2, pt1000=True)  # eg for a 2-wire pt1000.

    while True:
        print(f"pH: {rtd.read()}")

        time.sleep(1)


if __name__ == "__main__":
    main()