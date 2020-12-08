# Driver for the Anyleaf pH module

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, List, Union
import struct

import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_max31865 import MAX31865

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import serial
from serial.tools import list_ports

from . import filter


DISCRETE_PH_JUMP_THRESH = 0.2
DISCRETE_ORP_JUMP_THRESH = 30.0
# Compensate for temperature diff between readings and calibration.
PH_TEMP_C = -0.05694  # pH/(V*T). V is in volts, and T is in °C


# These 3 bits are used for requesting data from the water monitor via USB
# serial. They must match the codes in the Water Monitor firmware.
# READINGS_REQ_BIT = 69
READINGS_SIZE_WM = 20
READINGS_SIZE_EC = 11
SUCCESS_MSG = [50, 50, 50]
ERROR_MSG = [99, 99, 99]
# `OK_BIT` and `ERROR_BIT` are the preceding bit of each reading.
# They indicate a sensor error, not a serial comms error.
OK_BIT = 10
ERROR_BIT = 20
MSG_START_BITS = [100, 150]
MSG_END_BITS = [200]

SERIAL_TIMEOUT = 100_000  # loops


class CalSlot(Enum):
    """Keeps our calibration organized, so we track when to overwrite."""
    ONE = auto()
    TWO = auto()
    THREE = auto()


@dataclass
class OnBoard:
    """Specify onboard or offboard temperature source."""

    pass


@dataclass
class OffBoard:
    temp: float


@dataclass
class CalPt:
    V: float
    pH: float
    T: float


@dataclass
class CalPtOrp:
    V: float
    ORP: float


@dataclass
class CalPtT:
    V: float
    T: float  # Temp in C


@dataclass
class CalPtEc:
    reading: float
    ec: float
    T: float  # Temp in C


@dataclass
class PhSensor:
    adc: ADS
    filter: KalmanFilter
    dt: float
    last_meas: float  # To let discrete jumps bypass the filter.
    cal_1: CalPt
    cal_2: CalPt
    cal_3: Optional[CalPt]

    def __init__(self, i2c, dt: float, address=0x48):
        # `dt` is in seconds.
        adc = ADS.ADS1115(i2c, address=address)
        adc.gain = 2  # Set the ADC's voltage range to be +-2.048V.

        self.adc = adc
        self.filter = filter.create(dt)
        self.dt = dt  # Store for when we update the filter's Q.
        self.last_meas: 7.0
        self.cal_1 = CalPt(0., 7.0, 23.)
        self.cal_2 = CalPt(0.17, 4.0, 23.)
        self.cal_3 = None

    def predict(self) -> None:
        """Make a prediction using the Kalman filter. Not generally used
        directly."""
        self.filter.predict()

    def update(self, t: Union[OnBoard, OffBoard]) -> None:
        """Update the Kalman filter with a pH reading. Not generally used
        directly."""
        pH = self.read_raw(t)

        if abs(pH - self.last_meas) > DISCRETE_PH_JUMP_THRESH:
            self.filter.reset()

        self.filter.update(pH)

    def read(self, t: Union[OnBoard, OffBoard]) -> float:
        """Take a pH reading, using the Kalman filter. This reduces sensor 
        noise, and provides a more accurate reading. Optional parameter `t` allows
        you to pass a temp manually, eg from a temperature probe in the solution.
        Not passing this uses the on-chip temp sensor."""
        self.predict()
        self.update(t)
        # self.filter.x is mean, variance. We only care about the mean
        return self.filter.x[0][0]

    def read_raw(self, t: Union[OnBoard, OffBoard]) -> float:
        """Take a pH reading, without using the Kalman filter"""
        if isinstance(t, OnBoard):
            T = temp_from_voltage(AnalogIn(self.adc, ADS.P2).voltage)
        else:
            T = t.temp

        chan_ph = AnalogIn(self.adc, ADS.P0, ADS.P1)
        pH = ph_from_voltage(chan_ph.voltage, T, self.cal_1, self.cal_2, self.cal_3)

        self.last_meas = pH
        return pH

    def read_voltage(self) -> float:
        """Useful for getting calibration data"""
        return AnalogIn(self.adc, ADS.P0, ADS.P1).voltage

    def read_temp(self) -> float:
        """Useful for getting calibration data"""
        return temp_from_voltage(AnalogIn(self.adc, ADS.P2).voltage)

    def calibrate(
        self, slot: CalSlot, pH: float, t: Union[OnBoard, OffBoard]
    ) -> (float, float):
        """Calibrate by measuring voltage and temp at a given pH. Set the
        calibration, and return (Voltage, Temp). Optional parameter `t` allows
        you to pass a temp manually, eg from a temperature probe in the solution.
        Not passing this uses the on-chip temp sensor."""
        if isinstance(t, OnBoard):
            T = temp_from_voltage(AnalogIn(self.adc, ADS.P2).voltage)
        else:
            T = t.temp

        V = AnalogIn(self.adc, ADS.P0, ADS.P1).voltage
        pt = CalPt(V, pH, T)

        if slot == CalSlot.ONE:
            self.cal_1 = pt
        elif slot == CalSlot.TWO:
            self.cal_2 = pt
        else:
            self.cal_3 = pt

        print(f"Calibration voltage at pH {pH}, {T}°C: {V}V")
        print(
            "Calibration set. Store those values somewhere, and apply them in"
            f"future runs: `calibrate_all(CalPt({V}, {pH}, {T}), ...)`"
        )

        return V, T

    def calibrate_all(
        self, pt0: CalPt, pt1: CalPt, pt2: Optional[CalPt] = None
    ) -> None:
        self.cal_1 = pt0
        self.cal_2 = pt1
        self.cal_3 = pt2

    def reset_calibration(self):
        self.cal_1 = CalPt(0.0, 7.0, 23.0)
        self.cal_2 = CalPt(0.17, 4.0, 23.0)
        self.cal_3 = None


@dataclass
class OrpSensor:
    """These sensors operate in a similar, minus the conversion from
    voltage to measurement, not compensating for temp, and using only 1 cal pt.
    The adc will be empty if I2C has been freed."""

    adc: ADS
    filter: KalmanFilter
    dt: float
    last_meas: float  # To let discrete jumps bypass the filter.
    cal: CalPtOrp

    def __init__(self, i2c, dt: float, address=0x48):
        # `dt` is in seconds.
        adc = ADS.ADS1115(i2c, address=address)
        adc.gain = 2  # Set the ADC's voltage range to be +-2.048V.

        self.adc = adc
        self.filter = filter.create(dt)
        self.dt = dt  # Store for when we update the filter's Q.
        self.last_meas: 0.0
        self.cal = CalPtOrp(0.4, 400.0)

    def predict(self) -> None:
        """Make a prediction using the Kalman filter. Not generally used
        directly."""
        self.filter.predict()

    def update(self) -> None:
        """Update the Kalman filter with a pH reading. Not generally used
        directly."""
        ORP = self.read_raw()

        if abs(ORP - self.last_meas) > DISCRETE_ORP_JUMP_THRESH:
            self.filter.reset()

        self.filter.update(ORP)

    def read(self) -> float:
        """Take an ORP reading, using the Kalman filter. This reduces sensor
        noise, and provides a more accurate reading."""
        self.predict()
        self.update()
        # self.filter.x is mean, variance. We only care about the mean
        return self.filter.x[0][0]

    def read_raw(self) -> float:
        """Take an ORP reading, without using the Kalman filter"""
        chan_orp = AnalogIn(self.adc, ADS.P0, ADS.P1)
        ORP = orp_from_voltage(chan_orp.voltage, self.cal)

        self.last_meas = ORP
        return ORP

    def read_voltage(self) -> float:
        """Useful for getting calibration data"""
        return AnalogIn(self.adc, ADS.P0, ADS.P1).voltage

    def read_temp(self) -> float:
        """Useful for getting calibration data"""
        return temp_from_voltage(AnalogIn(self.adc, ADS.P2).voltage)

    def calibrate(self, ORP: float) -> float:
        """Calibrate by measuring voltage and temp at a given ORP. Set the
        calibration, and return Voltage."""
        V = AnalogIn(self.adc, ADS.P0, ADS.P1).voltage

        self.cal = CalPtOrp(V, ORP)

        print(f"Calibration voltage at ORP {ORP}: {V}V")
        print(
            "Calibration set. Store those values somewhere, and apply them in"
            f"future runs: `calibrate_all(CalPt({V}, {ORP}))`"
        )

        return V

    def calibrate_all(self, pt: CalPtOrp) -> None:
        self.cal = pt

    def reset_calibration(self):
        self.cal = CalPtOrp(0.4, 400.0)


class RtdType(Enum):
    PT100 = auto()
    PT1000 = auto()


class RtdWires(Enum):
    TWO = auto()
    THREE = auto()
    FOUR = auto()


@dataclass
class Rtd:
    sensor: MAX31865
    type_: RtdType
    wires: RtdWires
    cal: CalPtT

    def __init__(self, spi, cs, type_: RtdType, wires_: RtdWires):
        if type_ == RtdType.PT100:
            rtd_nominal = 100
            ref_resistor = 300
        elif type_ == RtdType.PT1000:
            rtd_nominal = 1_000
            ref_resistor = 3_000
        else:
            raise TypeError("Invalid RtdType: Must be `RtdType.PT100`, or `RtdType.PT1000`.")

        if wires_ == RtdWires.TWO:
            wires = 2
        elif wires_ == RtdWires.THREE:
            wires = 3
        elif wires_ == RtdWires.FOUR:
            wires = 4
        else:
            raise TypeError("Invalid RtdWires: Must be `RtdWires.TWO`, `RtdWires.THREE`, or `RtdWires.FOUR`.")

        self.sensor = MAX31865(
            spi,
            cs,
            wires=wires,
            rtd_nominal=rtd_nominal,
            ref_resistor=ref_resistor
        )

    def read(self) -> float:
        """Read measured temperature"""
        return self.sensor.temperature

    def read_resistance(self) -> float:
        """Read measured resistance of the RTD"""
        return self.sensor.resistance

    def calibrate(self) -> None:
        pass


@dataclass
class Readings:
    # todo: Should these (And the readings in general) be Optional[float] to deal
    # todo with hardware errors?
    pH: Optional[float]
    T: Optional[float]
    ec: Optional[float]
    ORP: Optional[float]

    @classmethod
    def from_bytes(cls, buf: bytes) -> 'Readings':
        """Read a 20-byte set. Each reading is 5 bytes: 1 for ok/error, the other
        4 for a float."""
        result = cls(None, None, None, None)

        if buf[0] == OK_BIT:
            result.T = struct.unpack('f', buf[1:5])

        if buf[5] == OK_BIT:
            result.pH = struct.unpack('f', buf[6:10])

        if buf[10] == OK_BIT:
            result.ORP = struct.unpack('f', buf[11:15])

        if buf[15] == OK_BIT:
            result.ec = struct.unpack('f', buf[16:20])

        return result


class CellConstant(Enum):
    """Cell constant, in 1/cm. Aka, K. Numerical value is the serialized bit value."""
    K0_01 = 0
    K0_1 = 1
    K1_0 = 2
    K10 = 3

    def value(self) -> float:
        """Return the conductivity value"""
        if self == CellConstant.K0_01:
            return 0.01
        elif self == CellConstant.K0_1:
            return 0.1
        elif self == CellConstant.K1_0:
            return 1.
        else:
            return 10.

class ExcMode(Enum):
    """Excitation mode: Always on, or only when measuring. Numerical value is
    the serialized bit value. This mirrors the rust equivalent"""
    READING_ONLY = 0,
    ALWAYS_ON = 1


@dataclass
class EcSensor:
    """An interface for our EC module, which communicates a serial message over UART."""
    ser: serial.Serial
    K: CellConstant
    cal: Optional[CalPtEc]
    excitation_mode: ExcMode


    def __init__(self, K: float=1.0, cal: Optional[CalPtEc]=None, exc_mode = ExcMode.READING_ONLY):
        self.ser = serial.Serial('/dev/ttyAMA0', 19200, timeout=10)
        # self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=10)

        if K == 0.01:
            self.K = CellConstant.K0_01
        elif K == 0.1:
            self.K = CellConstant.K0_1
        elif K == 1.0:
            self.K = CellConstant.K1_0
        elif K == 10.0:
            self.K = CellConstant.K10
        else:
            raise AttributeError("Cell constant (K) must be 0.01, 0.1, 1.0, or 10.0.")

        self.set_K(self.K)

        self.cal = cal

        self.excitation_mode = exc_mode
        self.set_excitation_mode(self.excitation_mode)

    def read(self) -> float:
        """Take an ec reading"""
        T = self.read_temp()
        T = self.read_temp()

        for _ in range(SERIAL_TIMEOUT):
            self.ser.write(MSG_START_BITS + 10 + [0, 0, 0, 0, 0, 0, 0] + MSG_END_BITS)
            response = self.ser.read(READINGS_SIZE_EC)
            if response:
                ec = response * self.K.value()  # µS/cm
                # todo: Calibration, temp compensation, and units

                return ec_from_voltage(ec, T)

    def read_temp(self) -> float:
        """Take an reading from the onboard air temperature sensor"""
        # todo DRY
        for _ in range(SERIAL_TIMEOUT):
            self.ser.write(MSG_START_BITS + 11 + [0, 0, 0, 0, 0, 0, 0] + MSG_END_BITS)
            response = self.ser.read(READINGS_SIZE_EC)
            if response:
                return Readings.from_bytes(response)

    def set_excitation_mode(self, mode: ExcMode) -> float:
        """Set probe conductivity constant"""
        # todo: Dry message sending
        self.K = K
        for _ in range(SERIAL_TIMEOUT):
            self.ser.write(MSG_START_BITS + 12 + int(mode) + [0, 0, 0, 0, 0] + MSG_END_BITS)
            response = self.ser.read(READINGS_SIZE_EC)
            if response:
               return # todo errors etc

        raise AttributeError("Problem getting data.")

    def set_K(self, K: CellConstant) -> float:
        """Set probe conductivity constant"""
        # todo: Dry message sending
        self.K = K
        for _ in range(SERIAL_TIMEOUT):
            self.ser.write(MSG_START_BITS + 13 + int(K) + [0, 0, 0, 0, 0] + MSG_END_BITS)
            response = self.ser.read(READINGS_SIZE_EC)
            if response:
                return # todo errors etc

        raise AttributeError("Problem getting data.")


@dataclass
class WaterMonitor:
    """We use this to pull readings from the Water Monitor over a USB COM serial connection.
    Note that this is not an equivalent for the `WaterMonitor` struct in the Rust drivers."""
    ser: serial.Serial

    def __init__(self):
        for port in list_ports.comports():
            # We set this serial number in the Water Monitor firmware.
            if port.serial_number == "WM":
                self.ser = serial.Serial(port.device, 9_600, timeout=10)
                return
        raise RuntimeError("Unable to find the Water Monitor. Is it connected over USB?")

    def read_all(self) -> Readings:
        """Read all sensors."""
        for _ in range(SERIAL_TIMEOUT):
            self.ser.write(MSG_START_BITS + MSG_END_BITS)
            response = self.ser.read(READINGS_SIZE_WM)
            if response:
                return Readings.from_bytes(response)

        raise AttributeError("Problem getting data.")

    # Alternatively, we could have special data requests for each reading, and only
    # send what's required, but this approach works, and doesn't have many practical
    # downsides.
    def read_ph(self) -> float:
        return self.read_all().pH

    def read_temp(self) -> float:
        return self.read_all().T

    def read_orp(self) -> float:
        return self.read_all().ORP

    def read_ec(self) -> float:
        return self.read_all().ec

    def close(self) -> None:
        """Close the serial port, leaving it available for other applications."""
        self.ser.close()


def lg(
    pt0: (float, float), pt1: (float, float), pt2: (float, float), X: float
) -> float:
    """Compute the result of a Lagrange polynomial of order 3.
Algorithm created from the `P(x)` eq
[here](https://mathworld.wolfram.com/LagrangeInterpolatingPolynomial.html)."""
    result = 0.0

    x = [pt0[0], pt1[0], pt2[0]]
    y = [pt0[1], pt1[1], pt2[1]]

    for j in range(3):
        c = 1.0
        for i in range(3):
            if j == i:
                continue
            c *= (X - x[i]) / (x[j] - x[i])
        result += y[j] * c

    return result


def ph_from_voltage(
    V: float, T: float, cal_0: CalPt, cal_1: CalPt, cal_2: Optional[CalPt],
) -> float:
    """Convert voltage to pH
    We model the relationship between sensor voltage and pH linearly
    using 2-pt calibration, or quadratically using 3-pt. Temperature
    compensated. Input `T` is in Celsius."""

    # We infer a -.05694 pH/(V*T) sensitivity linear relationship
    # (higher temp means higher pH/V ratio)
    T_diff = T - cal_0.T
    T_comp = PH_TEMP_C * T_diff  # pH / V

    if cal_2:
        result = lg((cal_0.V, cal_0.pH), (cal_1.V, cal_1.pH), (cal_2.V, cal_2.pH), V)
        return result + T_comp * V
    else:
        a = (cal_1.pH - cal_0.pH) / (cal_1.V - cal_0.V)
        b = cal_1.pH - a * cal_1.V
        return (a + T_comp) * V + b


def orp_from_voltage(V: float, cal: CalPtOrp) -> float:
    """Convert sensor voltage to ORP voltage
    We model the relationship between sensor voltage and pH linearly
    between the calibration point, and (0., 0.). Output is in mV."""
    a = cal.ORP / cal.V
    b = cal.ORP - a * cal.V
    return a * V + b

def ec_from_voltage(reading: float, cal: Optional[CalPtEc]) -> float:
    """Convert sensor voltage to ORP voltage
    We model the relationship between sensor voltage and pH linearly
    between the calibration point, and (0., 0.). Output is in mV."""
    if cal:
        a = cal.ec / cal.reading
        b = cal.ec - a * cal.reading
        return a * reading + b
    else:
        return reading


def temp_from_voltage(V: float) -> float:
    """Map voltage to temperature for the TI LM61, in °C
    Datasheet: https://datasheet.lcsc.com/szlcsc/Texas-Instruments-
    TI-LM61BIM3-NOPB_C132073.pdf"""
    return 100.0 * V - 60.0
