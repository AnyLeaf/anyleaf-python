# Driver for the Anyleaf pH module

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, List, Union

import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_max31865 import MAX31865

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from . import filter


DISCRETE_PH_JUMP_THRESH = 0.2
DISCRETE_ORP_JUMP_THRESH = 30.0
# Compensate for temperature diff between readings and calibration.
PH_TEMP_C = -0.05694  # pH/(V*T). V is in volts, and T is in °C


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
        self.cal_1 = CalPt(0, 7.0, 23)
        self.cal_2 = CalPt(0.17, 4.0, 23)
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
        self.cal_1 = CalPt(0.0, 7.0, 25.0)
        self.cal_2 = CalPt(0.17, 4.0, 25.0)
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
    pH: float
    T: float
    ec: float
    ORP: float

#
# @dataclass
# class WaterMonitor:
#     """We use this to pull data from the Water Monitor to an external program over I2C.
#     It interacts directly with the ADCs, and has no interaction to the Water Monitor's MCU."""
#     ph_temp: PhSensor  # at 0x48. Inludes the temp sensor at input A3.
#     orp_ec: OrpSensor  # at 0x49. Inlucdes the ec sensor at input A3.
#
#     def __init__(self, i2c, dt: float):
#         self.ph_temp = PhSensor(i2c, dt)
#         self.orp_ec = OrpSensor(i2c, dt, address=0x49)
#
#     def read_all(self) -> Readings:
#         """Read all sensors."""
#         T = self.ph_temp.read_temp_pt100()
#         pH = self.ph_temp.read(OffBoard(T))
#         ORP = self.orp_ec.read()
#         ec = self.orp_ec.read_ec(OffBoard(T))
#
#         return Readings(pH, T, ec, ORP)
#
#     def read_ph(self) -> float:
#         t = OffBoard(self.ph_temp.read_temp_pt100())
#         return self.ph_temp.read(t)
#
#     def read_temp(self) -> float:
#         return self.ph_temp.read_temp_pt100()
#
#     def read_orp(self) -> float:
#         return self.orp_ec.read()
#
#     def read_ec(self) -> float:
#         t = OffBoard(self.ph_temp.read_temp_pt100())
#         return sself.orp_ec.read_ec(t)


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


def temp_from_voltage(V: float) -> float:
    """Map voltage to temperature for the TI LM61, in °C
    Datasheet: https://datasheet.lcsc.com/szlcsc/Texas-Instruments-
    TI-LM61BIM3-NOPB_C132073.pdf"""
    return 100.0 * V - 60.0
