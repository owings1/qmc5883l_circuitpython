"""
QMC5883L 3-Axis Magnetic Sensor CircuitPython I2C Driver

https://github.com/owings1/qmc5883l_circuitpython

* Author: Doug Owings
* License: MIT License

Copyright (C) 2026 Doug Owings. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
======================================================================

Dependencies:

* Adafruit's Bus Device library:
  https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

* Adafruit's Register library:
  https://github.com/adafruit/Adafruit_CircuitPython_Register

Resources:

* Hardware Datasheet:
  https://www.qstcorp.com/upload/pdf/202202/%EF%BC%88%E5%B7%B2%E4%BC%A0%EF%BC%8913-52-04%20QMC5883L%20Datasheet%20Rev.%20A(1).pdf

* Adafruit CircuitPython:
  https://circuitpython.org/downloads
"""
from __future__ import annotations

import struct

from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_bits import ROBits, RWBits
from adafruit_register.i2c_struct import ROUnaryStruct, UnaryStruct
from micropython import const

try:
  from busio import I2C
except ImportError:
  pass

__version__ = '0.0.2'
__repo__ = 'https://github.com/owings1/qmc5883l_circuitpython'

QMC5883L_DEFAULT_ADDRESS = const(0x0D)
QMC5883L_CHIP_ID = const(0xFF)

# Oversampling
OSR_512 = const(0x00)
OSR_256 = const(0x01)
OSR_128 = const(0x02)
OSR_64 = const(0x03)

# Range/Gain
RANGE_2G = const(0x00)
RANGE_8G = const(0x01)
GAIN_VALUES = const((12_000, 3_000))

# Output Data Rate
ODR_10HZ = const(0x00)
ODR_50HZ = const(0x01)
ODR_100HZ = const(0x02)
ODR_200HZ = const(0x03)

# Mode
MODE_STANDBY = const(0x00)
MODE_CONTINUOUS = const(0x01)

_DATA_REGISTER = const(0x00)
_STATUS_REGISTER = const(0x06)
_TOUT_REGISTER = const(0x07)
_CTRL1_REGISTER = const(0x09)
_CTRL2_REGISTER = const(0x0A)
_FBR_REGISTER = const(0x0B)
_ID_REGISTER = const(0x0D)

STATUS_DRDY_BIT = const(0x00)
STATUS_OVL_BIT = const(0x01)
STATUS_DOR_BIT = const(0x02)
CTRL1_OSR_LOWEST_BIT = const(0x06)
CTRL1_RANGE_LOWEST_BIT = const(0x04)
CTRL1_ODR_LOWEST_BIT = const(0x02)
CTRL1_MODE_LOWEST_BIT = const(0x00)
CTRL2_SOFT_RST_BIT = const(0x07)
CTRL2_ROL_PNT_BIT = const(0x06)
CTRL2_INT_ENB_BIT = const(0x00)

_DATA_FORMAT = const('<3h')
_TOUT_FORMAT = const('<H')

FLAG_CALIBRATED = const(0x40)
_STATUS_IDX = const(0x07)
_FLAGS_IDX = const(0x08)
_CALIB_IDX = const(0x09)
_DEFAULT_CONFIG = const(0b10001) # OSR_512, RANGE_8G, ODR_200HZ, MODE_CONTINUOUS
_SETUP_CMD = const(b'\x0b\x01')

class QMC5883L:
  osr = RWBits(2, _CTRL1_REGISTER, CTRL1_OSR_LOWEST_BIT)
  range = RWBits(1, _CTRL1_REGISTER, CTRL1_RANGE_LOWEST_BIT)
  odr = RWBits(2, _CTRL1_REGISTER, CTRL1_ODR_LOWEST_BIT)
  mode = RWBits(2, _CTRL1_REGISTER, CTRL1_MODE_LOWEST_BIT)
  data_ready = ROBits(1, _STATUS_REGISTER, STATUS_DRDY_BIT)
  data_overflow = ROBits(1, _STATUS_REGISTER, STATUS_OVL_BIT)
  data_skip = ROBits(1, _STATUS_REGISTER, STATUS_DOR_BIT)
  temperature = ROUnaryStruct(_TOUT_REGISTER, _TOUT_FORMAT)
  chip_id = ROUnaryStruct(_ID_REGISTER, 'B')
  ctrl1_flags = UnaryStruct(_CTRL1_REGISTER, 'B')
  ctrl2_flags = UnaryStruct(_CTRL2_REGISTER, 'B')
  _int_enb = RWBits(1, _CTRL2_REGISTER, CTRL2_INT_ENB_BIT)
  _do_soft_reset = RWBits(1, _CTRL2_REGISTER, CTRL2_SOFT_RST_BIT)

  def __init__(self, i2c: I2C, address: int = QMC5883L_DEFAULT_ADDRESS, verify: bool = True) -> None:
    self.i2c_device = I2CDevice(i2c, address)
    # Buffer: [Reg] + [6 Data] + [Status] + [Flags] + [12 Calibration] = 21 bytes
    self.buf = bytearray(21)
    self.buf[0] = _DATA_REGISTER
    self.calibration = None
    if verify:
      self.verify()
    self.setup()

  def verify(self) -> None:
    if self.chip_id != QMC5883L_CHIP_ID:
      raise RuntimeError('Failed to find QMC5883L. Check address or chip type.')

  def setup(self) -> None:
    with self.i2c_device as device:
      device.write(_SETUP_CMD)
    self.ctrl1_flags = _DEFAULT_CONFIG

  def soft_reset(self) -> None:
    self._do_soft_reset = True
    self.setup()

  @property
  def interrupt_enabled(self) -> bool:
    return not self._int_enb

  @interrupt_enabled.setter
  def interrupt_enabled(self, value: bool) -> None:
    self._int_enb = not value

  @property
  def gain(self) -> int:
    return GAIN_VALUES[self.range]

  @property
  def overflow(self) -> bool:
    """
    Whether the last measurement was an overflow.
    """
    return bool(self.buf[_STATUS_IDX] & STATUS_OVL_BIT)

  @property
  def magnetic(self) -> tuple[float, float, float]:
    """
    The measurement values (X, Y, Z) in Gauss units with calibration
    offsets applied.
    """
    s = 1 / self.gain
    raw = self._magnetic_raw
    offset, scale = self.calibration
    return (
      (raw[0] * s - offset[0]) * scale[0],
      (raw[1] * s - offset[1]) * scale[1],
      (raw[2] * s - offset[2]) * scale[2])

  @property
  def _magnetic_raw(self) -> tuple[int, int, int]:
    if self.data_ready:
      with self.i2c_device as device:
        device.write_then_readinto(
          self.buf,
          self.buf,
          out_end=1,
          in_start=1,
          in_end=8)
    return struct.unpack_from(_DATA_FORMAT, self.buf, 1)

  @property
  def flags(self) -> int:
    return self.buf[_FLAGS_IDX]

  @property
  def calibrated(self) -> bool:
    """
    True if the calibration data has been set.
    """
    return bool(self.flags & FLAG_CALIBRATED)

  @property
  def calibration(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """
    The current calibration data as ((off_x, off_y, off_z), (scale_x, scale_y, scale_z)).
    Use this to save/restore calibration without re-running min/max routines.
    """
    return (
      struct.unpack_from('<3e', self.buf, _CALIB_IDX),
      struct.unpack_from('<3e', self.buf, _CALIB_IDX+6))

  @calibration.setter
  def calibration(self, data: tuple[tuple[float, float, float], tuple[float, float, float]]|None) -> None:
    if data is None:
      struct.pack_into('<6e', self.buf, _CALIB_IDX, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0)
      self.buf[_FLAGS_IDX] &= ~FLAG_CALIBRATED
    else:
      offset, scale = data
      struct.pack_into('<6e', self.buf, _CALIB_IDX, *offset, *scale)
      self.buf[_FLAGS_IDX] |= FLAG_CALIBRATED

  def calibrate(self, xmin: float, xmax: float, ymin: float, ymax: float, zmin: float, zmax: float) -> None:
    """
    Corrects for local magnetic interference (metal, batteries) using 
    the minimum and maximum Gauss values observed while rotating the 
    sensor through all possible orientations.
    """
    # 1. Hard-Iron Offset: The center of the sphere
    offset = (
      (xmax + xmin) / 2,
      (ymax + ymin) / 2,
      (zmax + zmin) / 2)
    # 2. Soft-Iron Scale: Makes the sphere perfectly round
    # Calculate the range (width) of each axis
    dx, dy, dz = (xmax - xmin), (ymax - ymin), (zmax - zmin)
    if dx <= 0 or dy <= 0 or dz <= 0:
      raise ValueError('Non-positive axis width')
    avg_d = (dx + dy + dz) / 3
    scale = (avg_d / dx, avg_d / dy, avg_d / dz)
    self.calibration = offset, scale
