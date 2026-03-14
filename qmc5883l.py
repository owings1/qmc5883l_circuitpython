"""
https://github.com/owings1/qmc5883l_circuitpython

QMC5883L 3-Axis Magnetic Sensor CircuitPython I2C Driver Implementation

Copyright (C) 2026 Doug Owings

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

MICROTESLAS_PER_GAUSS = const(100)
QMC5883L_DEFAULT_ADDRESS = const(0x0D)

# Oversampling
OSR_512 = const(0x00)
OSR_256 = const(0x01)
OSR_128 = const(0x02)
OSR_64 = const(0x03)

# Range
RANGE_2G = const(0x00)
RANGE_8G = const(0x01)
SENSITIVITY_VALUES = const((12_000, 3_000))

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

_DATA_FORMAT = '<3h'
_TOUT_FORMAT = '<H'

_DEFAULT_CONFIG = (
  (OSR_512 << CTRL1_OSR_LOWEST_BIT) |
  (RANGE_8G << CTRL1_RANGE_LOWEST_BIT) |
  (ODR_200HZ << CTRL1_ODR_LOWEST_BIT) |
  (MODE_CONTINUOUS << CTRL1_MODE_LOWEST_BIT))

_SETUP_CMD = struct.pack('2B', _FBR_REGISTER, 0x01)

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

  def __init__(self, i2c: I2C, address: int = QMC5883L_DEFAULT_ADDRESS) -> None:
    self.i2c_device = I2CDevice(i2c, address)
    self.buf = bytearray(1 + struct.calcsize(_DATA_FORMAT))
    self.buf[0] = _DATA_REGISTER
    self.setup()

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
  def sensitivity(self) -> int:
    try:
      return SENSITIVITY_VALUES[self.range]
    except KeyError:
      raise AttributeError

  @property
  def magnetic_raw(self) -> tuple[int, int, int]:
    if self.data_ready and not self.data_overflow:
      with self.i2c_device as device:
        device.write_then_readinto(
          self.buf,
          self.buf,
          out_end=1,
          in_start=1)
    return struct.unpack_from(_DATA_FORMAT, self.buf, 1)

  @property
  def magnetic_gauss(self) -> tuple[float, float, float]:
    scale = 1 / self.sensitivity
    return tuple(x * scale for x in self.magnetic_raw)

  @property
  def magnetic_ut(self) -> tuple[float, float, float]:
    scale = MICROTESLAS_PER_GAUSS
    return tuple(x * scale for x in self.magnetic_gauss)
