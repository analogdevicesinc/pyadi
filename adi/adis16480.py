# Copyright (C) 2019-2024 Analog Devices, Inc.
#
# SPDX short identifier: ADIBSD

from adi.attribute import attribute
import numpy as np
from adi.context_manager import context_manager
from adi.rx_tx import rx


class adis16480(rx, context_manager):
    """ ADIS16480 Ten Degrees of Freedom Inertial Sensor with Dynamic Orientation Outputs """

    _complex_data = False
    _rx_channel_names = [
        "anglvel_x",
        "anglvel_y",
        "anglvel_z",
        "accel_x",
        "accel_y",
        "accel_z",
        "magn_x",
        "magn_y",
        "magn_z",
        "pressure0",
        "temp0",
    ]
    _device_name = ""

    def __init__(self, uri="", device_name="adis16480"):
        context_manager.__init__(self, uri, self._device_name)

        compatible_parts = [
            "adis16375",
            "adis16480",
            "adis16485",
            "adis16488",
            "adis16490",
            "adis16495-1",
            "adis16495-2",
            "adis16495-3",
            "adis16497-1",
            "adis16497-2",
            "adis16497-3",
        ]

        if device_name not in compatible_parts:
            raise Exception(
                "Not a compatible device:"
                + str(device_name)
                + ".Please select from:"
                + str(compatible_parts)
            )
        else:
            self._ctrl = self._ctx.find_device(device_name)
            self._rxadc = self._ctx.find_device(device_name)
        
        if self._ctrl is None:
            print(
                "No device found with device_name = "
                + device_name
                + ". Searching for a device found in the compatible list."
            )
            for i in compatible_parts:
                self._ctrl = self._ctx.find_device(i)
                self._rxadc = self._ctx.find_device(i)
                if self._ctrl is not None:
                    print("Fond device = " + i + ". Will use this device instead.")
                    break
            if self._ctrl is None:
                raise Exception("No compatible device found")

        self.anglvel_x = self._anglvel_accel_channels(self._ctrl, "anglvel_x")
        self.anglvel_y = self._anglvel_accel_channels(self._ctrl, "anglvel_y")
        self.anglvel_z = self._anglvel_accel_channels(self._ctrl, "anglvel_z")
        self.accel_x = self._anglvel_accel_channels(self._ctrl, "accel_x")
        self.accel_y = self._anglvel_accel_channels(self._ctrl, "accel_y")
        self.accel_z = self._anglvel_accel_channels(self._ctrl, "accel_z")
        self.temp = self._temp_channel(self._ctrl, "temp0")
        self.pressure = self._pressure_channel(self._ctrl, "pressure0")
        self.magn_x = self._magn_channel(self._ctrl, "magn_x")
        self.magn_y = self._magn_channel(self._ctrl, "magn_y")
        self.magn_z = self._magn_channel(self._ctrl, "magn_z")

        rx.__init__(self)
        self.rx_buffer_size = 16  # Make default buffer smaller

    def __get_scaled_sensor(self, channel_name: str) -> float:
        raw = self._get_iio_attr(channel_name, "raw", False)
        scale = self._get_iio_attr(channel_name, "scale", False)

        return raw * scale
    
    def __get_scaled_sensor_temp(self, channel_name: str) -> float:
        raw = self._get_iio_attr(channel_name, "raw", False)
        scale = self._get_iio_attr(channel_name, "scale", False)
        offset = self._get_iio_attr(channel_name, "offset", False)

        return (raw + offset) * scale
    
    def get_anglvel_x(self):
        """Value returned in radians per second."""
        return self.__get_scaled_sensor("anglvel_x")

    anglvel_x_conv = property(get_anglvel_x, None)

    def get_anglvel_y(self):
        """Value returned in radians per second."""
        return self.__get_scaled_sensor("anglvel_y")

    anglvel_y_conv = property(get_anglvel_y, None)

    def get_anglvel_z(self):
        """Value returned in radians per second."""
        return self.__get_scaled_sensor("anglvel_z")

    anglvel_z_conv = property(get_anglvel_z, None)

    def get_accel_x(self):
        """Value returned in meters per squared second."""
        return self.__get_scaled_sensor("accel_x")

    accel_x_conv = property(get_accel_x, None)

    def get_accel_y(self):
        """Value returned in meters per squared second."""
        return self.__get_scaled_sensor("accel_y")

    accel_y_conv = property(get_accel_y, None)

    def get_accel_z(self):
        """Value returned in meters per squared second."""
        return self.__get_scaled_sensor("accel_z")

    accel_z_conv = property(get_accel_z, None)

    def get_magn_x(self):
        """Value returned in radians."""
        return self.__get_scaled_sensor("magn_x")

    magn_x_conv = property(get_magn_x, None)

    def get_magn_y(self):
        """Value returned in radians."""
        return self.__get_scaled_sensor("magn_y")

    magn_y_conv = property(get_magn_y, None)

    def get_magn_z(self):
        """Value returned in radians."""
        return self.__get_scaled_sensor("magn_z")

    magn_z_conv = property(get_magn_z, None)

    def get_temp(self):
        """Value returned in millidegrees Celsius."""
        return self.__get_scaled_sensor_temp("temp0")
    
    temp_conv = property(get_temp, None)

    def get_pressure(self):
        """Value returned in kilo Pascal."""
        return self.__get_scaled_sensor("pressure0")
    
    pressure_conv = property(get_pressure, None)
    
    @property
    def sample_rate(self):
        """sample_rate: Sample rate in samples per second"""
        return self._get_iio_dev_attr("sampling_frequency")

    @sample_rate.setter
    def sample_rate(self, value):
        self._set_iio_dev_attr_str("sampling_frequency", value)

    @property
    def current_timestamp_clock(self):
        """current_timestamp_clock: Source clock for timestamps"""
        return self._get_iio_dev_attr("current_timestamp_clock")

    @current_timestamp_clock.setter
    def current_timestamp_clock(self, value):
        self._set_iio_dev_attr_str("current_timestamp_clock", value)
    
    @property
    def anglvel_x_calibbias(self):
        """User calibration offset for gyroscope for the x-axis."""
        return self._get_iio_attr("anglvel_x", "calibbias", False)

    @anglvel_x_calibbias.setter
    def anglvel_x_calibbias(self, value):
        self._set_iio_attr("anglvel_x", "calibbias", False, value)

    @property
    def anglvel_y_calibbias(self):
        """User calibration offset for gyroscope for the y-axis."""
        return self._get_iio_attr("anglvel_y", "calibbias", False)

    @anglvel_y_calibbias.setter
    def anglvel_y_calibbias(self, value):
        self._set_iio_attr("anglvel_y", "calibbias", False, value)

    @property
    def anglvel_z_calibbias(self):
        """User calibration offset for gyroscope for the z-axis."""
        return self._get_iio_attr("anglvel_z", "calibbias", False)

    @anglvel_z_calibbias.setter
    def anglvel_z_calibbias(self, value):
        self._set_iio_attr("anglvel_z", "calibbias", False, value)

    @property
    def accel_x_calibbias(self):
        """User calibration offset for accelerometer for the x-axis."""
        return self._get_iio_attr("accel_x", "calibbias", False)

    @accel_x_calibbias.setter
    def accel_x_calibbias(self, value):
        self._set_iio_attr("accel_x", "calibbias", False, value)

    @property
    def accel_y_calibbias(self):
        """User calibration offset for accelerometer for the y-axis."""
        return self._get_iio_attr("accel_y", "calibbias", False)

    @accel_y_calibbias.setter
    def accel_y_calibbias(self, value):
        self._set_iio_attr("accel_y", "calibbias", False, value)

    @property
    def accel_z_calibbias(self):
        """User calibration offset for accelerometer for the z-axis."""
        return self._get_iio_attr("accel_z", "calibbias", False)

    @accel_z_calibbias.setter
    def accel_z_calibbias(self, value):
        self._set_iio_attr("accel_z", "calibbias", False, value)

    @property
    def magn_x_calibbias(self):
        """User calibration offset for magnetometer for the x-axis."""
        return self._get_iio_attr("magn_x", "calibbias", False)

    @magn_x_calibbias.setter
    def magn_x_calibbias(self, value):
        self._set_iio_attr("magn_x", "calibbias", False, value)

    @property
    def magn_y_calibbias(self):
        """User calibration offset for magnetometer for the y-axis."""
        return self._get_iio_attr("magn_y", "calibbias", False)

    @magn_y_calibbias.setter
    def magn_y_calibbias(self, value):
        self._set_iio_attr("magn_y", "calibbias", False, value)

    @property
    def magn_z_calibbias(self):
        """User calibration offset for magnetometer for the z-axis."""
        return self._get_iio_attr("magn_z", "calibbias", False)

    @magn_z_calibbias.setter
    def magn_z_calibbias(self, value):
        self._set_iio_attr("magn_z", "calibbias", False, value)

    @property
    def pressure_calibbias(self):
        """User calibration offset for pressure."""
        return self._get_iio_attr("pressure0", "calibbias", False)

    @pressure_calibbias.setter
    def pressure_calibbias(self, value):
        self._set_iio_attr("pressure0", "calibbias", False, value)

    #####
    @property
    def anglvel_x_calibscale(self):
        """Calibscale value for gyroscope for the x-axis."""
        return self._get_iio_attr("anglvel_x", "calibscale", False)

    @anglvel_x_calibscale.setter
    def anglvel_x_calibscale(self, value):
        self._set_iio_attr("anglvel_x", "calibscale", False, value)

    @property
    def anglvel_y_calibscale(self):
        """Calibscale value for gyroscope for the y-axis."""
        return self._get_iio_attr("anglvel_y", "calibscale", False)

    @anglvel_y_calibscale.setter
    def anglvel_y_calibscale(self, value):
        self._set_iio_attr("anglvel_y", "calibscale", False, value)

    @property
    def anglvel_z_calibscale(self):
        """Calibscale value for gyroscope for the z-axis."""
        return self._get_iio_attr("anglvel_z", "calibscale", False)

    @anglvel_z_calibscale.setter
    def anglvel_z_calibscale(self, value):
        self._set_iio_attr("anglvel_z", "calibscale", False, value)

    @property
    def accel_x_calibscale(self):
        """Calibscale value for accelerometer for the x-axis."""
        return self._get_iio_attr("accel_x", "calibscale", False)

    @accel_x_calibscale.setter
    def accel_x_calibscale(self, value):
        self._set_iio_attr("accel_x", "calibscale", False, value)

    @property
    def accel_y_calibscale(self):
        """Calibcale value for accelerometer for the y-axis."""
        return self._get_iio_attr("accel_y", "calibscale", False)

    @accel_y_calibscale.setter
    def accel_y_calibscale(self, value):
        self._set_iio_attr("accel_y", "calibscale", False, value)

    @property
    def accel_z_calibscale(self):
        """Calibscale for accelerometer for the z-axis."""
        return self._get_iio_attr("accel_z", "calibscale", False)

    @accel_z_calibscale.setter
    def accel_z_calibscale(self, value):
        self._set_iio_attr("accel_z", "calibscale", False, value)

    @property
    def anglvel_x_filter_low_pass_3db_frequency(self):
        """Bandwidth for gyroscope for the x-axis."""
        return self._get_iio_attr("anglvel_x", "filter_low_pass_3db_frequency", False)

    @anglvel_x_filter_low_pass_3db_frequency.setter
    def anglvel_x_filter_low_pass_3db_frequency(self, value):
        self._set_iio_attr("anglvel_x", "filter_low_pass_3db_frequency", False, value)

    @property
    def anglvel_y_filter_low_pass_3db_frequency(self):
        """Bandwidth for gyroscope for the y-axis."""
        return self._get_iio_attr("anglvel_y", "filter_low_pass_3db_frequency", False)

    @anglvel_y_filter_low_pass_3db_frequency.setter
    def anglvel_y_filter_low_pass_3db_frequency(self, value):
        self._set_iio_attr("anglvel_y", "filter_low_pass_3db_frequency", False, value)

    @property
    def anglvel_z_filter_low_pass_3db_frequency(self):
        """Bandwidth for gyroscope for the z-axis."""
        return self._get_iio_attr("anglvel_z", "filter_low_pass_3db_frequency", False)

    @anglvel_z_filter_low_pass_3db_frequency.setter
    def anglvel_z_filter_low_pass_3db_frequency(self, value):
        self._set_iio_attr("anglvel_z", "filter_low_pass_3db_frequency", False, value)

    @property
    def accel_x_filter_low_pass_3db_frequency(self):
        """Bandwidth for accelerometer for the x-axis."""
        return self._get_iio_attr("accel_x", "filter_low_pass_3db_frequency", False)

    @accel_x_filter_low_pass_3db_frequency.setter
    def accel_x_filter_low_pass_3db_frequency(self, value):
        self._set_iio_attr("accel_x", "filter_low_pass_3db_frequency", False, value)

    @property
    def accel_y_filter_low_pass_3db_frequency(self):
        """Bandwidth for accelerometer for the y-axis."""
        return self._get_iio_attr("accel_y", "filter_low_pass_3db_frequency", False)

    @accel_y_filter_low_pass_3db_frequency.setter
    def accel_y_filter_low_pass_3db_frequency(self, value):
        self._set_iio_attr("accel_y", "filter_low_pass_3db_frequency", False, value)

    @property
    def accel_z_filter_low_pass_3db_frequency(self):
        """Bandwidth for accelerometer for the z-axis."""
        return self._get_iio_attr("accel_z", "filter_low_pass_3db_frequency", False)

    @accel_z_filter_low_pass_3db_frequency.setter
    def accel_z_filter_low_pass_3db_frequency(self, value):
        self._set_iio_attr("accel_z", "filter_low_pass_3db_frequency", False, value)

    @property
    def magn_x_filter_low_pass_3db_frequency(self):
        """Bandwidth for magnetometer for the x-axis."""
        return self._get_iio_attr("magn_x", "filter_low_pass_3db_frequency", False)

    @magn_x_filter_low_pass_3db_frequency.setter
    def magn_x_filter_low_pass_3db_frequency(self, value):
        self._set_iio_attr("magn_x", "filter_low_pass_3db_frequency", False, value)

    @property
    def magn_y_filter_low_pass_3db_frequency(self):
        """Bandwidth for magnetometer for the y-axis."""
        return self._get_iio_attr("magn_y", "filter_low_pass_3db_frequency", False)

    @magn_y_filter_low_pass_3db_frequency.setter
    def magn_y_filter_low_pass_3db_frequency(self, value):
        self._set_iio_attr("magn_y", "filter_low_pass_3db_frequency", False, value)

    @property
    def magn_z_filter_low_pass_3db_frequency(self):
        """Bandwidth for magnetometer for the z-axis."""
        return self._get_iio_attr("magn_z", "filter_low_pass_3db_frequency", False)

    @magn_z_filter_low_pass_3db_frequency.setter
    def magn_z_filter_low_pass_3db_frequency(self, value):
        self._set_iio_attr("magn_z", "filter_low_pass_3db_frequency", False, value)

    class _temp_channel(attribute):
        """ADIS16480 temperature channel."""
        
        def __init__(self, ctrl, channel_name):
            self.name = channel_name
            self._ctrl = ctrl

        @property
        def raw(self):
            """ADIS16480 raw value"""
            return self._get_iio_attr(self.name, "raw", False)
        
        @property
        def scale(self):
            """ADIS16480 scale value"""
            return self._get_iio_attr(self.name, "scale", False)
        
        @property
        def offset(self):
            """ADIS16480 offset value"""
            return self._get_iio_attr(self.name, "offset", False)
        
    class _pressure_channel(attribute):
        """ADIS16480 pressure channel."""
        
        def __init__(self, ctrl, channel_name):
            self.name = channel_name
            self._ctrl = ctrl

        @property
        def raw(self):
            """ADIS16480 raw value"""
            return self._get_iio_attr(self.name, "raw", False)
        
        @property
        def scale(self):
            """ADIS16480 scale value"""
            return self._get_iio_attr(self.name, "scale", False)
        
        @property
        def calibbias(self):
            """ADIS16480 calibration offset"""
            return self._get_iio_attr(self.name, "calibbias", False)

        @calibbias.setter
        def calibbias(self, value):
            self._set_iio_attr(self.name, "calibbias", False, value)
    
    class _magn_channel(_pressure_channel):
        """ADIS16480 magnetometer channel."""

        def __init__(self, ctrl, channel_name):
            self.name = channel_name
            self._ctrl = ctrl

        @property
        def filter_low_pass_3db_frequency(self):
            """ADIS16480 channel bandwidth"""
            return self._get_iio_attr(self.name, "filter_low_pass_3db_frequency", False)

        @filter_low_pass_3db_frequency.setter
        def filter_low_pass_3db_frequency(self, value):
            self._set_iio_attr(self.name, "filter_low_pass_3db_frequency", False, value)

    class _anglvel_accel_channels(_magn_channel):
        """ADIS16480 gyro and accelerometer channels."""

        def __init__(self, ctrl, channel_name):
            self.name = channel_name
            self._ctrl = ctrl
        
        @property
        def calibscale(self):
            """ADIS16480 calibscale value"""
            return self._get_iio_attr(self.name, "calibscale", False)

        @calibscale.setter
        def calibscale(self, value):
            self._set_iio_attr(self.name, "calibscale", False, value)
