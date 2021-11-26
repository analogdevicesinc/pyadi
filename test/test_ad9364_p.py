import pytest

hardware = ["fmcomms4", "adrv9364", "pluto", "adrv9361"]
classname = "adi.ad9364"

params = dict(
    one_cw_tone_manual=dict(
        tx_lo=2300000000,
        rx_lo=2300000000,
        sample_rate=30720000,
        tx_rf_bandwidth=18000000,
        rx_rf_bandwidth=18000000,
        gain_control_mode_chan0="manual",
        rx_hardwaregain_chan0=0,
        tx_hardwaregain_chan0=0,
    ),
    change_attenuation_10dB_manual=dict(
        tx_lo=2300000000,
        rx_lo=2300000000,
        sample_rate=30720000,
        tx_rf_bandwidth=18000000,
        rx_rf_bandwidth=18000000,
        gain_control_mode_chan0="manual",
        rx_hardwaregain_chan0=0,
        tx_hardwaregain_chan0=-10,
    ),
    change_attenuation_5dB_manual=dict(
        tx_lo=2300000000,
        rx_lo=2300000000,
        sample_rate=30720000,
        tx_rf_bandwidth=18000000,
        rx_rf_bandwidth=18000000,
        gain_control_mode_chan0="manual",
        rx_hardwaregain_chan0=0,
        tx_hardwaregain_chan0=-5,
    ),
    change_rf_gain_5dB_manual=dict(
        tx_lo=2300000000,
        rx_lo=2300000000,
        sample_rate=30720000,
        tx_rf_bandwidth=18000000,
        rx_rf_bandwidth=18000000,
        gain_control_mode_chan0="manual",
        rx_hardwaregain_chan0=5,
        tx_hardwaregain_chan0=0,
    ),
    one_cw_tone_slow_attack=dict(
        tx_lo=2300000000,
        rx_lo=2300000000,
        sample_rate=30720000,
        tx_rf_bandwidth=18000000,
        rx_rf_bandwidth=18000000,
        gain_control_mode_chan0="slow_attack",
        tx_hardwaregain_chan0=-10,
    ),
    change_sampling_rate_60MSPS_slow_attack=dict(
        tx_lo=2300000000,
        rx_lo=2300000000,
        sample_rate=60710000,
        tx_rf_bandwidth=18000000,
        rx_rf_bandwidth=18000000,
        gain_control_mode_chan0="slow_attack",
        tx_hardwaregain_chan0=-10,
    ),
    change_sampling_rate_15MSPS_slow_attack=dict(
        tx_lo=2300000000,
        rx_lo=2300000000,
        sample_rate=15000000,
        tx_rf_bandwidth=18000000,
        rx_rf_bandwidth=18000000,
        gain_control_mode_chan0="slow_attack",
        tx_hardwaregain_chan0=-10,
    ),
    change_attenuation_0dB_slow_attack=dict(
        tx_lo=2300000000,
        rx_lo=2300000000,
        sample_rate=30720000,
        tx_rf_bandwidth=18000000,
        rx_rf_bandwidth=18000000,
        gain_control_mode_chan0="slow_attack",
        tx_hardwaregain_chan0=0,
    ),
    change_attenuation_20dB_slow_attack=dict(
        tx_lo=2300000000,
        rx_lo=2300000000,
        sample_rate=30720000,
        tx_rf_bandwidth=18000000,
        rx_rf_bandwidth=18000000,
        gain_control_mode_chan0="slow_attack",
        tx_hardwaregain_chan0=-20,
    ),
)


#########################################
@pytest.mark.iio_hardware(hardware)
@pytest.mark.parametrize("classname", [(classname)])
@pytest.mark.parametrize(
    "attr, start, stop, step, tol",
    [
        ("tx_hardwaregain_chan0", -89.75, 0.0, 0.25, 0),
        ("rx_lo", 70000000, 6000000000, 1, 8),
        ("tx_lo", 47000000, 6000000000, 1, 8),
        ("sample_rate", 2084000, 61440000, 1, 4),
    ],
)
def test_ad9364_attr(
    test_attribute_single_value, iio_uri, classname, attr, start, stop, step, tol
):
    test_attribute_single_value(iio_uri, classname, attr, start, stop, step, tol)


#########################################
@pytest.mark.iio_hardware(hardware)
@pytest.mark.parametrize("classname", [(classname)])
@pytest.mark.parametrize("attr, tol", [("loopback", 0)])
@pytest.mark.parametrize("val", [0, 1, 2])
def test_ad9364_loopback_attr(
    test_attribute_single_value_str, iio_uri, classname, attr, val, tol
):
    test_attribute_single_value_str(iio_uri, classname, attr, val, tol)


#########################################
@pytest.mark.iio_hardware(hardware, True)
@pytest.mark.parametrize("classname", [(classname)])
@pytest.mark.parametrize("channel", [0])
def test_ad9364_rx_data(test_dma_rx, iio_uri, classname, channel):
    test_dma_rx(iio_uri, classname, channel)


#########################################
@pytest.mark.iio_hardware(hardware)
@pytest.mark.parametrize("classname", [(classname)])
@pytest.mark.parametrize("channel", [0])
def test_ad9364_tx_data(test_dma_tx, iio_uri, classname, channel):
    test_dma_tx(iio_uri, classname, channel)


#########################################
@pytest.mark.iio_hardware(hardware)
@pytest.mark.parametrize("classname", [(classname)])
@pytest.mark.parametrize("channel", [0])
@pytest.mark.parametrize(
    "param_set",
    [
        dict(
            tx_lo=1000000000,
            rx_lo=1000000000,
            gain_control_mode_chan0="slow_attack",
            tx_hardwaregain_chan0=-20,
            sample_rate=4000000,
        )
    ],
)
def test_ad9364_cyclic_buffers(
    test_cyclic_buffer, iio_uri, classname, channel, param_set
):
    test_cyclic_buffer(iio_uri, classname, channel, param_set)


#########################################
@pytest.mark.iio_hardware(hardware)
@pytest.mark.parametrize("classname", [(classname)])
@pytest.mark.parametrize("channel", [0])
@pytest.mark.parametrize(
    "param_set",
    [
        dict(
            tx_lo=1000000000,
            rx_lo=1000000000,
            gain_control_mode_chan0="slow_attack",
            tx_hardwaregain_chan0=-20,
            sample_rate=4000000,
        )
    ],
)
def test_ad9364_cyclic_buffers_exception(
    test_cyclic_buffer_exception, iio_uri, classname, channel, param_set
):
    test_cyclic_buffer_exception(iio_uri, classname, channel, param_set)


#########################################
@pytest.mark.iio_hardware(hardware, True)
@pytest.mark.parametrize("classname", [(classname)])
@pytest.mark.parametrize("channel", [0])
def test_ad9364_loopback(test_dma_loopback, iio_uri, classname, channel):
    test_dma_loopback(iio_uri, classname, channel)


########################################
@pytest.mark.iio_hardware(hardware, True)
@pytest.mark.parametrize("classname", [(classname)])
@pytest.mark.parametrize("channel", [0])
@pytest.mark.parametrize(
    "param_set, sfdr_min",
    [
        (
            dict(
                tx_lo=1000000000,
                rx_lo=1000000000,
                gain_control_mode_chan0="slow_attack",
                tx_hardwaregain_chan0=-20,
                sample_rate=4000000,
            ),
            40,
        ),
        (params["one_cw_tone_manual"], 22),
        (params["one_cw_tone_slow_attack"], 37),
        (params["change_attenuation_20dB_slow_attack"], 44),
        (params["change_attenuation_0dB_slow_attack"], 18),
        (params["change_sampling_rate_60MSPS_slow_attack"], 49),
        (params["change_sampling_rate_15MSPS_slow_attack"], 49),
        (params["change_attenuation_10dB_manual"], 37),
        (params["change_attenuation_5dB_manual"], 30),
        (params["change_rf_gain_5dB_manual"], 22),
    ],
)
def test_ad9364_sfdr(test_sfdr, iio_uri, classname, channel, param_set, sfdr_min):
    test_sfdr(iio_uri, classname, channel, param_set, sfdr_min)


########################################
@pytest.mark.iio_hardware(hardware, True)
@pytest.mark.parametrize("classname", [(classname)])
@pytest.mark.parametrize("channel", [0])
@pytest.mark.parametrize(
    "param_set, dds_scale, min_rssi, max_rssi",
    [
        (params["one_cw_tone_manual"], 0.12, 24, 27),
        (params["one_cw_tone_manual"], 0.25, 18, 21),
        (params["one_cw_tone_manual"], 0.25, 17.5, 21.5),
        (params["one_cw_tone_manual"], 0.06, 30, 32.5),
        (params["change_rf_gain_5dB_manual"], 0.25, 21, 24),
        (params["change_attenuation_10dB_manual"], 0.25, 27, 30),
        (params["change_attenuation_5dB_manual"], 0.25, 22.5, 25.5),
    ],
)
def test_ad9364_dds_gain_check_vary_power(
    test_gain_check,
    iio_uri,
    classname,
    channel,
    param_set,
    dds_scale,
    min_rssi,
    max_rssi,
):
    test_gain_check(
        iio_uri, classname, channel, param_set, dds_scale, min_rssi, max_rssi
    )


#########################################
@pytest.mark.iio_hardware(hardware, True)
@pytest.mark.parametrize("classname", [(classname)])
@pytest.mark.parametrize("channel", [0])
@pytest.mark.parametrize(
    "param_set, dds_scale, min_rssi, max_rssi",
    [
        (params["one_cw_tone_slow_attack"], 0.06, 42.5, 47.5),
        (params["change_attenuation_20dB_slow_attack"], 0.06, 53.5, 56.5),
        (params["change_attenuation_0dB_slow_attack"], 0.06, 33.5, 37.5),
    ],
)
def test_ad9364_dds_gain_check_agc(
    test_gain_check,
    iio_uri,
    classname,
    channel,
    param_set,
    dds_scale,
    min_rssi,
    max_rssi,
):
    test_gain_check(
        iio_uri, classname, channel, param_set, dds_scale, min_rssi, max_rssi
    )


#########################################
@pytest.mark.iio_hardware(hardware, True)
@pytest.mark.parametrize("classname", [(classname)])
@pytest.mark.parametrize("channel", [0])
@pytest.mark.parametrize(
    "param_set, frequency, scale, peak_min",
    [
        (
            dict(
                tx_lo=1000000000,
                rx_lo=1000000000,
                gain_control_mode_chan0="slow_attack",
                tx_hardwaregain_chan0=-30,
                sample_rate=4000000,
            ),
            1000000,
            0.9,
            -40,
        ),
        (params["one_cw_tone_manual"], 2000000, 0.12, -47),
        (params["one_cw_tone_manual"], 2000000, 0.25, -40),
        (params["one_cw_tone_manual"], 2000000, 0.06, -53),
        (params["change_attenuation_10dB_manual"], 2000000, 0.25, -49),
        (params["change_attenuation_5dB_manual"], 2000000, 0.25, -44.5),
        (params["change_rf_gain_5dB_manual"], 2000000, 0.25, -35.5),
        (params["one_cw_tone_slow_attack"], 500000, 0.06, -40),
        (params["one_cw_tone_slow_attack"], 1000000, 0.06, -40),
        (params["one_cw_tone_slow_attack"], 2000000, 0.06, -40),
        (params["one_cw_tone_slow_attack"], 2000000, 0.12, -40),
        (params["one_cw_tone_slow_attack"], 3000000, 0.25, -40),
        (params["one_cw_tone_slow_attack"], 4000000, 0.5, -40),
        (params["change_sampling_rate_60MSPS_slow_attack"], 2000000, 0.06, -40),
        (params["change_sampling_rate_15MSPS_slow_attack"], 2000000, 0.06, -40),
        (params["change_attenuation_20dB_slow_attack"], 1000000, 0.06, -40),
        (params["change_attenuation_0dB_slow_attack"], 1000000, 0.06, -40),
    ],
)
def test_ad9364_dds_loopback(
    test_dds_loopback,
    iio_uri,
    classname,
    param_set,
    channel,
    frequency,
    scale,
    peak_min,
):
    test_dds_loopback(
        iio_uri, classname, param_set, channel, frequency, scale, peak_min
    )


#########################################
@pytest.mark.iio_hardware(hardware, True)
@pytest.mark.parametrize("classname", [(classname)])
@pytest.mark.parametrize("channel", [0])
@pytest.mark.parametrize(
    "param_set",
    [
        dict(
            tx_lo=1000000000,
            rx_lo=1000000000,
            gain_control_mode_chan0="slow_attack",
            tx_hardwaregain_chan0=-20,
            sample_rate=4000000,
        ),
        dict(
            tx_lo=2000000000,
            rx_lo=2000000000,
            gain_control_mode_chan0="slow_attack",
            tx_hardwaregain_chan0=-20,
            sample_rate=4000000,
        ),
        dict(
            tx_lo=3000000000,
            rx_lo=3000000000,
            gain_control_mode_chan0="slow_attack",
            tx_hardwaregain_chan0=-20,
            sample_rate=4000000,
        ),
        params["one_cw_tone_manual"],
        params["change_attenuation_10dB_manual"],
        params["change_attenuation_5dB_manual"],
        params["change_rf_gain_5dB_manual"],
        params["one_cw_tone_slow_attack"],
        params["change_attenuation_20dB_slow_attack"],
        params["change_attenuation_0dB_slow_attack"],
        params["change_sampling_rate_60MSPS_slow_attack"],
        params["change_sampling_rate_15MSPS_slow_attack"],
    ],
)
def test_ad9364_iq_loopback(test_iq_loopback, iio_uri, classname, channel, param_set):
    test_iq_loopback(iio_uri, classname, channel, param_set)


#########################################
@pytest.mark.iio_hardware(hardware, True)
@pytest.mark.parametrize("classname", [(classname)])
@pytest.mark.parametrize("channel", [0])
@pytest.mark.parametrize(
    "param_set, frequency1, scale1, peak_min1, frequency2, scale2, peak_min2",
    [(params["one_cw_tone_slow_attack"], 1000000, 0.06, -20, 2000000, 0.12, -40,)],
)
def test_ad9364_two_tone_loopback(
    test_dds_two_tone,
    iio_uri,
    classname,
    channel,
    param_set,
    frequency1,
    scale1,
    peak_min1,
    frequency2,
    scale2,
    peak_min2,
):
    test_dds_two_tone(
        iio_uri,
        classname,
        channel,
        param_set,
        frequency1,
        scale1,
        peak_min1,
        frequency2,
        scale2,
        peak_min2,
    )
