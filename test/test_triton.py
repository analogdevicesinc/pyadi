import adi
import test.dma_tests
import pytest
import test
import matplotlib.pyplot as plt
import numpy as np
from test.test_ad9084 import scale_field
from test.scpi import find_instrument
import subprocess
import pyvisa
import time
import test.instruments as instruments
# from scpi import signal




hardware = ["Triton"]
classname = "adi.Triton"

##########################################
# Dictionaries for Tests
##########################################

params = dict(
    loopback_test_1=dict(
                        tx_main_nco_frequencies=[8000000000, 8000000000, 8000000000, 8000000000, 8000000000, 8000000000, 8000000000, 8000000000, 8000000000, 8000000000, 8000000000, 8000000000, 8000000000, 8000000000, 8000000000, 8000000000],
                        rx_main_nco_frequencies=[-4800000000, -4800000000, -4800000000, -4800000000, -4800000000, -4800000000, -4800000000, -4800000000, -4800000000, -4800000000, -4800000000, -4800000000, -4800000000, -4800000000, -4800000000, -4800000000],
                        tx_channel_nco_frequencies=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_channel_nco_frequencies=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        tx_main_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_main_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        tx_channel_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_channel_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_dsa_gain = 0,
    ),
    loopback_test_2=dict(
                        tx_main_nco_frequencies=[10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000],
                        rx_main_nco_frequencies=[-2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000],
                        tx_channel_nco_frequencies=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_channel_nco_frequencies=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        tx_main_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_main_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        tx_channel_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_channel_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_dsa_gain = 0,
    ),
    loopback_test_3=dict(
                        tx_main_nco_frequencies=[12000000000, 12000000000, 12000000000, 12000000000, 12000000000, 12000000000, 12000000000, 12000000000, 12000000000, 12000000000, 12000000000, 12000000000, 12000000000, 12000000000, 12000000000, 12000000000],
                        rx_main_nco_frequencies=[-800000000, -800000000, -800000000, -800000000, -800000000, -800000000, -800000000, -800000000, -800000000, -800000000, -800000000, -800000000, -800000000, -800000000, -800000000, -800000000],
                        tx_channel_nco_frequencies=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_channel_nco_frequencies=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        tx_main_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_main_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        tx_channel_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_channel_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_dsa_gain = 0,
    ),
    filter_test_1=dict(
                        tx_main_nco_frequencies=[10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000],
                        rx_main_nco_frequencies=[-2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000],
                        tx_channel_nco_frequencies=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_channel_nco_frequencies=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        tx_main_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_main_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        tx_channel_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_channel_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_dsa_gain = 0,
    ),
    filter_test_2=dict(
                        tx_main_nco_frequencies=[10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000],
                        rx_main_nco_frequencies=[-2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000],
                        tx_channel_nco_frequencies=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_channel_nco_frequencies=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        tx_main_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_main_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        tx_channel_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_channel_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_dsa_gain = 0,
    ),
    dac_adc_loopback_30dB_attenuation=dict(
                        tx_main_nco_frequencies=[10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000],
                        rx_main_nco_frequencies=[-2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000],
                        tx_channel_nco_frequencies=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_channel_nco_frequencies=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        tx_main_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_main_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        tx_channel_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_channel_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_dsa_gain = -30,
    ),
    sfdr_test=dict(
                        tx_main_nco_frequencies=[10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000, 10000000000],
                        rx_main_nco_frequencies=[-2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000, -2800000000],
                        tx_channel_nco_frequencies=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_channel_nco_frequencies=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        tx_main_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_main_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        tx_channel_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_channel_nco_phases=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        rx_dsa_gain = 0,
    ),
)

##########################################
# DC Power Test - Preboot
##########################################

def test_current_preboot():
    rm = pyvisa.ResourceManager()
    rm.list_resources()
    inst = rm.open_resource('TCPIP::192.168.10.1::inst0::INSTR')

    inst.write("OUTP:PAIR PAR")
    inst.write("INST CH1")
    inst.write("SOUR:VOLT 12")
    inst.write("SOUR:CURR 30") # Change as needed
    print(inst.query("MEASure:CURR?"))

    current_value = float(inst.query("MEASure:CURR?"))

    assert current_value > 2
    assert current_value < 4.5

def test_voltage_preboot():
    rm = pyvisa.ResourceManager()
    rm.list_resources()
    inst = rm.open_resource('TCPIP::192.168.10.1::inst0::INSTR')

    inst.write("OUTP:PAIR PAR")
    inst.write("INST CH1")
    inst.write("SOUR:VOLT 12")
    inst.write("SOUR:CURR 30")  # Change as needed
    print(inst.query("MEASure:VOLT?"))

    voltage_value = float(inst.query("MEASure:VOLT?"))

    assert voltage_value > 11.8
    assert voltage_value < 12.2


# ##########################################
# # Automated bootup 
# ##########################################

@pytest.fixture(scope="module")
def test_bootup():
    import os
    import subprocess 

    dir_path = os.path.dirname(os.path.realpath(__file__))
    print(dir_path)
    boot_path = os.path.join(dir_path, 'vcu118_quad_ad9084_2023-09-28')
    # tcl_script_path = r"C:/Users/JChambli/Downloads/vcu118_quad_ad9084_2023-09-28/run_26p4.tcl"
    tcl_script_path = os.path.join(boot_path, 'run_me.tcl')

    # Path to Vivado Lab executable
    vivado_lab_executable = r"C:\Xilinx\Vivado_Lab\2022.2\bin\xsdb.bat"

    # Open Xilinx xsdb tool and
    # source run.tcl
    bitstream = os.path.join(boot_path, "system_top_26p4.bit")
    strip = os.path.join(boot_path, "simpleImage_26p4.strip" )
    script = f"""
    connect
    fpga -f "{bitstream.replace(os.sep, '/')}"
    after 1000
    target 3
    dow "{strip.replace(os.sep, '/')}"
    after 1000
    con
    disconnect
    """
    with open(tcl_script_path, 'w') as f:
        f.write(script)

    # Command to run Vivado Lab with the Tcl script
    command = [vivado_lab_executable, '-eval', 'source', tcl_script_path]
    print(command)

    # Run the command
    # subprocess.run(command)

    results = subprocess.run(command)
    time.sleep(140)  ## Wait for bootup
    print(dir(results))
    print(results)

    

# #########################################
# # DC Power Test - Postboot
# #########################################

def test_current_postboot():
    rm = pyvisa.ResourceManager()
    rm.list_resources()
    inst = rm.open_resource('TCPIP::192.168.10.1::inst0::INSTR')

    inst.write("OUTP:PAIR PAR")
    inst.write("INST CH1")
    inst.write("SOUR:VOLT 12")
    inst.write("SOUR:CURR 30") # Change as needed
    print(inst.query("MEASure:CURR?"))

    current_value = float(inst.query("MEASure:CURR?"))

    assert current_value > 14
    assert current_value < 18

def test_voltage_postboot():
    rm = pyvisa.ResourceManager()
    rm.list_resources()
    inst = rm.open_resource('TCPIP::192.168.10.1::inst0::INSTR')

    inst.write("OUTP:PAIR PAR")
    inst.write("INST CH1")
    inst.write("SOUR:VOLT 12")
    inst.write("SOUR:CURR 30") # Change as needed
    print(inst.query("MEASure:VOLT?"))

    voltage_value = float(inst.query("MEASure:VOLT?"))

    assert voltage_value > 11.8
    assert voltage_value < 12.2




# ##########################################
# # Ethernet connection test
# ##########################################

def test_iio_attr(iio_uri):
    print("iio_uri", iio_uri)

#########################################
# DDS Loopback Test 
#########################################

@pytest.mark.parametrize("classname", [(classname)])
# @pytest.mark.parametrize("channel", [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15], ids=lambda x: f"channel:{x}")
@pytest.mark.parametrize("channel", [0, 1], ids=lambda x: f"channel:{x}")
@pytest.mark.parametrize(
    "param_set, frequency, scale, peak_min, hpf_value, lpf_value",
    [
        (params["loopback_test_1"], 50000000, 0.9, -25, 5, 15),
        (params["loopback_test_2"], 50000000, 0.9, -25, 5, 15),
        (params["loopback_test_3"], 50000000, 0.9, -25, 5, 15),
        (params["filter_test_1"], 50000000, 0.9, -20, 5, 15),
        (params["filter_test_2"], 50000000, 0.9, -30, 11, 9),
        (params["dac_adc_loopback_30dB_attenuation"], 50000000, 0.9, -50, 5, 15),
    ]
)
def test_Triton_dds_loopback(
    test_dds_loopback,
    iio_uri,
    classname,
    param_set,
    channel,
    frequency,
    scale,
    peak_min,
    hpf_value,
    lpf_value,
):
    dev = adi.Triton("ip:192.168.2.1", calibration_board_attached=True)
    ## Set low pass and high pass filter values
    dev.hpf_ctrl = hpf_value
    dev.lpf_ctrl = lpf_value
    ## Set cal board loopback state
    dev.gpio_ctrl_ind = 0
    dev.gpio_5045_v1 = 1
    dev.gpio_5045_v2 = 0
    dev.gpio_ctrl_rx_combined = 0
    ## Param set and DDS test
    param_set = scale_field(param_set, iio_uri)
    test_dds_loopback(
        iio_uri, classname, param_set, channel, frequency, scale, peak_min
    )


#########################################
# SFDR Test 
#########################################
    
@pytest.mark.parametrize("classname", [(classname)])
# @pytest.mark.parametrize("channel", [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15], ids=lambda x: f"channel:{x}")
@pytest.mark.parametrize("channel", [0, 1], ids=lambda x: f"channel:{x}")
@pytest.mark.parametrize(
    "param_set, hpf_value, lpf_value",
    [
        (params["sfdr_test"], 5, 15),   
    ],
)
@pytest.mark.parametrize("sfdr_min", [60])
def test_Triton_sfdr(test_sfdr, iio_uri, classname, channel, param_set, sfdr_min, hpf_value, lpf_value):
    dev = adi.Triton("ip:192.168.2.1", calibration_board_attached=True)
    ## Set low pass and high pass filter values
    dev.hpf_ctrl = hpf_value
    dev.lpf_ctrl = lpf_value 
    ## SFDR test
    test_sfdr(iio_uri, classname, channel, param_set, sfdr_min)


