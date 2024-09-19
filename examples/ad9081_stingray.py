# Copyright (C) 2020 Analog Devices, Inc.
#
# SPDX short identifier: ADIBSD

import time
import sys
import adi
import math
import matplotlib.pyplot as plt
import numpy as np
from enum import Enum

url = "local:" if len(sys.argv) == 1 else sys.argv[1]
ssh = adi.sshfs(address=url, username="root", password="analog")

tx_offload_base_addr    = 0x9c440000
rx_offload_base_addr    = 0x9c450000
rx_dma_base_addr        = 0x9c420000
control_reg_offload     = 0x88

conv = adi.ad9081(url)
tddn = adi.tddn(url)

conv._rxadc.set_kernel_buffers_count(1)

# Set NCOs

conv.rx_channel_nco_frequencies = [0] * 4
conv.tx_channel_nco_frequencies = [0] * 4

conv.rx_main_nco_frequencies = [100000000] * 4 
conv.tx_main_nco_frequencies = [100000000] * 4 

conv.rx_enabled_channels = [0,1]
conv.tx_enabled_channels = [0,1]
conv.rx_nyquist_zone     = ["odd"] * 4

conv.tx_cyclic_buffer = True
conv.rx_cyclic_buffer = False
conv.tx_ddr_offload   = False
conv.rx_ddr_offload   = False

stdout, stderr = ssh._run(f"busybox devmem 0x{tx_offload_base_addr + control_reg_offload:02x} 32 0x2")
stdout, stderr = ssh._run(f"busybox devmem 0x{rx_offload_base_addr + control_reg_offload:02x} 32 0x2")

frame_length_ms           = 1
rx_time                   = 0.533
generated_signal_time     = 0.1 
blank_time                = 0.05
use_tx_mxfe_en            = 1
tx_time                   = generated_signal_time - blank_time
capture_range             = 1


N_tx                      = int((generated_signal_time * conv.tx_sample_rate) / 1000)
N_rx                      = int((rx_time * conv.tx_sample_rate) / 1000)

conv.rx_buffer_size       = N_rx

tddn.startup_delay_ms     = 0
tddn.frame_length_ms      = frame_length_ms
tddn.burst_count          = 0

TX_OFFLOAD_SYNC = 0 
RX_OFFLOAD_SYNC = 1 
TDD_ENABLE      = 2
RX_MXFE_EN      = 3
TX_MXFE_EN      = 4
TX_STINGRAY_EN  = 5
if use_tx_mxfe_en: 
    print("0x001B:", conv.ad9081_register_read(0x001B))
    conv.ad9081_register_write(0x001B,0xf)
    print("0x001B:", conv.ad9081_register_read(0x001B))
    # 
    print("0x0321:", conv.ad9081_register_read(0x0321))
    conv.ad9081_register_write(0x0321,0x00)
    print("0x0321:", conv.ad9081_register_read(0x0321))
    tddn.channel[TX_MXFE_EN].on_ms    = 0
    tddn.channel[TX_MXFE_EN].off_ms   = tx_time
    tddn.channel[TX_MXFE_EN].polarity = 0
    tddn.channel[TX_MXFE_EN].enable   = 1
else :
    tddn.channel[TX_MXFE_EN].on_ms    = 0
    tddn.channel[TX_MXFE_EN].off_ms   = 0
    tddn.channel[TX_MXFE_EN].polarity = 1
    tddn.channel[TX_MXFE_EN].enable   = 1
 

for chan in [TDD_ENABLE,RX_MXFE_EN,TX_STINGRAY_EN]:
    tddn.channel[chan].on_ms    = 0
    tddn.channel[chan].off_ms   = 0
    tddn.channel[chan].polarity = 1
    tddn.channel[chan].enable   = 1



for chan in [RX_OFFLOAD_SYNC, TX_OFFLOAD_SYNC]:
    tddn.channel[chan].on_raw   = 0
    tddn.channel[chan].off_raw  = 10
    tddn.channel[chan].polarity = 0
    tddn.channel[chan].enable   = 1

tddn.enable = 1

print(f"TX Sampling_rate: {conv.tx_sample_rate}")
print(f"RX Sampling_rate: {conv.rx_sample_rate}")

print(f"TX buffer length: {N_tx}")
print(f"RX buffer length: {N_rx}")

print(f"Generated signal time[ms]: {generated_signal_time}")
print(f"TX_transmit time[ms]: {tx_time}")

print(f"RX_recieve time[ms]:  {((1/conv.rx_sample_rate) * N_rx)*1000}")

print(f"TDD_frame time[ms]: {tddn.frame_length_ms}")

fs = int(conv.tx_sample_rate)
A = 0.9 * 2**15  # -6 dBFS
B = 1e5
T = N_tx / fs
t = np.linspace(0, T, N_tx, endpoint=False)
tx_sig = A * np.sin(2 * math.pi * B * t)
conv.tx([tx_sig,tx_sig])

tddn.sync_soft  = 0
tddn.sync_soft  = 1

rx_sig = np.zeros((capture_range,2,N_rx)) 
    
for r in range(capture_range):
    rx_sig[r]  = conv.rx()

for r in range(capture_range):
    plt.suptitle(f"Capture number: {r}")
    plt.plot(rx_sig[0][0], label='Input Channel 1')
    plt.plot(np.append(tx_sig*0.04, np.zeros(abs(len(rx_sig[0][0]) - len(tx_sig))))-3000, label='Generated Signal')
plt.legend()
plt.show()

tddn.enable = 0

for chan in [TX_OFFLOAD_SYNC, RX_OFFLOAD_SYNC, TDD_ENABLE, RX_MXFE_EN, TX_MXFE_EN, TX_STINGRAY_EN]:
    tddn.channel[chan].on_ms = 0
    tddn.channel[chan].off_ms = 0
    tddn.channel[chan].polarity = 0
    tddn.channel[chan].enable = 1

tddn.enable = 1
tddn.enable = 0

conv.tx_destroy_buffer()
conv.rx_destroy_buffer()





