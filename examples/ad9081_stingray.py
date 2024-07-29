# Copyright (C) 2020 Analog Devices, Inc.
#
# SPDX short identifier: ADIBSD

import time
import sys
import adi
import math
import matplotlib.pyplot as plt
import numpy as np

url = "ip:analog-2.local" if len(sys.argv) == 1 else sys.argv[1]
ssh = adi.sshfs(address=url, username="root", password="analog")

tx_offload_base_addr = 0x9c440000
rx_offload_base_addr = 0x9c450000
tx_dma_base_addr     = 0x9c430000
rx_dma_base_addr     = 0x9c420000
reset_reg_offload    = 0x84
control_reg_offload  = 0x88
control_reg_dma      = 0x400
flags_reg_dma        = 0x40c

# Reset the DMAs
ssh._run(f"busybox devmem 0x{tx_dma_base_addr + control_reg_dma:02x} 32 0x0")
# ssh._run(f"busybox devmem 0x{rx_dma_base_addr + control_reg_dma:02x} 32 0x0")
# 
# Reset the TX offload
ssh._run(f"busybox devmem 0x{tx_offload_base_addr + reset_reg_offload:02x} 32 0x0")
ssh._run(f"busybox devmem 0x{tx_offload_base_addr + reset_reg_offload:02x} 32 0x1")

# Reset the RX offload
# ssh._run(f"busybox devmem 0x{rx_offload_base_addr + reset_reg_offload:02x} 32 0x0")
# ssh._run(f"busybox devmem 0x{rx_offload_base_addr + reset_reg_offload:02x} 32 0x1")

conv = adi.ad9081(url)
tddn = adi.tddn(url)

conv._rxadc.set_kernel_buffers_count(1)

conv.ad9081_register_write(0x001B,0xf)
conv.ad9081_register_write(0x0321,0x00)

# Set NCOs

conv.rx_channel_nco_frequencies = [0] * 4
conv.tx_channel_nco_frequencies = [0] * 4

conv.rx_main_nco_frequencies = [100000000] * 4
conv.tx_main_nco_frequencies = [100000000] * 4

conv.rx_enabled_channels = [0,1]
conv.tx_enabled_channels = [0,1]
conv.rx_nyquist_zone = ["odd"] * 4

conv.rx_cyclic_buffer = False
conv.tx_cyclic_buffer = True

conv.tx_ddr_offload = False
conv.rx_ddr_offload = False

# make sure that the offloads are in one-shot mode 

stdout, stderr = ssh._run(f"busybox devmem 0x{tx_offload_base_addr + control_reg_offload:02x} 32 0x2")
stdout, stderr = ssh._run(f"busybox devmem 0x{rx_offload_base_addr + control_reg_offload:02x} 32 0x2")


samples_per_frame_desired = 2**15
frame_length_ms = 20
pulse_width_high = samples_per_frame_desired / conv.tx_sample_rate * 100

tddn.enable = 0
tddn.startup_delay_ms = 0
tddn.frame_length_ms = frame_length_ms
tddn.burst_count = 0 

# TDD ENABLE

tddn.channel[2].on_ms    = 0
tddn.channel[2].off_ms   = 0
tddn.channel[2].polarity = 1
tddn.channel[2].enable   = 1 

# TX OFFFLOAD SYNC
tddn.channel[0].on_ms = 0
tddn.channel[0].off_ms = pulse_width_high
tddn.channel[0].polarity = 0
tddn.channel[0].enable = 1

# RX OFFFLOAD SYNC
tddn.channel[1].on_ms = 0
tddn.channel[1].off_ms = pulse_width_high 
tddn.channel[1].polarity = 0
tddn.channel[1].enable = 1

for chan in [3,4,5] :
    tddn.channel[chan].on_ms = 0
    tddn.channel[chan].off_ms = 0
    tddn.channel[chan].polarity = 1
    tddn.channel[chan].enable = 1
 
tddn.enable = 1
# Create and send data to TX DMA

fs = int(conv.tx_sample_rate)


samples_per_frame = math.ceil(pulse_width_high * fs / 1000)

N_tx = int(samples_per_frame)
N_rx = N_tx
conv.rx_buffer_size = N_rx

print(f"TX Sampling_rate: {conv.tx_sample_rate}")
print(f"RX Sampling_rate: {conv.rx_sample_rate}")

print(f"TX buffer length: {N_tx}")
print(f"RX buffer length: {N_rx}")

print(f"Samples per frame: {samples_per_frame}")

print(f"TX_transmit time[ms]: {pulse_width_high}")
print(f"RX_recieve time[ms]: {((1/conv.rx_sample_rate) * N_rx)*1000}")
print(f"TDD_frame time[ms]: {frame_length_ms}")

A = 0.9 * 2**15  # -6 dBFS
data_gap = 1500
tx_sig  = np.concatenate((np.zeros(int(data_gap/2)),np.linspace(0,A,N_tx-data_gap),np.zeros(int(data_gap/2))),axis=None)

# A = 0.9 * 2**15  # -6 dBFS
# B = 1e6
# N = int(samples_per_frame)
# T = N / fs
# t = np.linspace(-T / 2, T / 2, N, endpoint=False)
# tx_sig = A * np.sin(2 * math.pi * B * t)
# 
conv.tx([tx_sig, tx_sig])

tddn.sync_soft  = 0
tddn.sync_soft  = 1
 
rx_t = np.linspace(0,N_rx, N_rx , endpoint=False)

for r in range(200):
    rx_sig  = conv.rx()
    plt.plot(rx_t, np.abs(rx_sig[0]))
    plt.ylabel("Amplitude")
    plt.xlabel("Samples")
    plt.title(f"Capture number: {r}")
    plt.pause(0.3)
    if r % 2 ==1 :
        conv.tx_destroy_buffer()
        tx_sig  = np.concatenate((np.zeros(int(data_gap/2)),np.linspace(A,0,N_tx-data_gap),np.zeros(int(data_gap/2))),axis=None)
        conv.tx([tx_sig, tx_sig])

    if r % 2 ==0 :
        conv.tx_destroy_buffer()
        tx_sig  = np.concatenate((np.zeros(int(data_gap/2)),np.linspace(0,A,N_tx-data_gap),np.zeros(int(data_gap/2))),axis=None)
        conv.tx([tx_sig, tx_sig])
plt.show()

tddn.enable = 0
   
for chan in [1,2,3,4,5] :
    tddn.channel[chan].on_ms = 0
    tddn.channel[chan].off_ms = 0
    tddn.channel[chan].polarity = 0
    tddn.channel[chan].enable = 1

tddn.enable = 1
tddn.enable = 0
conv.tx_destroy_buffer()
conv.rx_destroy_buffer()