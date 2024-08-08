# Copyright (C) 2020 Analog Devices, Inc.
#
# SPDX short identifier: ADIBSD

import time
import sys
import adi
import math
import matplotlib.pyplot as plt
import numpy as np

def twos_complement(hexstr, bits):
    value = int(hexstr, 16)
    if value & (1 << (bits - 1)):
        value -= 1 << bits
    return value

url = "local:" if len(sys.argv) == 1 else sys.argv[1]
ssh = adi.sshfs(address=url, username="root", password="analog")

tx_offload_base_addr    = 0x9c440000
rx_offload_base_addr    = 0x9c450000
tx_dma_base_addr        = 0x9c430000
rx_dma_base_addr        = 0x9c420000
control_reg_offload     = 0x88
x_length_dma            = 0x418
control_reg_dma         = 0x400
offload_transfer_length = 0x1c

ssh._run(f"busybox devmem 0x{tx_dma_base_addr + control_reg_dma:02x} 32 0x0")
ssh._run(f"busybox devmem 0x{rx_dma_base_addr + control_reg_dma:02x} 32 0x0")

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

conv.rx_enabled_channels = [0,1,2]
conv.tx_enabled_channels = [0,1,2]
conv.rx_nyquist_zone = ["odd"] * 4

conv.rx_cyclic_buffer = False
conv.tx_ddr_offload = False
conv.rx_ddr_offload = False
samples_per_frame_desired = 24576


stdout, stderr = ssh._run(f"busybox devmem 0x{tx_offload_base_addr + control_reg_offload:02x} 32 0x1")
conv.tx_cyclic_buffer = False
tx_sig = np.zeros(samples_per_frame_desired)
conv.tx([tx_sig,tx_sig,tx_sig])
conv.tx_destroy_buffer()
conv.tx_cyclic_buffer = True

stdout, stderr = ssh._run(f"busybox devmem 0x{tx_offload_base_addr + control_reg_offload:02x} 32 0x2")
stdout, stderr = ssh._run(f"busybox devmem 0x{rx_offload_base_addr + control_reg_offload:02x} 32 0x2")

frame_length_ms = 1
pulse_width_high = (samples_per_frame_desired / conv.tx_sample_rate) * 1000
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
tddn.channel[0].off_raw = 1
tddn.channel[0].polarity = 0
tddn.channel[0].enable = 1

# RX OFFFLOAD SYNC
tddn.channel[1].on_ms = 0
tddn.channel[1].off_raw = 1
tddn.channel[1].polarity = 0
tddn.channel[1].enable = 1

for chan in [3,4,5] :
    tddn.channel[chan].on_ms = 0
    tddn.channel[chan].off_ms = 0
    tddn.channel[chan].polarity = 1
    tddn.channel[chan].enable = 1
 
tddn.enable = 1

fs = int(conv.tx_sample_rate)

samples_per_frame = math.ceil(pulse_width_high * (fs / 1000))

N_tx = int(samples_per_frame)

N_rx = 2*N_tx
conv.rx_buffer_size = N_rx
length_dma_rx  =  int((N_rx*16)-1)
length_offload_rx =  int((length_dma_rx/64))

print(f"TX Sampling_rate: {conv.tx_sample_rate}")
print(f"RX Sampling_rate: {conv.rx_sample_rate}")

print(f"TX buffer length: {N_tx}")
print(f"RX buffer length: {N_rx}")

print(f"Samples per frame: {samples_per_frame}")

print(f"TX_transmit time[ms]: {pulse_width_high}")
print(f"RX_recieve time[ms]: {((1/conv.rx_sample_rate) * N_rx)*1000}")
print(f"TDD_frame time[ms]: {frame_length_ms}")


A = 0.9 * 2**15  # -6 dBFS
B = 1e4
N = int(samples_per_frame_desired)
T = N / fs
t = np.linspace(0, T, N, endpoint=False)
tx_sig = A * np.sin(2 * math.pi * B * t)
conv.tx([tx_sig,tx_sig,tx_sig])


# stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + x_length_dma:02x} 32 {length_dma_rx}")
# ssh._run(f"busybox devmem 0x{rx_offload_base_addr + offload_transfer_length:02x} 32 {length_offload_rx}")

# stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + 0x40c:02x} 32 0x6")
# stdout, stderr = ssh._run(f"busybox devmem 0x{tx_dma_base_addr + 0x40c:02x} 32 0x7")


# stdout, stderr = ssh._run(f"busybox devmem 0x{tx_dma_base_addr + x_length_dma:02x} 32 ")
length_dma_tx     = int((samples_per_frame_desired*16)-1)
length_offload_tx = int((length_dma_tx/64))
# stdout, stderr = ssh._run(f"busybox devmem 0x{tx_dma_base_addr + x_length_dma:02x} 32 {length_dma_tx}")
# ssh._run(f"busybox devmem 0x{tx_offload_base_addr + offload_transfer_length:02x} 32 {length_offload_tx}")


print(f"computed rx_dma_base_addr + x_length_dma: {length_dma_rx:02x}")
print(f"computed tx_dma_base_addr + x_length_dma:  {length_dma_tx:02x}")
print(f"computed tx_offload_base_addr + offload_transfer_length: {length_offload_tx:02x}")
print(f"computed rx_offload_base_addr + offload_transfer_length: {length_offload_rx:02x}")
   
stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + x_length_dma:02x} 32")
print(f"rx_dma_base_addr + x_length_dma: 0x{rx_dma_base_addr + x_length_dma:02x} (Relative {x_length_dma:02x}): {stdout}")
stdout, stderr = ssh._run(f"busybox devmem 0x{tx_dma_base_addr + x_length_dma:02x} 32")
print(f"tx_dma_base_addr + x_length_dma: 0x{tx_dma_base_addr + x_length_dma:02x} (Relative {x_length_dma:02x}): {stdout}")

stdout, stderr = ssh._run(f"busybox devmem 0x{tx_offload_base_addr + offload_transfer_length:02x} 32")
print(f"tx_offload_base_addr + offload_transfer_length: 0x{tx_offload_base_addr + offload_transfer_length:02x} (Relative {offload_transfer_length:02x}): {stdout}")
stdout, stderr = ssh._run(f"busybox devmem 0x{rx_offload_base_addr + offload_transfer_length:02x} 32")
print(f"rx_offload_base_addr + offload_transfer_length: 0x{rx_offload_base_addr + offload_transfer_length:02x} (Relative {offload_transfer_length:02x}): {stdout}")



tddn.sync_soft  = 0
tddn.sync_soft  = 1


rx_t = np.linspace(0,N_rx, N_rx , endpoint=False)
capture_range = 200
enabled_channels = 3
rx_sig = np.zeros((capture_range,enabled_channels,N_rx)) 

for r in range(capture_range):
    rx_sig[r]  = conv.rx()

fig, (ch1, ch2) = plt.subplots(1, 2)

for r in range(capture_range):
    plt.suptitle(f"Capture number: {r}")
    ch1.plot(rx_t, np.abs(rx_sig[r][0]))
    ch1.set_title("Channel 1 data")
    ch2.plot(rx_t, np.abs(rx_sig[r][1]))
    ch2.set_title("Channel 2 data")
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





