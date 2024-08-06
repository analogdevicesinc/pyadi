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

conv.rx_enabled_channels = [0]
conv.tx_enabled_channels = [0]
conv.rx_nyquist_zone = ["odd"] * 4

conv.rx_cyclic_buffer = False
conv.tx_ddr_offload = False
conv.rx_ddr_offload = False
samples_per_frame_desired = 256


stdout, stderr = ssh._run(f"busybox devmem 0x{tx_offload_base_addr + control_reg_offload:02x} 32 0x1")
conv.tx_cyclic_buffer = False
tx_sig = np.zeros(samples_per_frame_desired)
conv.tx(tx_sig)
conv.tx_destroy_buffer()
conv.tx_cyclic_buffer = True

stdout, stderr = ssh._run(f"busybox devmem 0x{tx_offload_base_addr + control_reg_offload:02x} 32 0x2")
stdout, stderr = ssh._run(f"busybox devmem 0x{rx_offload_base_addr + control_reg_offload:02x} 32 0x2")

tddn.enable = 0
frame_length_ms = 1
pulse_width_high = (samples_per_frame_desired / conv.tx_sample_rate) * 1000
tddn.startup_delay_ms = 0
tddn.frame_length_ms = frame_length_ms
tddn.burst_count = 3

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
tddn.channel[1].on_ms = 0.0015
tddn.channel[1].off_ms = pulse_width_high +0.0015
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
N_rx = N_tx

print(f"TX Sampling_rate: {conv.tx_sample_rate}")
print(f"RX Sampling_rate: {conv.rx_sample_rate}")

print(f"TX buffer length: {N_tx}")
print(f"RX buffer length: {N_rx}")

print(f"Samples per frame: {samples_per_frame}")

print(f"TX_transmit time[ms]: {pulse_width_high}")
print(f"RX_recieve time[ms]: {((1/conv.rx_sample_rate) * N_rx)*1000}")
print(f"TDD_frame time[ms]: {frame_length_ms}")


A = 0.9 * 2**15  # -6 dBFS
B = 1e6
N = int(samples_per_frame_desired)
T = N / fs
t = np.linspace(0, T, N, endpoint=False)
tx_sig = A * np.sin(2 * math.pi * B * t)
conv.tx(tx_sig)

length_dma_rx  =  int((samples_per_frame_desired*16)-1)
length_offload_rx =  int((length_dma_rx/64))

stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + x_length_dma:02x} 32 {length_dma_rx}")
ssh._run(f"busybox devmem 0x{rx_offload_base_addr + offload_transfer_length:02x} 32 {length_offload_rx}")


stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + 0x40c:02x} 32 0x6")
stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + 0x400:02x} 32 0x1")

stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + x_length_dma:02x} 32")
print(f"RX DMA x_length_dma: 0x{rx_dma_base_addr + x_length_dma:02x} (Relative {rx_dma_base_addr:02x}): {stdout}")
stdout, stderr = ssh._run(f"busybox devmem 0x{tx_dma_base_addr + x_length_dma:02x} 32")
print(f"TX DMA x_length_dma: 0x{tx_dma_base_addr + x_length_dma:02x} (Relative {tx_dma_base_addr:02x}): {stdout}")


stdout, stderr = ssh._run(f"busybox devmem 0x{rx_offload_base_addr + offload_transfer_length:02x} 32")
print(f"TX DMA x_length_dma: 0x{rx_offload_base_addr + offload_transfer_length:02x} (Relative {offload_transfer_length:02x}): {stdout}")

stdout, stderr = ssh._run(f"busybox devmem 0x{tx_offload_base_addr + offload_transfer_length:02x} 32")
print(f"TX DMA x_length_dma: 0x{tx_offload_base_addr + offload_transfer_length:02x} (Relative {offload_transfer_length:02x}): {stdout}")

stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + 0x410:02x} 32")
print(f"TX DMA x_length_dma: 0x{rx_dma_base_addr + 0x410:02x} (Relative {0x410:02x}): {stdout}")
stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + 0x400:02x} 32")
print(f"TX DMA x_length_dma: 0x{rx_dma_base_addr + 0x400:02x} (Relative {0x400:02x}): {stdout}")
stdout, stderr = ssh._run(f"busybox devmem 0x{tx_dma_base_addr + 0x40c:02x} 32")
print(f"TX DMA x_length_dma: 0x{tx_dma_base_addr + 0x40c:02x} (Relative {0x40c:02x}): {stdout}")
stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + 0x40c:02x} 32")
print(f"rX DMA x_length_dma: 0x{rx_dma_base_addr + 0x40c:02x} (Relative {0x40c:02x}): {stdout}")

rx_buf1_base_adress = 42213376
rx_buf2_base_adress = rx_buf1_base_adress + length_dma_rx + 1
rx_buf3_base_adress = rx_buf2_base_adress + length_dma_rx + 1

print(f"{rx_buf1_base_adress:02x}")
print(f"{rx_buf2_base_adress:02x}")
print(f"{rx_buf3_base_adress:02x}")

stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + 0x410:02x} 32 {rx_buf1_base_adress}")
stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + 0x408:02x} 32 0x1")
stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + 0x410:02x} 32 {rx_buf2_base_adress}")
stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + 0x408:02x} 32 0x1")
stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + 0x410:02x} 32 {rx_buf3_base_adress}")
stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + 0x408:02x} 32 0x1")

# to test with 4 submits 

tddn.sync_soft  = 0
tddn.sync_soft  = 1

time.sleep(3)

stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + 0x428:02x} 32")
print(f"RX DMA TRANSFER_DONE: 0x{rx_dma_base_addr + 0x428:02x} (Relative {0x428:02x}): {stdout}")


cnt=0
rx_buf_1 = [0 for x in range(N_rx)]
rx_buf_2 = [0 for x in range(N_rx)]
rx_buf_3 = [0 for x in range(N_rx)]

for i in range(0,length_dma_rx+1,16):
    stdout, stderr = ssh._run(f"busybox devmem 0x{rx_buf1_base_adress + i:02x} 16")
    rx_buf_1[cnt] = twos_complement(stdout, 16) 
    stdout, stderr = ssh._run(f"busybox devmem 0x{rx_buf2_base_adress + i:02x} 16")
    rx_buf_2[cnt] = twos_complement(stdout, 16) 
    stdout, stderr = ssh._run(f"busybox devmem 0x{rx_buf3_base_adress + i:02x} 16")
    rx_buf_3[cnt] = twos_complement(stdout, 16) 
    cnt+=1

rx_t = np.linspace(0,N_rx, N_rx , endpoint=False)

plt.plot(rx_t,rx_buf_1)
plt.plot(rx_t,rx_buf_2)
plt.plot(rx_t,rx_buf_3)
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





