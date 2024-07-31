# Copyright (C) 2020 Analog Devices, Inc.
#
# SPDX short identifier: ADIBSD

import time
import sys
import adi
import math
import matplotlib.pyplot as plt
import numpy as np

url = "local:"
ssh_url = "local:" if len(sys.argv) == 1 else sys.argv[1]
ssh = adi.sshfs(address=ssh_url, username="root", password="analog")

tx_offload_base_addr    = 0x9c440000
rx_offload_base_addr    = 0x9c450000
tx_dma_base_addr        = 0x9c430000
rx_dma_base_addr        = 0x9c420000
tddn_base_adress        = 0x9c460000
reset_reg_offload       = 0x84
control_reg_offload     = 0x88
x_length_dma            = 0x418
control_reg_dma         = 0x400
flags_reg_dma           = 0x40c
offload_transfer_length = 0x1c

# Reset the DMAs
ssh._run(f"busybox devmem 0x{tx_dma_base_addr + control_reg_dma:02x} 32 0x0")
ssh._run(f"busybox devmem 0x{rx_dma_base_addr + control_reg_dma:02x} 32 0x0")

# Reset the TX offload
ssh._run(f"busybox devmem 0x{tx_offload_base_addr + reset_reg_offload:02x} 32 0x0")
ssh._run(f"busybox devmem 0x{tx_offload_base_addr + reset_reg_offload:02x} 32 0x1")

# Reset the RX offload
ssh._run(f"busybox devmem 0x{rx_offload_base_addr + reset_reg_offload:02x} 32 0x0")
ssh._run(f"busybox devmem 0x{rx_offload_base_addr + reset_reg_offload:02x} 32 0x1")

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

samples_per_frame_desired = 245760
frame_length_ms = 1
pulse_width_high = (samples_per_frame_desired / conv.tx_sample_rate) * 100

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

# with open("tdd_regdump.txt", "w") as f:
#     for i in range(0,0x0b0,4):
#            stdout, stderr = ssh._run(f"busybox devmem 0x{tddn_base_adress + i:02x}")
#            print(f"0x{tddn_base_adress + i:02x} (Relative {i:02x}): {stdout}")
#            f.write(f"0x{tddn_base_adress + i:02x}, (Relative {i:02x}), {stdout}\n")

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
data_gap = int((0.05*samples_per_frame_desired)/2)
tx_sig  = np.concatenate((np.zeros(int(data_gap/2)),np.linspace(0,A,N_tx-data_gap),np.zeros(int(data_gap/2))),axis=None)

conv.tx([tx_sig, tx_sig])

tddn.sync_soft  = 0
tddn.sync_soft  = 1

rx_t = np.linspace(0,N_rx, N_rx , endpoint=False)
fig, (ch1, ch2) = plt.subplots(1, 2)

stdout, stderr = ssh._run(f"busybox devmem 0x{tx_dma_base_addr + x_length_dma:02x} 32")
print(f"TX DMA x_length_dma: 0x{tx_dma_base_addr + x_length_dma:02x} (Relative {x_length_dma:02x}): {stdout}")

stdout, stderr = ssh._run(f"busybox devmem 0x{rx_dma_base_addr + x_length_dma:02x} 32")
print(f"RX DMA x_length_dma: 0x{rx_dma_base_addr + x_length_dma:02x} (Relative {x_length_dma:02x}): {stdout}")

lenth_offload = int(int(stdout, base=16) / 64)

stdout, stderr = ssh._run(f"busybox devmem 0x{rx_offload_base_addr + offload_transfer_length:02x} 32")
print(f"rx_offload offload_transfer_length: 0x{rx_offload_base_addr + offload_transfer_length:02x} (Relative {offload_transfer_length:02x}): {stdout}")

stdout, stderr = ssh._run(f"busybox devmem 0x{rx_offload_base_addr + offload_transfer_length:02x} 32 {lenth_offload}")

stdout, stderr = ssh._run(f"busybox devmem 0x{rx_offload_base_addr + offload_transfer_length:02x} 32")
print(f"rx_offload offload_transfer_length: 0x{rx_offload_base_addr + offload_transfer_length:02x} (Relative {offload_transfer_length:02x}): {stdout}")

for r in range(100):
    rx_sig  = conv.rx()
    ch1.plot(rx_t, np.abs(rx_sig[0]))
    ch1.set_title("Channel 1 data")
    ch2.plot(rx_t, np.abs(rx_sig[1]))
    ch2.set_title("Channel 2 data")
    fig.suptitle(f"Capture number: {r}")
    
    # if r % 2 ==1 :
        # conv.tx_destroy_buffer()
        # # tx_sig  = np.concatenate((np.zeros(data_gap),np.linspace(A,0,N_tx-(2*data_gap)),np.zeros(data_gap)),axis=None)
        # conv.tx([tx_sig, tx_sig])

    # if r % 2 ==0 :
        # conv.tx_destroy_buffer()
        # # tx_sig  = np.concatenate((np.zeros(data_gap),np.linspace(0,A,N_tx-(2*data_gap)),np.zeros(data_gap)),axis=None)
        # conv.tx([tx_sig, tx_sig])
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