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
reset_reg_offload  = 0x84
control_reg_offload = 0x88

# with open("tdd_settings1.txt", "w") as f:
#     for i in range(0,0x0200,4):
#            stdout, stderr = ssh._run(f"busybox devmem 0x{tx_offload_base_addr + i:02x}")
#            print(f"0x{tx_offload_base_addr + i:02x} (Relative {i:02x}): {stdout}")
#            f.write(f"0x{tx_offload_base_addr + i:02x}, (Relative {i:02x}), {stdout}\n")

# ssh._run(f"busybox devmem 0x{tx_offload_base_addr + reset_reg_offload:02x} 32 0x0")
# ssh._run(f"busybox devmem 0x{tx_offload_base_addr + reset_reg_offload:02x} 32 0x1")

# with open("tdd_settings2.txt", "w") as f:
#     for i in range(0,0x0200,4):
#            stdout, stderr = ssh._run(f"busybox devmem 0x{tx_offload_base_addr + i:02x}")
#            print(f"0x{tx_offload_base_addr + i:02x} (Relative {i:02x}): {stdout}")
#            f.write(f"0x{tx_offload_base_addr + i:02x}, (Relative {i:02x}), {stdout}\n")       


# Make sure that the offload is not in one-shot-mode


conv = adi.ad9081(url)
tddn = adi.tddn(url)

conv._rxadc.set_kernel_buffers_count(1)

print("0x001B:", conv.ad9081_register_read(0x001B))
conv.ad9081_register_write(0x001B,0xf)
print("0x001B:", conv.ad9081_register_read(0x001B))

print("0x0321:", conv.ad9081_register_read(0x0321))
conv.ad9081_register_write(0x0321,0x00)
print("0x0321:", conv.ad9081_register_read(0x0321))

print("--Setting up chip")

# Set NCOs
conv.rx_channel_nco_frequencies = [0] * 4
conv.tx_channel_nco_frequencies = [0] * 4

conv.rx_main_nco_frequencies = [100000000] * 4
conv.tx_main_nco_frequencies = [100000000] * 4

conv.rx_enabled_channels = [0,1]
conv.tx_enabled_channels = [0,1]
conv.rx_nyquist_zone = ["odd"] * 4

conv.tx_cyclic_buffer = True
conv.tx_ddr_offload = False
time.sleep(1)

# make sure that the offload is in one-shot mode 

stdout, stderr = ssh._run(f"busybox devmem 0x{tx_offload_base_addr + control_reg_offload:02x} 32")
print(f"0x{tx_offload_base_addr + control_reg_offload:02x} (Relative {control_reg_offload:02x}): {stdout}")
stdout, stderr = ssh._run(f"busybox devmem 0x{tx_offload_base_addr + control_reg_offload:02x} 32 0x2")
stdout, stderr = ssh._run(f"busybox devmem 0x{tx_offload_base_addr + control_reg_offload:02x} 32")
print(f"0x{tx_offload_base_addr + control_reg_offload:02x} (Relative {control_reg_offload:02x}): {stdout}")

samples_per_frame_desired = 2**10
frame_length_ms = samples_per_frame_desired / conv.tx_sample_rate * 1000
print(f"frame_length_ms: {frame_length_ms}")

tddn.enable = 0
tddn.startup_delay_ms = 0
tddn.frame_length_ms = frame_length_ms

tddn.channel[2].on_ms    = 0
tddn.channel[2].off_ms   = 0
tddn.channel[2].polarity = 1
tddn.channel[2].enable   = 1 

for chan in [0,1] :
    tddn.channel[chan].on_ms = 0
    tddn.channel[chan].off_ms = frame_length_ms/2
    tddn.channel[chan].polarity = 0
    tddn.channel[chan].enable = 1

for chan in [3,4,5] :
    tddn.channel[chan].on_ms = 0.00050
    tddn.channel[chan].off_ms = frame_length_ms/2 + 0.00050
    tddn.channel[chan].polarity = 0
    tddn.channel[chan].enable = 1

tddn.enable = 1
# Create and send data to TX DMA

fs = int(conv.tx_sample_rate)
print(f"Sampling_rate: {fs}")

samples_per_frame = math.ceil(tddn.frame_length_ms * fs / 1000)
print(f"Samples per frame: {samples_per_frame}")

N = int(samples_per_frame/2)

assert N < 2**15

A = 0.9 * 2**15  # -6 dBFS

tx_sig  = np.linspace(A,0,N)
tx_sig2 = np.linspace(A,0,N)
conv.tx([tx_sig,tx_sig2])

time.sleep(1)

tddn.sync_soft  = 0
tddn.sync_soft  = 1

print("An infinite loop untill you type 's'")

while True:
    user_input = input("Enter 's' to quit the loop: ")
    print("You entered:", user_input)
   
    if user_input.lower() == 's':
        tddn.enable = 0
       
        tddn.channel[2].on_ms    = 0
        tddn.channel[2].off_ms   = 0
        tddn.channel[2].polarity = 0
        tddn.channel[2].enable   = 1 
        
        tddn.enable = 1
        tddn.enable = 0
        conv.tx_destroy_buffer()
        break       
 