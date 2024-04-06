# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: MIT

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer, FallingEdge, Edge
from helper import float16_to_8bit_bytes, binary_strings_to_float16
import numpy as np


@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")
    
    # Initialize the clock
    clock_period = 83.33
    clock = Clock(dut.clk, clock_period, units="ns")
    cocotb.start_soon(clock.start())
    cocotb.start_soon(monitor_tx(dut))


    # Reset
    dut._log.info("Reset")
    dut.ena.value = 1
    dut.ui_in.value = 1
    dut.uio_in.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)
    await send_byte(dut, "11111110")
    await Timer(1000, units='ns')
    print("uo_out:", dut.uo_out.value)
    for i in range(8):
            float16 = np.float16(i+1)
            print("Sending float16:", float16)
            high, low = float16_to_8bit_bytes(float16)
            # Send the high byte of the float
            await send_byte(dut, format(high, '08b'))
            await ClockCycles(dut.clk, 1)
            # Send the low byte of the float
            await send_byte(dut, format(low, '08b'))
            await ClockCycles(dut.clk, 1)
    print("Sent all float16 values")
    print("uo_out:", dut.uio_out.value)
    await Timer(12000, units='us') #should have received all float16 values back by this time
    print("FIRST PASS DONE")

    #second pass 

    await send_byte(dut, "11111110")
    await Timer(1000, units='ns')
    print("uo_out:", dut.uo_out.value)
    for i in range(8):
            float16 = np.float16(i+1)
            print("Sending float16:", float16)
            high, low = float16_to_8bit_bytes(float16)
            # Send the high byte of the float
            await send_byte(dut, format(high, '08b'))
            await ClockCycles(dut.clk, 1)
            # Send the low byte of the float
            await send_byte(dut, format(low, '08b'))
            await ClockCycles(dut.clk, 1)
    print("Sent all float16 values")
    print("uo_out:", dut.uio_out.value)
    await Timer(12000, units='us') #should have received all float16 values back by this time
    print("SECOND PASS DONE")

async def send_byte(dut, byte):
    print("Byte to send:", byte)
    # Send start bit
    dut.ui_in[0].value = 0
    await Timer(104, units='us')  # Delay for 1 bit time at 9600 baud
    #Send data bits
    for i in range(7, -1, -1):
        dut.ui_in[0].value = int(byte[i])
        await Timer(104, units='us')  # Delay for 1 bit time at 9600 baud
    #Send stop bit
    dut.ui_in[0].value = 1
    await Timer(104, units='us')  # Delay for 1 bit time at 9600 baud

async def monitor_tx(dut):
    while True:
        await Edge(dut.uo_out)
        data = ""
        for _ in range(8):
            await Timer(104, units='us')  # Wait for 1 bit time at 9600 baud
            data = str(dut.uo_out[0].value.integer) + data
        await Timer(104, units='us')  # Wait for stop bit
        stop_bit = dut.uo_out[0].value.integer
        dut._log.info(f"Received byte from TX: {data}")