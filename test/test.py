# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: MIT

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")
    
    # Initialize the clock
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())

    # Reset
    dut._log.info("Reset")
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # Case 1: ui_in is 1, expect uo_out to be all 1's
    dut._log.info("Test with ui_in = 1")
    dut.ui_in.value = 1  # Set ui_in to 1
    await ClockCycles(dut.clk, 1)  # Wait one clock cycle
    assert dut.uo_out.value == 0xFF, f"Expected uo_out to be 0xFF, got {hex(dut.uo_out.value)}"

    # Reset before next test
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # Case 2: ui_in is not 1, specifically testing with previous value of 20
    dut._log.info("Test with ui_in = 20")
    dut.ui_in.value = 20  # Revert to previous test condition
    dut.uio_in.value = 30
    await ClockCycles(dut.clk, 1)  # Wait one clock cycle
    # Expect uo_out to be 0x03 since our Verilog code now sets only two LSBs if ui_in is not 1
    assert dut.uo_out.value == 0x03, f"Expected uo_out to be 0x03, got {hex(dut.uo_out.value)}"
