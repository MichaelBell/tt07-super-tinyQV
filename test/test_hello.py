# SPDX-FileCopyrightText: © 2024 Michael Bell
# SPDX-License-Identifier: MIT

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer
import cocotb.utils

async def reset(dut, latency=1, ui_in=0):
  # Reset
  dut._log.info(f"Reset, latency {latency}")
  dut.ena.value = 1
  dut.ui_in_base.value = ui_in
  dut.uio_in.value = 0
  dut.rst_n.value = 1
  dut.uart_rx.value = 1
  await ClockCycles(dut.clk, 2)
  dut.rst_n.value = 0
  dut.latency_cfg.value = latency
  await ClockCycles(dut.clk, 1)
  assert dut.uio_oe.value == 0
  await ClockCycles(dut.clk, 9)
  dut.rst_n.value = 1
  await ClockCycles(dut.clk, 1)
  assert dut.uio_oe.value == 0b11001001

async def receive_string(dut, str):
    for char in str:
        dut._log.debug(f"Wait for: {char}")

        for _ in range(5000):
            await ClockCycles(dut.clk, 8)
            if dut.uart_tx.value == 0:
                break
        else:
            # Should have started by now
            assert dut.uart_tx.value == 0
        
        uart_byte = ord(char)
        bit_time = 8680
        await Timer(bit_time / 2, "ns")
        assert dut.uart_tx.value == 0
        for i in range(8):
            await Timer(bit_time, "ns")
            assert dut.uart_tx.value == (uart_byte & 1)
            uart_byte >>= 1
        await Timer(bit_time, "ns")
        assert dut.uart_tx.value == 1

async def read_string(dut):
    str = ""
    while not str.endswith('\r'):
        for _ in range(25000):
            await ClockCycles(dut.clk, 8)
            if dut.uart_tx.value == 0:
                break
        else:
            # Should have started by now
            dut._log.info(f"Received before fail: {str}")
            assert dut.uart_tx.value == 0
        
        uart_byte = 0
        bit_time = 8680
        await Timer(bit_time / 2, "ns")
        assert dut.uart_tx.value == 0
        for i in range(8):
            await Timer(bit_time, "ns")
            uart_byte |= dut.uart_tx.value << i
        await Timer(bit_time, "ns")
        assert dut.uart_tx.value == 1
        str += chr(uart_byte)
        dut._log.debug(f"Recvd: {chr(uart_byte)}")
    return str

@cocotb.test()
async def test_hello(dut):
    dut._log.debug("Start")
  
    # Our example module doesn't use clock and reset, but we show how to use them here anyway.
    clock = Clock(dut.clk, 15.624, units="ns")
    cocotb.start_soon(clock.start())

    for latency in range(1, 6):
        start_time = cocotb.utils.get_sim_time("ns")
        await reset(dut, latency)

        # Should output: Hello, world!\n
        await receive_string(dut, "Hello, world!\r\n")

        await receive_string(dut, "Hello 3\r\n")
        await receive_string(dut, "Hello 36\r\n")
        run_time = int(cocotb.utils.get_sim_time("ns") - start_time)
        dut._log.info(f"Took {run_time}ns at latency {latency}")

        if latency == 1 or latency == 5:
            s = await read_string(dut)
            dut._log.info(f"Received: {s}")
