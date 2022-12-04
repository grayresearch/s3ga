# S3GA: simple scalable serial FPGA
# By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC. All rights reserved.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge
import random

@cocotb.test()
async def test_pipe(dut):
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.fork(clock.start())

    for i in range(100):
        r = random.randrange(16)
        dut.i.value = r
        await FallingEdge(dut.clk)
        assert dut.o.value == r
