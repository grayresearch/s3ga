# S3GA: simple scalable serial FPGA
# By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC. All rights reserved.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge
import math
import random

# cfg_ram parameters
M = 8
W = 7
CFG_W = 4
SEGS = M*math.ceil(W/CFG_W)
ST_WAIT = 0
ST_CFG = 1
ST_PASS = 2

async def reset(dut):
    dut.rst.value = 1
    dut.cfg_i.value = 0
    await ClockCycles(dut.clk, M)
    dut.rst.value = 0

@cocotb.test()
async def test_cfg_ram(dut):
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.fork(clock.start())

    await reset(dut)
    assert dut.cfg_o.value == 0

    # test reset resets the ram
    for i in range(2*M):
        await FallingEdge(dut.clk)
        assert dut.cfg_o.value == 0
        assert dut.o.value == 0
        assert dut.st.value == ST_WAIT

    # -> config state
    dut.cfg_i.value = 1
    await FallingEdge(dut.clk)
    assert dut.cfg_o.value == 0

    # configure to (0x80,0x91,...,0xF7) & 0x7F
    for i in range(SEGS):
        dut.cfg_i.value = i
        assert dut.st.value == ST_CFG
        await FallingEdge(dut.clk)
        assert dut.cfg_o.value == 0

    # last segment received -> pass state
    assert dut.st.value == ST_PASS
    dut.cfg_i.value = 0

    # test ram outputs
    for i in range(M):
        assert dut.o.value == (((i+8)<<4) + i) & ((1<<W)-1)
        await FallingEdge(dut.clk)
        assert dut.st.value == ST_PASS
        assert dut.cfg_o.value == 0

    # test pass through of more config data
    for i in range(4<<CFG_W):
        r = random.randrange(16)
        dut.cfg_i.value = r
        await FallingEdge(dut.clk)
        assert dut.cfg_o.value == r
