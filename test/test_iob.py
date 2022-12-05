# S3GA: simple scalable serial FPGA
# By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC. All rights reserved.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge
from functools import reduce
import math
import random
import s3ga
from s3ga import seg

# IOB parameters (IOB32)
M = 8
CFG_W = 4
IO_I_W = 16
IO_O_W = 16
I_W = 6
O_W = 4

i_xbar = [[m*O_W + i for i in range(O_W)] for m in range(IO_I_W//O_W)] + \
         [[0         for i in range(O_W)] for m in range(IO_I_W//O_W,M)]

o_sels = [i for i in range(IO_O_W)]

async def reset(dut):
    dut.rst.value = 1
    dut.cfg.value = 1
    await ClockCycles(dut.clk, 2*M)
    dut.rst.value = 0
    cocotb.fork(m_counter(dut))

ticks = 0
async def m_counter(dut):
    global ticks
    while True:
        await RisingEdge(dut.clk)
        dut.m.value = ticks = (ticks + 1) % M

@cocotb.test()
async def test_iob(dut):
    iob = s3ga.IOB(M, IO_I_W, IO_O_W, I_W, O_W, CFG_W, i_xbar, o_sels)

    clock = Clock(dut.clk, 10, units="ns")
    cocotb.fork(clock.start())

    dut.cfg_i.value = 0
    dut.io_i = 0
    dut.i = 0
    await reset(dut)
    assert dut.cfg_o.value == 0

    # test reset resets the IOB
    for i in range(2*M):
        await FallingEdge(dut.clk)
        assert dut.o.value == 0

    # configure IOB
    for cfg_i in iob.cfg():
        dut.cfg_i.value = cfg_i
        await RisingEdge(dut.clk)

    # end configuration
    dut.cfg_i.value = 0
    global ticks
    while ticks != 0:
        await RisingEdge(dut.clk)
    dut.cfg.value = 0   # configuration done

    # burn an M-cycle getting things aligned
    while ticks != M-1:
        await RisingEdge(dut.clk)

    # test parallel inputs -> serial outputs
    for i in tests():
        dut.io_i.value = i
        await RisingEdge(dut.clk)
        for m in range(M):
            await FallingEdge(dut.clk)
            e = reduce(lambda a,b: a|b, [seg(i, i_xbar[m][j], 1) << j for j in range(O_W)])
            assert dut.o.value.integer == e

def tests():
    yield 0
    for j in range(IO_I_W):
        yield 1<<j
    for _ in range(1000):
        yield random.randrange(1<<IO_I_W)
