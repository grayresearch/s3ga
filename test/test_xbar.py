# S3GA: simple scalable serial FPGA
# By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC. All rights reserved.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge
import math
import random

# xbar parameters
M     = 8 # M contexts
DELAY = 1 # no. of output pipeline stages
I_W   = 4 # input  width
O_W   = 4 # output width 
CFG_W = 4 # config I/O width

SEL_W = (I_W-1).bit_length()
SEGS  = M * O_W * SEL_W // CFG_W

# cfg state machine
ST_WAIT = 0
ST_CFG  = 1
ST_PASS = 2

# M=8 different input permutations
map = [ [0,0,0,0], [0,1,2,3], [3,2,1,0], [3,0,1,2],
        [1,2,3,0], [2,1,0,3], [1,2,1,2], [3,3,3,3] ] 

async def reset(dut):
    dut.rst.value = 1
    await ClockCycles(dut.clk, M+1)
    dut.rst.value = 0

@cocotb.test()
async def test_xbar(dut):
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.fork(clock.start())

    dut.cfg_i.value = 0
    dut.i.value = 0
    await reset(dut)
    assert dut.cfg_o.value == 0

    # test reset resets the xbar
    for i in range(2*M):
        await FallingEdge(dut.clk)
        assert dut.cfg_o.value == 0
        assert dut.o.value == 0
        assert dut.selects.st.value == ST_WAIT

    # config ram -> config state
    dut.cfg_i.value = 1
    await FallingEdge(dut.clk)
    assert dut.cfg_o.value == 0
    assert dut.selects.st.value == ST_CFG

    # config map[] into xbar.selects
    for i in range(2):
        for m in range(M):
            dut.cfg_i.value = (map[m][2*i+1] << 2) | map[m][2*i] 
            await FallingEdge(dut.clk)

    # config ram -> pass state
    dut.cfg_i.value = 0
    assert dut.selects.st.value == ST_PASS

    # check cfg_ram correctly loaded
    for m in range(M):
        expect = 0
        for i in range(4):
            expect = (expect << 2) | map[m][3-i]
        assert dut.sels.value == expect
        await FallingEdge(dut.clk)
        assert dut.cfg_o.value == 0

    # check crossbar works for various inputs across M contexts / mappings
    for i in range(1<<I_W):
        for m in range(M):
            expect = 0
            for j in range(O_W-1,-1,-1):
                expect = (expect << 1) | ((i >> map[m][j]) & 1)
            dut.i.value = i
            await FallingEdge(dut.clk)
            assert dut.o.value == expect

    for k in range(100):
        i = random.randrange(1<<I_W)
        for m in range(M):
            expect = 0
            for j in range(O_W-1,-1,-1):
                expect = (expect << 1) | ((i >> map[m][j]) & 1)
            dut.i.value = i
            await FallingEdge(dut.clk)
            assert dut.o.value == expect

    # test pass through of more config data
    for i in range(4<<CFG_W):
        r = random.randrange(16)
        dut.cfg_i.value = r
        await FallingEdge(dut.clk)
        assert dut.cfg_o.value == r
