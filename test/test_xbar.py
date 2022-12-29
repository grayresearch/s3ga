# S3GA: simple scalable serial FPGA
# By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC.

# SPDX-FileCopyrightText: 2022 Gray Research LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge
import math
import random
from s3ga import seg

# xbar parameters
M     = 4 # M contexts
DELAY = 1 # no. of output pipeline stages
I_W   = 4 # input  width
O_W   = 4 # output width 
CFG_W = 5 # config I/O width
SEG_W = CFG_W-1 # config I/O width

SEL_W = (I_W-1).bit_length()
SEGS  = M * O_W * SEL_W // SEG_W

# M=8 different input permutations
map8 = [ [0,0,0,0], [0,1,2,3], [3,2,1,0], [3,0,1,2],
        [1,2,3,0], [2,1,0,3], [1,2,1,2], [3,3,3,3] ] 

# M=4 different input permutations
map4 = [ [0,0,0,0], [0,1,2,3], [1,2,1,3], [3,0,1,2] ]

map = map4 if M==4 else map8

async def reset(dut):
    dut.rst.value = 1
    dut.cfg.value = 1
    dut.m.value = 0
    await ClockCycles(dut.clk, M+1)
    dut.rst.value = 0
    cocotb.fork(m_counter(dut))

ticks = 0
async def m_counter(dut):
    global ticks
    while True:
        await RisingEdge(dut.clk)
        dut.m.value = ticks = (ticks + 1) % M

@cocotb.test()
async def test_xbar(dut):
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.fork(clock.start())

    dut.cfg_i.value = 0
    dut.i.value = 0
    await reset(dut)
    assert dut.cfgd.value == 0

    # test reset resets the xbar
    for i in range(2*M):
        await FallingEdge(dut.clk)
        assert dut.cfgd.value == 0
        assert dut.o.value == 0

    # config map[] into xbar.selects
    for i in range(2):
        for m in range(M):
            dut.cfg_i.value = (1<<SEG_W) | (map[m][2*i+1] << 2) | map[m][2*i] 
            await FallingEdge(dut.clk)

    # configured
    dut.cfg_i.value = 0
    assert dut.cfgd.value == 1

    # check cfg_ram correctly loaded
    for m in range(M):
        expect = 0
        for i in range(4):
            expect = (expect << 2) | map[m][3-i]
        assert dut.sels.value == expect
        await FallingEdge(dut.clk)

    # check crossbar works for various inputs across M contexts / mappings
    for i in range(1<<I_W):
        for m in range(M):
            expect = 0
            for j in range(O_W-1,-1,-1):
                expect = (expect << 1) | seg(i, map[m][j], 1)
            dut.i.value = i
            await FallingEdge(dut.clk)
            assert dut.o.value == expect

    for k in range(100):
        i = random.randrange(1<<I_W)
        for m in range(M):
            expect = 0
            for j in range(O_W-1,-1,-1):
                expect = (expect << 1) | seg(i, map[m][j], 1)
            dut.i.value = i
            await FallingEdge(dut.clk)
            assert dut.o.value == expect
