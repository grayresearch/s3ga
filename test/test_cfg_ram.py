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
from functools import reduce
import math
import random

# cfg_ram parameters
M = 4
W = 11
CFG_W = 5
SEG_W = CFG_W-1
W_SEGS = math.ceil(W/SEG_W)
SEGS = M*W_SEGS

async def reset(dut):
    dut.rst.value = 1
    dut.cfg.value = 0
    dut.cfg_i.value = 0
    dut.m.value = 0
    await ClockCycles(dut.clk, SEGS+1)
    dut.rst.value = 0
    cocotb.fork(m_counter(dut))

ticks = 0
async def m_counter(dut):
    global ticks
    while True:
        await RisingEdge(dut.clk)
        dut.m.value = ticks = (ticks + 1) % M

@cocotb.test()
async def test_cfg_ram(dut):
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.fork(clock.start())

    await reset(dut)
    assert dut.cfgd.value == 0

    # test reset resets the ram
    for i in range(2*M):
        await RisingEdge(dut.clk)
        await FallingEdge(dut.clk)
        assert dut.cfgd.value == 0
        assert dut.o.value == 0

    # configure something else
    for i in range(SEGS):
        dut.cfg_i.value = (1<<SEG_W)|i
        await FallingEdge(dut.clk)
        assert dut.cfgd.value == 0

    # configure this
    dut.cfg.value = 1
    for i in range(SEGS):
        assert dut.cfgd.value == 0
        dut.cfg_i.value = (1<<SEG_W)|i
        await FallingEdge(dut.clk)

    assert dut.cfgd.value == 1
    dut.cfg_i.value = 0

    # test ram outputs
    for m in range(M):
        e = reduce(lambda a,b: a|b, [(((s*M + m) & ((1<<SEG_W)-1)) << (s*SEG_W)) for s in range(W_SEGS)]) & ((1<<W)-1) 
        assert dut.o.value == e
        await FallingEdge(dut.clk)
        assert dut.cfgd.value == 1
