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
import s3ga
from s3ga import seg

# IOB parameters (IOB32)
M = 8
CFG_W = 5
IO_I_W = 16
IO_O_W = 16
I_W = 7
O_W = 4

i_xbar = [[m*O_W + i for i in range(O_W)] for m in range(IO_I_W//O_W)] + \
         [[0         for i in range(O_W)] for m in range(IO_I_W//O_W,M)]

o_sels = [i for i in range(IO_O_W)]

async def reset(dut):
    dut.rst.value = 1
    dut.cfg.value = 1
    dut.m.value = 0
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

    dut.grst.value = 1
    dut.cfg_i.value = 0
    dut.io_i = 0
    dut.i = 0
    await reset(dut)
    assert dut.cfgd.value == 0

    # test reset resets the IOB
    for i in range(2*M):
        await RisingEdge(dut.clk)
        assert dut.o.value == 0

    # configure IOB
    for cfg_i in iob.cfg():
        dut.cfg_i.value = cfg_i
        await RisingEdge(dut.clk)

    # end configuration
    dut.grst.value = 0
    dut.cfg_i.value = 0

    # test parallel -> serial inputs
    for i in input_tests():
        dut.io_i.value = i
        for m in range(M):
            await FallingEdge(dut.clk)
            e = reduce(lambda a,b: a|b, [seg(i, i_xbar[m][j], 1) << j for j in range(O_W)])
            assert dut.o.value.integer == e
    dut.io_i.value = 0
    await RisingEdge(dut.clk)

    # test serial -> parallel outputs
    for i in output_tests():
        for m in range(M):
            dut.i.value = seg(i, m, I_W)
            await RisingEdge(dut.clk)
        await FallingEdge(dut.clk)
        e = reduce(lambda a,b: a|b, [seg(i, o_sels[j], 1) << j for j in range(IO_O_W)])
        assert dut.io_o.value.integer == e

def input_tests():
    yield 0
    for j in range(IO_I_W):
        yield 1<<j
    for _ in range(1000):
        yield random.randrange(1<<IO_I_W)

def output_tests():
    for j in range(M*I_W):
        yield 1<<j
    yield 0
    for _ in range(1000):
        yield random.randrange(1<<(M*I_W))
