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
import s3ga

# switch parameters (X128 switch)
M = 4
B = 4
DELAY = 1
UP_I_W = 12
UP_O_W = 8
DN_I_W = 4
DN_O_W = 6
CFG_W = 5

async def reset(dut):
    dut.rst.value = 1
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
async def test_switch(dut):
    sw = s3ga.Switch(M, B, DELAY, UP_I_W, UP_O_W, DN_I_W, DN_O_W, CFG_W)

    clock = Clock(dut.clk, 10, units="ns")
    cocotb.fork(clock.start())

    dut.cfg.value = 1
    dut.cfg_i.value = 0
    dut.up_i.value = 0
    dut.dn_is.value = 0
    await reset(dut)
    assert dut.cfgd.value == 0

    # test reset resets the switch
    for i in range(2*M):
        await FallingEdge(dut.clk)
        assert dut.cfgd.value == 0
        assert dut.up_o.value == 0
        assert dut.dn_os.value == 0

    # configure switch
    while dut.m.value != 0:
        await RisingEdge(dut.clk)
    for cfg_i in sw.cfg():
        dut.cfg_i.value = cfg_i
        await RisingEdge(dut.clk)

    dut.cfg_i.value = 0
    await FallingEdge(dut.clk)
    assert dut.cfgd.value == 1

    # switch tests
    for (m, up_i, dn_is, up_o, dn_os) in switch_tests(sw, 1000):
        dut.up_i.value = up_i
        dut.dn_is.value = dn_is
        await FallingEdge(dut.clk)
        assert dut.up_o.value == up_o and dut.dn_os.value == dn_os, \
            "test({0:1d},{1:08x},{2:08x},{3:08x},{4:08x}) -> ({5:08x},{6:08x})".format( \
            m, up_i, dn_is, up_o, dn_os, dut.up_o.value.integer, dut.dn_os.value.integer)


def switch_tests(sw, n):
    for e in switch_bit_tests(sw):
        yield e
    for e in switch_random_tests(sw, n):
        yield e

def switch_bit_tests(sw):
    for u in range(UP_I_W+1):
        for d in range(B*DN_I_W+1):
            up_i = 1<<u if u < UP_I_W else 0
            dn_is = 1<<d if d < B*DN_I_W else 0
            for m in range(M):
                (up_o,dn_os) = sw.eval(m, up_i, dn_is)
                yield (m, up_i, dn_is, up_o, dn_os)

def switch_random_tests(sw, n):
    for _ in range(n):
        up_i = random.randrange(1<<UP_I_W)
        dn_is = random.randrange(1<<(B*DN_I_W))
        for m in range(M):
            (up_o,dn_os) = sw.eval(m, up_i, dn_is)
            yield (m, up_i, dn_is, up_o, dn_os)
