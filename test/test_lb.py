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

# lb parameters (LB)
M = 4
B = 4
K = 4
G = 8
I = 4
CFG_W = 5

# some 4-LUTs and 3,3-LUTs (where input #3=sel)
def inc0(a,_,_2,sel):
    return 1-a if sel else a

def inc(a,_,ci,sel):
    return (a^ci) if sel else (1 if a+ci>1 else 0)

def add0(a,b,_,sel):
    return (a^b) if sel else (1 if a+b>1 else 0)

def add(a,b,ci,sel):
    return (a^b^ci) if sel else (1 if a+b+ci>1 else 0)

def xor4(a,b,c,d):
    return a^b^c^d

def xor3(a,b,c,_):
    return a^b^c

# xor(6b counter)
inc_xor_luts_m8 = [
    [  0, 0,-1,-1, inc0 ],
    [  1, 0,-1,-1, inc  ],
    [  2, 0,-1,-1, inc  ],
    [  3, 0,-1,-1, inc  ],
    [  4, 0,-1,-1, inc  ],
    [  5, 0,-1,-1, inc  ],
    [  0, 1, 2, 3, xor4 ],
    [  4, 5, 6,-1, xor3 ]
]

inc_xor_luts_m4 = [
    [  0, 0,-1,-1, inc0 ],
    [  1, 0,-1,-1, inc  ],
    [  2, 0,-1,-1, inc  ],
    [  0, 1, 2, 0, xor3 ]
]

async def reset(dut):
    dut.rst.value = 1
    dut.m.value = 0
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
async def test_lb(dut):
    lb = s3ga.LB(M, B, K, G, I, CFG_W, inc_xor_luts_m4)

    clock = Clock(dut.clk, 10, units="ns")
    cocotb.fork(clock.start())

    dut.grst.value = 1
    dut.cfg.value = 1
    dut.cfg_i.value = 0
    dut.globs.value = 0
    dut.locals.value = 0
    dut.half_i.value = 0
    await reset(dut)
    assert dut.cfgd.value == 0

    # test reset resets the lb
    for i in range(2*M):
        await FallingEdge(dut.clk)
        assert dut.o.value == 0

    # configure lb
    while dut.m.value != 0:
        await RisingEdge(dut.clk)
        await FallingEdge(dut.clk)

    for cfg_i in lb.cfg():
        dut.cfg_i.value = cfg_i
        await RisingEdge(dut.clk)

    # let LB come out of internal cfg hold
    dut.cfg_i.value = 0
    global ticks
    while ticks != 0:
        await RisingEdge(dut.clk)
    dut.cfg.value = 0   # configuration done
    dut.grst.value = 0

    # test the counter / xor results
    for i in range(1,(8 if M==4 else 64)):
        o = 0
        for m in range(M):
            dut.locals.value = dut.o.value
            await FallingEdge(dut.clk)
            if m < (3 if M==4 else 6):
                o |= (dut.o.value << m)
        assert o == i
        assert xor_reduce(i) == dut.o.value

def xor_reduce(i):
    return reduce(lambda a,b: a^b, [(i>>j) & 1 for j in range(i.bit_length())])
