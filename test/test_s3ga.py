# S3GA: simple scalable serial FPGA
# By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC. All rights reserved.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, FallingEdge
from functools import reduce
import math
import random
import s3ga

# s3ga parameters (S3GA<N=128>)
N = 128
M = 8
B = 4
K = 4
LB_IB = 3
CFG_W = 4
IO_I_W = 16
IO_O_W = 16
UP_I_WS = 60600
UP_O_WS = 40400

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
inc_xor_luts = [
	[  0, 0,-1,-1, inc0 ],
	[  1, 0,-1,-1, inc  ],
	[  2, 0,-1,-1, inc  ],
	[  3, 0,-1,-1, inc  ],
	[  4, 0,-1,-1, inc  ],
	[  5, 0,-1,-1, inc  ],
	[  0, 1, 2, 3, xor4 ],
	[  4, 5, 6,-1, xor3 ]
]

async def reset(dut):
	dut.rst.value = 1
	await ClockCycles(dut.clk, 2*M)
	dut.rst.value = 0

@cocotb.test()
async def test_s3ga(dut):
	s3 = s3ga.S3GA(N, M, B, K, LB_IB, CFG_W, inc_xor_luts)

	clock = Clock(dut.clk, 10, units="ns")
	cocotb.fork(clock.start())

	dut.cfg_i.value = 0
	dut.io_i.value = 0
	await reset(dut)

	# test reset resets the S3GA
	for i in range(2*M):
		await FallingEdge(dut.clk)

	await ClockCycles(dut.clk, 10*M)
'''
	# configure S3GA
	for cfg_i in lb.cfg():
		dut.cfg_i.value = cfg_i
		await RisingEdge(dut.clk)
'''
