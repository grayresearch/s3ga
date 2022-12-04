# S3GA: simple scalable serial FPGA
# By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC. All rights reserved.

from enum import IntEnum
import random
import math

class CfgRamState(IntEnum):
	ST_WAIT	= 0
	ST_CFG	= 1
	ST_PASS	= 2

class Switch:
	def __init__(self, M, B, DELAY, UP_I_W, UP_O_W, DN_I_W, DN_O_W, CFG_W):
		self.M = M
		self.B = B
		self.UP_I_W = UP_I_W
		self.UP_O_W = UP_O_W
		self.DN_I_W = DN_I_W
		self.DN_O_W = DN_O_W
		self.DELAY = DELAY
		self.CFG_W = CFG_W
		self.DN_X_W = DN_X_W = UP_I_W + (B-1)*DN_I_W

		# random switch xbar maps
		self.up_map = [[random.randrange(B*DN_I_W) for i in range(UP_O_W)] for m in range(M)]
		self.dns_map = [[[random.randrange(DN_X_W) for i in range(DN_O_W)] for m in range(M)] for b in range(B)]
		# with 0th context = {i:i}
		self.up_map[0] = [i for i in range(UP_O_W)]
		for b in range(B):
			self.dns_map[b][0] = [i for i in range(DN_O_W)]

	# yield switch's configuration bitstream, catenation of B+1 crossbar bitstreams
	def cfg(self):
		dn_sel_w = (self.DN_X_W-1).bit_length()
		dn_sels_w = self.DN_O_W * dn_sel_w
		dn_segs = math.ceil(dn_sels_w/self.CFG_W)

		for b in range(self.B):
			yield 1						# start
			for seg in range(dn_segs):
				for m in range(self.M):
					f = 0
					for i in range(self.DN_O_W-1,-1,-1):
						f = (f << dn_sel_w) | self.dns_map[b][m][i]
					yield self.cfg_i(f, seg)
			for i in range(self.M-2):	# M-1, -1 more for the cfg_o reg
				yield 0					# pad

		up_sel_w = (self.B*self.DN_I_W-1).bit_length()
		up_sels_w = self.UP_O_W * up_sel_w
		up_segs = math.ceil(up_sels_w/self.CFG_W)
		yield 1							# start
		for seg in range(up_segs):
			for m in range(self.M):
				f = 0
				for i in range(self.UP_O_W-1,-1,-1):
					f = (f << up_sel_w) | self.up_map[m][i]
				yield self.cfg_i(f, seg)
		for i in range(self.M-2):
			yield 0						# pad

	def cfg_i(self, f, i):
		return (f >> (i*self.CFG_W)) & ((1<<self.CFG_W)-1)

	def eval(self, m, up_i, dn_is):
		dn_os = 0
		for b in range(self.B-1,-1,-1):
			for i in range(self.DN_O_W-1,-1,-1):
				# The input select mapping for down outputs is tricky: each down
				# output's xbar input is the catenation of the OTHER down inputs
				# and the up inputs.
				idx = self.dns_map[b][m][i]
				if idx < (self.B-1)*self.DN_I_W:
					idx += self.DN_I_W if idx >= b*self.DN_I_W else 0	# skip self input bank b
					bit = (dn_is >> idx) & 1
				else:
					idx -= (self.B-1)*self.DN_I_W
					bit = (up_i >> idx) & 1
				dn_os = (dn_os << 1) | bit
		up_o = 0
		for i in range(self.UP_O_W-1,-1,-1):
			bit = (dn_is >> self.up_map[m][i]) & 1
			up_o = (up_o<<1) | bit
        return (up_o,dn_os)
