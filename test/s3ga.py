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

from enum import IntEnum
from functools import reduce
import random
import math

class S3GA:
    def __init__(self, N, M, B, K, LB_IB, CFG_W, luts):
        self.N = N
        self.M = M
        self.B = B
        self.K = K
        self.LB_IB = LB_IB
        self.CFG_W = CFG_W
        self.luts = luts


class LB:
    def __init__(self, M, B, K, G, I, CFG_W, luts):
        self.M = M
        self.B = B
        self.K = K
        self.G = G
        self.I = I
        self.SEG_W = CFG_W - 1
        self.tick = 0
        self.luts = luts
        self.obuf = [0]*M
        self.half_q = 0
        self.swizzle_local_input_indices()

    def swizzle_local_input_indices(self):
        for m in range(self.M):
            for k in range(self.K):
                idx = self.luts[m][k]
                if 0 <= idx and idx < self.M:
                    idx = (self.M + m-idx - 1) % self.M
                    self.luts[m][k] = idx

    # yield LB's configuration bitstream
    def cfg(self):
        lb_in_w = self.G + 1
        lb_sel_w = (lb_in_w-1).bit_length()
        lut_in_w = self.I*self.M + self.M
        lut_sel_w = (lut_in_w-1).bit_length()
        lut_w = self.I*lb_sel_w + self.K*lut_sel_w + (1<<self.K) + 3
        segs = math.ceil(lut_w/self.SEG_W)

        for s in range(segs):
            for m in range(self.M):
                lut = self.luts[m]
                f = self.mask(lut[self.K])
                # LUT input mux selects
                for k in range(self.K):
                    idx = lut[k]
                    if idx < 0:     # special input half_q/1?
                        idx = lut_in_w-1
                    f |= idx << ((1<<self.K) + k*lut_sel_w) 
                # LB input mux selects not yet implemented, currently zero
                # 3 FDRE control bits not yet implemented
                f = f << 3
                yield (1<<self.SEG_W) | seg(f, s, self.SEG_W)

    def eval(self):
        lut = self.luts[self.tick]
        self.tick = (self.tick + 1) % self.M

        inp = 0
        for k in range(self.K-1,-1,-1):
            idx = lut[k]
            if idx == -1:
                assert k >= self.K-2
                b = 1 if k==self.K-1 else self.half_q
            else:
                b = self.obuf[idx]
            inp = (inp<<1) | b
        m = self.mask(lut[self.K])
        o = (m >> inp) & 1
        self.half_q = (m >> (inp&~(1<<self.K-1))) & 1
        self.obuf = [o] + self.obuf[0:self.M-1]
        return o

    def mask(self, fn):
        o = 0
        for i in range((1<<self.K)-1,-1,-1):
            b = [1 if (i & (1<<j)) else 0 for j in range(self.K)]
            t = fn(*b)
            o = (o << 1) | t
        return o


class IOB:
    def __init__(self, M, IO_I_W, IO_O_W, I_W, O_W, CFG_W, i_xbar, o_sels):
        assert len(i_xbar) == M and len(i_xbar[0]) == O_W and len(o_sels) == IO_O_W
        self.M = M
        self.IO_I_W = IO_I_W
        self.IO_O_W = IO_O_W
        self.I_W = I_W
        self.O_W = O_W
        self.SEG_W = CFG_W - 1
        self.i_xbar = Xbar(M, IO_I_W, O_W, CFG_W, i_xbar) # IO_I_W!
        self.o_sels = o_sels

    # yield IOB's configuration bitstream
    def cfg(self):
        # parallel to serial input crossbar
        for cfg_i in self.i_xbar.cfg():
            yield cfg_i

        # serial to parallel output selects
        sel_w = (self.I_W-1).bit_length()
        tick_w = (self.M-1).bit_length()
        sels = reduce(lambda a,b: a|b,
            [(self.o_sels[i] % self.I_W) << i*sel_w for i in range(self.IO_O_W)])
        ticks = reduce(lambda a,b: a|b,
            [(self.o_sels[i] // self.I_W) << i*tick_w for i in range(self.IO_O_W)])
        f = (ticks << (self.IO_O_W * sel_w)) | sels;
        segs = math.ceil(self.IO_O_W * (sel_w + tick_w) / self.SEG_W)
        for s in range(segs):
            yield (1<<self.SEG_W) | seg(f, s, self.SEG_W)

class Xbar:
    def __init__(self, M, I_W, O_W, CFG_W, xbar):
        assert len(xbar) == M and len(xbar[0]) == O_W
        self.M = M
        self.I_W = I_W
        self.O_W = O_W
        self.SEG_W = CFG_W - 1
        self.xbar = xbar

    # yield Xbar's configuration bitstream
    def cfg(self):
        sel_w = (self.I_W-1).bit_length()
        sels_w = self.O_W * sel_w
        segs = math.ceil(sels_w/self.SEG_W)

        for s in range(segs):
            for m in range(self.M):
                f = reduce(lambda a,b: a|b,
                    [self.xbar[m][i] << (sel_w*i) for i in range(self.O_W)])
                yield (1<<self.SEG_W) | seg(f, s, self.SEG_W)

    def eval(self, m, inp):
        return reduce(lambda a,b: a|b,
            [((inp >> self.xbar[m][i]) & 1) << i for i in range(self.O_W)])


class Switch:
    def __init__(self, M, B, DELAY, UP_I_W, UP_O_W, DN_I_W, DN_O_W, CFG_W):
        self.M = M
        self.B = B
        self.UP_I_W = UP_I_W
        self.UP_O_W = UP_O_W
        self.DN_I_W = DN_I_W
        self.DN_O_W = DN_O_W
        self.DELAY = DELAY
        self.SEG_W = CFG_W - 1
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
        dn_segs = math.ceil(dn_sels_w/self.SEG_W)

        for b in range(self.B):
            for s in range(dn_segs):
                for m in range(self.M):
                    f = 0
                    for i in range(self.DN_O_W-1,-1,-1):
                        f = (f << dn_sel_w) | self.dns_map[b][m][i]
                    yield (1<<self.SEG_W) | seg(f, s, self.SEG_W)

        up_sel_w = (self.B*self.DN_I_W-1).bit_length()
        up_sels_w = self.UP_O_W * up_sel_w
        up_segs = math.ceil(up_sels_w/self.SEG_W)
        for s in range(up_segs):
            for m in range(self.M):
                f = 0
                for i in range(self.UP_O_W-1,-1,-1):
                    f = (f << up_sel_w) | self.up_map[m][i]
                yield (1<<self.SEG_W) | seg(f, s, self.SEG_W)

    def eval(self, m, up_i, dn_is):
        dn_os = 0
        for b in range(self.B-1,-1,-1):
            for i in range(self.DN_O_W-1,-1,-1):
                # The input select mapping for down outputs is tricky: each down
                # output's xbar input is the catenation of the OTHER down inputs
                # and the up inputs.
                idx = self.dns_map[b][m][i]
                if idx < (self.B-1)*self.DN_I_W:
                    idx += self.DN_I_W if idx >= b*self.DN_I_W else 0   # skip self input bank b
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


# return ith w-bit segment of e
def seg(e, i, w):
    return (e >> (i*w)) & ((1<<w)-1)
