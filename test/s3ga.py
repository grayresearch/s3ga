# S3GA: simple scalable serial FPGA
# By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC. All rights reserved.

from enum import IntEnum
import random
import math

class CfgRamState(IntEnum):
    ST_WAIT = 0
    ST_CFG  = 1
    ST_PASS = 2

class LB:
    def __init__(self, M, B, K, G, I, CFG_W, luts):
        self.M = M
        self.B = B
        self.K = K
        self.G = G
        self.I = I
        self.CFG_W = CFG_W
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
        lb_in_w = self.G + self.B - 2
        lb_sel_w = (lb_in_w-1).bit_length()
        lut_in_w = self.I*self.M + self.M
        lut_sel_w = (lut_in_w-1).bit_length()
        lut_w = self.I*lb_sel_w + self.K*lut_sel_w + (1<<self.K) + 3
        segs = math.ceil(lut_w/self.CFG_W)

        yield 1                     # start

        for seg in range(segs):
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
                yield cfg_seg(f, seg, self.CFG_W)

        for i in range(self.M-2):   # M-1, -1 more for the cfg_o reg
            yield 0                 # pad

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
            yield 1                     # start
            for seg in range(dn_segs):
                for m in range(self.M):
                    f = 0
                    for i in range(self.DN_O_W-1,-1,-1):
                        f = (f << dn_sel_w) | self.dns_map[b][m][i]
                    yield cfg_seg(f, seg, self.CFG_W)
            for i in range(self.M-2):   # M-1, -1 more for the cfg_o reg
                yield 0                 # pad

        up_sel_w = (self.B*self.DN_I_W-1).bit_length()
        up_sels_w = self.UP_O_W * up_sel_w
        up_segs = math.ceil(up_sels_w/self.CFG_W)
        yield 1                         # start
        for seg in range(up_segs):
            for m in range(self.M):
                f = 0
                for i in range(self.UP_O_W-1,-1,-1):
                    f = (f << up_sel_w) | self.up_map[m][i]
                yield cfg_seg(f, seg, self.CFG_W)
        for i in range(self.M-2):
            yield 0                     # pad

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


def cfg_seg(f, i, cfg_w):
    return (f >> (i*cfg_w)) & ((1<<cfg_w)-1)
