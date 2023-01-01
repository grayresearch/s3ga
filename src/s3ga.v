// S3GA: simple scalable serial FPGA
// By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC.

// SPDX-FileCopyrightText: 2022 Gray Research LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`include "s3ga.h"

// Common clocking signals seen by every submodule
//  clk
//  rst     // reset; transitions only at start of an M-cycle
//  grst    // global (programmable logic) reset; asserted til S3GA is configured; ditto
//  m       // cycle % M
//  cfg_i   // config data segment
//  tock    // cycle % M == M-1
// Across the huge S3GA hierarchy of cores, care is taken to ensure
// every leaf configurable module (switch, lb, iob) sees identical
// (rst, grst, m, cfg_i) -- and so does the external SoC.

// S3GA: simple scalable serial FPGA

module s3ga #(
    parameter N         = 256,          // N logical LUTs
    parameter M         = 4,            // M contexts
    parameter B         = 4,            // subcluster branching factor
    parameter B_SUB     = 4,            // sub-subcluster branching factor
    parameter K         = 4,            // K-input LUTs
    parameter LB_IB     = K,            // no. of LB input buffers
    parameter CFG_W     = 5,            // config I/O width
    parameter IO_I_W    = 32,           // parallel IO input  width
    parameter IO_O_W    = 64,           // parallel IO output width
    parameter MACRO_N   = 0             // subcluster macro size (if non-zero)
) (
`ifdef USE_POWER_PINS
    inout               vccd1,          // User area 1 1.8V supply
    inout               vssd1,          // User area 1 digital ground
`endif
    input               clk,            // clock
    input               rst,            // sync reset -- > M+log4(N)+1 cycles please
    output              done,           // S3GA configuration "done"
    output              tock,           // cycle % M == M-1
    input  `V(CFG_W)    cfg_i,          // config chain input -- must negate rst first
    input  `V(IO_I_W)   io_i,           // parallel IO inputs
    output `V(IO_O_W)   io_o            // parallel IO outputs
);
    localparam LEVEL    = $clog2(N/M)/$clog2(B_SUB);
    localparam UP_I_WS  = 08_08_16_32 / 100**(4-LEVEL) * 100; // up switch input widths
    localparam UP_O_WS  = 04_04_08_16 / 100**(4-LEVEL) * 100; // up switch output widths

    reg  `CNT(3)    cfg_st;             // state: 0: await cfgd; 1: await tock; 2: done
    reg  `CNT(M)    m;                  // local cycle % M
    reg             rst_;               // local reset
    reg             grst;               // local global reset
    reg             tock_;              // local tock
    wire            cfgd;               // S3GA is completely configured
    wire `V(CFG_W)  cfg_i_;             // delay-matched cfg_i

    always @(posedge clk) begin
        m     <= rst ? 0 : m + 1'b1;
        tock_ <= rst ? 0 : (m == M-2);

        // align reset to next M-cycle
        if (rst)
            rst_ <= 1;
        else if (tock_)
            rst_ <= rst;

        if (rst) begin
            cfg_st <= 2'd0;             // -> await cfgd
            grst <= 1;
        end
        else if (cfg_st == 2'd0 && cfgd)
            cfg_st <= 2'd1;             // -> await tock
        else if (cfg_st == 2'd1 && tock_) begin
            cfg_st <= 2'd2;             // -> done
            grst <= 0;
        end
    end

    // delay match inputs' LEVEL pipeline stages from here to all configurable elements
    // so that first-tick synchronized cfg_i signals arrive there synchronized;
    // similarly delay S3GA outputs to sync tock with io_i latching and and io_o switching
    pipe #(.W(CFG_W), .DELAY(M-LEVEL)) iq(.clk, .i(cfg_i), .o(cfg_i_));
    pipe #(.W(2),     .DELAY(LEVEL))   oq(.clk, .i({!grst,tock_}), .o({done,tock}));

    cluster #(.N(N), .M(M), .B(B), .B_SUB(B_SUB), .K(K), .LB_IB(LB_IB), .CFG_W(CFG_W),
              .IO_I_W(IO_I_W), .IO_O_W(IO_O_W),
              .UP_I_WS(UP_I_WS), .UP_O_WS(UP_O_WS), .UP_O_DELAY(0),
              .ID(0), .MACRO_N(MACRO_N))
        c(
`ifdef USE_POWER_PINS
          .vccd1, .vssd1,
`endif
          .clk, .rst(rst_), .grst, .m, .cfg(1'b1), .cfgd, .cfg_i(cfg_i_),
          .io_i, .io_o, .up_i(1'b0), .up_o());
endmodule


// A cluster is an IOB, or a cluster of LBs, or a switch and B sub-clusters

module cluster #(
    parameter N         = 256,          // N logical LUTs
    parameter M         = 4,            // M contexts
    parameter B         = 4,            // subcluster branching factor
    parameter B_SUB     = 4,            // sub-subcluster branching factor
    parameter K         = 4,            // K-input LUTs
    parameter LB_IB     = K,            // no. of LB input buffers
    parameter CFG_W     = 5,            // config I/O width
    parameter IO_I_W    = 0,            // parallel IO input  width
    parameter IO_O_W    = 0,            // parallel IO output width
    parameter UP_I_WS   = 08_08_16,     // up switch serial input  widths
    parameter UP_O_WS   = 04_04_08,     // up switch serial output widths
    parameter UP_O_DELAY = 0,           // default up_o delay
    parameter ID        = 0,            // cluster identifier ::= ID of its first LB
    parameter MACRO_N   = 0,            // subcluster macro size (if non-zero)

    parameter UP_I_W    = UP_I_WS%100,  // up switch serial input  width
    parameter UP_O_W    = UP_O_WS%100,  // up switch serial output width
    parameter DN_I_W    = UP_O_WS/100%100, // down switches' serial input  width
    parameter DN_O_W    = UP_I_WS/100%100  // down switches' serial output width
) (
`ifdef USE_POWER_PINS
    inout               vccd1,          // User area 1 1.8V supply
    inout               vssd1,          // User area 1 digital ground
`endif
    input               clk,            // clock
    input               rst,            // sync reset
    input               grst,           // S3GA configuration in progress
    input  `CNT(M)      m,              // cycle % M
    input               cfg,            // config enable
    output              cfgd,           // cluster is configured
    input  `V(CFG_W)    cfg_i,          // config input
    input  `V(IO_I_W)   io_i,           // parallel IO inputs
    output `V(IO_O_W)   io_o,           // parallel IO outputs
    input  `V(UP_I_W)   up_i,           // up switch serial inputs
    output `V(UP_O_W)   up_o            // up switch serial outputs
);
    localparam LEVEL    = $clog2(N/M)/$clog2(B);

    wire `V(UP_O_W) up_o_;              // up switch serial output (pre-delay)

    wire `V(B+1)    cfgs;               // local config enables
    assign cfgs[0] = cfg;

    // globally synchronized (delay matched) control signals tree, so that every
    // configurable module and cfg_ram receives {rst,grst,m,cfg_i} on the same tick
    wire            rst_q;
    wire            grst_q;
    wire `CNT(M)    m_q;
    wire `V(CFG_W)  cfg_i_q;
    pipe #(.W(2+$clog2(M)+CFG_W), .DELAY(1))
        q(.clk, .i({rst,grst,m,cfg_i}), .o({rst_q,grst_q,m_q,cfg_i_q}));

    genvar i, j;
    generate
    if (IO_I_W == 0 && IO_O_W == 0 && N == MACRO_N && N == 64) begin : x
        x64 c(
`ifdef USE_POWER_PINS
              .vccd1, .vssd1,
`endif
              .clk, .rst, .grst, .m, .cfg, .cfgd, .cfg_i, .io_i, .io_o, .up_i, .up_o);
    end
    else if (M*UP_I_W == IO_O_W) begin : io
        iob #(.M(M), .CFG_W(CFG_W), .IO_I_W(IO_I_W), .IO_O_W(IO_O_W), .I_W(UP_I_W), .O_W(UP_O_W))
            iob_(.clk, .rst(rst_q), .grst(grst_q), .m(m_q), .cfg, .cfgd, .cfg_i(cfg_i_q),
                 .io_i, .io_o, .i(up_i), .o(up_o_));
    end
    else if (N == B*M) begin : leaf
        // s3ga<32> => { lb<8> lb<8> lb<8> lb<8> } directly, sans switch<32>
        wire `V(B)      halfs;          // half-LUT cascade chains, 0->1->2->3->0
        for (i = 0; i < B; i=i+1) begin : lbs
            lb #(.M(M), .B(B), .K(K), .G(UP_I_W), .I(LB_IB), .CFG_W(CFG_W), .ID(ID+i))
                lb_(.clk, .rst(rst_q), .grst(grst_q), .m(m_q),
                    .cfg(cfgs[i]), .cfgd(cfgs[i+1]), .cfg_i(cfg_i_q),
                    .globs(up_i), .locals(up_o_),
                    .half_i(halfs[(i+B-1)%B]), .half_o(halfs[i]), .o(up_o_[i]));
        end
        assign cfgd = cfgs[B];
        assign io_o = 0;
    end
    else begin : subs
        // recurse to B subclusters sized N/B
        wire            rst_qq;         // further pipelined across hierarchy
        wire            grst_qq;
        wire `CNT(M)    m_qq;
        wire `V(CFG_W)  cfg_i_qq;
        pipe #(.W(2+$clog2(M)+CFG_W), .DELAY(LEVEL-1)) // LEVEL-1 cycles further delayed
            qq(.clk, .i({rst_q,grst_q,m_q,cfg_i_q}), .o({rst_qq,grst_qq,m_qq,cfg_i_qq}));

        wire `NV(B,DN_I_W) dn_is;       // down switches' serial inputs
        wire `NV(B,DN_O_W) dn_os;       // down switches' serial outputs

        switch #(.M(M), .B(B), .DELAY(1), .UP_I_W(UP_I_W), .UP_O_W(UP_O_W),
                 .DN_I_W(DN_I_W), .DN_O_W(DN_O_W), .CFG_W(CFG_W))
            sw(.clk, .rst(rst_qq), .m(m_qq), .cfg(cfgs[B]), .cfgd, .cfg_i(cfg_i_qq),
               .up_i, .up_o(up_o_), .dn_is, .dn_os);

        // first subcluster may have IO
        cluster #(.N(N/B), .M(M), .B(B_SUB), .B_SUB(B_SUB), .K(K), .LB_IB(LB_IB), .CFG_W(CFG_W),
                  .IO_I_W(IO_I_W), .IO_O_W(IO_O_W), .UP_I_WS(UP_I_WS/100), .UP_O_WS(UP_O_WS/100),
                  .UP_O_DELAY(0), .ID(ID), .MACRO_N(MACRO_N))
            c0(
`ifdef USE_POWER_PINS
              .vccd1, .vssd1,
`endif
              .clk, .rst(rst_q), .grst(grst_q), .m(m_q),
              .cfg(cfgs[0]), .cfgd(cfgs[1]), .cfg_i(cfg_i_q), .io_i, .io_o,
              .up_i(dn_os`at(0,DN_O_W)), .up_o(dn_is`at(0,DN_I_W)));

        // other subclusters don't
        for (i = 1; i < B; i=i+1) begin : cs
            cluster #(.N(N/B), .M(M), .B(B_SUB), .B_SUB(B_SUB), .K(K), .LB_IB(LB_IB), .CFG_W(CFG_W),
                      .IO_I_W(0), .IO_O_W(0), .UP_I_WS(UP_I_WS/100), .UP_O_WS(UP_O_WS/100),
                      .UP_O_DELAY(0), .ID(ID+i*N/B), .MACRO_N(MACRO_N))
                c(
`ifdef USE_POWER_PINS
                  .vccd1, .vssd1,
`endif
                  .clk, .rst(rst_q), .grst(grst_q), .m(m_q),
                  .cfg(cfgs[i]), .cfgd(cfgs[i+1]), .cfg_i(cfg_i_q), .io_i(1'b0), .io_o(),
                  .up_i(dn_os`at(i,DN_O_W)), .up_o(dn_is`at(i,DN_I_W)));
        end
    end
    endgenerate
    pipe #(.W(UP_O_W), .DELAY(UP_O_DELAY)) up_o_pipe(.clk, .i(up_o_), .o(up_o));
endmodule


// Configurable M-context serial interconnect switch
//
//  Each cycle, for each of M contexts,
//      for each of B down switches,
//          for each of DN_O_W serial net outputs,
//              select one of the UP_I_W inputs or the other B's DN_I_W inputs;
//      and for each of UP_O_W serial net outputs (if any),
//          select one of the B's DN_I_W's inputs.

module switch #(
    parameter M         = 4,            // M contexts
    parameter B         = 4,            // no. of down switches (subcluster branch factor)
    parameter DELAY     = 1,            // no. of output pipeline stages 
    parameter UP_I_W    = 12,           // up switch (*) input  width  (*: 0 if none)
    parameter UP_O_W    = 8,            // up switch (*) output width  ""
    parameter DN_I_W    = 4,            // down switches input  width
    parameter DN_O_W    = 6,            // down switches output width
    parameter CFG_W     = 5             // config I/O width
) (
    input               clk,
    input               rst,
    input  `CNT(M)      m,
    input               cfg,
    output              cfgd,
    input  `V(CFG_W)    cfg_i,
    input  `V(UP_I_W)   up_i,
    output `V(UP_O_W)   up_o,
    input  `NV(B,DN_I_W) dn_is,
    output `NV(B,DN_O_W) dn_os
);
    localparam DN_X_W   = UP_I_W + (B-1)*DN_I_W;
    wire `V(DN_X_W) is[0:B-1];
    wire `V(B+1)    cfgs;
    assign cfgs[0] = cfg;

    genvar i, j;
    generate
        // down outputs' crossbars
        for (i = 0; i < B; i=i+1) begin : dns
            // ith xbar input is concat of dn_is[j] and up_i for j!=i
            for (j = 0; j < B-1; j=j+1) begin : is_
                assign is[i][j*DN_I_W +: DN_I_W] = dn_is`at(j+(j>=i),DN_I_W);
            end
            if (UP_I_W > 0)
                assign is[i][(B-1)*DN_I_W +: UP_I_W] = up_i;

            xbar #(.M(M), .DELAY(DELAY), .I_W(DN_X_W), .O_W(DN_O_W), .CFG_W(CFG_W))
                x(.clk, .rst, .m, .cfg(cfgs[i]), .cfgd(cfgs[i+1]), .cfg_i,
                  .i(is[i]), .o(dn_os`at(i,DN_O_W)));
        end

        // optional up outputs' crossbar
        if (UP_O_W > 0) begin : up
            xbar #(.M(M), .DELAY(DELAY), .I_W(B*DN_I_W), .O_W(UP_O_W), .CFG_W(CFG_W))
                x(.clk, .rst, .m, .cfg(cfgs[B]), .cfgd, .cfg_i, .i(dn_is), .o(up_o));
        end
        else begin
            assign cfgd = cfgs[B];
        end
    endgenerate
endmodule


// Configurable M-context crossbar

module xbar #(
    parameter M         = 4,            // M contexts
    parameter DELAY     = 1,            // no. of output pipeline stages
    parameter I_W       = 4,            // input  width
    parameter O_W       = 4,            // output width 
    parameter CFG_W     = 5             // config I/O width
) (
    input               clk,
    input               rst,
    input  `CNT(M)      m,
    input               cfg,
    output              cfgd,
    input  `V(CFG_W)    cfg_i,
    input  `V(I_W)      i,
    output `V(O_W)      o
);
    localparam SEL_W    = $clog2(I_W);
    wire `NV(O_W,SEL_W) sels;
    `comb`V(O_W)        o_;

    cfg_ram #(.M(M), .W(O_W*SEL_W), .CFG_W(CFG_W))
        selects(.clk, .rst, .m, .cfg, .cfgd, .cfg_i, .o(sels));

    integer j;
    always @* begin
        for (j = 0; j < O_W; j=j+1)
            o_[j] = i[sels`at(j,SEL_W)];
    end
    pipe #(.W(O_W), .DELAY(DELAY)) o_pipe(.clk, .i(o_), .o);
endmodule


// Configurable M-context logic block

module lb #(
    parameter M         = 4,            // M contexts
    parameter B         = 4,            // no. of peer LBs, including this one
    parameter K         = 4,            // K-input LUTs
    parameter G         = 8,            // no. of global inputs
    parameter I         = K,            // no. of input buffers
    parameter CFG_W     = 5,            // config I/O width
    parameter ID        = 0             // unique ID (in system, or macro)
) (
    input               clk,
    input               rst,
    input               grst,
    input  `CNT(M)      m,
    input               cfg,
    output              cfgd,
    input  `V(CFG_W)    cfg_i,
    input  `V(G)        globs,          // global serial inputs
    input  `V(B)        locals,         // local serial inputs (LBs in this cluster)
    input               half_i,         // half-LUT cascade in
    output `comb        half_o,         // half-LUT cascade out
    output `comb        o
);
    // LB IMUXs
    localparam LB_IN_W  = G-1 + 1;      // each LB input selects G-1 of G globals and 1 local
    localparam LB_SEL_W = $clog2(LB_IN_W);
    `comb`V(LB_IN_W)    lb_ins;
    `comb`V(I)          imuxs;

    // input and output buffers
    reg  `NV(I,M)       ibufs;
    reg                 half_q;         // prev tick's half_lut output

    // flip-flops
    reg  `V(M)          qs;             // last M LUT outputs a.k.a. output flip-flops
    reg                 ff_rst;         // flip-flops' reset
    reg                 ff_ce;          // flip-flops' clock enable

    // LUT inputs, IMUXs, and outputs
    localparam LUT_IN_W = I*M;
    localparam LUT_SEL_W= $clog2(LUT_IN_W);
    `comb`V(LUT_IN_W)   ins;
    `comb`V(K)          idx;
    `comb               lut;

    // LUT configuration frames
    localparam LUT_W    = I*LB_SEL_W + K*LUT_SEL_W + (1<<K) + 3/*{fde,fds,fd}*/;
    wire `NV(I,LB_SEL_W) lb_in_sels;    // logic block input selects
    wire `NV(K,LUT_SEL_W) lut_in_sels;  // LUT input selects
    wire `V(1<<K)       mask;           // LUT mask
    wire                fd;             // D flip-flop
    wire                fds;            // D-FF reset value
    wire                fde;            // D-FF clock enable
    cfg_ram #(.M(M), .W(LUT_W), .CFG_W(CFG_W))
        luts(.clk, .rst, .m, .cfg, .cfgd, .cfg_i, .o({fde,fds,fd,lb_in_sels,lut_in_sels,mask}));

    // LB input multiplexers
    integer i, j;
    always @* begin
        for (i = 0; i < I; i=i+1) begin
            // ith LB input: any of G-1 of the global inputs, or 1 of the local inputs
            lb_ins[0] = locals[i];
            for (j = 0; j < G-1; j=j+1)
                lb_ins[j+1] = globs[j + (j>=i)];
            imuxs[i] = lb_ins[lb_in_sels`at(i,LB_SEL_W)];
        end
    end

    // LB input buffers, output buffers, flip-flop nets
    always @(posedge clk) begin
        for (i = 0; i < I; i=i+1)
            ibufs`at(i,M) <= {ibufs`at(i,M),imuxs[i]};
        qs <= {qs,o};                   // output flip-flops
        half_q <= (m == M-1) ? half_i : half_o;   // half-LUT cascade

        // Obtain next M-cycle's flip-flops' RST and CE from the last global nets
        // of the last cycle of this M-cycle. REVIEW: make this configurable?
        if (m == M-1) begin
            ff_rst <= grst | globs[G-1];
            ff_ce  <= grst | globs[G-2];
        end
    end

    // lookup table and "FDRE/FDSE flip-flop"
    always @* begin
        // LUT inputs
        for (i = 0; i < K; i=i+1) begin
            ins = ibufs;

            // half-LUT cascade special inputs
            if (i == K-2)
                ins[LUT_IN_W-1] = half_q;
            else if (i == K-1)
                ins[LUT_IN_W-1] = 1'b1;

            idx[i] = ins[lut_in_sels`at(i,LUT_SEL_W)];
        end
        // LUT / half-LUT outputs
        // REVIEW: SPEED
        lut = ~grst & mask[idx];
        half_o = ~grst & mask[idx[K-2:0]];

        // optional output flip-flop:
        if (!fd)                        // combinational?
            o = lut;
        else if (ff_rst)                // reset?
            o = fds;                    //  Q <= set/reset value
        else if (!fde || ff_ce)         // flop enabled?
            o = lut;                    //  Q <= D (so to speak)
        else                            // flop not enabled
            o = qs[M-1];                //  Q unchanged
    end
endmodule


// Configurable M-context IO block
//
// Crossbar parallel inputs into serial outputs;
// crossbar serial inputs into parallel outputs.

module iob #(
    parameter M         = 4,            // M contexts
    parameter CFG_W     = 5,            // config I/O width
    parameter IO_I_W    = 16,           // parallel IO input  width
    parameter IO_O_W    = 16,           // parallel IO output width
    parameter I_W       = 7,            // serial input  width
    parameter O_W       = 4             // serial output width
) (
    input               clk,
    input               rst,
    input               grst,
    input  `CNT(M)      m,
    input               cfg,
    output              cfgd,
    input  `V(CFG_W)    cfg_i,
    input  `V(IO_I_W)   io_i,           // per M-cycle
    output reg `V(IO_O_W) io_o,         // per M-cycle
    input  `V(I_W)      i,
    output `V(O_W)      o
);
    wire                cfg_;
    reg  `V(IO_I_W)     io_i_q;
    reg  `V(IO_O_W)     io_o_;          // prior pending output nets
    `comb`V(IO_O_W)     io_o_nxt;       // current pending output nets

    // crossbar parallel inputs into serial outputs
    xbar #(.M(M), .DELAY(0), .I_W(IO_I_W), .O_W(O_W), .CFG_W(CFG_W))
        x(.clk, .rst, .m, .cfg, .cfgd(cfg_), .cfg_i, .i(io_i_q), .o(o));

    // crossbar serial inputs into parallel outputs
    // output configuration frame (1 context only)
    localparam SEL_W    = $clog2(I_W);
    localparam TICK_W   = $clog2(M);
    wire `NV(IO_O_W,SEL_W)  sels;       // output selects
    wire `NV(IO_O_W,TICK_W) ticks;      // output ticks
    cfg_ram #(.M(1), .W(IO_O_W*(SEL_W+TICK_W)), .CFG_W(CFG_W))
        sels_(.clk, .rst, .m(1'b0), .cfg(cfg_), .cfgd, .cfg_i, .o({ticks,sels}));

    // output muxes and flops: for each output bit, register some input net,
    // on some tick, accumulating them in io_o_ (cumulative prior) and
    // io_o_nxt (current)
    integer j;
    always @* begin
        io_o_nxt = io_o_;
        for (j = 0; j < IO_O_W; j=j+1)
            if (ticks`at(j,TICK_W) == m)
                io_o_nxt[j] = i[sels`at(j,SEL_W)];
    end

    // register IO inputs and outputs as next M-cycle starts
    always @(posedge clk) begin
        if (grst || m == M-1)
            io_o_ <= 0;
        else
            io_o_ <= io_o_nxt;

        if (grst)
            io_i_q <= 0;
        else if (m == M-1)
            io_i_q <= io_i;

        if (grst)
            io_o <= 0;
        else if (m == M-1)
            io_o <= io_o_nxt;
    end
endmodule


// Configuration ram with M W-bit contexts
//
// Config FSM: reset/0 -> configure * -> configured
// Config data is received in frames each of M CFG_W-1-bit segments;
// M lsbs' segments then M next-lsbs' segments, etc.
// The msb of each cfg_i word is 'valid' bit for that segment.
// Between frames there can be idle periods (cfg_i = 0)
// When USE_SR=1, the segments within a frame must be received on successive
// clock cycles, and the first segment of a frame must arrive when m==0.
// Example config frames' layout (assume M=8, W=4, CFG_W=3):
//  000*, 1,[0][1:0], 1,[1][1:0], ..., 1,[7][1:0], 000*, 1,[0][3:2], 1,[1][3:2], ..., 1,[7][3:2]

module cfg_ram #(
    parameter M         = 4,            // M switch contexts
    parameter W         = 11,           // output width
    parameter CFG_W     = 5,            // config data I/O width: { valid:1; segment:4; }
    parameter USE_SR    = 1             // use shift register RAM
) (
    input               clk,
    input               rst,
    input  `CNT(M)      m,              // unused when USE_SR=1
    input               cfg,            // 0 => await config; 1 => configure (and run)
    output reg          cfgd,           // ram is configured
    input  `V(CFG_W)    cfg_i,          // config data
    output `V(W)        o
);
    localparam SEG_W    = CFG_W-1;      // config data segment width
    localparam CFG_SEGS = M * `SEGS(W,SEG_W); // no. of segments for entire cfg_ram
    reg  `CNT(CFG_SEGS) seg;            // config segment counter, in [0,CFG_SEGS).

    `comb   cfg_v;
    always @* cfg_v = cfg && !cfgd && cfg_i[SEG_W]; // configuring, valid segment

    always @(posedge clk) begin
        if (rst) begin                  // reset: start config, pass no config data
            cfgd <= 0;
            seg <= 0;
        end
        else if (cfg_v) begin
            if (seg != CFG_SEGS-1)      // terminal count?
                seg <= seg + 1'b1;      // next segment/frame
            else
                cfgd <= 1;              // config complete
        end
    end

    reg `V(W) ram[0:M-1];
    integer i;

    generate if (USE_SR) begin : sr     // shift register RAM
        assign o = ram[M-1];

        // REVIEW: replace this SR input mux with a vector of sdfxtp_1 enabled
        // by a shift register of per-frame write enables.
        `comb`V(W) ram_in;
        always @* begin
            // recirculate the last shift register tap back to the front;
            // during reset or config, merge in new config segments
            ram_in = ram[M-1];
            if (cfg_v)                  // load a segment
                ram_in[seg/M*SEG_W +: SEG_W] = cfg_i[0+:SEG_W]; // will clip when W%CFG_W != 0
            if (rst)
                ram_in = {`SEGS(W,SEG_W){cfg_i[0+:SEG_W]}}; // = 0; -- saves area
        end
        always @(posedge clk) begin
            for (i = 0; i < M; i=i+1)
                ram[i] <= (i == 0) ? ram_in : ram[i-1];
        end
    end
    else begin : ra                     // DFF RAM
        // here can load a segment on any tick
        always @(posedge clk) begin
            if (rst) begin
                for (i = 0; i < M; i=i+1)
                    ram[i] <= 0;
            end
            else if (cfg_v)
                ram[seg%M]`at(seg/M,SEG_W) <= cfg_i[0+:SEG_W];
        end

        // pipeline ram data out (when M != 1) else just ram[0]
        pipe #(.W(W), .DELAY(M!=1)) o_pipe(.clk, .i(ram[(m+1)%M]), .o);
    end endgenerate
endmodule


// Pipeline register(s)
// o == i after DELAY clock cycles, DELAY >= 0

module pipe #(
    parameter W         = 4,
    parameter DELAY     = 1
) (
    input               clk,
    input  `V(W)        i,
    output `V(W)        o
);
    generate
    if (DELAY == 0) begin
        assign o = i;
    end
    else begin : q
        reg `V(W) qs[0:DELAY-1];
        integer j;
        always @(posedge clk) begin
            for (j = 0; j < DELAY; j=j+1)
                qs[j] <= (j == 0) ? i : qs[j-1];
        end
        assign o = qs[DELAY-1];
    end
    endgenerate
endmodule
