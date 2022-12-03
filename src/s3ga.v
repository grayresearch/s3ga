// S3GA: simple scalable serial FPGA
// By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC. All rights reserved.

`include "s3ga.h"

// S3GA: simple scalable serial FPGA

module s3ga #(
    parameter N         = 2048,         // N logical LUTs
    parameter M         = 8,            // M contexts
    parameter B         = 4,            // subcluster branching factor
    parameter K         = 4,            // K-input LUTs
    parameter LB_IB     = 3,            // no. of LB input buffers
    parameter CFG_W     = 4,            // config I/O width
    parameter IO_I_W    = 32,           // parallel IO input  width
    parameter IO_O_W    = 32,           // parallel IO output width
    parameter UP_I_WS   = 06_12_24_00,  // up switch serial input  widths
    parameter UP_O_WS   = 04_08_16_00,  // up switch serial output widths

    localparam UP_I_W   = UP_I_WS%100,  // up switch serial input  width
    localparam UP_O_W   = UP_O_WS%100,  // up switch serial output width
    localparam DN_I_W   = UP_O_WS/100%100, // down switches' serial input  width
    localparam DN_O_W   = UP_I_WS/100%100  // down switches' serial output width
) (
    input               clk,            // clock
    input               rst,            // sync reset
    input  `V(CFG_W)    cfg_i,          // config chain input
    output `V(CFG_W)    cfg_o,          // config chain output
    input  `V(IO_I_W)   io_i,           // parallel IO inputs
    output `V(IO_O_W)   io_o,           // parallel IO outputs
    input  `V(UP_I_W)   up_i,           // up switch serial inputs
    output `V(UP_O_W)   up_o            // up switch serial outputs
);
    wire `V(CFG_W)      cfgs[0:B];      // local config chain outputs
    assign cfgs[0] = cfg_i;


    genvar i, j;
    generate
    if (N == B*M) begin
        // s3ga<32> => { lb<8> lb<8> lb<8> lb<8> } directly, sans switch<32>
        for (i = 0; i < B; i=i+1) begin : lbs
            wire `V(B-1) peers;
            for (j = 0; j < B-1; j=j+1)
                assign peers[j] = up_o[i + (j>=i)];
            lb #(.M(M), .B(B), .K(K), .G(UP_I_W), .I(LB_IB), .CFG_W(CFG_W))
                b(.clk, .rst, .cfg_i(cfgs[i]), .cfg_o(cfgs[i+1]),
                  .globals(up_i), .peers, .o(up_o[i]));
        end
        assign cfg_o = cfgs[B];
        // TODO: IO
        assign io_o = up_o;
    end
    else begin
        // recurse to B subclusters sized N/B
        wire `NV(B,DN_I_W)  dn_is;      // down switches' serial inputs
        wire `NV(B,DN_O_W)  dn_os;      // down switches' serial outputs

        switch #(.M(M), .B(B), .UP_I_W(UP_I_W), .UP_O_W(UP_O_W), .DN_I_W(DN_I_W),.DN_O_W(DN_O_W))
            sw(.clk, .rst, .cfg_i(cfgs[B]), .cfg_o(cfg_o), .up_i, .up_o, .dn_is, .dn_os);

        for (i = 0; i < B; i=i+1) begin : cs
            s3ga #(.N(N/B), .M(M), .B(B), .K(K), .LB_IB(LB_IB), .CFG_W(CFG_W),
                   .IO_I_W(IO_I_W), .IO_O_W(IO_O_W), .UP_I_WS(UP_I_WS/100), .UP_O_WS(UP_O_WS/100))
                c(.clk, .rst, .cfg_i(cfgs[i]), .cfg_o(cfgs[i+1]), .io_i('0), .io_o(),
                  .up_i(dn_os`at(i,DN_O_W)), .up_o(dn_is`at(i,DN_I_W)));
        end
        // TODO: IO
        assign io_o = dn_is;
    end
    endgenerate
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
    parameter M         = 8,            // M contexts
    parameter B         = 4,            // no. of down switches (subcluster branch factor)
    parameter UP_I_W    = 12,           // up   switch(*)  input  width  (*: 0 if none)
    parameter UP_O_W    = 8,            // up   switch(*)  output width  ""
    parameter DN_I_W    = 4,            // down switch(es) input  width
    parameter DN_O_W    = 6,            // down switch(es) output width
    parameter CFG_W     = 4             // config I/O width
) (
    input               clk,
    input               rst,
    input  `V(CFG_W)    cfg_i,
    output `V(CFG_W)    cfg_o,
    input  `V(UP_I_W)   up_i,
    output `V(UP_O_W)   up_o,
    input  `NV(B,DN_I_W) dn_is,
    output `NV(B,DN_O_W) dn_os
);
    localparam DN_X_W   = UP_I_W + (B-1)*DN_I_W;
    wire `V(DN_X_W)     is[0:B-1];
    wire `V(CFG_W)      cfgs[0:B];
    assign cfgs[0] = cfg_i;

    genvar i, j;
    generate
        // down outputs' crossbars
        for (i = 0; i < B; i=i+1) begin : dns
            // ith xbar input is concat of up_i and dn_is[j] for j!=i
            if (UP_I_W > 0)
                assign is[i][0 +: UP_I_W] = up_i;
            for (j = 0; j < B-1; j=j+1)
                assign is[i][UP_I_W + j*DN_I_W +: DN_I_W] = dn_is`at(j+(j>=i),DN_I_W);

            xbar #(.M(M), .I_W(DN_X_W), .O_W(DN_O_W))
                x(.clk, .rst, .cfg_i(cfgs[i]), .cfg_o(cfgs[i+1]), .i(is[i]), .o(dn_os`at(i,DN_O_W)));
        end

        // optional up outputs' crossbar
        if (UP_O_W > 0) begin : up
            xbar #(.M(M), .I_W(B*DN_I_W), .O_W(UP_O_W))
                x(.clk, .rst, .cfg_i(cfgs[B]), .cfg_o, .i(dn_is), .o(up_o));
        end
        else begin
            assign cfg_o = cfgs[B];
        end
    endgenerate
endmodule


// Configurable M-context crossbar

module xbar #(
    parameter M         = 8,            // M contexts
    parameter B         = 4,            // no. of down switches (subcluster branch factor)
    parameter I_W       = 4,            // input  width
    parameter O_W       = 4,            // output width 
    parameter CFG_W     = 4             // config I/O width
) (
    input               clk,
    input               rst,
    input  `V(CFG_W)    cfg_i,
    output `V(CFG_W)    cfg_o,
    input  `V(I_W)      i,
    output `comb`V(O_W) o
);
    localparam SEL_W    = $clog2(I_W);
    wire `NV(O_W,SEL_W) sels;

    cfg_ram #(.M(M), .W(O_W*SEL_W), .CFG_W(CFG_W))
        selects(.clk, .rst, .cfg_i, .cfg_o, .o(sels));

    integer j;
    always @* begin
        for (j = 0; j < O_W; j=j+1)
            o[j] = i[sels`at(j,SEL_W)];
    end
endmodule


// Configurable M-context logic block

module lb #(
    parameter M         = 8,            // M contexts
    parameter B         = 4,            // no. of peer LBs, including this one
    parameter K         = 4,            // K-input LUTs
    parameter G         = 6,            // no. of global inputs
    parameter I         = 3,            // no. of input buffers
    parameter CFG_W     = 4             // config I/O width
) (
    input               clk,
    input               rst,
    input  `V(CFG_W)    cfg_i,
    output `V(CFG_W)    cfg_o,
    input  `V(G)        globals,
    input  `V(B-1)      peers,
    output `comb        o
);
    // LB IMUXs
    localparam LB_IN_W  = G+B-2;
    localparam LB_SEL_W = $clog2(LB_IN_W);
    `comb`NV(I,LB_IN_W) lb_inss;
    `comb`V(LB_IN_W)    lb_ins;
    `comb`V(I)          imuxs;

    // input and output buffers
    reg  `NV(I,M)       ibufs;
    reg  `V(M)          obuf;

    // LUT inputs, IMUXs, and output
    localparam LUT_IN_W = I*M + M;
    localparam LUT_SEL_W= $clog2(LUT_IN_W);
    `comb`V(LUT_IN_W)   ins;
    `comb`V(K)          idx;
    `comb               lut;

    // LUT configuration frames
    localparam LUT_W    = I*LB_SEL_W + K*LUT_SEL_W + (1<<K) + 3/*{fde,fds,fd}*/;
    wire `NV(I,LB_SEL_W) lb_in_sels;    // logic block input selects
    wire `NV(K,LUT_SEL_W) lut_in_sels;  // LUT input selects
    wire `V(1<<K)       mask;           // LUT mask
    wire                fd;             // D flip-flop?
    wire                fds;            // D-FF reset value
    wire                fde;            // D-FF clock enable?
    cfg_ram #(.M(M), .W(LUT_W), .CFG_W(CFG_W))
        luts(.clk, .rst, .cfg_i, .cfg_o, .o({lb_in_sels,lut_in_sels,mask,fde,fds,fd}));

    // LB input multiplexers
    integer i, j;
    always @* begin
        for (i = 0; i < I; i=i+1) begin
            // ith LB input is one of G globals or B-2 of the B-1 peer LB outputs
            lb_inss`at(i,LB_IN_W) = globals << (B-2);
            for (j = 0; j < B-2; j=j+1)
                lb_inss[i*LB_IN_W + j] = peers[j + (j>=i)];
            lb_ins = lb_inss`at(i,LB_IN_W);
            imuxs[i] = lb_ins[lb_in_sels`at(i,LB_SEL_W)];
        end
    end
    // LB input buffers and output buffer
    always @(posedge clk) begin
        for (i = 0; i < I; i=i+1)
            ibufs[i] <= {ibufs[i],imuxs[i]};
        obuf <= {obuf,lut};
    end
    always @* begin
        // LUT inputs
        ins = {ibufs,obuf};
        for (i = 0; i < K; i=i+1)
            idx[i] = ins[lut_in_sels[i]];
        // LUT output
        lut = mask[idx];

        o = lut;

/*      TODO
        // optional output register
        if (!fd)
            o = lut;
        else if (rst)
            o = fds;
        else
            o = (clk && (!fde || ce)) ? lut : obuf[N-1];
*/
    end
endmodule


// Configuration ram with M W-bit contexts using serial shift register memory
//
// Config FSM: reset/0 -> wait-for-1 * -> configure * -> passthru/cfg_i.
// Config data is received in CFG_W segments: M lsbs' segments then M next-lsbs' segments, etc.
// Example config frame layout (assume M=8, W=4, CFG_W=2):
//  0* (ignored), 1, [0][1:0], [1][1:0], ..., [7][1:0], [0][3:2], [1][3:2], ..., [7][3:2].

module cfg_ram #(
    parameter M         = 8,            // M switch contexts
    parameter W         = 8,            // output width
    parameter CFG_W     = 4             // config I/O width
) (
    input               clk,
    input               rst,
    input  `V(CFG_W)    cfg_i,
    output reg`V(CFG_W) cfg_o,
    output `V(W)        o
);
    // configuration state machine
    localparam ST_WAIT  = 0;            // wait for 1 signaling start of config frame
    localparam ST_CFG   = 1;            // receive config data and configure ram
    localparam ST_PASS  = 2;            // pass more config data to next cfg_ram
    // assert(W%CFG_W == 0);
    localparam CFG_SEGS = M * `SEGS(W,CFG_W); // no. of config frame segments

    reg  `CNT(ST_PASS)  st;             // config FSM state
    reg  `CNT(CFG_SEGS) seg;            // config segment counter, in [0,CFG_SEGS).

    always @(posedge clk) begin
        if (rst) begin                  // reset: pass no config data
            st <= ST_WAIT;
            seg <= '0;
            cfg_o <= '0;
        end
        else if (st == ST_WAIT) begin   // await start of config frame
            if (cfg_i == 1)
                st <= ST_CFG;
        end
        else if (st == ST_CFG) begin    // receive config data segments
            if (seg != CFG_SEGS-1)
                seg <= seg + 1'b1;
            else
                st <= ST_PASS;
        end
        else begin                      // pass subsequent config data
            cfg_o <= cfg_i;
        end
    end

    // serial shift register ram
    `comb`V(W)          ram_in;
    reg  `NV(M,W)       ram;

    assign o = ram`at(M-1,W);

    always @* begin
        // recirculate the last shift register tap back to the front;
        // during config, merge in new config segments
        ram_in = ram`at(M-1,W);
        if (st == ST_CFG)
            ram_in`at(seg/M,CFG_W) = cfg_i;
    end
    always @(posedge clk)
        ram <= {ram,ram_in};
endmodule


// Pipeline register(s)
//
// o == i after DELAY clock cycles, DELAY >= 0

module pipe #(
    parameter W         = 1,
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
    else begin
        reg  `NV(DELAY,W) qs;
        integer j;
        always @(posedge clk) begin
            for (j = 0; j < DELAY; j=j+1)
                qs[j] <= (j == 0) ? i : qs[j-1];
        end
        assign o = qs[DELAY-1];
    end
    endgenerate
endmodule