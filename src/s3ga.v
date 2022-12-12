// S3GA: simple scalable serial FPGA
// By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC. All rights reserved.

`include "s3ga.h"

// Common clocking signals seen by every submodule
//  clk
//  rst     // reset; transitions only at start of an M-cycle
//  cfg     // config pending;  ""
//  m       // cycle % M
//  tock    // cycle % M == M-1
// Across the huge S3GA hierarchy of cores, care is taken to ensure
// every leaf configurable module (switch, lb, iob) sees identical
// (rst, cfg, m) -- and so does the external SoC.

// S3GA: simple scalable serial FPGA

module s3ga #(
    parameter N         = 128,         // N logical LUTs
    parameter M         = 8,            // M contexts
    parameter B         = 4,            // subcluster branching factor
    parameter K         = 4,            // K-input LUTs
    parameter LB_IB     = 3,            // no. of LB input buffers
    parameter CFG_W     = 4,            // config I/O width
    parameter IO_I_W    = 32,           // parallel IO input  width
    parameter IO_O_W    = 32            // parallel IO output width
) (
    input               clk,            // clock
    input               rst,            // sync reset -- > M+log4(N)+1 cycles please
    output              tock,           // cycle % M == M-1
    output              cfg,            // configuration in progress
    input  `V(CFG_W)    cfg_i,          // config chain input -- must negate rst first
    input  `V(IO_I_W)   io_i,           // parallel IO inputs
    output `V(IO_O_W)   io_o            // parallel IO outputs
);
    localparam LEVEL    = $clog2(N/M)/$clog2(B);
    localparam UP_I_WS  = 06_06_12_24 / 100**(4-LEVEL) * 100; // up switch input widths
    localparam UP_O_WS  = 04_04_08_16 / 100**(4-LEVEL) * 100; // up switch output widths

    reg  `CNT(3)        cfg_st;         // state: 0: (need cfg_o=1); 1: (need tock); 2: config'd
    reg  `CNT(M)        m;              // local cycle % M
    reg                 rst_;           // local reset
    reg                 tock_;          // local tock
    reg                 cfg_;           // local cfg
    wire `V(CFG_W)      cfg_o;

    always @(posedge clk) begin
        m     <= rst ? '0 : m + 1'b1;
        tock_ <= rst ? '0 : (m == M-2);

        if (rst)
            rst_ <= 1;
        else if (tock_)
            rst_ <= rst;

        if (rst) begin
            cfg_st <= 2'd0;             // -> await cfg_o
            cfg_ <= 1;
        end
        else if (cfg_st == 2'd0 && cfg_o) begin
            cfg_st <= 2'd1;             // -> await tock
        end
        else if (cfg_st == 2'd1 && tock_) begin
            cfg_st <= 2'd2;             // -> done
            cfg_ <= 0;
        end
    end

    pipe #(.W(2), .DELAY(LEVEL)) q(.clk, .i({tock_,cfg_}), .o({tock,cfg}));

    cluster #(.N(N), .M(M), .B(B), .K(K), .LB_IB(LB_IB), .CFG_W(CFG_W),
              .IO_I_W(IO_I_W), .IO_O_W(IO_O_W), .UP_I_WS(UP_I_WS), .UP_O_WS(UP_O_WS), .ID(0))
        c(.clk, .rst(rst_), .m, .cfg(cfg_), .cfg_i, .cfg_o, .io_i, .io_o, .up_i('0), .up_o());
endmodule


// A cluster is an IOB, or a cluster of LBs, or a switch and B sub-clusters

module cluster #(
    parameter N         = 128,          // N logical LUTs
    parameter M         = 8,            // M contexts
    parameter B         = 4,            // subcluster branching factor
    parameter K         = 4,            // K-input LUTs
    parameter LB_IB     = 3,            // no. of LB input buffers
    parameter CFG_W     = 4,            // config I/O width
    parameter IO_I_W    = 32,           // parallel IO input  width
    parameter IO_O_W    = 32,           // parallel IO output width
    parameter UP_I_WS   = 06_06_00,     // up switch serial input  widths
    parameter UP_O_WS   = 04_04_00,     // up switch serial output widths
    parameter ID        = 0,            // cluster identifier ::= ID of its first LB

    localparam UP_I_W   = UP_I_WS%100,  // up switch serial input  width
    localparam UP_O_W   = UP_O_WS%100,  // up switch serial output width
    localparam DN_I_W   = UP_O_WS/100%100, // down switches' serial input  width
    localparam DN_O_W   = UP_I_WS/100%100  // down switches' serial output width
) (
    input               clk,            // clock
    input               rst,            // sync reset
    input  `CNT(M)      m,              // cycle % M
    input               cfg,            // configuration in progress
    input  `V(CFG_W)    cfg_i,          // config chain input
    output `V(CFG_W)    cfg_o,          // config chain output
    input  `V(IO_I_W)   io_i,           // parallel IO inputs
    output `V(IO_O_W)   io_o,           // parallel IO outputs
    input  `V(UP_I_W)   up_i,           // up switch serial inputs
    output `V(UP_O_W)   up_o            // up switch serial outputs
);
    localparam LEVEL    = $clog2(N/M)/$clog2(B);

    wire `V(CFG_W)      cfgs[0:B];      // local config chain outputs
    assign cfgs[0] = cfg_i;

    // globally synchronized control signals tree
    wire                rst_q;          // pipelined hierarchically across hierarchy
    wire                cfg_q;
    wire `CNT(M)        m_q;
    wire                rst_qq;         // delay balanced control sigs, upper level switches
    wire                cfg_qq;
    wire `CNT(M)        m_qq;

    // one cycle delayed
    pipe #(.W($clog2(M)+2), .DELAY(1))
        q(.clk, .i({m,cfg,rst}), .o({m_q,cfg_q,rst_q}));
    // LEVEL-1 cycles further delayed
    pipe #(.W($clog2(M)+2), .DELAY(LEVEL-1))
        qq(.clk, .i({m_q,cfg_q,rst_q}), .o({m_qq,cfg_qq,rst_qq}));

    genvar i, j;
    generate
    if (N == B*M && ID == 0) begin : io
        // first leaf cluster is the IO block
        iob #(.M(M), .CFG_W(CFG_W), .IO_I_W(IO_I_W), .IO_O_W(IO_O_W), .I_W(UP_I_W), .O_W(UP_O_W))
            iob_(.clk, .rst(rst_q), .m(m_q), .cfg(cfg_q), .cfg_i, .cfg_o, .io_i, .io_o, .i(up_i), .o(up_o));
    end
    else if (N == B*M) begin : leaf
        // s3ga<32> => { lb<8> lb<8> lb<8> lb<8> } directly, sans switch<32>
        wire `V(B)      halfs;          // half-LUT cascade chains, 0->1->2->3->0
        for (i = 0; i < B; i=i+1) begin : lbs
            wire `V(B-1) peers;
            for (j = 0; j < B-1; j=j+1)
                assign peers[j] = up_o[i + (j>=i)];
            lb #(.M(M), .B(B), .K(K), .G(UP_I_W), .I(LB_IB), .CFG_W(CFG_W))
                lb_(.clk, .rst(rst_q), .m(m_q), .cfg(cfg_q), .cfg_i(cfgs[i]), .cfg_o(cfgs[i+1]),
                  .globs(up_i), .peers, .half_i(halfs[(i+B-1)%B]), .half_o(halfs[i]), .o(up_o[i]));
        end
        assign cfg_o = cfgs[B];
        assign io_o = '0;
    end
    else begin : subs
        // recurse to B subclusters sized N/B
        wire `NV(B,DN_I_W)  dn_is;      // down switches' serial inputs
        wire `NV(B,DN_O_W)  dn_os;      // down switches' serial outputs
        wire `NV(B,IO_O_W)  io_os;      // sub-clusters' IO outputs

        switch #(.M(M), .B(B), .DELAY(1), .UP_I_W(UP_I_W), .UP_O_W(UP_O_W),
                 .DN_I_W(DN_I_W), .DN_O_W(DN_O_W), .CFG_W(CFG_W))
            sw(.clk, .rst(rst_qq), .m(m_qq), .cfg(cfg_qq), .cfg_i(cfgs[B]), .cfg_o(cfg_o), .up_i, .up_o, .dn_is, .dn_os);

        for (i = 0; i < B; i=i+1) begin : cs
            cluster #(.N(N/B), .M(M), .B(B), .K(K), .LB_IB(LB_IB), .CFG_W(CFG_W),
                      .IO_I_W(IO_I_W), .IO_O_W(IO_O_W),
                      .UP_I_WS(UP_I_WS/100), .UP_O_WS(UP_O_WS/100), .ID(ID+i*N/B))
                c(.clk, .rst(rst_q), .m(m_q), .cfg(cfg_q), .cfg_i(cfgs[i]), .cfg_o(cfgs[i+1]),
                   .io_i, .io_o(io_os`at(i,IO_O_W)),
                   .up_i(dn_os`at(i,DN_O_W)), .up_o(dn_is`at(i,DN_I_W)));
        end
        assign io_o = io_os`at(0,IO_O_W); // IO output, if any, is from first sub-cluster
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
    parameter DELAY     = 1,            // no. of output pipeline stages 
    parameter UP_I_W    = 12,           // up switch (*) input  width  (*: 0 if none)
    parameter UP_O_W    = 8,            // up switch (*) output width  ""
    parameter DN_I_W    = 4,            // down switches input  width
    parameter DN_O_W    = 6,            // down switches output width
    parameter CFG_W     = 4             // config I/O width
) (
    input               clk,
    input               rst,
    input  `CNT(M)      m,
    input               cfg,
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
            // ith xbar input is concat of dn_is[j] and up_i for j!=i
            for (j = 0; j < B-1; j=j+1) begin : is_
                assign is[i][j*DN_I_W +: DN_I_W] = dn_is`at(j+(j>=i),DN_I_W);
            end
            if (UP_I_W > 0)
                assign is[i][(B-1)*DN_I_W +: UP_I_W] = up_i;

            xbar #(.M(M), .DELAY(DELAY), .I_W(DN_X_W), .O_W(DN_O_W), .CFG_W(CFG_W))
                x(.clk, .rst, .m, .cfg, .cfg_i(cfgs[i]), .cfg_o(cfgs[i+1]), .i(is[i]), .o(dn_os`at(i,DN_O_W)));
        end

        // optional up outputs' crossbar
        if (UP_O_W > 0) begin : up
            xbar #(.M(M), .DELAY(DELAY), .I_W(B*DN_I_W), .O_W(UP_O_W), .CFG_W(CFG_W))
                x(.clk, .rst, .m, .cfg, .cfg_i(cfgs[B]), .cfg_o, .i(dn_is), .o(up_o));
        end
        else begin
            assign cfg_o = cfgs[B];
        end
    endgenerate
endmodule


// Configurable M-context crossbar

module xbar #(
    parameter M         = 8,            // M contexts
    parameter DELAY     = 1,            // no. of output pipeline stages
    parameter I_W       = 4,            // input  width
    parameter O_W       = 4,            // output width 
    parameter CFG_W     = 4             // config I/O width
) (
    input               clk,
    input               rst,
    input  `CNT(M)      m,
    input               cfg,
    input  `V(CFG_W)    cfg_i,
    output `V(CFG_W)    cfg_o,
    input  `V(I_W)      i,
    output `V(O_W)      o
);
    localparam SEL_W    = $clog2(I_W);
    wire `NV(O_W,SEL_W) sels;
    `comb`V(O_W)        o_;

    cfg_ram #(.M(M), .W(O_W*SEL_W), .CFG_W(CFG_W))
        selects(.clk, .rst, .m, .cfg_i, .cfg_o, .o(sels));

    integer j;
    always @* begin
        for (j = 0; j < O_W; j=j+1)
            o_[j] = i[sels`at(j,SEL_W)];
    end
    pipe #(.W(O_W), .DELAY(DELAY)) o_pipe(.clk, .i(o_), .o);
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
    input               cfg,
    input  `CNT(M)      m,
    input  `V(CFG_W)    cfg_i,
    output `V(CFG_W)    cfg_o,
    input  `V(G)        globs,          // global inputs
    input  `V(B-1)      peers,          // serial outputs from peer LB8s in this X32 cluster
    input               half_i,         // half-LUT cascade in
    output `comb        half_o,         // half-LUT cascade out
    output `comb        o
);
    // LB IMUXs
    localparam LB_IN_W  = G+B-2;
    localparam LB_SEL_W = $clog2(LB_IN_W);
    `comb`V(LB_IN_W)    lb_ins;
    `comb`V(I)          imuxs;

    // input and output buffers
    reg  `NV(I,M)       ibufs;
    reg  `V(M)          obuf;
    reg                 half_q;         // prev tick's half_lut output

    // LUT inputs, IMUXs, and outputs
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
        luts(.clk, .rst, .m, .cfg_i, .cfg_o, .o({lb_in_sels,lut_in_sels,mask,fde,fds,fd}));

    // LB input multiplexers
    integer i, j;
    always @* begin
        for (i = 0; i < I; i=i+1) begin
            // ith LB input is one of G globals or B-2 of the B-1 peer LB outputs
            lb_ins = globs << (B-2);
            for (j = 0; j < B-2; j=j+1)
                lb_ins[j] = peers[j + (j>=i)];
            imuxs[i] = lb_ins[lb_in_sels`at(i,LB_SEL_W)];
        end
    end

    // LB input buffers and output buffer
    always @(posedge clk) begin
        for (i = 0; i < I; i=i+1)
            ibufs`at(i,M) <= {ibufs`at(i,M),imuxs[i]};
        obuf <= {obuf,lut};
        half_q <= (m == M-1) ? half_i : half_o;   // half-LUT cascade
    end

    // lookup table and "FDRE/FDSE flip-flop"
    always @* begin
        // LUT inputs
        for (i = 0; i < K; i=i+1) begin
            ins = {ibufs,obuf};

            // half-LUT cascade special inputs
            if (i == K-2)
                ins[LUT_IN_W-1] = half_q;
            else if (i == K-1)
                ins[LUT_IN_W-1] = 1'b1;

            idx[i] = ins[lut_in_sels`at(i,LUT_SEL_W)];
        end
        // LUT / half-LUT outputs
        // REVIEW: SPEED
        lut = ~cfg & mask[idx];
        half_o = ~cfg & mask[idx[K-2:0]];

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


// Configurable M-context IO block
//
// Crossbar parallel inputs into serial outputs;
// crossbar serial inputs into parallel outputs.

module iob #(
    parameter M         = 8,            // M contexts
    parameter CFG_W     = 4,            // config I/O width
    parameter IO_I_W    = 16,           // parallel IO input  width
    parameter IO_O_W    = 16,           // parallel IO output width
    parameter I_W       = 6,            // serial input  width
    parameter O_W       = 4             // serial output width
) (
    input               clk,
    input               rst,
    input  `CNT(M)      m,
    input               cfg,
    input  `V(CFG_W)    cfg_i,
    output `V(CFG_W)    cfg_o,
    input  `V(IO_I_W)   io_i,           // per M-cycle
    output reg `V(IO_O_W) io_o,         // per M-cycle
    input  `V(I_W)      i,
    input  `V(O_W)      o
);
    wire `V(CFG_W)      cfg_;
    reg  `V(IO_O_W)     io_o_;          // prior pending output nets
    `comb`V(IO_O_W)     io_o_nxt;       // current pending output nets

    // register IO inputs and outputs as next M-cycle starts
    reg  `V(IO_I_W)     io_i_q;
    always @(posedge clk) begin
        if (cfg)
            io_i_q <= '0;
        else if (m == M-1)
            io_i_q <= io_i;

        if (cfg)
            io_o <= '0;
        else if (m == M-1)
            io_o <= io_o_nxt;
    end

    // crossbar parallel inputs into serial outputs
    xbar #(.M(M), .DELAY(0), .I_W(IO_I_W), .O_W(O_W), .CFG_W(CFG_W))
        x(.clk, .rst, .m, .cfg, .cfg_i, .cfg_o(cfg_), .i(io_i_q), .o(o));

    // crossbar serial inputs into parallel outputs
    // output configuration frame (1 context only)
    localparam SEL_W    = $clog2(I_W);
    localparam TICK_W   = $clog2(M);
    wire `NV(IO_O_W,SEL_W)  sels;       // output selects
    wire `NV(IO_O_W,TICK_W) ticks;      // output ticks
    cfg_ram #(.M(1), .W(IO_O_W*(SEL_W+TICK_W)), .CFG_W(CFG_W))
        sels_(.clk, .rst, .m('0), .cfg_i(cfg_), .cfg_o, .o({ticks,sels}));

    // output muxes and flops: for each output bit, register some input net,
    // on some tick, accumulating them in io_o_ (cumulative prior) and
    // io_o_nxt (current)
    integer j;
    always @* begin
        io_o_nxt = io_o_;
        for (j = 0; j < IO_O_W; j=j+1)
            if (ticks`at(j,TICK_W) == m)
                io_o_nxt[j] <= i[sels`at(j,SEL_W)];
    end
    always @(posedge clk) io_o_ <= cfg ? '0 : io_o_nxt;
endmodule


// Configuration ram with M W-bit contexts
//
// Config FSM: reset/0 -> wait-for-1 * -> configure * -> passthru/cfg_i.
// Config data is received in CFG_W segments: M lsbs' segments then M next-lsbs' segments, etc.
// Example config frame layout (assume M=8, W=4, CFG_W=2):
//  0* (ignored), 1, [0][1:0], [1][1:0], ..., [7][1:0], [0][3:2], [1][3:2], ..., [7][3:2].

module cfg_ram #(
    parameter M         = 8,            // M switch contexts
    parameter W         = 11,           // output width
    parameter CFG_W     = 4,            // config I/O width
    parameter USE_SR    = 1             // use shift register RAM
) (
    input               clk,
    input               rst,
    input  `CNT(M)      m,              // presently unused
    input  `V(CFG_W)    cfg_i,
    output reg`V(CFG_W) cfg_o,
    output `V(W)        o
);
    // configuration state machine
    localparam ST_WAIT  = 0;            // wait for 1 signaling start of config frame
    localparam ST_CFG   = 1;            // receive config data and configure ram
    localparam ST_PASS  = 2;            // pass more config data to next cfg_ram
    localparam ST_CNT   = 3;            // no. of states
    localparam CFG_SEGS = M * `SEGS(W,CFG_W); // no. of config frame segments

    reg  `CNT(ST_CNT)   st;             // config FSM state
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

    reg `V(W) ram[0:M-1];
    integer i;

    generate if (USE_SR) begin : sr     // shift register RAM
        // ignores m! -- must carefully synchronize configuration timing
//      localparam WW_W = `SEGS(W,CFG_W) * CFG_W; // widen W to CFG_W multiple
        assign o = ram[M-1];

        `comb`V(W) ram_in;
        always @* begin
            // recirculate the last shift register tap back to the front;
            // during reset or config, merge in new config segments
            ram_in = ram[M-1];
            if (st == ST_CFG)
                ram_in[seg/M*CFG_W +: CFG_W] = cfg_i;   // may clip last segment when W%CFG_W != 0
            if (rst)
                ram_in = '0;
        end
        always @(posedge clk) begin
            for (i = 0; i < M; i=i+1)
                ram[i] <= (i == 0) ? ram_in : ram[i-1];
        end
    end
    else begin : ra                     // DFF RAM
        // uses m! -- configure a segment on any tick
        reg `V(W) ram[0:M-1];
        always @(posedge clk) begin
            if (rst) begin
                for (i = 0; i < M; i=i+1)
                    ram[i] <= '0;
            end
            else if (st == ST_CFG)
                ram[seg%M]`at(seg/M,CFG_W) <= cfg_i;
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
