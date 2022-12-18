// S3GA: simple scalable serial FPGA
// By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC.

// SPDX-FileCopyrightText: 2022 Gray Research LLC
// SPDX-FileCopyrightText: 2020 Efabless Corporation
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

`default_nettype none

/*
S3GA interface

S3GA is configured over the Wishbone bus. Once configured, S3GA may be
use IOs via Wishbone CSRs, or external IO pads io_in and io_out, or
via the logic analyzer IOs.

Wishbone CSRs
Name        Addr    RW      Description
---------------------------------------
s3ctl       0x0     RW      S3GA control register
 .reset     [0]      W        write 1: reset S3GA (but not other WB CSRs)
                    R         1 => reset busy
 .cfg_done  [1]     R         1 => S3GA fully configured
 .overrun   [3]     R         1 => write during reset busy or config busy
 .clk_sel   [5:4]   RW        S3GA clk is
                                0 => WB wbs_clk_i
                                1 => IO io_in[35]
                                2 => LA la_data_in[64]
                                // TBD: 3 => single step
 .rst_sel   [7:6]   RW        S3GA rst is
                                0 => WB CSR s3ctl.reset
                                1 => IO io_in[34]
                                2 => LA la_data_in[65]
 .io_sel    [9:8]   RW        S3GA io_i is
                                0 => WB s3_in
                                1 => IO io_in
                                2 => LA la_data_in
s3cfg       0x4      W      S3GA configuration register: write config data
                    R       1 => config data busy
s3oem       0x8     RW      S3GA output enable mask
s3in        0x10    RW      S3GA input register [31:0]
s3out0      0x20    R       S3GA output register [31:0]
s3out1      0x24    R       S3GA output register [47:32]

Logic analyzer (when s3ctl.*_sel=2)
Name        Dir Bits        Description
---------------------------------------
la_io_i     in  [31:0]      S3GA io_i[31:0]
la_clk      in  [32]        S3GA clk
la_rst      in  [33]        S3GA rst
la_io_o     out [111:64]    S3GA io_o[47:0]

Configuration sequence, approx:
    ; init pointers
        la a0,s3ga      ; CSRs base
        la a1,cfg_data
        la a2,cfg_data_end
    ; reset S#GA
        li a3,1
        sw a3,0(a0)     ; reset
    1:  lw a3,0(a0)
        bnz a3,1b       ; spin while reset busy
    ; send a config word
    2:  lw a3,0(a1)
        addi a1,a1,4
        sw a3,4(a0)     ; send a config data word
    3:  lw a3,4(a0)
        bnz a3,3b       ; spin while config data send busy
        blt a1,a2,2b    ; send another config data word?
    ; await config done
        li a4,2
    4:  lw a3,0(a0)
        bne a3,a4,4b
    ; S3GA is up and running
*/

module s3ga_proj #(
    parameter N         = 128,          // N logical LUTs
    parameter M         = 8,            // M contexts
    parameter CFG_W     = 5,            // config I/O width: {last,data[3:0]}
    parameter IO_I_W    = 32,           // parallel IO input  width
    parameter IO_O_W    = 48            // parallel IO output width
) (
`ifdef USE_POWER_PINS
    inout vccd1,    // User area 1 1.8V supply
    inout vssd1,    // User area 1 digital ground
`endif

    // Wishbone target ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output `comb wbs_ack_o,
    output `comb [31:0] wbs_dat_o,

    // logic analyzer
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // IRQ
    output [2:0] irq
);
    // CSR address lsbs
    localparam [5:0] s3ctl  = 6'h00;
    localparam [5:0] s3cfg  = 6'h04;
    localparam [5:0] s3oem  = 6'h08;
    localparam [5:0] s3in_  = 6'h10;
    localparam [5:0] s3out0 = 6'h20;
    localparam [5:0] s3out1 = 6'h24;
    localparam IO_MIN       = 8;        // only use IOs in [IO_MIN,IO_MAX);
    localparam IO_MAX       = 36;       // i.e. avoid IOs[37,36,7:0]
    localparam SEG_W        = CFG_W-1;  // config data segment width

    // Wishbone interface, CSRs
    reg `CNT(2*M+1) rst_cnt;            // S3GA reset counter in [0,2*M]
    reg `CNT(M+1) cfg_cnt;              // S3GA config counter in [0,M]
    reg overrun;                        // s3ctl[3]
    reg `CNT(4) clk_sel;                // s3ctl[5:4]
    reg `CNT(4) rst_sel;                // s3ctl[7:6]
    reg `CNT(4) io_sel;                 // s3ctl[9:8]
    reg `V(32) cfg_data;                // s3cfg CSR: 32b config data loadable shift reg
    reg `V(32) oem;                     // s3oem: io_out output enable S3 outputs mask
    reg `V(IO_I_W) s3in;                // s3in CSR
    reg `V(IO_O_W-32) s3out1_;          // snapshot of second half of outputs
    `comb wbs_v;                        // valid WB request
    `comb rst_busy;                     // S3GA reset busy
    `comb cfg_busy;                     // S3GA send config data busy
    `comb cfg_v;                        // S3GA config segment valid

    // S3GA signals
    wire done;                          // S3GA full config complete, up and running
    wire tock;                          // S3GA M-cycle ends: last tick
    reg  tick;                          // S3GA M-cycle begins: first tick
    wire `V(IO_O_W) io_o;               // S3GA IO out

    always @* begin
        // Wishbone: asserts cyc and stb, in the same cycle, target responds
        // (read:) with ack and read data, or
        // (write:) latches WB write data inputs on posedge clk
        //
        // WB: 3.10: "If the SLAVE guarantees it can keep pace with all MASTER interfaces
        // and if the [ERR_I] and [RTY_I] signals are not used, then the SLAVE’s [ACK_O]
        // signal MAY be tied to the logical AND of the SLAVE’s [STB_I] and [CYC_I] inputs.
        // The interface will function normally under these circumstances."
        wbs_ack_o = wbs_cyc_i && wbs_stb_i;

        // CSR reads
        case (wbs_adr_i[5:0])
        s3ctl:  wbs_dat_o = {io_sel,rst_sel,clk_sel,overrun,1'b0,done,rst_busy};
        s3cfg:  wbs_dat_o = cfg_busy;
        s3oem:  wbs_dat_o = oem;
        s3in_:  wbs_dat_o = s3in;
        s3out0: wbs_dat_o = io_o;
        s3out1: wbs_dat_o = s3out1_;
        default:wbs_dat_o = '0;
        endcase

        // S3GA state machines for reset, and to transfer one config word
        // as M segments in M contiguous beats, starting after a tock
        rst_busy = rst_cnt != 2*M;      // reset busy until its terminal count 
        cfg_busy = cfg_cnt != M;        // config send busy until its TC
        cfg_v    = cfg_busy && (cfg_cnt != 0 || tick);  // sync cfg_v to start of M-cycle
    end
    always @(posedge wb_clk_i) begin
        tick <= tock;
        if (wb_rst_i) begin
            rst_cnt <= '0;              // start reset counter
            cfg_cnt <= M;               // end config counter
            overrun <= 0;
            clk_sel <= '0;
            rst_sel <= '0;
            io_sel <= '0;
            cfg_data <= '0;
            oem <= '0;
            s3in <= '0;
        end
        else begin
            // reset, config
            if (rst_busy)
                rst_cnt <= rst_cnt + 1'b1;
            if (cfg_v) begin
                cfg_cnt <= cfg_cnt + 1;
                cfg_data <= cfg_data >> SEG_W;
            end

            // CSR writes (must follow above)
            if (wbs_ack_o && wbs_we_i && &wbs_sel_i[3:0]) begin
                case (wbs_adr_i[5:0])
                s3ctl: begin
                    if (wbs_dat_i[0])
                        rst_cnt <= '0;
                    {io_sel,rst_sel,clk_sel,overrun} <= wbs_dat_i[9:3];
                end
                s3cfg: begin cfg_data <= wbs_dat_i; cfg_cnt <= '0; end
                s3oem: oem <= wbs_dat_i;
                s3in_: s3in <= wbs_dat_i;
                default:;
                endcase
                if (rst_busy || cfg_busy)
                    overrun <= 1;       // tsk
            end
            // when reading s3out0, snapshot s3out1 read response too, so that
            // the two are synchronized when sequentially read by software
            if (wbs_ack_o && !wbs_we_i && wbs_adr_i[5:0] == s3out0)
                s3out1_ <= io_o >> 32;
        end
    end

    // S3GA inputs from WB CSRs, project inputs, or LA inputs
    `comb clk;
    `comb rst;
    `comb `V(IO_I_W) io_i;
    `comb `V(CFG_W) cfg_i;

    always @* begin
        clk = clk_sel==2'd1 ? io_in[IO_MAX-1] : clk_sel==2'd2 ? la_data_in[32] : wb_clk_i;
        rst = rst_sel==2'd1 ? io_in[IO_MAX-2] : rst_sel==2'd2 ? la_data_in[33] : rst_busy;
        case (io_sel)
        default:io_i = s3in;
        2'd1:   io_i = io_in[IO_MAX-1:IO_MIN];
        2'd2:   io_i = la_data_in[IO_I_W-1:0];
        endcase

        cfg_i = cfg_v ? {1'b1,cfg_data[0+:SEG_W]} : '0;
    end

    s3ga #(.N(N), .M(M), .CFG_W(CFG_W), .IO_I_W(IO_I_W), .IO_O_W(IO_O_W))
        s(.clk, .rst, .done, .tock, .cfg_i, .io_i, .io_o);

    // project IOs
    // output first 28 S3GA outputs on io_out[:8]
    assign io_out = io_o << IO_MIN;
    // output next 14 S3GA outputs, doubled, on io_oeb[:8] (under oem);
    // thus S3GA can configurably dynamically tristate (pairs of) output wires
    genvar i;
    generate for (i = 0; i < `MPRJ_IO_PADS; i=i+1) begin : oebs
        assign io_oeb[i] = rst           ? 1'b1
                         : i < IO_MIN    ? 1'b0
                         : i >= IO_MAX   ? 1'b0
                         : oem[i-IO_MIN] ? io_o[IO_MAX-IO_MIN+(i-IO_MIN)/2]
                         : 1'b0;
    end endgenerate

    // logic analyzer. REVIEW: bring out some debug signals
    assign la_data_out = io_o << 64;

    // other
    assign irq = '0;
endmodule


`default_nettype wire
