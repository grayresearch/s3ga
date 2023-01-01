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

`include "s3ga.h"

`ifndef MPRJ_IO_PADS
`include "defines.h"
`endif

/*
Caravel S3GA interface

Out of a preponderance of caution, the interface of last resort is the
the logic analyzer. S3GA may be configured by the logic analyzer or over
the Wishbone bus. Once configured, S3GA may use IOs via Wishbone CSRs,
MPRJ IOs, or logic analyzer IOs.

Logic analyzer signals
Name        Dir Bits    Description
-----------------------------------
la_ctl      in  7:0
  .clk_sel  in  1:0     S3GA clk select: 0:wb_clk_i 1:la_clk 2:io_clk 3:user_clock2
  .i_sel    in  3:2     S3GA io_i[31: 0] input select: 0:s3_in0 1:s3_in0 2:la_in  3:io_in
                        S3GA io_i[63:32] input select: 0:s3_in1 1:io_in  2:s3_in0 3:la_in when IO_I_W>32
  .clk      in  4       S3GA clock when la.clk_sel==0; la_* inputs sampled on 0=>1 transition
  .rst      in  5       S3GA reset
  .cfg      in  6       S3GA cfg CE: send la_in as a config data frame
  .oem      in  7       S3GA output enable mask CE: oem <= la_in (MPRJ output enable mask)
--          --  15:8    --
la_status   out 23:16
  .rst_busy out 16      S3GA reset busy
  .cfg_busy out 17      S3GA config data frame busy
  .done     out 18      S3GA fully configured, up and running
  --        --  23:19   --
--          --  31:24   --
la_in       in  63:32   la_in => config data frame when la_ctl.cfg on posedge clk
                        la_in => oem when la_ctl.oem on posedge clk
                        la_in => S3GA io_i[31: 0] when i_sel==
                        la_in => S3GA io_i[63:32] when
la_io_out0  out 95:64   la_io_out0 <= S3GA io_o[31:0]
la_io_out1  out 127:96  la_io_out1 <= S3GA io_o[63:32]

Wishbone CSRs
Name        Addr    RW      Description
---------------------------------------
s3_ctl      0x0     RW      S3GA control register
 .rst       [0]      W        write 1: reset S3GA (but not other WB CSRs)
                    R         1 => reset busy
 .cfg_done  [1]     R         1 => S3GA fully configured
s3_cfg      0x4      W      S3GA configuration register: write config data
                    R       1 => config data busy
s3_oem      0x8     RW      S3GA output enable mask
s3_in0      0x10    RW      S3GA input  register => io_i[31:0] / io_i[63:32]   per la_ctl.i_sel
s3_in1      0x14    RW      S3GA input  register => io_i[63:32] when IO_I_W>32 per la_ctl.i_sel
s3_out0     0x20    R       S3GA output register <= io_o[31: 0]
s3_out1     0x24    R       S3GA output register <= io_o[63:32] when IO_O_W>32

MPRJ I/O Signals
Name        Dir Pins    Description
-----------------------------------
io_clk      in  35      external S3GA fabric clock input
io_in       in  34:8    external inputs  => S3GA io_i per la_ctl.i_sel
io_out      out 34:8    external outputs <= S3GA io_o[25:0]
io_oeb      out 34:8    io_oeb[i] = ~(oem[(i-8)] & S3GA io_o[26+(i-8)/4] "approx" )

Wishbone configuration sequence, approx:
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

LA configuration sequence, approx:
    config() {
        LA* la;
        la->ctl = 0;
        la->io_in = 0;
        la->oem = 0;

        la->ctl = '{ clk=0; rst=1; others=0; };
        la->ctl = '{ clk=1; rst=1; others=0; };
        la->ctl = '{ clk=0; others=0; };

        do { clock(); } while (la->status.rst_busy);

        for (uint32* pcfg = pcfg_base; pcfg < pcfg_end; ++pcfg) {
            la->cfg_data = *pcfg;
            la->ctl = '{ clk=0; cfg=1; others=0; }
            la->ctl = '{ clk=1; cfg=1; others=0; }
            la->ctl = '{ clk=0; others=0; }
            do { clock(); } while (la->status.cfg_busy);
        }

        do { clock(); } while (!la->status.cfgd);
        // up and running
    }
    clock() {
        la->ctl = '{ clk=1; others=0; };
        la->ctl = '{ clk=0; others=0; };
    }
*/

module s3ga_proj #(
    parameter N         = 1024,         // N logical LUTs
    parameter M         = 4,            // M contexts
    parameter CFG_W     = 5,            // config I/O width: {last,data[3:0]}
    parameter IO_I_W    = 32,           // parallel IO input  width
    parameter IO_O_W    = 64,           // parallel IO output width
    parameter MACRO_N   = 64            // reuse a macro for cluster<N=MACRO_N>
) (
`ifdef USE_POWER_PINS
    inout  vccd1,   // User area 1 1.8V supply
    inout  vssd1,   // User area 1 digital ground
`endif
    // Wishbone
    input  wb_clk_i,
    input  wb_rst_i,
    input  wbs_stb_i,
    input  wbs_cyc_i,
    input  wbs_we_i,
    input  [3:0] wbs_sel_i,
    input  [31:0] wbs_dat_i,
    input  [31:0] wbs_adr_i,
    output reg wbs_ack_o,               // @wb_clk_i
    output reg [31:0] wbs_dat_o,        // @wb_clk_i
    // logic analyzer
    input  [127:0] la_data_in,
    output `comb [127:0] la_data_out,
    input  [127:0] la_oenb,
    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output `comb [`MPRJ_IO_PADS-1:0] io_out,
    output `comb [`MPRJ_IO_PADS-1:0] io_oeb,
    // other
    input  user_clock2,
    output `comb [2:0] user_irq
);
    ///////////////////////////////////////////////////////////////////////////
    // signals and state

    // S3GA inputs
    wire clk = wb_clk_i;                // clock input mux -- CLKFIX!!
    `comb use_wb_clk;                   // la_ctl.clk_sel == 0: use wb_clk_i
    `comb use_la_clk;                   // la_ctl.clk_sel == 1: use la_clk
    reg `V(IO_I_W) io_i;                // S3GA IO inputs       @clk
    `comb `V(CFG_W) cfg_i;              // S3GA config segment  @clk

    // S3GA outputs
    wire done;                          // S3GA full config complete, up and running
    wire tock;                          // S3GA M-cycle ends: last tick
    wire `V(IO_O_W) io_o;               // S3GA IO out

    // reset and config FSMs
    localparam SEG_W        = CFG_W-1;  // config data segment width
    localparam SEGS         = 32/SEG_W; // no. of segments in a 32b config frame
    reg tick;                           // S3GA M-cycle begins: first tick
    reg `CNT(2*M+1) rst_cnt;            // S3GA reset counter in [0,2*M]
    reg `CNT(SEGS+1) cfg_cnt;           // S3GA config counter in [0,SEGS]
    reg `V(M*SEG_W) cfg_data;           // config data frame shift register
    `comb rst_req;                      // S3GA reset request
    `comb rst_busy;                     // S3GA reset busy
    `comb cfg_busy;                     // S3GA send config data busy
    `comb cfg_v;                        // S3GA config segment valid

    // logic analyzer interface         // see top of file block comment
    reg `CNT(4) la_clk_sel;             // all la_* inputs @wb_clk_i alas
    reg `CNT(4) la_i_sel;
    reg la_clk;
    reg la_rst;
    reg la_cfg;
    reg la_oem;
    reg `V(32) la_in;

    // Wishbone CSRs                    // see top of file block comment
    localparam [5:0] s3_ctl  = 6'h00;
    localparam [5:0] s3_cfg  = 6'h04;
    localparam [5:0] s3_oem  = 6'h08;
    localparam [5:0] s3_in0  = 6'h10;
    localparam [5:0] s3_in1  = 6'h14;
    localparam [5:0] s3_out0 = 6'h20;
    localparam [5:0] s3_out1 = 6'h24;
    localparam S3_IN_W = (IO_I_W>32) ? 64 : 32;
    reg `V(S3_IN_W) s3_in;              // s3_in CSR(s)
    reg `V(IO_O_W-32) s3_out1_q;        // s3_out CSR(s)
    `comb wb_req;                       // WB read/write request: first cycle of WB transaction
    `comb wb_write;                     // valid WB write
    `comb wb_rst;                       // WB write to s3_ctl.rst=1
    `comb wb_cfg;                       // WB write to s3_cfg
    `comb wb_oem;                       // WB write to s3_oem

    // MPRJ IOs
    localparam MPRJ_IO_MIN  = 8;        // only use IOs in [MPRJ_IO_MIN,MPRJ_IO_MAX);
    localparam MPRJ_IO_MAX  = 36;       // i.e. avoid IOs[37,36,7:0]
    localparam MPRJ_IO_W    = MPRJ_IO_MAX - MPRJ_IO_MIN;
    wire io_clk = io_in[MPRJ_IO_MIN];   // MPRJ S3GA clock input

    // other state
    reg `V(MPRJ_IO_W) oem = 0;          // MPRJ io_oeb mask ([i] => oeb[MPRJ_IO_MIN+i] enabled)

    ///////////////////////////////////////////////////////////////////////////
    // Clock mux and reset and config FSMs -- see top of file block comment
    //
    // State here can be managed by logic analyzer or Wishbone interfaces,
    // so take care to sample requests per current la_clk_sel.

    reg wb_rst_i_q;
    always @(posedge wb_clk_i) wb_rst_i_q <= wb_rst_i;

    always @* begin
        use_wb_clk = la_clk_sel == 2'd0;
        use_la_clk = la_clk_sel == 2'd1;

        // REVIEW: temporary expediency due to MPW-8 hold violations -- CLKFIX!!
        /*
        case (la_clk_sel)
        default: clk = wb_clk_i;
        2'd1:    clk = la_clk;
        2'd2:    clk = io_clk;
        2'd3:    clk = user_clock2;
        endcase
        */

        // S3GA reset *request*
        rst_req = wb_rst_i_q || (use_wb_clk && wb_rst) || (use_la_clk && la_rst);

        // S3GA state machines for reset, and to transfer one config data frame
        // as M segments in M contiguous beats, starting after a tock
        rst_busy = rst_cnt != 2*M;      // reset busy until its terminal count 
        cfg_busy = cfg_cnt != SEGS;     // config send busy until its TC
        cfg_v    = cfg_busy && (cfg_cnt != 0 || tick);  // sync cfg_v to start of M-cycle
        cfg_i    = cfg_v ? {1'b1,cfg_data[0+:SEG_W]} : 0;
    end
    always @(posedge clk) begin
        tick <= tock;
        if (rst_req) begin
            rst_cnt <= 0;               // start reset counter
            cfg_cnt <= SEGS;            // end config counter
            cfg_data <= 0;
            oem <= 0;
        end
        else begin
            if (rst_busy)
                rst_cnt <= rst_cnt + 1'b1;

            if (cfg_v) begin
                // send another config data segment
                cfg_cnt  <= cfg_cnt + 1'b1;
                cfg_data <= cfg_data >> SEG_W;
            end
            else if (use_wb_clk && wb_cfg || use_la_clk && la_cfg) begin
                // start send config data frame: register config data frame
                cfg_cnt  <= 0;
                cfg_data <= la_cfg ? la_in : wbs_dat_i;
            end

            if (use_wb_clk && wb_oem || use_la_clk && la_oem)
                oem <= la_oem ? la_in : wbs_dat_i;
        end
    end


    ///////////////////////////////////////////////////////////////////////////
    // Logic analyzer interface

    // inputs [15:0] and [63:32] -- registered to mitigate downstream hold violations
    always @(posedge wb_clk_i) begin
        {la_oem,la_cfg,la_rst,la_clk,la_i_sel,la_clk_sel} <= la_data_in[7:0];
        la_in <= la_data_in[63:32];
    end
    always @* begin
        // outputs [31:16] and [127:64]
        la_data_out[15:0] = 0;
        la_data_out[23:16] = {5'b0,done,cfg_busy,rst_busy};
        la_data_out[63:24] = 0;
        la_data_out[127:64] = io_o;
    end

    ///////////////////////////////////////////////////////////////////////////
    // Wishbone CSRs

    // Wishbone: asserts cyc and stb; in response:
    // (read:)  next cycle ack, with registered CSR read data output, or
    // (write:) next cycle ack, and (this cycle) register CSR write data on posedge clk 
    //
    // REVIEW: previously, this FSM responded to the WB request in the same cycle, per
    // WB: 3.10: "If the SLAVE guarantees it can keep pace with all MASTER interfaces
    // and if the [ERR_I] and [RTY_I] signals are not used, then the SLAVE’s [ACK_O]
    // signal MAY be tied to the logical AND of the SLAVE’s [STB_I] and [CYC_I] inputs.
    // The interface will function normally under these circumstances."
    // Out of caution, to be more like user_proj_example, and seeking more timing
    // margin, replaced that with current logic,/ which acks, and drives read data,
    // on the second clock cycle.

    always @* begin
        // Writes to CSRs { s3_ctl, s3_cfg, s3_oem } impact the interface's
        // reset and config FSMs, and interact with logic analyzer outputs.
        // These are handled above in the FSMs block of the module.
        wb_req   = !wb_rst_i_q && wbs_cyc_i && wbs_stb_i && !wbs_ack_o/* first cycle of transaction*/;
        wb_write = wb_req && wbs_we_i && &wbs_sel_i;
        wb_rst   = wb_write && wbs_adr_i[5:0] == s3_ctl && wbs_dat_i[0];
        wb_cfg   = wb_write && wbs_adr_i[5:0] == s3_cfg;
        wb_oem   = wb_write && wbs_adr_i[5:0] == s3_oem;
    end
    always @(posedge wb_clk_i) begin
        if (wb_rst_i_q) begin
            wbs_ack_o <= 0;
            wbs_dat_o <= 0;
            s3_in <= 0;
            s3_out1_q <= 0;
        end
        else if (wbs_ack_o) begin
            wbs_ack_o <= 0;             // ack for one cycle
        end
        else if (wb_req) begin
            wbs_ack_o <= 1;
            if (wbs_we_i && &wbs_sel_i) begin
                // CSR write
                case (wbs_adr_i[5:0])
                // { s3_ctl, s3_cfg, s3_oem } writes are handled above
                s3_in0: s3_in[0+:32] <= wbs_dat_i;
                s3_in1: s3_in <= {wbs_dat_i,s3_in[0+:32]}; // S3_IN_W in [32,64]
                default: ; // writes to s3_ctl, s3_cfg, s3_oem are handled above
                endcase
            end
            else if (!wbs_we_i) begin
                // CSR read
                case (wbs_adr_i[5:0])
                s3_ctl:  wbs_dat_o <= {done,rst_busy};
                s3_cfg:  wbs_dat_o <= cfg_busy;
                s3_oem:  wbs_dat_o <= oem;
                s3_in0:  wbs_dat_o <= s3_in;
                s3_in1:  wbs_dat_o <= s3_in >> 32;
                s3_out0: begin wbs_dat_o <= io_o; s3_out1_q <= io_o >> 32; end
                s3_out1: wbs_dat_o <= s3_out1_q;
                default: wbs_dat_o <= 0;
                endcase
            end
        end
    end

    ///////////////////////////////////////////////////////////////////////////
    // IOs

    `comb `V(32) io_in_32;
    integer i, j;
    always @* begin
        // S3GA IO inputs
        io_in_32 = io_in >> MPRJ_IO_MIN;

        // MPRJ IO outputs
        io_out = io_o << (MPRJ_IO_MIN + (IO_I_W==16 ? 16 : 0)); // if only 16b I/O, don't overlap them
        io_oeb = {`MPRJ_IO_PADS{1'b1}}; // default to input
        for (i = MPRJ_IO_MIN; i < MPRJ_IO_MAX; i=i+1) begin
            // given enough S3GA IOs, the io_o msbs are per-nybble dynamic output enables -- disabled
            j = MPRJ_IO_W + (i-MPRJ_IO_MIN)/4;
            io_oeb[i] = rst_busy || ~oem[i-MPRJ_IO_MIN]; // disabled: || (j<IO_O_W ? ~io_o[j] : 0);
        end
        // other
        user_irq = 0;
    end
 
    // resynchronize S3GA inputs to @clk; REVIEW CDC
    always @(posedge clk) begin
        case (la_i_sel)
        default: io_i <= s3_in;
        2'd1:    io_i <= {io_in,s3_in[0+:32]};
        2'd2:    io_i <= {s3_in[0+:32],la_in};
        2'd3:    io_i <= {la_in,io_in_32};
        endcase
    end

    ///////////////////////////////////////////////////////////////////////////
    // S3GA instance

    s3ga #(.N(N), .M(M), .CFG_W(CFG_W), .IO_I_W(IO_I_W), .IO_O_W(IO_O_W), .MACRO_N(MACRO_N))
        s(
`ifdef USE_POWER_PINS
          .vccd1, .vssd1,
`endif
          .clk, .rst(rst_busy), .done, .tock, .cfg_i, .io_i, .io_o);
endmodule

`default_nettype wire
