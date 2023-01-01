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

module x64 #(
    parameter N         = 64,           // N logical LUTs
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
    cluster #(.N(N), .M(M), .B(B), .B_SUB(B_SUB), .K(K), .LB_IB(LB_IB), .CFG_W(CFG_W),
              .IO_I_W(IO_I_W), .IO_O_W(IO_O_W),
              .UP_I_WS(UP_I_WS), .UP_O_WS(UP_O_WS), .UP_O_DELAY(1), .ID(1), .MACRO_N(0))
        c(
`ifdef USE_POWER_PINS
          .vccd1, .vssd1,
`endif
          .clk, .rst, .grst, .m, .cfg, .cfgd, .cfg_i, .io_i, .io_o, .up_i, .up_o);
endmodule
