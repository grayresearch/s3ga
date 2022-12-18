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

`define default_netname none
`timescale 1ns/1ps

`define MAX(A,B)    ((A) >= (B) ? (A) : (B))
`define V(W)        [`MAX((W)-1,0):0]           /* 1D bit vector, min width=1 */
`define CNT(W)      `V($clog2(W))               /* counter in [0,W) (NOT [0,W]!) */
`define SEGS(N,M)   (((N) + ((M)-1)) / (M))     /* number of full or partial W-bit segments in N bits */
`define comb        reg                         /* combinational: assigned in an always @* block */

// alas, in 2022, cannot assume all Verilog tools implement module ports w/ 2D arrays
`define NV(N,W)     [`MAX((N)*(W)-1,0):0]       /* flatten N x W-bit 2D array to 1D */
`define at(i,W)     [(i)*W+:W]                  /* access 2D array */
