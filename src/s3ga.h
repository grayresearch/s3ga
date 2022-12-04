// S3GA: simple scalable serial FPGA
// By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC. All rights reserved.

`define default_netname none
`timescale 1ns/1ps

`define MAX(A,B)    ((A) >= (B) ? (A) : (B))
`define V(W)        [`MAX((W)-1,0):0]           /* 1D bit vector, min width=1 */
`define CNT(W)      [$clog2(W)-1:0]             /* counter in [0,W) (NOT [0,W]!) */
`define SEGS(N,M)   (((N) + ((M)-1)) / (M))     /* number of full or partial W-bit segments in N bits */
`define comb        reg                         /* combinational: assigned in an always @* block */

// alas, in 2022, cannot assume all Verilog tools implement module ports w/ 2D arrays
`define NV(N,W)     [`MAX((N)*(W)-1,0):0]       /* flatten N x W-bit 2D array to 1D */
`define at(i,W)     [(i)*W+:W]                  /* access 2D array */
