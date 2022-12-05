// S3GA: simple scalable serial FPGA
// By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC. All rights reserved.

module dump();
    initial begin
        $dumpfile("lb.vcd");
        $dumpvars(0, lb);
        #1;
    end
endmodule
