// S3GA: simple scalable serial FPGA
// By Jan Gray. Copyright (C) 2021-2022 Gray Research LLC. All rights reserved.

module dump();
    initial begin
        $dumpfile("cfg_ram.vcd");
        $dumpvars(0, cfg_ram);
        #1;
    end
endmodule
