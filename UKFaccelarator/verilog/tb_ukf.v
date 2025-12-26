module tb_ukf;
    reg clk = 0;
    reg rst = 0;

    always #5 clk = ~clk;

    initial begin
        $dumpfile("ukf.vcd");
        $dumpvars(0, tb_ukf);
        #20
        rst = 0;
        #1000
        $finish
    end
endmodule