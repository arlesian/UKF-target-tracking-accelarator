`timescale 1ns/1ps

module tb_ukf;
    localparam signed [31:0] FXP_ONE = 32'sd65536;

    reg clk;
    reg rst_n;
    reg start;
    reg signed [31:0] meas_x;
    reg signed [31:0] meas_y;
    reg signed [31:0] meas_z;

    wire done;
    wire busy;
    wire [4:0] state_dbg;
    wire [7:0] sigma_index_dbg;
    wire phase_predict_dbg;
    wire phase_update_dbg;
    wire signed [31:0] est_x;
    wire signed [31:0] est_y;
    wire signed [31:0] est_z;
    real est_x_real;
    real est_y_real;
    real est_z_real;

    ukf dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .meas_x(meas_x),
        .meas_y(meas_y),
        .meas_z(meas_z),
        .done(done),
        .busy(busy),
        .state_dbg(state_dbg),
        .sigma_index_dbg(sigma_index_dbg),
        .phase_predict_dbg(phase_predict_dbg),
        .phase_update_dbg(phase_update_dbg),
        .est_x(est_x),
        .est_y(est_y),
        .est_z(est_z)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    initial begin
        $dumpfile("ukf.vcd");
        $dumpvars(0, tb_ukf);

        rst_n  = 1'b0;
        start  = 1'b0;
        meas_x = 12 * FXP_ONE;
        meas_y = -3 * FXP_ONE;
        meas_z = 5 * FXP_ONE;

        #20;
        rst_n = 1'b1;

        #20;
        start = 1'b1;

        wait(done == 1'b1);
        #20;
        start = 1'b0;

        #50;
        $finish;
    end

    initial begin
        $display("Starting SR-UKF accelerator architectural simulation");
    end

    always @(*) begin
        est_x_real = $itor(est_x) / FXP_ONE;
        est_y_real = $itor(est_y) / FXP_ONE;
        est_z_real = $itor(est_z) / FXP_ONE;
    end

    always @(posedge clk) begin
        $display(
            "t=%0t busy=%b done=%b state=%0d sigma=%0d pred=%b upd=%b est=(%0f,%0f,%0f)",
            $time,
            busy,
            done,
            state_dbg,
            sigma_index_dbg,
            phase_predict_dbg,
            phase_update_dbg,
            est_x_real,
            est_y_real,
            est_z_real
        );
    end
endmodule
