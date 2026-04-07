`timescale 1ns/1ps

module wc_unit #(
    parameter DATA_W = 32,
    parameter FRAC_W = 16
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    output reg  done,
    output reg signed [DATA_W-1:0] w0m_half,
    output reg signed [DATA_W-1:0] w0c_full,
    output reg signed [DATA_W-1:0] wi
);
    localparam signed [DATA_W-1:0] W0M_HALF_C = 32'sd8192;   // 0.125 in Q16.16
    localparam signed [DATA_W-1:0] W0C_FULL_C = 32'sd24576;  // 0.375 in Q16.16
    localparam signed [DATA_W-1:0] WI_C       = 32'sd4096;   // 0.0625 in Q16.16

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done     <= 1'b0;
            w0m_half <= '0;
            w0c_full <= '0;
            wi       <= '0;
        end else begin
            done <= start;
            if (start) begin
                w0m_half <= W0M_HALF_C;
                w0c_full <= W0C_FULL_C;
                wi       <= WI_C;
            end
        end
    end
endmodule

module cordic_unit #(
    parameter DATA_W = 32,
    parameter FRAC_W = 16,
    parameter N_DIM  = 6
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire signed [DATA_W-1:0] state_in [0:N_DIM-1],
    output reg  done,
    output reg  signed [DATA_W-1:0] spread
);
    integer i;
    reg [1:0] latency;
    reg signed [DATA_W-1:0] abs_sum;

    function automatic signed [DATA_W-1:0] abs_fixed;
        input signed [DATA_W-1:0] value;
        begin
            abs_fixed = value[DATA_W-1] ? -value : value;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done    <= 1'b0;
            latency <= 2'd0;
            spread  <= 32'sd65536;
        end else begin
            done <= 1'b0;
            if (start) begin
                abs_sum = 32'sd0;
                for (i = 0; i < N_DIM; i = i + 1) begin
                    abs_sum = abs_sum + (abs_fixed(state_in[i]) >>> 4);
                end
                spread  <= 32'sd32768 + abs_sum + (32'sd65536 >>> 2);
                latency <= 2'd2;
            end else if (latency != 2'd0) begin
                latency <= latency - 2'd1;
                if (latency == 2'd1) begin
                    done <= 1'b1;
                end
            end
        end
    end
endmodule

module spg_unit #(
    parameter DATA_W = 32,
    parameter N_DIM  = 6
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire signed [DATA_W-1:0] base_state [0:N_DIM-1],
    input  wire signed [DATA_W-1:0] spread,
    output reg  valid,
    output reg  done,
    output reg  center_pair,
    output reg  [7:0] pair_index,
    output reg  signed [DATA_W-1:0] sigma_a [0:N_DIM-1],
    output reg  signed [DATA_W-1:0] sigma_b [0:N_DIM-1]
);
    integer i;
    reg running;
    reg [7:0] phase;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            running     <= 1'b0;
            phase       <= 8'd0;
            valid       <= 1'b0;
            done        <= 1'b0;
            center_pair <= 1'b0;
            pair_index  <= 8'd0;
            for (i = 0; i < N_DIM; i = i + 1) begin
                sigma_a[i] <= '0;
                sigma_b[i] <= '0;
            end
        end else begin
            valid <= 1'b0;
            done  <= 1'b0;

            if (start && !running) begin
                running <= 1'b1;
                phase   <= 8'd0;
            end else if (running) begin
                valid       <= 1'b1;
                center_pair <= (phase == 8'd0);
                pair_index  <= phase;

                for (i = 0; i < N_DIM; i = i + 1) begin
                    sigma_a[i] <= base_state[i];
                    sigma_b[i] <= base_state[i];
                end

                if (phase != 8'd0) begin
                    sigma_a[phase - 8'd1] <= base_state[phase - 8'd1] + spread;
                    sigma_b[phase - 8'd1] <= base_state[phase - 8'd1] - spread;
                end

                if (phase == N_DIM[7:0]) begin
                    running <= 1'b0;
                    done    <= 1'b1;
                end else begin
                    phase <= phase + 8'd1;
                end
            end
        end
    end
endmodule

module shared_compute_fabric #(
    parameter DATA_W = 32,
    parameter FRAC_W = 16,
    parameter N_IN   = 6,
    parameter N_OUT  = 6
)(
    input  wire clk,
    input  wire rst_n,
    input  wire valid_i,
    input  wire mode_h,
    input  wire center_pair_i,
    input  wire [7:0] pair_index_i,
    input  wire signed [DATA_W-1:0] sigma_a_i [0:N_IN-1],
    input  wire signed [DATA_W-1:0] sigma_b_i [0:N_IN-1],
    output reg  valid_o,
    output reg  center_pair_o,
    output reg  [7:0] pair_index_o,
    output reg  signed [DATA_W-1:0] vec_a_o [0:N_OUT-1],
    output reg  signed [DATA_W-1:0] vec_b_o [0:N_OUT-1]
);
    integer i;
    localparam signed [DATA_W-1:0] DT_Q = 32'sd6554; // 0.1 in Q16.16

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_o       <= 1'b0;
            center_pair_o <= 1'b0;
            pair_index_o  <= 8'd0;
            for (i = 0; i < N_OUT; i = i + 1) begin
                vec_a_o[i] <= '0;
                vec_b_o[i] <= '0;
            end
        end else begin
            valid_o       <= valid_i;
            center_pair_o <= center_pair_i;
            pair_index_o  <= pair_index_i;

            if (valid_i) begin
                if (!mode_h) begin
                    vec_a_o[0] <= sigma_a_i[0] + (($signed(sigma_a_i[3]) * DT_Q) >>> FRAC_W);
                    vec_a_o[1] <= sigma_a_i[1] + (($signed(sigma_a_i[4]) * DT_Q) >>> FRAC_W);
                    vec_a_o[2] <= sigma_a_i[2] + (($signed(sigma_a_i[5]) * DT_Q) >>> FRAC_W);
                    vec_a_o[3] <= sigma_a_i[3];
                    vec_a_o[4] <= sigma_a_i[4];
                    vec_a_o[5] <= sigma_a_i[5];

                    vec_b_o[0] <= sigma_b_i[0] + (($signed(sigma_b_i[3]) * DT_Q) >>> FRAC_W);
                    vec_b_o[1] <= sigma_b_i[1] + (($signed(sigma_b_i[4]) * DT_Q) >>> FRAC_W);
                    vec_b_o[2] <= sigma_b_i[2] + (($signed(sigma_b_i[5]) * DT_Q) >>> FRAC_W);
                    vec_b_o[3] <= sigma_b_i[3];
                    vec_b_o[4] <= sigma_b_i[4];
                    vec_b_o[5] <= sigma_b_i[5];
                end else begin
                    vec_a_o[0] <= sigma_a_i[0];
                    vec_a_o[1] <= sigma_a_i[1];
                    vec_a_o[2] <= sigma_a_i[2];
                    vec_a_o[3] <= '0;
                    vec_a_o[4] <= '0;
                    vec_a_o[5] <= '0;

                    vec_b_o[0] <= sigma_b_i[0];
                    vec_b_o[1] <= sigma_b_i[1];
                    vec_b_o[2] <= sigma_b_i[2];
                    vec_b_o[3] <= '0;
                    vec_b_o[4] <= '0;
                    vec_b_o[5] <= '0;
                end
            end else begin
                for (i = 0; i < N_OUT; i = i + 1) begin
                    vec_a_o[i] <= '0;
                    vec_b_o[i] <= '0;
                end
            end
        end
    end
endmodule

module uacc_unit #(
    parameter DATA_W = 32,
    parameter FRAC_W = 16,
    parameter N_DIM  = 6
)(
    input  wire clk,
    input  wire rst_n,
    input  wire clear,
    input  wire en,
    input  wire signed [DATA_W-1:0] weight_a,
    input  wire signed [DATA_W-1:0] weight_b,
    input  wire signed [DATA_W-1:0] vec_a [0:N_DIM-1],
    input  wire signed [DATA_W-1:0] vec_b [0:N_DIM-1],
    output wire signed [DATA_W-1:0] sum_o [0:N_DIM-1]
);
    integer i;
    reg signed [DATA_W+12-1:0] acc [0:N_DIM-1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < N_DIM; i = i + 1) begin
                acc[i] <= '0;
            end
        end else if (clear) begin
            for (i = 0; i < N_DIM; i = i + 1) begin
                acc[i] <= '0;
            end
        end else if (en) begin
            for (i = 0; i < N_DIM; i = i + 1) begin
                acc[i] <= acc[i]
                    + (($signed(vec_a[i]) * $signed(weight_a)) >>> FRAC_W)
                    + (($signed(vec_b[i]) * $signed(weight_b)) >>> FRAC_W);
            end
        end
    end

    generate
        genvar g;
        for (g = 0; g < N_DIM; g = g + 1) begin : GEN_ACC_OUT
            assign sum_o[g] = acc[g][DATA_W-1:0];
        end
    endgenerate
endmodule

module pacc_unit #(
    parameter DATA_W = 32,
    parameter FRAC_W = 16,
    parameter N_DIM  = 6
)(
    input  wire clk,
    input  wire rst_n,
    input  wire clear,
    input  wire en,
    input  wire signed [DATA_W-1:0] weight,
    input  wire signed [DATA_W-1:0] vec_i [0:N_DIM-1],
    input  wire signed [DATA_W-1:0] mean_i [0:N_DIM-1],
    output reg  signed [DATA_W-1:0] spread_metric
);
    integer i;
    reg signed [DATA_W-1:0] delta_abs_sum;

    function automatic signed [DATA_W-1:0] abs_fixed;
        input signed [DATA_W-1:0] value;
        begin
            abs_fixed = value[DATA_W-1] ? -value : value;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spread_metric <= '0;
        end else if (clear) begin
            spread_metric <= '0;
        end else if (en) begin
            delta_abs_sum = '0;
            for (i = 0; i < N_DIM; i = i + 1) begin
                delta_abs_sum = delta_abs_sum + abs_fixed(vec_i[i] - mean_i[i]);
            end
            spread_metric <= spread_metric + (($signed(delta_abs_sum) * $signed(weight)) >>> FRAC_W);
        end
    end
endmodule

module sigma_buffer #(
    parameter DATA_W = 32,
    parameter N_DIM  = 6,
    parameter DEPTH  = 13
)(
    input  wire clk,
    input  wire wr_en0,
    input  wire [$clog2(DEPTH)-1:0] wr_addr0,
    input  wire signed [DATA_W-1:0] wr_vec0 [0:N_DIM-1],
    input  wire wr_en1,
    input  wire [$clog2(DEPTH)-1:0] wr_addr1,
    input  wire signed [DATA_W-1:0] wr_vec1 [0:N_DIM-1],
    input  wire [$clog2(DEPTH)-1:0] rd_addr,
    output wire signed [DATA_W-1:0] rd_vec [0:N_DIM-1]
);
    integer i;
    reg signed [DATA_W-1:0] mem [0:DEPTH-1][0:N_DIM-1];

    always @(posedge clk) begin
        if (wr_en0) begin
            for (i = 0; i < N_DIM; i = i + 1) begin
                mem[wr_addr0][i] <= wr_vec0[i];
            end
        end
        if (wr_en1) begin
            for (i = 0; i < N_DIM; i = i + 1) begin
                mem[wr_addr1][i] <= wr_vec1[i];
            end
        end
    end

    generate
        genvar g;
        for (g = 0; g < N_DIM; g = g + 1) begin : GEN_RD
            assign rd_vec[g] = mem[rd_addr][g];
        end
    endgenerate
endmodule

module qr_unit #(
    parameter DATA_W = 32
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire signed [DATA_W-1:0] spread_metric,
    output reg  done,
    output reg  signed [DATA_W-1:0] qr_factor
);
    reg [1:0] latency;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done      <= 1'b0;
            latency   <= 2'd0;
            qr_factor <= '0;
        end else begin
            done <= 1'b0;
            if (start) begin
                qr_factor <= spread_metric + 32'sd4096;
                latency   <= 2'd2;
            end else if (latency != 2'd0) begin
                latency <= latency - 2'd1;
                if (latency == 2'd1) begin
                    done <= 1'b1;
                end
            end
        end
    end
endmodule

module cholupdate_unit #(
    parameter DATA_W = 32
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire signed [DATA_W-1:0] qr_factor,
    output reg  done,
    output reg  signed [DATA_W-1:0] chol_factor
);
    reg [1:0] latency;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done        <= 1'b0;
            latency     <= 2'd0;
            chol_factor <= '0;
        end else begin
            done <= 1'b0;
            if (start) begin
                chol_factor <= qr_factor - (qr_factor >>> 3);
                latency     <= 2'd2;
            end else if (latency != 2'd0) begin
                latency <= latency - 2'd1;
                if (latency == 2'd1) begin
                    done <= 1'b1;
                end
            end
        end
    end
endmodule

module trisolve_unit #(
    parameter DATA_W = 32,
    parameter N_STATE = 6,
    parameter N_MEAS  = 3
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire signed [DATA_W-1:0] chol_factor,
    input  wire signed [DATA_W-1:0] x_prior [0:N_STATE-1],
    input  wire signed [DATA_W-1:0] z_mean  [0:N_MEAS-1],
    input  wire signed [DATA_W-1:0] z_obs   [0:N_MEAS-1],
    output reg  done,
    output reg  signed [DATA_W-1:0] x_post [0:N_STATE-1]
);
    reg [2:0] latency;
    reg signed [DATA_W-1:0] innovation [0:N_MEAS-1];
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done    <= 1'b0;
            latency <= 3'd0;
            for (i = 0; i < N_STATE; i = i + 1) begin
                x_post[i] <= '0;
            end
        end else begin
            done <= 1'b0;
            if (start) begin
                for (i = 0; i < N_MEAS; i = i + 1) begin
                    innovation[i] <= z_obs[i] - z_mean[i];
                end

                x_post[0] <= x_prior[0] + ((z_obs[0] - z_mean[0]) >>> 1);
                x_post[1] <= x_prior[1] + ((z_obs[1] - z_mean[1]) >>> 1);
                x_post[2] <= x_prior[2] + ((z_obs[2] - z_mean[2]) >>> 1);
                x_post[3] <= x_prior[3] + ((z_obs[0] - z_mean[0]) >>> 3) + (chol_factor >>> 6);
                x_post[4] <= x_prior[4] + ((z_obs[1] - z_mean[1]) >>> 3) + (chol_factor >>> 6);
                x_post[5] <= x_prior[5] + ((z_obs[2] - z_mean[2]) >>> 3) + (chol_factor >>> 6);
                latency   <= 3'd3;
            end else if (latency != 3'd0) begin
                latency <= latency - 3'd1;
                if (latency == 3'd1) begin
                    done <= 1'b1;
                end
            end
        end
    end
endmodule

module ukf #(
    parameter N_STATE = 6,
    parameter N_MEAS  = 3,
    parameter N_SIGMA = (2 * N_STATE) + 1,
    parameter DATA_W  = 32,
    parameter FRAC_W  = 16
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire signed [DATA_W-1:0] meas_x,
    input  wire signed [DATA_W-1:0] meas_y,
    input  wire signed [DATA_W-1:0] meas_z,
    output reg  done,
    output reg  busy,
    output reg  [4:0] state_dbg,
    output reg  [7:0] sigma_index_dbg,
    output reg  phase_predict_dbg,
    output reg  phase_update_dbg,
    output reg  signed [DATA_W-1:0] est_x,
    output reg  signed [DATA_W-1:0] est_y,
    output reg  signed [DATA_W-1:0] est_z
);
    localparam S_IDLE             = 5'd0;
    localparam S_META             = 5'd1;
    localparam S_PREDICT_STREAM   = 5'd2;
    localparam S_PREDICT_LATCH    = 5'd3;
    localparam S_PREDICT_SPREAD   = 5'd4;
    localparam S_PREDICT_QR       = 5'd5;
    localparam S_UPDATE_STREAM    = 5'd6;
    localparam S_UPDATE_LATCH     = 5'd7;
    localparam S_UPDATE_SPREAD    = 5'd8;
    localparam S_UPDATE_QR        = 5'd9;
    localparam S_UPDATE_CHOL      = 5'd10;
    localparam S_UPDATE_SOLVE     = 5'd11;
    localparam S_DONE             = 5'd12;

    integer i;
    reg [4:0] state;

    reg wc_start;
    reg cordic_start;
    reg spg_start;
    reg qr_start;
    reg chol_start;
    reg trisolve_start;
    reg wc_ready;
    reg cordic_ready;

    reg spg_mode_h;
    reg state_acc_clear;
    reg meas_acc_clear;
    reg pred_pacc_clear;
    reg meas_pacc_clear;

    reg pred_buf_wr0;
    reg pred_buf_wr1;
    reg meas_buf_wr0;
    reg meas_buf_wr1;

    reg [$clog2(N_SIGMA)-1:0] pred_wr_addr0;
    reg [$clog2(N_SIGMA)-1:0] pred_wr_addr1;
    reg [$clog2(N_SIGMA)-1:0] meas_wr_addr0;
    reg [$clog2(N_SIGMA)-1:0] meas_wr_addr1;
    reg [$clog2(N_SIGMA)-1:0] pred_rd_addr;
    reg [$clog2(N_SIGMA)-1:0] meas_rd_addr;
    reg [$clog2(N_SIGMA)-1:0] pred_wr_ptr;
    reg [$clog2(N_SIGMA)-1:0] meas_wr_ptr;

    reg signed [DATA_W-1:0] x_state [0:N_STATE-1];
    reg signed [DATA_W-1:0] x_prior [0:N_STATE-1];
    reg signed [DATA_W-1:0] z_mean [0:N_MEAS-1];
    reg signed [DATA_W-1:0] z_obs [0:N_MEAS-1];
    reg signed [DATA_W-1:0] spg_base_state [0:N_STATE-1];

    wire wc_done;
    wire signed [DATA_W-1:0] w0m_half;
    wire signed [DATA_W-1:0] w0c_full;
    wire signed [DATA_W-1:0] wi;

    wire cordic_done;
    wire signed [DATA_W-1:0] sigma_spread;

    wire spg_valid;
    wire spg_done;
    wire spg_center_pair;
    wire [7:0] spg_pair_index;
    wire signed [DATA_W-1:0] spg_sigma_a [0:N_STATE-1];
    wire signed [DATA_W-1:0] spg_sigma_b [0:N_STATE-1];

    wire fabric_valid;
    wire fabric_center_pair;
    wire [7:0] fabric_pair_index;
    wire signed [DATA_W-1:0] fabric_vec_a [0:N_STATE-1];
    wire signed [DATA_W-1:0] fabric_vec_b [0:N_STATE-1];

    reg signed [DATA_W-1:0] state_weight_a;
    reg signed [DATA_W-1:0] state_weight_b;
    reg state_acc_en;

    reg signed [DATA_W-1:0] meas_weight_a;
    reg signed [DATA_W-1:0] meas_weight_b;
    reg meas_acc_en;

    wire signed [DATA_W-1:0] state_mean_sum [0:N_STATE-1];
    wire signed [DATA_W-1:0] meas_mean_sum_full [0:N_STATE-1];

    wire signed [DATA_W-1:0] pred_rd_vec [0:N_STATE-1];
    wire signed [DATA_W-1:0] meas_rd_vec_full [0:N_STATE-1];
    wire signed [DATA_W-1:0] meas_rd_vec [0:N_MEAS-1];
    reg  signed [DATA_W-1:0] pred_pacc_weight;
    reg  signed [DATA_W-1:0] meas_pacc_weight;
    reg  pred_pacc_en;
    reg  meas_pacc_en;
    wire signed [DATA_W-1:0] pred_spread_metric;
    wire signed [DATA_W-1:0] meas_spread_metric;
    reg  signed [DATA_W-1:0] qr_metric_in;

    wire qr_done;
    wire signed [DATA_W-1:0] qr_factor;
    wire chol_done;
    wire signed [DATA_W-1:0] chol_factor;
    wire trisolve_done;
    wire signed [DATA_W-1:0] x_post [0:N_STATE-1];

    wc_unit #(
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W)
    ) wc_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(wc_start),
        .done(wc_done),
        .w0m_half(w0m_half),
        .w0c_full(w0c_full),
        .wi(wi)
    );

    cordic_unit #(
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W),
        .N_DIM(N_STATE)
    ) cordic_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(cordic_start),
        .state_in(x_state),
        .done(cordic_done),
        .spread(sigma_spread)
    );

    spg_unit #(
        .DATA_W(DATA_W),
        .N_DIM(N_STATE)
    ) spg_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(spg_start),
        .base_state(spg_base_state),
        .spread(sigma_spread),
        .valid(spg_valid),
        .done(spg_done),
        .center_pair(spg_center_pair),
        .pair_index(spg_pair_index),
        .sigma_a(spg_sigma_a),
        .sigma_b(spg_sigma_b)
    );

    shared_compute_fabric #(
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W),
        .N_IN(N_STATE),
        .N_OUT(N_STATE)
    ) fabric_inst (
        .clk(clk),
        .rst_n(rst_n),
        .valid_i(spg_valid),
        .mode_h(spg_mode_h),
        .center_pair_i(spg_center_pair),
        .pair_index_i(spg_pair_index),
        .sigma_a_i(spg_sigma_a),
        .sigma_b_i(spg_sigma_b),
        .valid_o(fabric_valid),
        .center_pair_o(fabric_center_pair),
        .pair_index_o(fabric_pair_index),
        .vec_a_o(fabric_vec_a),
        .vec_b_o(fabric_vec_b)
    );

    uacc_unit #(
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W),
        .N_DIM(N_STATE)
    ) state_uacc (
        .clk(clk),
        .rst_n(rst_n),
        .clear(state_acc_clear),
        .en(state_acc_en),
        .weight_a(state_weight_a),
        .weight_b(state_weight_b),
        .vec_a(fabric_vec_a),
        .vec_b(fabric_vec_b),
        .sum_o(state_mean_sum)
    );

    uacc_unit #(
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W),
        .N_DIM(N_STATE)
    ) meas_uacc (
        .clk(clk),
        .rst_n(rst_n),
        .clear(meas_acc_clear),
        .en(meas_acc_en),
        .weight_a(meas_weight_a),
        .weight_b(meas_weight_b),
        .vec_a(fabric_vec_a),
        .vec_b(fabric_vec_b),
        .sum_o(meas_mean_sum_full)
    );

    sigma_buffer #(
        .DATA_W(DATA_W),
        .N_DIM(N_STATE),
        .DEPTH(N_SIGMA)
    ) pred_buffer (
        .clk(clk),
        .wr_en0(pred_buf_wr0),
        .wr_addr0(pred_wr_addr0),
        .wr_vec0(fabric_vec_a),
        .wr_en1(pred_buf_wr1),
        .wr_addr1(pred_wr_addr1),
        .wr_vec1(fabric_vec_b),
        .rd_addr(pred_rd_addr),
        .rd_vec(pred_rd_vec)
    );

    sigma_buffer #(
        .DATA_W(DATA_W),
        .N_DIM(N_STATE),
        .DEPTH(N_SIGMA)
    ) meas_buffer (
        .clk(clk),
        .wr_en0(meas_buf_wr0),
        .wr_addr0(meas_wr_addr0),
        .wr_vec0(fabric_vec_a),
        .wr_en1(meas_buf_wr1),
        .wr_addr1(meas_wr_addr1),
        .wr_vec1(fabric_vec_b),
        .rd_addr(meas_rd_addr),
        .rd_vec(meas_rd_vec_full)
    );

    pacc_unit #(
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W),
        .N_DIM(N_STATE)
    ) pred_pacc (
        .clk(clk),
        .rst_n(rst_n),
        .clear(pred_pacc_clear),
        .en(pred_pacc_en),
        .weight(pred_pacc_weight),
        .vec_i(pred_rd_vec),
        .mean_i(x_prior),
        .spread_metric(pred_spread_metric)
    );

    pacc_unit #(
        .DATA_W(DATA_W),
        .FRAC_W(FRAC_W),
        .N_DIM(N_MEAS)
    ) meas_pacc (
        .clk(clk),
        .rst_n(rst_n),
        .clear(meas_pacc_clear),
        .en(meas_pacc_en),
        .weight(meas_pacc_weight),
        .vec_i(meas_rd_vec),
        .mean_i(z_mean),
        .spread_metric(meas_spread_metric)
    );

    qr_unit #(
        .DATA_W(DATA_W)
    ) qr_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(qr_start),
        .spread_metric(qr_metric_in),
        .done(qr_done),
        .qr_factor(qr_factor)
    );

    cholupdate_unit #(
        .DATA_W(DATA_W)
    ) chol_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(chol_start),
        .qr_factor(qr_factor),
        .done(chol_done),
        .chol_factor(chol_factor)
    );

    trisolve_unit #(
        .DATA_W(DATA_W),
        .N_STATE(N_STATE),
        .N_MEAS(N_MEAS)
    ) trisolve_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(trisolve_start),
        .chol_factor(chol_factor),
        .x_prior(x_prior),
        .z_mean(z_mean),
        .z_obs(z_obs),
        .done(trisolve_done),
        .x_post(x_post)
    );

    generate
        genvar gm;
        for (gm = 0; gm < N_MEAS; gm = gm + 1) begin : GEN_MEAS_SLICE
            assign meas_rd_vec[gm] = meas_rd_vec_full[gm];
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= S_IDLE;
            done              <= 1'b0;
            busy              <= 1'b0;
            state_dbg         <= S_IDLE;
            sigma_index_dbg   <= 8'd0;
            phase_predict_dbg <= 1'b0;
            phase_update_dbg  <= 1'b0;
            est_x             <= '0;
            est_y             <= '0;
            est_z             <= '0;

            wc_start          <= 1'b0;
            cordic_start      <= 1'b0;
            spg_start         <= 1'b0;
            qr_start          <= 1'b0;
            chol_start        <= 1'b0;
            trisolve_start    <= 1'b0;
            wc_ready          <= 1'b0;
            cordic_ready      <= 1'b0;
            spg_mode_h        <= 1'b0;

            state_acc_clear   <= 1'b0;
            meas_acc_clear    <= 1'b0;
            pred_pacc_clear   <= 1'b0;
            meas_pacc_clear   <= 1'b0;

            state_acc_en      <= 1'b0;
            meas_acc_en       <= 1'b0;
            pred_pacc_en      <= 1'b0;
            meas_pacc_en      <= 1'b0;

            pred_buf_wr0      <= 1'b0;
            pred_buf_wr1      <= 1'b0;
            meas_buf_wr0      <= 1'b0;
            meas_buf_wr1      <= 1'b0;

            pred_wr_addr0     <= '0;
            pred_wr_addr1     <= '0;
            meas_wr_addr0     <= '0;
            meas_wr_addr1     <= '0;
            pred_rd_addr      <= '0;
            meas_rd_addr      <= '0;
            pred_wr_ptr       <= '0;
            meas_wr_ptr       <= '0;

            state_weight_a    <= '0;
            state_weight_b    <= '0;
            meas_weight_a     <= '0;
            meas_weight_b     <= '0;
            pred_pacc_weight  <= '0;
            meas_pacc_weight  <= '0;
            qr_metric_in      <= '0;

            z_obs[0]          <= '0;
            z_obs[1]          <= '0;
            z_obs[2]          <= '0;

            x_state[0]        <= 32'sd655360;
            x_state[1]        <= 32'sd327680;
            x_state[2]        <= 32'sd131072;
            x_state[3]        <= 32'sd32768;
            x_state[4]        <= -32'sd16384;
            x_state[5]        <= 32'sd8192;

            for (i = 0; i < N_STATE; i = i + 1) begin
                x_prior[i]        <= '0;
                spg_base_state[i] <= '0;
            end
            for (i = 0; i < N_MEAS; i = i + 1) begin
                z_mean[i] <= '0;
            end
        end else begin
            wc_start        <= 1'b0;
            cordic_start    <= 1'b0;
            spg_start       <= 1'b0;
            qr_start        <= 1'b0;
            chol_start      <= 1'b0;
            trisolve_start  <= 1'b0;
            state_acc_clear <= 1'b0;
            meas_acc_clear  <= 1'b0;
            pred_pacc_clear <= 1'b0;
            meas_pacc_clear <= 1'b0;
            state_acc_en    <= 1'b0;
            meas_acc_en     <= 1'b0;
            pred_pacc_en    <= 1'b0;
            meas_pacc_en    <= 1'b0;
            pred_buf_wr0    <= 1'b0;
            pred_buf_wr1    <= 1'b0;
            meas_buf_wr0    <= 1'b0;
            meas_buf_wr1    <= 1'b0;
            done            <= 1'b0;

            state_dbg       <= state;

            if (wc_done) begin
                wc_ready <= 1'b1;
            end
            if (cordic_done) begin
                cordic_ready <= 1'b1;
            end

            case (state)
                S_IDLE: begin
                    busy              <= 1'b0;
                    phase_predict_dbg <= 1'b0;
                    phase_update_dbg  <= 1'b0;
                    sigma_index_dbg   <= 8'd0;
                    if (start) begin
                        busy              <= 1'b1;
                        wc_ready          <= 1'b0;
                        cordic_ready      <= 1'b0;
                        z_obs[0]          <= meas_x;
                        z_obs[1]          <= meas_y;
                        z_obs[2]          <= meas_z;
                        wc_start          <= 1'b1;
                        cordic_start      <= 1'b1;
                        state             <= S_META;
                    end
                end

                S_META: begin
                    if (wc_ready && cordic_ready) begin
                        phase_predict_dbg <= 1'b1;
                        phase_update_dbg  <= 1'b0;
                        spg_mode_h        <= 1'b0;
                        state_acc_clear   <= 1'b1;
                        pred_wr_ptr       <= '0;
                        for (i = 0; i < N_STATE; i = i + 1) begin
                            spg_base_state[i] <= x_state[i];
                        end
                        spg_start <= 1'b1;
                        state     <= S_PREDICT_STREAM;
                    end
                end

                S_PREDICT_STREAM: begin
                    if (fabric_valid) begin
                        sigma_index_dbg <= fabric_pair_index;
                        if (fabric_center_pair) begin
                            state_weight_a <= w0m_half;
                            state_weight_b <= w0m_half;
                            pred_buf_wr0   <= 1'b1;
                            pred_wr_addr0  <= pred_wr_ptr;
                            pred_wr_ptr    <= pred_wr_ptr + 1'b1;
                        end else begin
                            state_weight_a <= wi;
                            state_weight_b <= wi;
                            pred_buf_wr0   <= 1'b1;
                            pred_buf_wr1   <= 1'b1;
                            pred_wr_addr0  <= pred_wr_ptr;
                            pred_wr_addr1  <= pred_wr_ptr + 1'b1;
                            pred_wr_ptr    <= pred_wr_ptr + 2'd2;
                        end
                        state_acc_en <= 1'b1;
                    end

                    if (fabric_valid && (fabric_pair_index == N_STATE[7:0])) begin
                        state <= S_PREDICT_LATCH;
                    end
                end

                S_PREDICT_LATCH: begin
                    for (i = 0; i < N_STATE; i = i + 1) begin
                        x_prior[i] <= state_mean_sum[i];
                    end
                    pred_pacc_clear <= 1'b1;
                    pred_rd_addr    <= '0;
                    state           <= S_PREDICT_SPREAD;
                end

                S_PREDICT_SPREAD: begin
                    pred_pacc_en <= 1'b1;
                    if (pred_rd_addr == '0) begin
                        pred_pacc_weight <= w0c_full;
                    end else begin
                        pred_pacc_weight <= wi;
                    end

                    if (pred_rd_addr == N_SIGMA - 1) begin
                        qr_metric_in <= pred_spread_metric;
                        qr_start <= 1'b1;
                        state    <= S_PREDICT_QR;
                    end else begin
                        pred_rd_addr <= pred_rd_addr + 1'b1;
                    end
                end

                S_PREDICT_QR: begin
                    if (qr_done) begin
                        phase_predict_dbg <= 1'b0;
                        phase_update_dbg  <= 1'b1;
                        spg_mode_h        <= 1'b1;
                        meas_acc_clear    <= 1'b1;
                        meas_wr_ptr       <= '0;
                        for (i = 0; i < N_STATE; i = i + 1) begin
                            spg_base_state[i] <= x_prior[i];
                        end
                        spg_start <= 1'b1;
                        state     <= S_UPDATE_STREAM;
                    end
                end

                S_UPDATE_STREAM: begin
                    if (fabric_valid) begin
                        sigma_index_dbg <= fabric_pair_index;
                        if (fabric_center_pair) begin
                            meas_weight_a <= w0m_half;
                            meas_weight_b <= w0m_half;
                            meas_buf_wr0  <= 1'b1;
                            meas_wr_addr0 <= meas_wr_ptr;
                            meas_wr_ptr   <= meas_wr_ptr + 1'b1;
                        end else begin
                            meas_weight_a <= wi;
                            meas_weight_b <= wi;
                            meas_buf_wr0  <= 1'b1;
                            meas_buf_wr1  <= 1'b1;
                            meas_wr_addr0 <= meas_wr_ptr;
                            meas_wr_addr1 <= meas_wr_ptr + 1'b1;
                            meas_wr_ptr   <= meas_wr_ptr + 2'd2;
                        end
                        meas_acc_en <= 1'b1;
                    end

                    if (fabric_valid && (fabric_pair_index == N_STATE[7:0])) begin
                        state <= S_UPDATE_LATCH;
                    end
                end

                S_UPDATE_LATCH: begin
                    for (i = 0; i < N_MEAS; i = i + 1) begin
                        z_mean[i] <= meas_mean_sum_full[i];
                    end
                    meas_pacc_clear <= 1'b1;
                    meas_rd_addr    <= '0;
                    state           <= S_UPDATE_SPREAD;
                end

                S_UPDATE_SPREAD: begin
                    meas_pacc_en <= 1'b1;
                    if (meas_rd_addr == '0) begin
                        meas_pacc_weight <= w0c_full;
                    end else begin
                        meas_pacc_weight <= wi;
                    end

                    if (meas_rd_addr == N_SIGMA - 1) begin
                        qr_metric_in <= meas_spread_metric;
                        qr_start <= 1'b1;
                        state    <= S_UPDATE_QR;
                    end else begin
                        meas_rd_addr <= meas_rd_addr + 1'b1;
                    end
                end

                S_UPDATE_QR: begin
                    if (qr_done) begin
                        chol_start <= 1'b1;
                        state      <= S_UPDATE_CHOL;
                    end
                end

                S_UPDATE_CHOL: begin
                    if (chol_done) begin
                        trisolve_start <= 1'b1;
                        state          <= S_UPDATE_SOLVE;
                    end
                end

                S_UPDATE_SOLVE: begin
                    if (trisolve_done) begin
                        for (i = 0; i < N_STATE; i = i + 1) begin
                            x_state[i] <= x_post[i];
                        end
                        est_x <= x_post[0];
                        est_y <= x_post[1];
                        est_z <= x_post[2];
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    if (!start) begin
                        phase_update_dbg <= 1'b0;
                        state            <= S_IDLE;
                    end
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end
endmodule
