module uACC #(
    parameter DATA_W    = 32,
    parameter INT_BITS  = 8,
    parameter FRAC_BITS = 24,
    parameter N_STATE   = 6,
)(
    input  wire                         clk,
    input  wire                         rstn,
    input  wire                         en,

    input  wire [DATA_W-1:0]            w,      // weight
    input  wire [N_STATE-1:0][DATA_W-1:0] x_in1, // sigma point
    input  wire [N_STATE-1:0][DATA_W-1:0] x_in2, // sigma point

    output reg  [N_STATE-1:0][DATA_W-1:0] sum,  // accumulated mean
    output reg                            valid // indicates sum is valid
);

    // --------------------------------------------------
    // MUL outputs: w * x_in[j]
    // --------------------------------------------------
    wire [N_STATE-1:0][DATA_W-1:0] wx;
    reg  [N_STATE-1:0][DATA_W+8-1:0] acc; // wider accumulator to prevent overflow

    genvar i;
    generate
        for (i = 0; i < N_STATE; i = i + 1) begin : GEN_MUL
            MUL #(
                .DATA_W(DATA_W),
                .INT_BITS(INT_BITS),
                .FRAC_BITS(FRAC_BITS)
            ) mul_inst (
                .a(w),
                .b(x_in[i]),
                .p(wx[i])
            );
        end
    endgenerate

    // --------------------------------------------------
    // Accumulation logic
    // --------------------------------------------------
    integer j;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            valid <= 1'b0;
            for (j = 0; j < N_STATE; j = j + 1)
                acc[j] <= '0;
        end else begin
            if (en) begin
                for (j = 0; j < N_STATE; j = j + 1)
                    acc[j] <= acc[j] + wx[j];
                valid <= 1'b1;
            end else begin
                // reset accumulator when en deasserts
                valid <= 1'b0;
                for (j = 0; j < N_STATE; j = j + 1)
                    acc[j] <= '0;
            end
        end
    end

    assign sum = acc[DATA_W-1:0]; // truncate accumulator to DATA_W bits

endmodule
