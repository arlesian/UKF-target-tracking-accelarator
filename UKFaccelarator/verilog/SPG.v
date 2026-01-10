module SPG #(
    parameter INT_BITS     = 8, // including sign bit
    parameter FRAC_BITS    = 24,
    parameter N_STATE      = 6,
    parameter DATA_W       = 32
) (
    input  wire                     clk,
    input  wire                     rstn,
    input  wire                     en,

    input  wire [DATA_W-1:0]         gamma,
    input  wire [N_STATE-1:0][DATA_W-1:0] x,   // latched once
    input  wire [N_STATE-1:0][DATA_W-1:0] s,   // one column per cycle

    output reg                      valid,
    output reg  [N_STATE-1:0][DATA_W-1:0] sp1,
    output reg  [N_STATE-1:0][DATA_W-1:0] sp2
);
    // --------------------------------------------------
    // this module computes the sigma points using the previous state esimate x and cholesky factor s
    // sp1 = x + gamma * s
    // sp2 = x - gamma * s
    // input:
    //   gamma: scaling factor
    //   x:     state vector (latched once at start)
    //   s:     one column of the state transition matrix per cycle
    // output:
    //   sp1:   computed sigma point 1
    //   sp2:   computed sigma point 2
    // --------------------------------------------------

    // --------------------------------------------------
    // FSM
    // --------------------------------------------------
    localparam IDLE = 1'd0;
    localparam RUN  = 1'd1;

    reg state;

    // --------------------------------------------------
    // Registers
    // --------------------------------------------------
    reg [N_STATE-1:0][DATA_W-1:0] x_reg;

    // --------------------------------------------------
    // Datapath signals
    // --------------------------------------------------
    wire [N_STATE-1:0][DATA_W-1:0] gamma_s;

    genvar i;
    generate
        for (i = 0; i < N_STATE; i = i + 1) begin : GEN_MULT
            MUL #(
                .DATA_W(DATA_W),
                .INT_BITS(INT_BITS),
                .FRAC_BITS(FRAC_BITS)
            ) mul_inst (
                .a(gamma),
                .b(s[i]),
                .p(gamma_s[i])
            );
        end
    endgenerate


    // --------------------------------------------------
    // State transition logic
    // --------------------------------------------------
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state   <= IDLE;
            col_cnt <= 0;
        end else begin
            case (state)
                IDLE: begin
                    if (en) begin
                        x_reg   <= x;
                        state   <= RUN;
                    end
                end

                RUN: begin
                    if (!en) begin
                        state <= IDLE;
                    end
                end

            endcase
        end
    end

    // --------------------------------------------------
    // Output logic
    // --------------------------------------------------
    always @(posedge clk) begin
        if (state == RUN) begin
            valid <= 1'b1;
            for (int j = 0; j < N_STATE; j = j + 1) begin
                sp1[j] <= x_reg[j] + gamma_s[j];
                sp2[j] <= x_reg[j] - gamma_s[j];
            end
        end else begin
            valid <= 1'b0;
            for (int j = 0; j < N_STATE; j = j + 1) begin
                sp1[j] <= 0;
                sp2[j] <= 0;
            end
        end
    end

endmodule
