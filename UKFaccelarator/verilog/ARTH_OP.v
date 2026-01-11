module MUL #(
    parameter DATA_W = 32,
    parameter INT_BITS = 8, // including sign bit
    parameter FRAC_BITS = 24
 ) (
     input  wire [DATA_W-1:0]         a,
     input  wire [DATA_W-1:0]         b,

     output wire [DATA_W-1:0]         p
 );
 
    // --------------------------------------------------
    // multiplier
    // --------------------------------------------------
    wire signed [2*DATA_W-1:0] mult_full;
    assign mult_full = $signed(a) * $signed(b); // Q8.24 x Q8.24 = Q16.48
    assign p = mult_full >>> FRAC_BITS; // truncate to Q8.24
    // NOTE: may have to add saturation logic
    // NOTE: may be better to pipeline this multiplication or make a separate module
endmodule

// NOTE: no need for div; just inv -> mul

// multi-clk inv module using the newton-raphson method(combinatorial delay is too big)
// x_k+1 = x_k * ( 2 - dd * x_k ) 
module INV #(
    parameter DATA_W    = 32,
    parameter INT_BITS  = 8,
    parameter FRAC_BITS = 24,
    parameter N_ITER    = 3
)(
    input  wire                 clk,
    input  wire                 rstn,
    input  wire                 start,
    input  wire [DATA_W-1:0]    dd,     // Q8.24, positive

    output reg  [DATA_W-1:0]    q,      // Q8.24
    output reg                  done
);

    localparam IDLE  = 3'd0;
    localparam INIT  = 3'd1;
    localparam MUL1  = 3'd2;
    localparam SUB   = 3'd3;
    localparam MUL2  = 3'd4;
    localparam CHECK = 3'd5;
    localparam DONE  = 3'd6;

    reg [2:0] state;
    reg [1:0] iter;

    reg [DATA_W-1:0] x;      // current estimate
    reg [DATA_W-1:0] t1;     // d*x
    reg [DATA_W-1:0] t2;     // 2 - d*x

    wire signed [2*DATA_W-1:0] mult_full;

    // shared multiplier
    // could use MUL, but using full mult result is better here
    assign mult_full = $signed(dd) * $signed(x);

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state <= IDLE;
            done  <= 1'b0;
        end else begin
            case (state)

                IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        iter  <= 0;
                        state <= INIT;
                    end
                end

                INIT: begin
                    x     <= (1 << FRAC_BITS); // x0 = 1.0
                    state <= MUL1;
                end

                MUL1: begin
                    t1    <= mult_full >>> FRAC_BITS; // d*x
                    state <= SUB;
                end

                SUB: begin
                    t2    <= (2 << FRAC_BITS) - t1;
                    state <= MUL2;
                end

                MUL2: begin
                    x     <= ($signed(x) * $signed(t2)) >>> FRAC_BITS;
                    state <= CHECK;
                end

                CHECK: begin
                    if (iter == N_ITER-1)
                        state <= DONE;
                    else begin
                        iter  <= iter + 1;
                        state <= MUL1;
                    end
                end

                DONE: begin
                    q    <= x;
                    done <= 1'b1;
                    if (!start)
                        state <= IDLE;
                end

            endcase
        end
    end

endmodule
