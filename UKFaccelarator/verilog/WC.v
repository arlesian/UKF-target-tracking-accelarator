module WC #(
    parameter N_STATE = 6,
    parameter DATA_W  = 32,
    parameter INT_BITS = 2, // unsigned, so no sign bit
    parameter FRAC_BITS = 30 
 ) (
    input  wire                      clk,
    input  wire                      rstn,
    input  wire                      en,
    input  wire            [DATA_W-1:0] alpha,
    input  wire            [DATA_W-1:0] beta,
    input  wire            [DATA_W-1:0] kappa,

    output wire            [DATA_W-1:0] w0m,
    output wire            [DATA_W-1:0] w0c,
    output wire            [DATA_W-1:0] w
    );

    // --------------------------------------------------
    // weight calculations
    // ** This WC assumes two-lane sigma point calculation **************************************
    // ** x_0 is passed onto two lanes simultaneously, thus to prevent duplicated accumulation **
    // ** w_0(c) and w_0(m) are pre-divided by 2 ************************************************
    // L = N_STATE, lambda = alpha^2 * (L + kappa) - L
    // W_0(m) = lambda / 2 * (L + lambda) // divided by 2 because x_0 is added twice
    // W_0(c) = lambda / 2 * (L + lambda) + (1 - alpha^2 + beta)/2 // divided by 2 because x_0 is added twice
    // W_i(m) = W_i(c) = 1 / (2 * (L + lambda)) for i = 1,...,2L
    // INV and DIV take long, so use fsm
    // --------------------------------------------------

    localparam IDLE     = 3'b0;
    localparam CALC_A   = 3'b1; // A = alpha^2
    localparam CALC_B   = 3'b2; // B = L+k
    localparam CALC_C   = 3'b3; // C = A * B
    localparam CALC_w   = 3'b4; // D = INV(C), lambda = C - L
    localparam CALC_w0m = 3'b5; // E = lambda * D, F = (1 - A + beta) / 2
    localparam CALC_w0c = 3'b6; // w0c = E + F

    reg [DATA_W-1:0] A;
    reg [DATA_W-1:0] B;
    reg [DATA_W-1:0] C;
    reg [DATA_W-1:0] D;
    reg [DATA_W-1:0] E;
    reg [DATA_W-1:0] F;
    reg [DATA_W-1:0] lambda;

    reg [DATA_W-1:0] mul_a;
    reg [DATA_W-1:0] mul_b;
    reg [DATA_W-1:0] p;
    reg [DATA_W-1:0] dividend;
    reg [DATA_W-1:0] inv;

    reg [2:0]        state;

    wire             done;
    wire             start;

    assign start = ( state == CALC_w );

    MUL #(
        .DATA_W(DATA_W)
        .INT_BITS(INT_BITS),
        .FRAC_BITS(FRAC_BITS)
    ) wc_mul (
        .a(mul_a),
        .b(mul_b),
        .p(p)
    )

    INV #(
        .DATA_W(DATA_W)
        .INT_BITS(INT_BITS),
        .FRAC_BITS(FRAC_BITS)
    ) wc_inv (
        .clk(clk),
        .rstn(rstn),
        .start(start),
        .dd(dividend),

        .q(inv),
        .done(done)
    )



    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state <= IDLE;
        end else begin
            case (state)
                IDLE: begin
                    if (en)
                        state <= CALC_A;
                end
                CALC_A: begin
                    mul_a <= alpha;
                    mul_b <= alpha;
                    A <= p;
                    state <= CALC_B
                end
                CALC_B: begin
                    B <= N_STATE + kappa;
                    state <= CALC_C
                end
                CALC_C: begin
                    mul_a <= A;
                    mul_b <= B;
                    C <= p;
                    state <= CALC_w;
                end
                CALC_w: begin
                    dividend <= C;
                    D <= inv;
                    lambda <= C - L;
                    if (done) begin
                        state <= CALC_w0m;
                    end else begin
                        state <= CALC_w
                    end
                end
                CALC_w0m: begin
                    mul_a <= lambda;
                    mul_b <= D;
                    E <= p;
 // NEED: finish fsm logic
                end
                    


            endcase
        end
    end

endmodule