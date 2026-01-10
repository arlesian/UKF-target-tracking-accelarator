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
    // L = N_STATE, lambda = alpha^2 * (L + kappa) - L
    // W_0(m) = lambda / 2 * (L + lambda) // divided by 2 because x_0 is added twice
    // W_0(c) = lambda / 2 * (L + lambda) + (1 - alpha^2 + beta) // divided by 2 because x_0 is added twice
    // W_i(m) = W_i(c) = 1 / (2 * (L + lambda)) for i = 1,...,2L
    // --------------------------------------------------

endmodule