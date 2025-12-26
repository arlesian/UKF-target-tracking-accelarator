module ukf #(
    parameter N_STATE = 6,   // spherical coords for position and velocity
    parameter N_SIGMA = 2*N_STATE + 1,
    parameter DATA_W  = 32   // fixed-point width
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Control interface
    input  wire                 start,
    output wire                 done,

    // (Optional) debug outputs
    output wire [3:0]           state_dbg
);
    typedef enum logic [3:0] {
        S_IDLE          = 4'd0,
        S_GEN_SIGMA     = 4'd1,
        S_PROPAGATE     = 4'd2,
        S_MEAN_COV      = 4'd3,
        S_MEAS_TRANS    = 4'd4,
        S_INNOVATION    = 4'd5,
        S_UPDATE        = 4'd6,
        S_DONE          = 4'd7
    } ukf_state_t;

    ukf_state_t state, state_next;
    
endmodule