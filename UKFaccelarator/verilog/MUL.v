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