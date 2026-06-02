module fsm_led (
    input  wire clk,
    input  wire rst,
    output reg  led
);

always @(posedge clk or posedge rst) begin
    if (rst)
        led <= 1'b0;
    else
        led <= ~led;
end

endmodule
