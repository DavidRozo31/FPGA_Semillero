module fsm_led (
    input  wire clk,
    input  wire rst,
    output reg  led
);

localparam STATE_OFF = 1'b0;
localparam STATE_ON  = 1'b1;

reg state;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state <= STATE_OFF;
        led   <= 1'b0;
    end else begin
        case (state)
            STATE_OFF: begin
                state <= STATE_ON;
                led   <= 1'b0;
            end
            default: begin
                state <= STATE_OFF;
                led   <= 1'b1;
            end
        endcase
    end
end

endmodule
