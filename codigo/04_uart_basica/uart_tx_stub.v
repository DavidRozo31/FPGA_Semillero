module uart_tx_stub (
    input  wire clk,
    input  wire rst,
    input  wire start,
    input  wire [7:0] data,
    output reg  tx,
    output reg  busy
);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        tx   <= 1'b1;
        busy <= 1'b0;
    end else begin
        if (start)
            busy <= 1'b1;
        else
            busy <= 1'b0;

        tx <= data[0];
    end
end

endmodule
