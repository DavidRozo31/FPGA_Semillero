module uart_tx_stub #(
    parameter integer CLKS_PER_BIT = 16
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       start,
    input  wire [7:0] data,
    output reg        tx,
    output reg        busy
);

reg [3:0] bit_index;
reg [7:0] shift_reg;
reg [15:0] clk_count;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        tx        <= 1'b1;
        busy      <= 1'b0;
        bit_index <= 4'd0;
        shift_reg <= 8'd0;
        clk_count <= 16'd0;
    end else begin
        if (!busy) begin
            tx <= 1'b1;

            if (start) begin
                busy      <= 1'b1;
                bit_index <= 4'd0;
                shift_reg <= data;
                clk_count <= 16'd0;
                tx        <= 1'b0;
            end
        end else begin
            if (clk_count == CLKS_PER_BIT - 1) begin
                clk_count <= 16'd0;

                if (bit_index < 4'd8) begin
                    tx        <= shift_reg[0];
                    shift_reg <= {1'b0, shift_reg[7:1]};
                    bit_index <= bit_index + 4'd1;
                end else if (bit_index == 4'd8) begin
                    tx        <= 1'b1;
                    bit_index <= bit_index + 4'd1;
                end else begin
                    busy <= 1'b0;
                    tx   <= 1'b1;
                end
            end else begin
                clk_count <= clk_count + 16'd1;
            end
        end
    end
end

endmodule
