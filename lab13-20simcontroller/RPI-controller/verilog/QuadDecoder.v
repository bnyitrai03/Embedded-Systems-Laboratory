module QuadDecoder (
    input clk,
    input A,
    input B,
    input rst,

    output reg signed [15:0] count
);

    localparam S00 = 2'b00,
               S01 = 2'b01,
               S10 = 2'b10,
               S11 = 2'b11;

    reg [1:0] state = S00;
    reg [1:0] sync = 2'b00;
    reg [1:0] AB = 2'b00;

    // Two-stage input synchronizer (A and B are in a different clock domain)
    always @(posedge clk) begin
        sync <= {A, B};
        AB <= sync;
    end

    // State machine: direction and count update
    always @(posedge clk) begin
        if (rst) begin
            count <= 0;
            state <= S00;
        end else begin
            case (state)
                S00: begin
                    if (AB == 2'b01) begin
                        count <= count - 1;
                        state <= S01;
                    end
                    else if (AB == 2'b10) begin
                        count <= count + 1;
                        state <= S10;
                    end
                end
                S01: begin
                    if (AB == 2'b00) begin
                        count <= count + 1;
                        state <= S00;
                    end
                    else if (AB == 2'b11) begin
                        count <= count - 1;
                        state <= S11;
                    end
                end
                S10: begin
                    if (AB == 2'b00) begin
                        count <= count - 1;
                        state <= S00;
                    end
                    else if (AB == 2'b11) begin
                        count <= count + 1;
                        state <= S11;
                    end
                end
                S11: begin
                    if (AB == 2'b01) begin
                        count <= count + 1;
                        state <= S01;
                    end
                    else if (AB == 2'b10) begin
                        count <= count - 1;
                        state <= S10;
                    end
                end
            endcase
        end
    end

endmodule
