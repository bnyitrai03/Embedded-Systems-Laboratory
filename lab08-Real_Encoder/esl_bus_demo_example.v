`timescale 1 ps / 1 ps

module esl_bus_demo_example (
    input  wire       clk,
    input  wire       A,
    input  wire       B,
    input  wire       rst,

    output reg signed  [31:0] count,
    output reg        dir
);

    localparam S00 = 2'b00,
               S01 = 2'b01,
               S10 = 2'b10,
               S11 = 2'b11;

    reg [1:0] state;
    reg [1:0] sync;
    reg [1:0] AB;

    // Two-stage input synchronizer
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sync <= 2'b00;
            AB   <= 2'b00;
        end else begin
            sync <= {A, B};
            AB   <= sync;
        end
    end

    // State machine: direction and count update
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            count <= 7'd0;
            state <= S00;
            dir   <= 1'b0;
        end else begin
            case (state)
                S00: begin
                    if (AB == 2'b01) begin
                        count <= count - 1'b1;
                        state <= S01;
                        dir   <= 1'b0;
                    end else if (AB == 2'b10) begin
                        count <= count + 1'b1;
                        state <= S10;
                        dir   <= 1'b1;
                    end
                end

                S01: begin
                    if (AB == 2'b00) begin
                        count <= count + 1'b1;
                        state <= S00;
                        dir   <= 1'b1;
                    end else if (AB == 2'b11) begin
                        count <= count - 1'b1;
                        state <= S11;
                        dir   <= 1'b0;
                    end
                end

                S10: begin
                    if (AB == 2'b00) begin
                        count <= count - 1'b1;
                        state <= S00;
                        dir   <= 1'b0;
                    end else if (AB == 2'b11) begin
                        count <= count + 1'b1;
                        state <= S11;
                        dir   <= 1'b1;
                    end
                end

                S11: begin
                    if (AB == 2'b01) begin
                        count <= count + 1'b1;
                        state <= S01;
                        dir   <= 1'b1;
                    end else if (AB == 2'b10) begin
                        count <= count - 1'b1;
                        state <= S10;
                        dir   <= 1'b0;
                    end
                end

            endcase
        end
    end

endmodule