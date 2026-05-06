module QuadDecoder (
    input FPGA_CLK1_50,
    input [3:0] SW,

    output [7:0] LED
);

    localparam S00 = 2'b00,
               S01 = 2'b01,
               S10 = 2'b10,
               S11 = 2'b11;
					
	 // mapped signals
	 wire rst = SW[0];
	 wire A = SW[1];
	 wire B = SW[2];
	 
	 reg dir = 0;
	 reg [6:0] count = 0;
     reg [1:0] state = S00;
     reg [1:0] sync = 2'b00;
     reg [1:0] AB = 2'b00;

    // Two-stage input synchronizer (A and B are in a different clock domain)
    always @(posedge FPGA_CLK1_50) begin
        sync <= {A, B};
        AB <= sync;
    end

    // State machine: direction and count update
    always @(posedge FPGA_CLK1_50) begin
        if (rst) begin
            count <= 0;
            state <= S00;
            dir <= 1'b0;
        end else begin
            case (state)
                S00: begin
                    if (AB == 2'b01) begin
                        count <= count - 1;
                        state <= S01;
                        dir <= 1'b0;
                    end
                    else if (AB == 2'b10) begin
                        count <= count + 1;
                        state <= S10;
                        dir <= 1'b1;
                    end
                end
                S01: begin
                    if (AB == 2'b00) begin
                        count <= count + 1;
                        state <= S00;
                        dir <= 1'b1;
                    end
                    else if (AB == 2'b11) begin
                        count <= count - 1;
                        state <= S11;
                        dir <= 1'b0;
                    end
                end
                S10: begin
                    if (AB == 2'b00) begin
                        count <= count - 1;
                        state <= S00;
                        dir <= 1'b0;
                    end
                    else if (AB == 2'b11) begin
                        count <= count + 1;
                        state <= S11;
                        dir <= 1'b1;
                    end
                end
                S11: begin
                    if (AB == 2'b01) begin
                        count <= count + 1;
                        state <= S01;
                        dir <= 1'b1;
                    end
                    else if (AB == 2'b10) begin
                        count <= count - 1;
                        state <= S10;
                        dir <= 1'b0;
                    end
                end
            endcase
        end
    end
	 
	 // pin mapping
	 assign LED[6:0] = count[6:0];
	 assign LED[7] = dir;

endmodule