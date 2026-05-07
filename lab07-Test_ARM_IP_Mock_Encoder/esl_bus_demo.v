`timescale 1 ps / 1 ps

module esl_bus_demo #(
        parameter LED_WIDTH  = 8,
        parameter DATA_WIDTH = 32
    ) (
        input  wire [7:0]                slave_address,      // avs_s0.address
        input  wire                      slave_read,         //       .read
        output reg  [DATA_WIDTH-1:0]     slave_readdata,     //       .readdata
        input  wire                      slave_write,        //       .write
        input  wire [DATA_WIDTH-1:0]     slave_writedata,    //       .writedata
        input  wire                      clk,                // clock.clk
        input  wire                      reset,              // reset.reset
        input  wire [(DATA_WIDTH/8)-1:0] slave_byteenable,

        input  wire [3:0]                user_input,         // SW[3:0]
        output wire [LED_WIDTH-1:0]      user_output         // LED[7:0]
    );

    // ------------------------------------------------------------
    // Switch mapping
    // ------------------------------------------------------------
    wire quad_rst;
    wire quad_A;
    wire quad_B;

    assign quad_rst   = user_input[0];   // SW[0] = reset
    assign quad_A     = user_input[1];   // SW[1] = A
    assign quad_B     = user_input[2];   // SW[2] = B

    // ------------------------------------------------------------
    // QuadDecoder outputs
    // ------------------------------------------------------------
    wire [6:0] quad_count;
    wire       quad_dir;

    // ------------------------------------------------------------
    // Decoder instance
    // ------------------------------------------------------------
    esl_bus_demo_example encoder_inst (
        .clk   (clk),
        .A     (quad_A),
        .B     (quad_B),
        .rst   (quad_rst),
        .count (quad_count),
        .dir   (quad_dir)
    );

    // ------------------------------------------------------------
    // LED mapping
    //
    // LED[6:0] = counter value
    // LED[7]   = direction
    // ------------------------------------------------------------
    assign user_output[6:0] = quad_count;
    assign user_output[7]   = quad_dir;

    // ------------------------------------------------------------
    // Avalon read/write logic
    //
    // ------------------------------------------------------------
    always @(posedge clk or posedge reset) begin
      if (reset) begin
        slave_readdata <= 32'b0;
      end else if (slave_read) begin
        slave_readdata <= {24'b0, quad_dir, quad_count};
      end
    end

endmodule