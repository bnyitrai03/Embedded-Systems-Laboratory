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

        input  wire                      pitch_enc_a,
        input  wire                      pitch_enc_b,
        input  wire                      yaw_enc_a,
        input  wire                      yaw_enc_b,
        input wire                       reset_button
    );

    // ------------------------------------------------------------
    // Encoder values declared
    // ------------------------------------------------------------
    wire [6:0] pitch_count;
    wire       pitch_dir;
    wire [6:0] yaw_count;
    wire       yaw_dir;

    // ------------------------------------------------------------
    // Decoder instance
    // ------------------------------------------------------------

    esl_bus_demo_example pitch_decoder (
      .clk(clk),
      .A(pitch_enc_a),
      .B(pitch_enc_b),
      .rst(reset_button),
      .count(pitch_count),
      .dir(pitch_dir)
   );

    esl_bus_demo_example yaw_decoder (
      .clk(clk),
      .A(yaw_enc_a),
      .B(yaw_enc_b),
      .rst(reset_button),
      .count(yaw_count),
      .dir(yaw_dir)
    );

    // ------------------------------------------------------------
    // Avalon read logic
    //
    // ------------------------------------------------------------
    always @(posedge clk or posedge reset) begin
      if (reset) begin
        slave_readdata <= 32'b0;
      end else if (slave_read) begin
        slave_readdata <= {16'b0, yaw_dir, yaw_count, pitch_dir, pitch_count};
      end
    end

endmodule