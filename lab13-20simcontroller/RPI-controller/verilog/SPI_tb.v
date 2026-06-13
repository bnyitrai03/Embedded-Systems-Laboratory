`timescale 1ns / 1ps

module SPI_tb;

  // Clock and reset
  reg clk = 0;
  reg rst = 1;

  // SPI signals
  reg spi_clk = 0;
  reg spi_pico = 0;
  reg spi_cs = 1;
  wire spi_poci;

  // Encoder inputs
  reg signed [15:0] pitch_encoder_count = 0;
  reg signed [15:0] yaw_encoder_count = 0;

  // Motor outputs
  wire [13:0] pitch_pwm_duty;
  wire        pitch_dir;
  wire        pitch_pwm_enable;

  wire [13:0] yaw_pwm_duty;
  wire        yaw_dir;
  wire        yaw_pwm_enable;

  // Test variables
  reg [31:0] rx_word;
  reg [15:0] yaw_cmd;
  reg [15:0] pitch_cmd;

  SPI dut (
    .clk(clk),
    .rst(rst),
    .spi_clk(spi_clk),
    .spi_pico(spi_pico),
    .spi_cs(spi_cs),
    .pitch_encoder_count(pitch_encoder_count),
    .yaw_encoder_count(yaw_encoder_count),
    .pitch_pwm_duty(pitch_pwm_duty),
    .pitch_dir(pitch_dir),
    .pitch_pwm_enable(pitch_pwm_enable),
    .yaw_pwm_duty(yaw_pwm_duty),
    .yaw_dir(yaw_dir),
    .yaw_pwm_enable(yaw_pwm_enable),
    .spi_poci(spi_poci)
  );

  always #5 clk = ~clk;

  task spi_transfer32;
    input  [31:0] tx_word;
    output [31:0] rx_word_out;
    integer i;
    begin
      rx_word_out = 0;

      spi_cs = 1;
      spi_clk = 0;
      #100;

      spi_cs = 0;   // start frame
      #100;

      for (i = 31; i >= 0; i = i - 1) begin
        spi_pico = tx_word[i];   // put next MOSI bit

        #50;
        spi_clk = 1;             // rising edge: sample MISO
        #50;
        rx_word_out[i] = spi_poci;

        #50;
        spi_clk = 0;             // falling edge: slave shifts next bit
        #50;
      end

      spi_cs = 1;   // end frame
      #100;
    end
  endtask

  initial begin
    $dumpfile("SPI_tb.vcd");
    $dumpvars(0, SPI_tb);

    // Reset
    #50;
    rst = 1;
    #100;
    rst = 0;
    #100;

    // Test: read encoder values
    pitch_encoder_count = -25;
    yaw_encoder_count   =  100;
    spi_transfer32(0, rx_word);
    $display("Expected yaw = %0d", yaw_encoder_count);
    $display("Expected pitch = %0d", pitch_encoder_count);
    $display("SPI read: pitch = %0d yaw = %0d", $signed(rx_word[15:0]), $signed(rx_word[31:16]));

    // Test: send motor command
    // yaw = dir 1, enable 1, duty 1000
    // pitch = dir 0, enable 1, duty 500
    yaw_cmd   = {1'b1, 1'b1, 14'd1000};
    pitch_cmd = {1'b0, 1'b1, 14'd500};
    spi_transfer32({yaw_cmd, pitch_cmd}, rx_word);
    $display("Expected values:");
    $display("yaw_dir = %0d, yaw_enable = %0d, yaw_duty = %0d", 1, 1, 1000);
    $display("pitch_dir = %0d, pitch_enable = %0d, pitch_duty = %0d", 0, 1, 500);
    $display("Simulated values:");
    $display("yaw_dir = %0d, yaw_enable = %0d, yaw_duty = %0d", yaw_dir, yaw_pwm_enable, yaw_pwm_duty);
    $display("pitch_dir = %0d, pitch_enable = %0d, pitch_duty = %0d", pitch_dir, pitch_pwm_enable, pitch_pwm_duty);

    #100;
    $finish;
  end

endmodule