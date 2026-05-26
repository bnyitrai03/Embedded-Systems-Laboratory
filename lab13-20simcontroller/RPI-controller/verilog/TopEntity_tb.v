`timescale 1 ns / 1 ps

module TopEntity_tb;

  // System clock and reset button
  reg clk = 0;
  reg btn1 = 1;  // active-high reset

  // SPI signals (master side)
  reg  SPI_CLK = 0;
  reg  SPI_CS = 1;   // active low
  reg  SPI_PICO = 0;   // MOSI
  wire SPI_POCI;       // MISO

  // For SPI transactions
  reg [31:0] rx_word;
  reg [15:0] yaw_cmd;
  reg [15:0] pitch_cmd;

  // Encoder inputs
  reg PITCH_ENC_A = 0, PITCH_ENC_B = 0;
  reg YAW_ENC_A = 0, YAW_ENC_B = 0;

  // PWM outputs
  wire PITCH_DIRA, PITCH_DIRB, PITCH_PWM_VAL;
  wire YAW_DIRA,   YAW_DIRB,   YAW_PWM_VAL;

  // Generate main 50 MHz system clock (20 ns period)
  always #10 clk = ~clk;

  // Generate SPI clock (2 MHz -> 250 ns period)
  always #125 SPI_CLK = ~SPI_CLK;

  // Instantiate DUT
  TopEntity dut (
    .clk          (clk),
    .btn1         (btn1),

    .SPI_CLK      (SPI_CLK),
    .SPI_PICO     (SPI_PICO),
    .SPI_CS       (SPI_CS),

    .PITCH_ENC_A  (PITCH_ENC_A),
    .PITCH_ENC_B  (PITCH_ENC_B),

    .YAW_ENC_A    (YAW_ENC_A),
    .YAW_ENC_B    (YAW_ENC_B),

    .PITCH_DIRA   (PITCH_DIRA),
    .PITCH_PWM_VAL(PITCH_PWM_VAL),
    .PITCH_DIRB   (PITCH_DIRB),

    .YAW_DIRA     (YAW_DIRA),
    .YAW_PWM_VAL  (YAW_PWM_VAL),
    .YAW_DIRB     (YAW_DIRB),

    .SPI_POCI     (SPI_POCI)
  );

  // SPI mode 0, 32-bit transfer
  task spi_transfer32(
    input  [31:0] mosi_word,
    output [31:0] miso_word
  );
    integer i;
    reg [31:0] miso_tmp;
    begin
      miso_tmp = 0;
      // Wait for a known falling edge to align phase
      @(negedge SPI_CLK);
      // Start transaction: CS low
      SPI_CS = 0;

      // SPI_CLK is already toggling in its own always block
      for (i = 31; i >= 0; i = i - 1) begin
        // Put MOSI bit on line before rising edge
        SPI_PICO = mosi_word[i];

         // Wait for rising edge: master samples MISO, slave samples MOSI
        @(posedge SPI_CLK);
        miso_tmp[i] = SPI_POCI;

        // Wait for falling edge, where slave will shift next bit
        @(negedge SPI_CLK);
      end

      // End transaction
      SPI_CS = 1;
      SPI_PICO = 0;
      miso_word = miso_tmp;
      #500;
    end
  endtask

  initial begin
    // Optional waves
    $dumpfile("TopEntity_tb.vcd");
    $dumpvars(0, TopEntity_tb);

    // Reset
    btn1 = 1;
    #200;
    // Adding Initial values
    SPI_CS = 1;
    SPI_PICO = 0;
    PITCH_ENC_A = 0;
    PITCH_ENC_B = 0;
    YAW_ENC_A = 0;
    YAW_ENC_B = 0;
    btn1 = 0;
    #600;

    // Move pitch encoder forward: 00 -> 10 -> 11 -> 01 -> 00
    PITCH_ENC_A = 0; PITCH_ENC_B = 0; #500;
    PITCH_ENC_A = 1; PITCH_ENC_B = 0; #500;
    PITCH_ENC_A = 1; PITCH_ENC_B = 1; #500;
    PITCH_ENC_A = 0; PITCH_ENC_B = 1; #500;
    PITCH_ENC_A = 0; PITCH_ENC_B = 0; #500;
    // Move one back
    PITCH_ENC_A = 0; PITCH_ENC_B = 1; #500;
    $display("T=%0t pitch count=%0d", $time, dut.pitch_decoder.count);

    // Move yaw encoder backward: 00 -> 10 -> 11 -> 01 -> 00
    YAW_ENC_A = 0; YAW_ENC_B = 0; #500;
    YAW_ENC_A = 0; YAW_ENC_B = 1; #500;
    YAW_ENC_A = 1; YAW_ENC_B = 1; #500;
    YAW_ENC_A = 1; YAW_ENC_B = 0; #500;
    YAW_ENC_A = 0; YAW_ENC_B = 0; #500;
    // Move one forward
    YAW_ENC_A = 1; YAW_ENC_B = 0; #500;
    $display("T=%0t yaw count=%0d", $time, dut.yaw_decoder.count);

    // Read encoder via SPI (send 0 as command)
    spi_transfer32(0, rx_word);
    $display("T=%0t SPI read: pitch=%0d yaw=%0d", $time, $signed(rx_word[15:0]), $signed(rx_word[31:16]));

    // Wait a bit
    #500;

    // Build a motor command
    yaw_cmd   = {1'b1, 1'b1, 14'd1000}; // yaw: dir=1, enable=1, duty=1000
    pitch_cmd = {1'b0, 1'b1, 14'd500};  // pitch: dir=0, enable=1, duty=500

    spi_transfer32({yaw_cmd, pitch_cmd}, rx_word);
    $display("T=%0t Received motor control values: pitch_dir=%b pitch_enable=%b pitch_duty=%d | yaw_dir=%b yaw_enable=%b yaw_duty=%d",
      $time,
      dut.pitch_dir, dut.pitch_pwm_enable, dut.pitch_pwm_duty,
      dut.yaw_dir,   dut.yaw_pwm_enable,   dut.yaw_pwm_duty
    );
    $display("T=%0t PWM values: pitch: INA=%b INB=%b | yaw: INA=%b INB=%b",
      $time,
      dut.PITCH_DIRA, dut.PITCH_DIRB,
      dut.YAW_DIRA, dut.YAW_DIRB,
    );

    #200000;
    $finish;
  end

endmodule