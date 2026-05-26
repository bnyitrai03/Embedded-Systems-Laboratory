module SPI (
    input  wire        clk,
    input  wire        rst,
    input  wire        spi_clk,
    input  wire        spi_pico,
    input  wire        spi_cs,

    input  wire signed [15:0] pitch_encoder_count,
    input  wire signed [15:0] yaw_encoder_count,

    output wire [13:0] pitch_pwm_duty,
    output wire        pitch_dir,
    output wire        pitch_pwm_enable,

    output wire [13:0] yaw_pwm_duty,
    output wire        yaw_dir,
    output wire        yaw_pwm_enable,

    output wire        spi_poci
);

  /*
   * 32-bit full-duplex SPI frame, mode 0
   *
   * Raspberry Pi -> FPGA:
   *   [31:16] yaw   : bit15 dir, bit14 enable, bit13:0 PWM
   *   [15:0]  pitch : bit15 dir, bit14 enable, bit13:0 PWM
   *
   * FPGA -> Raspberry Pi:
   *   [31:16] signed yaw encoder count
   *   [15:0]  signed pitch encoder count
   */

  reg [31:0] rx_shift = 32'd0;
  reg [31:0] tx_shift = 32'd0;
  reg [5:0]  bit_count = 6'd0;

  reg [13:0] pitch_pwm_duty_r   = 14'd0;
  reg        pitch_dir_r        = 1'b0;
  reg        pitch_pwm_enable_r = 1'b0;

  reg [13:0] yaw_pwm_duty_r     = 14'd0;
  reg        yaw_dir_r          = 1'b0;
  reg        yaw_pwm_enable_r   = 1'b0;

  assign pitch_pwm_duty   = pitch_pwm_duty_r;
  assign pitch_dir        = pitch_dir_r;
  assign pitch_pwm_enable = pitch_pwm_enable_r;

  assign yaw_pwm_duty     = yaw_pwm_duty_r;
  assign yaw_dir          = yaw_dir_r;
  assign yaw_pwm_enable   = yaw_pwm_enable_r;

  assign spi_poci = tx_shift[31];

  // Start of SPI frame: preload encoder snapshot for MISO
  always @(negedge spi_cs or posedge rst) begin
    if (rst) begin
      tx_shift  <= 0;
      rx_shift  <= 0;
      bit_count <= 0;
    end else begin
      tx_shift  <= {yaw_encoder_count, pitch_encoder_count};
      rx_shift  <= 0;
      bit_count <= 0;
    end
  end

  // Sample MOSI on rising edge (mode 0)
  always @(posedge spi_clk or posedge rst) begin
    if (rst) begin
      rx_shift  <= 0;
      bit_count <= 0;
    end else if (!spi_cs) begin
      rx_shift <= {rx_shift[30:0], spi_pico};
      if (bit_count < 32)
        bit_count <= bit_count + 1;
    end
  end

  // Shift MISO on falling edge so next bit is ready before next rising edge
  always @(negedge spi_clk or posedge rst) begin
    if (rst) begin
      tx_shift <= 0;
    end else if (!spi_cs) begin
      tx_shift <= {tx_shift[30:0], 1'b0};
    end
  end

  // Accept command only if exactly 32 bits were received
  always @(posedge spi_cs or posedge rst) begin
    if (rst) begin
      yaw_pwm_duty_r      <= 0;
      yaw_dir_r           <= 0;
      yaw_pwm_enable_r    <= 0;
      pitch_pwm_duty_r    <= 0;
      pitch_dir_r         <= 0;
      pitch_pwm_enable_r  <= 0;
    end else if (bit_count == 32) begin
      yaw_dir_r           <= rx_shift[31];
      yaw_pwm_enable_r    <= rx_shift[30];
      yaw_pwm_duty_r      <= rx_shift[29:16];

      pitch_dir_r         <= rx_shift[15];
      pitch_pwm_enable_r  <= rx_shift[14];
      pitch_pwm_duty_r    <= rx_shift[13:0];
    end
  end

endmodule