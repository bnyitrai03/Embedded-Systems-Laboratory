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
   * 32-bit full-duplex SPI frame, mode 0.
   *
   * Raspberry Pi -> FPGA:
   *   [31:16] yaw   bit15 dir, bit14 enable, bit13..0 PWM
   *   [15:0]  pitch bit15 dir, bit14 enable, bit13..0 PWM
   *
   * FPGA -> Raspberry Pi:
   *   [31:16] signed yaw encoder count
   *   [15:0]  signed pitch encoder count
   */

  // Clock synchronization
  reg [2:0] SPI_CLKr = 3'b000;
  always @(posedge clk)
    SPI_CLKr <= {SPI_CLKr[1:0], spi_clk};

  wire SPI_CLK_risingedge  = (SPI_CLKr[2:1] == 2'b01);
  wire SPI_CLK_fallingedge = (SPI_CLKr[2:1] == 2'b10);

  reg [2:0] SPI_CSr = 3'b111;
  always @(posedge clk)
    SPI_CSr <= {SPI_CSr[1:0], spi_cs};

  wire SPI_CS_active       = ~SPI_CSr[1];
  wire SPI_CS_startmessage = (SPI_CSr[2:1] == 2'b10);
  wire SPI_CS_endmessage   = (SPI_CSr[2:1] == 2'b01);

  reg [1:0] SPI_PICOr = 2'b00;
  always @(posedge clk)
    SPI_PICOr <= {SPI_PICOr[0], spi_pico};

  wire SPI_PICO_data = SPI_PICOr[1];

  reg [31:0] rx_shift = 32'h00000000;
  reg [31:0] tx_shift = 32'h00000000;
  reg [5:0]  bit_count = 6'd0;

  reg [13:0] pitch_pwm_duty_r   = 14'd0;
  reg        pitch_dir_r        = 1'b0;
  reg        pitch_pwm_enable_r = 1'b0;

  reg [13:0] yaw_pwm_duty_r     = 14'd0;
  reg        yaw_dir_r          = 1'b0;
  reg        yaw_pwm_enable_r   = 1'b0;

  always @(posedge clk) begin
    if (rst) begin
      tx_shift <= 32'h00000000;
      rx_shift <= 32'h00000000;
      bit_count <= 6'd0;
      yaw_pwm_duty_r <= 14'd0;
      yaw_dir_r <= 1'b0;
      yaw_pwm_enable_r <= 1'b0;
      pitch_pwm_duty_r <= 14'd0;
      pitch_dir_r <= 1'b0;
      pitch_pwm_enable_r <= 1'b0;
    end else begin
      if (!SPI_CS_active) begin
        tx_shift <= {yaw_encoder_count, pitch_encoder_count};
        rx_shift <= 32'h00000000;
        bit_count <= 6'd0;
      end else if (SPI_CS_startmessage) begin
        /*
         * Snapshot encoders at chip-select assertion so all 32 return bits
         * belong to the same sample.
         */
        tx_shift <= {yaw_encoder_count, pitch_encoder_count};
        rx_shift <= 32'h00000000;
        bit_count <= 6'd0;
      end else if (SPI_CS_active && SPI_CLK_risingedge) begin
        rx_shift <= {rx_shift[30:0], SPI_PICO_data};
        if (bit_count < 6'd32) begin
          bit_count <= bit_count + 6'd1;
        end
      end

      /*
       * SPI mode 0: the master samples POCI on rising edges. The slave shifts
       * the next output bit on falling edges.
       */
      if (SPI_CS_active && SPI_CLK_fallingedge) begin
        tx_shift <= {tx_shift[30:0], 1'b0};
      end

      /*
       * Only commit a new motor command after a complete 32-bit frame. Short or
       * noisy transfers are ignored, leaving the previous command active.
       */
      if (SPI_CS_endmessage && bit_count == 6'd32) begin
        yaw_dir_r          <= rx_shift[31];
        yaw_pwm_enable_r   <= rx_shift[30];
        yaw_pwm_duty_r     <= rx_shift[29:16];
        pitch_dir_r        <= rx_shift[15];
        pitch_pwm_enable_r <= rx_shift[14];
        pitch_pwm_duty_r   <= rx_shift[13:0];
      end
    end
  end

  assign pitch_pwm_duty   = pitch_pwm_duty_r;
  assign pitch_dir        = pitch_dir_r;
  assign pitch_pwm_enable = pitch_pwm_enable_r;

  assign yaw_pwm_duty     = yaw_pwm_duty_r;
  assign yaw_dir          = yaw_dir_r;
  assign yaw_pwm_enable   = yaw_pwm_enable_r;

  assign spi_poci = tx_shift[31];

endmodule
