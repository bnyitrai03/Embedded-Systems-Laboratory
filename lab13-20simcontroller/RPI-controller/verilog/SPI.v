module SPI (
    input  wire        clk,
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

  // Clock synchronization
  reg [2:0] SPI_CLKr;
  always @(posedge clk)
    SPI_CLKr <= {SPI_CLKr[1:0], SPI_CLK};

  wire SPI_CLK_risingedge  = (SPI_CLKr[2:1] == 2'b01);
  wire SPI_CLK_fallingedge = (SPI_CLKr[2:1] == 2'b10);

  reg [2:0] SPI_CSr;
  always @(posedge clk)
    SPI_CSr <= {SPI_CSr[1:0], SPI_CS};

  wire SPI_CS_active       = ~SPI_CSr[1];
  wire SPI_CS_startmessage = (SPI_CSr[2:1] == 2'b10);
  wire SPI_CS_endmessage   = (SPI_CSr[2:1] == 2'b01);

  reg [1:0] SPI_PICOr;
  always @(posedge clk)
    SPI_PICOr <= {SPI_PICOr[0], SPI_PICO};

  wire SPI_PICO_data = SPI_PICOr[1];

  // 32-bit receive and transmit shift registers
  reg [31:0] rx_shift = 0;
  reg [31:0] tx_shift = 0;

  // Latched output signals
  reg [13:0] pitch_pwm_duty_r   = 14'd0;
  reg        pitch_dir_r        = 1'b0;
  reg        pitch_pwm_enable_r = 1'b0;

  reg [13:0] yaw_pwm_duty_r     = 14'd0;
  reg        yaw_dir_r          = 1'b0;
  reg        yaw_pwm_enable_r   = 1'b0;


  reg [2:0] bitcnt;
  reg       byte_received;
  reg [7:0] byte_data_received;

  always @(posedge clk) begin
    if (~SPI_CS_active)
      bitcnt <= 3'b000;
    else if (SPI_CLK_risingedge) begin
      bitcnt <= bitcnt + 3'b001;
      byte_data_received <= {byte_data_received[6:0], SPI_PICO_data};
    end
  end

  always @(posedge clk)
    byte_received <= SPI_CS_active && SPI_CLK_risingedge && (bitcnt == 3'b111);

  reg [31:0] data_sent;
  reg        response_loaded;
  reg [7:0]  cnt;

  always @(posedge clk)
    if (SPI_CS_startmessage)
      cnt <= cnt + 8'h1;

  always @(posedge clk) begin
    if (!SPI_CS_active) begin
      data_sent       <= 32'h00000000;
      response_loaded <= 1'b0;
    end else if (byte_received && byte_data_received == 8'h01) begin
      data_sent       <= pitch_count;
      response_loaded <= 1'b1;
    end else if (SPI_CLK_fallingedge && response_loaded) begin
      response_loaded <= 1'b0;
    end else if (SPI_CLK_fallingedge) begin
      data_sent <= {data_sent[30:0], 1'b0};
    end
  end

  assign pitch_pwm_duty   = pitch_pwm_duty_r;
  assign pitch_dir        = pitch_dir_r;
  assign pitch_pwm_enable = pitch_pwm_enable_r;

  assign yaw_pwm_duty     = yaw_pwm_duty_r;
  assign yaw_dir          = yaw_dir_r;
  assign yaw_pwm_enable   = yaw_pwm_enable_r;
  
  assign SPI_POCI = data_sent[31];

endmodule