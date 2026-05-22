module TopEntity (
    input  clk,
    input  btn1,        // reset button

    input  SPI_CLK,
    input  SPI_PICO,
    input  SPI_CS,

    input  PITCH_ENC_A,
    input  PITCH_ENC_B,

    input YAW_ENC_A,
    input YAW_ENC_B,

    output PITCH_DIRA,
    output PITCH_PWM_VAL,
    output PITCH_DIRB,

    output YAW_DIRA,
    output YAW_PWM_VAL,
    output YAW_DIRB,

    output SPI_POCI
);

  wire signed [15:0] pitch_count;
  wire [13:0]        pitch_pwm_duty;
  wire               pitch_dir;
  wire               pitch_pwm_enable;

  wire signed [15:0] yaw_count;
  wire [13:0]        yaw_pwm_duty;
  wire               yaw_dir;
  wire               yaw_pwm_enable;

  SPI spi_comm(
    .clk(clk),
    .rst(btn1),
    .spi_clk(SPI_CLK),
    .spi_pico(SPI_PICO),
    .spi_cs(SPI_CS),
    .spi_poci(SPI_POCI),

    .pitch_encoder_count(pitch_count),
    .pitch_pwm_duty(pitch_pwm_duty),
    .pitch_dir(pitch_dir),
    .pitch_pwm_enable(pitch_pwm_enable),

    .yaw_encoder_count(yaw_count),
    .yaw_pwm_duty(yaw_pwm_duty),
    .yaw_dir(yaw_dir),
    .yaw_pwm_enable(yaw_pwm_enable)
  );

  QuadDecoder pitch_decoder (
    .clk(clk),
    .A(PITCH_ENC_A),
    .B(PITCH_ENC_B),
    .rst(btn1),
    .count(pitch_count)
  );

  PWM pitch_pwm (
    .clk(clk),
    .rst(btn1),
    .enable(pitch_pwm_enable),
    .dir(pitch_dir),
    .duty(pitch_pwm_duty),
    .INA(PITCH_DIRA),
    .INB(PITCH_DIRB),
    .C(PITCH_PWM_VAL)
  );

  QuadDecoder yaw_decoder (
    .clk(clk),
    .A(YAW_ENC_A),
    .B(YAW_ENC_B),
    .rst(btn1),
    .count(yaw_count)
  );

  PWM yaw_pwm (
    .clk(clk),
    .rst(btn1),
    .enable(yaw_pwm_enable),
    .dir(yaw_dir),
    .duty(yaw_pwm_duty),
    .INA(YAW_DIRA),
    .INB(YAW_DIRB),
    .C(YAW_PWM_VAL)
  );

endmodule
