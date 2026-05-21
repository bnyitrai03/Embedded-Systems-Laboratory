
module TopEntity (
    input  clk,
    input  SPI_CLK,
    input  SPI_PICO,
    input  SPI_CS,
    input  PITCH_ENC_A,
    input  PITCH_ENC_B,
    input  btn1,
    output SPI_POCI,
    output led2
);

  reg [2:0] SPI_CLKr;
  always @(posedge clk) SPI_CLKr <= {SPI_CLKr[1:0], SPI_CLK};
  wire SPI_CLK_risingedge = (SPI_CLKr[2:1] == 2'b01);
  wire SPI_CLK_fallingedge = (SPI_CLKr[2:1] == 2'b10);

  reg [2:0] SPI_CSr;
  always @(posedge clk) SPI_CSr <= {SPI_CSr[1:0], SPI_CS};
  wire SPI_CS_active = ~SPI_CSr[1];
  wire SPI_CS_startmessage = (SPI_CSr[2:1] == 2'b10);
  wire SPI_CS_endmessage = (SPI_CSr[2:1] == 2'b01);

  reg [1:0] SPI_PICOr;
  always @(posedge clk) SPI_PICOr <= {SPI_PICOr[0], SPI_PICO};
  wire SPI_PICO_data = SPI_PICOr[1];

  reg [2:0] bitcnt;
  reg byte_received;
  reg [7:0] byte_data_received;

  wire signed [31:0] pitch_count;
  wire pitch_dir;

  QuadDecoder pitch_decoder (
    .clk(clk),
    .A(PITCH_ENC_A),
    .B(PITCH_ENC_B),
    .rst(btn1),
    .count(pitch_count),
    .dir(pitch_dir)
  );

  always @(posedge clk) begin
    if (~SPI_CS_active) bitcnt <= 3'b000;
    else if (SPI_CLK_risingedge) begin
      bitcnt <= bitcnt + 3'b001;
      byte_data_received <= {byte_data_received[6:0], SPI_PICO_data};
    end
  end

  always @(posedge clk) byte_received <= SPI_CS_active && SPI_CLK_risingedge && (bitcnt == 3'b111);

  reg led2;
  always @(posedge clk) if (byte_received) led2 <= byte_data_received[0];

  reg [7:0] byte_data_sent;
  reg [7:0] cnt;
  always @(posedge clk) if (SPI_CS_startmessage) cnt <= cnt + 8'h1;

  always @(posedge clk) begin
    if (!SPI_CS_active) begin
      byte_data_sent <= 8'h00;
    end else begin
      if (byte_received && byte_data_received == 8'h01) begin
        byte_data_sent <= pitch_count[7:0];
      end else if (SPI_CLK_fallingedge) begin
        byte_data_sent <= {byte_data_sent[6:0], 1'b0};
      end
    end
  end


  assign SPI_POCI = byte_data_sent[7];

endmodule