module TopEntity (
    input  wire clk,
    output reg  led1 = 0,
    output reg  led2 = 0,
    output reg  led3 = 0
);

  reg [31:0] count1 = 0;
  reg [31:0] count2 = 0;
  reg [31:0] count3 = 0;

  always @(posedge clk) begin
    // LED1 toggles every 8,000,000 cycles
    if (count1 == 8_000_000) begin
      count1 <= 0;
      led1   <= ~led1;
    end else begin
      count1 <= count1 + 1;
    end

    // LED2 toggles every 2,000,000 cycles
    if (count2 == 2_000_000) begin
      count2 <= 0;
      led2   <= ~led2;
    end else begin
      count2 <= count2 + 1;
    end

    // LED3 toggles every 16,000,000 cycles
    if (count3 == 16_000_000) begin
      count3 <= 0;
      led3   <= ~led3;
    end else begin
      count3 <= count3 + 1;
    end
  end

endmodule