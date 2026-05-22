`timescale 1 ps / 1 ps

module PWM #(
    parameter CLK_FREQ = 50000000,
    parameter PWM_FREQ = 20000
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire        dir,
    input  wire [13:0] duty,

    output reg         INA,
    output reg         INB,
    output reg         C // ie PWM
);
    // 2500 ticks for one period with the default 50 MHz / 20 kHz settings.
    localparam PWM_PERIOD = CLK_FREQ / PWM_FREQ;

    reg [13:0] counter;

    // Counts one full PWM period, then starts again from zero.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            counter <= 14'd0;
        end else begin
            if (counter >= PWM_PERIOD - 1) begin
                counter <= 14'd0;
            end else begin
                counter <= counter + 1'b1;
            end
        end
    end

    // INA and INB set the motor direction, C is the PWM speed signal.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            INA <= 1'b0;
            INB <= 1'b0;
            C   <= 1'b0;
        end else begin
            if (enable) begin
                // dir = 0 gives one direction, dir = 1 gives the other.
                INA <= ~dir;
                INB <= dir;
                // duty is expressed directly in PWM clock ticks, max 2500.
                C   <= (counter < duty);
            end else begin
                // Safe stop state when the module is disabled.
                INA <= 1'b0;
                INB <= 1'b0;
                C   <= 1'b0;
            end
        end
    end

endmodule
