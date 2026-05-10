`timescale 1 ns / 1 ps

module PWM_tb;

    reg clk;
    reg rst;
    reg enable;
    reg dir;
    reg [31:0] duty;

    wire INA;
    wire INB;
    wire C;

    initial begin
        $dumpfile("lab09-PWM_IP/PWM_tb.vcd");
        $dumpvars(0, PWM_tb);
    end

    // Smaller values make the simulation easier to read.
    PWM #(
        .CLK_FREQ(100),
        .PWM_FREQ(10)
    ) dut (
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .dir(dir),
        .duty(duty),
        .INA(INA),
        .INB(INB),
        .C(C)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    initial begin
        rst    = 1'b1;
        enable = 1'b0;
        dir    = 1'b0;
        duty   = 32'd0;

        #30;
        rst = 1'b0;

        // Disabled: all outputs should stay low.
        #50;

        // One direction, 50 percent duty cycle.
        enable = 1'b1;
        dir    = 1'b0;
        duty   = 32'd5;
        #150;

        // Same direction, smaller duty cycle.
        duty = 32'd2;
        #150;

        // Reverse direction, same PWM duty.
        dir = 1'b1;
        #150;

        // Stop again.
        enable = 1'b0;
        #50;

        $finish;
    end

endmodule
