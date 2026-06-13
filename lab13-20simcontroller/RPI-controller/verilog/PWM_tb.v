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
        $dumpfile("PWM_tb.vcd");
        $dumpvars(0, PWM_tb);
    end

    // Smaller values make the simulation easier to read
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
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        rst    = 1;
        enable = 0;
        dir    = 0;
        duty   = 0;

        #30;
        rst = 0;
        #50;

        // One direction, 50 percent duty cycle
        enable = 1;
        dir    = 0;
        duty   = 5;
        #150;

        // Same direction, smaller duty cycle
        duty = 2;
        #150;

        // Reverse direction, same PWM duty
        dir = 1;
        #150;

        // Stop again
        enable = 0;
        #50;

        $finish;
    end

endmodule
