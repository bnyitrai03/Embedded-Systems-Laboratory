`timescale 1 ps / 1 ps

module PWM_wrapper #(
    parameter DATA_WIDTH = 32
) (
    input  wire [7:0]                slave_address,      // avs_s0.address
    input  wire                      slave_read,         //       .read
    output reg  [DATA_WIDTH-1:0]     slave_readdata,     //       .readdata
    input  wire                      slave_write,        //       .write
    input  wire [DATA_WIDTH-1:0]     slave_writedata,    //       .writedata
    input  wire                      clk,                // clock.clk
    input  wire                      reset,              // reset.reset
    input  wire [(DATA_WIDTH/8)-1:0] slave_byteenable,

    output wire                      INA,
    output wire                      INB,
    output wire                      C
);

    localparam REG_CONTROL = 8'd0;
    localparam REG_DUTY    = 8'd1;

    reg [31:0] control_reg;
    reg [31:0] duty_reg;

    wire enable;
    wire dir;

    assign enable = control_reg[0];
    assign dir    = control_reg[1];

    // Write registers from the Avalon bus.
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            control_reg <= 32'd0;
            duty_reg    <= 32'd0;
        end else if (slave_write) begin
            case (slave_address)
                REG_CONTROL: control_reg <= slave_writedata;
                REG_DUTY:    duty_reg    <= slave_writedata;
                default: ;
            endcase
        end
    end

    // Read the same values back, mostly for debugging from Linux.
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            slave_readdata <= 32'd0;
        end else if (slave_read) begin
            case (slave_address)
                REG_CONTROL: slave_readdata <= control_reg;
                REG_DUTY:    slave_readdata <= duty_reg;
                default:     slave_readdata <= 32'd0;
            endcase
        end
    end

    PWM pwm_inst (
        .clk(clk),
        .rst(reset),
        .enable(enable),
        .dir(dir),
        .duty(duty_reg),
        .INA(INA),
        .INB(INB),
        .C(C)
    );

endmodule
