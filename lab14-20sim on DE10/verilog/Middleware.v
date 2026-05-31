`timescale 1 ps / 1 ps

module Middleware #(
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

    input  wire                      pitch_enc_a,
    input  wire                      pitch_enc_b,
    input wire                       pitch_pwm_val,
    input wire                       pitch_dir_a,
    input wire                       pitch_dir_b,

    input  wire                      yaw_enc_a,
    input  wire                      yaw_enc_b,
    input wire                       yaw_pwm_val,
    input wire                       yaw_dir_a,
    input wire                       yaw_dir_b,

    input wire                       reset_button
);

    localparam READ  = 0,
               WRITE = 1;

    // Encoder values
    wire signed [15:0] pitch_count;
    wire signed [15:0] yaw_count;

    // Direction values
    reg        pitch_enable;
    reg        pitch_dir;
    reg [13:0] pitch_duty;

    reg        yaw_enable;
    reg        yaw_dir;
    reg [13:0] yaw_duty;

    //
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            slave_readdata <= 0;
        end else if (slave_read) begin
            case (slave_address)
                READ: slave_readdata <= {yaw_count, pitch_count};
            endcase
        end
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            slave_readdata <= 0;

            yaw_enable   <= 0;
            yaw_dir      <= 0;
            yaw_duty     <= 0;

            pitch_enable <= 0;
            pitch_dir    <= 0;
            pitch_duty   <= 0;
        end else begin
            if (slave_write) begin
                case (slave_address)
                    WRITE: begin
                        yaw_dir      <= slave_writedata[15];
                        yaw_enable   <= slave_writedata[14];
                        yaw_duty     <= slave_writedata[13:0];

                        pitch_dir    <= slave_writedata[31];
                        pitch_enable <= slave_writedata[30];
                        pitch_duty   <= slave_writedata[29:16];
                    end
                endcase
            end

            if (slave_read) begin
                case (slave_address)
                    READ: slave_readdata <= {yaw_count, pitch_count};
                endcase
            end
        end
    end

    PWM yaw_pwm (
        .clk(clk),
        .rst(reset_button),
        .enable(yaw_enable),
        .dir(yaw_dir),
        .duty(yaw_duty),
        .INA(yaw_dir_a),
        .INB(yaw_dir_b),
        .C(yaw_pwm_val)
    );

    PWM pitch_pwm (
        .clk(clk),
        .rst(reset_button),
        .enable(pitch_enable),
        .dir(pitch_dir),
        .duty(pitch_duty),
        .INA(pitch_dir_a),
        .INB(pitch_dir_b),
        .C(pitch_pwm_val)
    );

    QuadDecoder pitch_decoder (
      .clk(clk),
      .A(pitch_enc_a),
      .B(pitch_enc_b),
      .rst(reset_button),
      .count(pitch_count),
      .dir(pitch_dir)
    );

    QuadDecoder yaw_decoder (
      .clk(clk),
      .A(yaw_enc_a),
      .B(yaw_enc_b),
      .rst(reset_button),
      .count(yaw_count),
      .dir(yaw_dir)
    );

endmodule