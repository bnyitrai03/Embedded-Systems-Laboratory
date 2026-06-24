`include "QuadDecoder.v"
`timescale 1ns / 1ps
module QuadDecoder_tb;
  reg clk, A, B, rst;
  wire signed [31:0] count;
  wire dir;

  QuadDecoder dut (
      .clk (clk),
      .A(A),
      .B(B),
      .rst(rst),
      .count(count),
      .dir(dir)
  );

  // generate clock signal
  initial begin
    forever begin
      clk = 0;
      #1;
      clk = ~clk;
      #1;
    end
  end

  initial begin
    $dumpfile("quad_signals.vcd");
    $dumpvars(0, QuadDecoder_tb);

    // initial values
    A = 0;
    B = 0;
    rst = 1;
    // hold reset
    #10;
    rst = 0;

    // forward: 00 -> 10 -> 11 -> 01 -> 00
    #10; A = 1; B = 0;
    #10; A = 1; B = 1;
    #10; A = 0; B = 1;
    #10; A = 0; B = 0;

    // reverse: 00 -> 01 -> 11 -> 10 -> 00
    #10; A = 0; B = 1;
    #10; A = 1; B = 1;
    #10; A = 1; B = 0;
    #10; A = 0; B = 0;

    // reverse: 00 -> 01 -> 11 -> 10 -> 00
    #10; A = 0; B = 1;
    #10; A = 1; B = 1;
    #10; A = 1; B = 0;
    #10; A = 0; B = 0;

    // forward: 00 -> 10 -> 11 -> 01 -> 00
    #10; A = 1; B = 0;
    #10; A = 1; B = 1;
    #10; A = 0; B = 1;
    #10; A = 0; B = 0;

    #10;
    $finish;
  end
endmodule