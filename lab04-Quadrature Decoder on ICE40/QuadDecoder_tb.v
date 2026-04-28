`include "QuadDecoder.v"
`timescale 1ns / 1ps
module QuadDecoder_tb;
  reg clk, A, B, rst;

  QuadDecoder dut (
      .clk (clk),
      .A(A),
      .B(B),
      .rst(rst)
  );

  // generate input signals
  initial begin
    forever begin
      clk = 0;
      #1;
      clk = ~clk;
      #1;
    end
  end

// Start of your testbench script
  initial begin
    $dumpfile("quad_signals.vcd");
    $dumpvars(0, QuadDecoder_tb);

    // rst = 0; // (Hmmm... why would this exist)
    #50000;
    $finish;
  end
endmodule