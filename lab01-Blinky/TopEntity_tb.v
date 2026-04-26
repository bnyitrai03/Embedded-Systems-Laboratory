`include "TopEntity.v" // Include YOUR entity.
`timescale 1ns / 1ps  // Time unit = period, precision
module TopEntity_tb;
  reg [0:0] clk;
  wire led1;
  integer i;
  TopEntity dut ( // <- TopEntity dut (Device Under Test) UPDATE TopEntity when relevant!
      .clk (clk),
      .led1(led1),
      .led2(led2),
      .led3(led3)
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
    $dumpfile("signals.vcd");  // Name of the signal dump file
    $dumpvars(0, TopEntity_tb);  // Signals to dump
    // rst = 0;
    // #10;
    // rst = 1;
    // #10;
    // rst = 0; // (Hmmm... why would this exist)
    #50000; // You might find this to be a bit short for your given simulation, please think of how long you would have to simulate the get usefull data!
    $finish;  // end simulation
  end
endmodule
