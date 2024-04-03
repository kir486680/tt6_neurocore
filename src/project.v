/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`define default_netname none

module tt_um_example (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output reg [7:0] uo_out,   // Dedicated outputs - Changed to 'reg' to allow procedural assignments
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // will go high when the design is enabled
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // Modification: Use procedural block to dynamically assign uo_out
  always @(ui_in or rst_n) begin
    if (!rst_n) begin
      uo_out = 0; // Reset condition, setting everything to 0
    end else if (ui_in == 8'h01) begin // Check if ui_in is 1
      uo_out = 8'hFF; // Set all bits of uo_out to 1's
    end else begin
      uo_out = 8'h03; // Only the two least significant bits are set to 1's, the rest are 0's
    end
  end

  // Keep other assignments as is
  assign uio_out = 0;
  assign uio_oe  = 0;

endmodule
