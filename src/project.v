/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`define default_netname none
 `include "define.v"
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


      // Parameters for the systolic array
  reg start;
  reg load;
  wire block_multiply_done;
  //reg [`DATA_W-1:0] block_a1, block_a2, block_a3, block_a4, block_b1, block_b2, block_b3, block_b4;
  reg [`DATA_W-1:0] block_a1, block_a2, block_a3, block_a4, block_b1, block_b2, block_b3, block_b4;
  reg [`DATA_W-1:0] block_result1, block_result2, block_result3, block_result4;
  systolic_array systolic_array_inst (
      .block_a1(block_a1),
      .block_a2(block_a2),
      .block_a3(block_a3),
      .block_a4(block_a4),
      .block_b1(block_b1),
      .block_b2(block_b2),
      .block_b3(block_b3),
      .block_b4(block_b4),
      .clk(CLK),
      .rst(RESET),
      .start(start),
      .load(load),
      .block_multiply_done(block_multiply_done),
      .block_result1(block_result1),
      .block_result2(block_result2),
      .block_result3(block_result3),
      .block_result4(block_result4)
  );

      //state machine params
  localparam IDLE_MUL = 0, LOAD = 1, START = 2, DONE_MUL = 3;
  reg [2:0] current_mul_state = IDLE_MUL;
  reg [2:0] next_mul_state = IDLE_MUL;



     // Update current_mul_state and send_data in a single always block
  always @(posedge clk) begin
    if (!rst_n) begin
       current_mul_state <= IDLE_MUL;
       block_a1 <= 16'b0011110000000000;
       block_a2 <= 16'b0011110000000000;
       block_a3 <= 16'b0011110000000000;
       block_a4 <= 16'b0011110000000000;
       block_b1 <= 16'b0011110000000000;
       block_b2 <= 16'b0011110000000000;
       block_b3 <= 16'b0011110000000000;
       block_b4 <= 16'b0011110000000000;
           
    end else begin
        current_mul_state <= next_mul_state;
        $display("current_mul_state: %d", current_mul_state);
    end
end

     // Calculate next state and send_data based on current state and inputs
always @(*) begin
  if (!rst_n) begin
      start = 0;
      uo_out = 0;
      load = 0;
      next_mul_state = IDLE_MUL;
  end
  case (current_mul_state)
      IDLE_MUL: begin
          if (ui_in == 8'h01) begin
              //uo_out = 8'hFF;
             $display("input recognized");
              next_mul_state = LOAD;
          end else begin
              next_mul_state = IDLE_MUL;
          end
          
      end
      LOAD: begin
          next_mul_state = START;
          load = 1;
          start = 0;
          $display("loading");
      end
      START: begin
          load = 0;
          start = 1;
          $display("starting");
          if (block_multiply_done) begin
            
              next_mul_state = DONE_MUL;
          end else begin
              next_mul_state = START;
          end
          uo_out = 8'hFF;
          // Other START state logic
      end
      DONE_MUL: begin
        $display("done multiplying");
          // DONE_MUL state logic
          
          next_mul_state = IDLE_MUL;
      end
      default: next_mul_state = IDLE_MUL;
  endcase
end





  // Keep other assignments as is
  assign uio_out = 0;
  assign uio_oe  = 0;

endmodule





module systolic_array (
    input [`DATA_W-1:0] block_a1, // the block of matrix A
    input [`DATA_W-1:0] block_a2, // the block of matrix A
    input [`DATA_W-1:0] block_a3, // the block of matrix A
    input [`DATA_W-1:0] block_a4, // the block of matrix A
    input [`DATA_W-1:0] block_b1, // the block of matrix B
    input [`DATA_W-1:0] block_b2, // the block of matrix B
    input [`DATA_W-1:0] block_b3, // the block of matrix B
    input [`DATA_W-1:0] block_b4, // the block of matrix B
    input clk, rst, start, load,
    output reg [4:0] counter,
    output reg block_multiply_done,
    output reg [`DATA_W-1:0] block_result1, // the result matrix
    output reg [`DATA_W-1:0] block_result2, // the result matrix
    output reg [`DATA_W-1:0] block_result3, // the result matrix
    output reg [`DATA_W-1:0] block_result4 // the result matrix

);

    wire [`DATA_W-1:0] block_a[0:`J*`K-1]; // the block of matrix A
    wire [`DATA_W-1:0] block_b[0:`J*`K-1]; // the block of matrix B
    assign block_a[0] = block_a1;
    assign block_a[1] = block_a2;
    assign block_a[2] = block_a3;
    assign block_a[3] = block_a4;
    assign block_b[0] = block_b1;
    assign block_b[1] = block_b2;
    assign block_b[2] = block_b3;
    assign block_b[3] = block_b4;

        // Parameters for dimensions
    parameter ROWS = `J; // Number of rows
    parameter COLS = `K; // Number of columns

    wire [`DATA_W-1:0] north_south_wires[ROWS-1:0][COLS-1:0];
    wire [`DATA_W-1:0] east_west_wires[ROWS-1:0][COLS-1:0];


    //create a wire to connect to the output of all the members of last row north_sout
    wire [`DATA_W-1:0] north_south_wires_last_row[COLS-1:0];
    genvar k;
    generate
        for (k = 0; k < COLS; k = k + 1) begin: assign_last_row
            assign north_south_wires_last_row[k] = north_south_wires[ROWS-1][k];
        end
    endgenerate

    // Shift registers
    reg [`DATA_W-1:0] shift_registers[ROWS-1:0][COLS+ROWS-1:0];
    reg [`DATA_W-1:0] shift_registers_last_row[ROWS+ROWS-2:0][COLS-1:0];//takes 2n-1 to complete the product

    
    // Load matrix A into shift registers with offset when compute is true
    integer i;
    integer j;
    always @(posedge clk) begin
        if (!rst) begin
            for (i = 0; i < ROWS; i++) begin
                for (j = 0; j < COLS + ROWS - 1; j++) begin
                    shift_registers[i][j] <= `DATA_W'd0;
                end
            end
            for (i = 0; i < ROWS + ROWS -1; i++) begin
                for (j = 0; j < COLS; j++) begin
                    shift_registers_last_row[i][j] <= `DATA_W'd0;
                end
            end
            counter <= 5'd0;
            block_multiply_done <= 1'b0;
           
        end
        else if (load) begin
            block_multiply_done <= 1'b0;
            for (i = 0; i < ROWS; i++) begin
        // Initialize the entire row to 0 first
                for (j = 0; j < COLS + ROWS - 1; j++) begin
                    shift_registers[i][j] <= `DATA_W'd0;
                end
                // Load matrix A into the shift register
                for (j = 0; j < COLS; j++) begin
                    shift_registers[i][j + i] <= block_a[i * COLS + j];
                end
            end
        end
        else if (start) begin
            for (i = 0; i < ROWS; i++) begin
                for (j = 0; j < COLS + ROWS - 1; j++) begin
                    shift_registers[i][j] <= shift_registers[i][j+1];
                end
                // Load new data into the leftmost position 
                shift_registers[i][COLS + ROWS - 2] <= `DATA_W'd0; 
            end

            for (i = 0; i < ROWS + ROWS -1; i++) begin
                for (j = 0; j < COLS; j++) begin
                    if (i == 0) begin
                        shift_registers_last_row[i][j] <= north_south_wires[ROWS-1][j];
                    end else begin
                        shift_registers_last_row[i][j] <= shift_registers_last_row[i-1][j];
                    end
                end
            end
            if (counter == 5'd5) begin
                //this is wayyy too manual right now. need to figure out how to do this in a loop
                block_result1 <= shift_registers_last_row[2][0];
                block_result2 <= shift_registers_last_row[1][0];
                block_result3 <= shift_registers_last_row[1][1];
                block_result4 <= shift_registers_last_row[0][1];
                counter <= 5'd0;
                block_multiply_done <= 1'b1;
            end else if(block_multiply_done != 1'b1)begin
                counter <= counter + 1;
            end
        end
    end





    // These P blocks are hardcoded which is not optimal
    // Block P1 (top-left corner)
    block P1(
        .inp_north(`DATA_W'd0),
        .inp_west(shift_registers[0][0]),
        .weight_in(block_b[0]),
        .outp_south(north_south_wires[0][0]),
        .outp_east(east_west_wires[0][0]),
        .clk(clk),
        .rst(rst),
        .compute(start),
        .weight_en(load)
    );

    // Block P2 (top-right corner)
    block P2(
        .inp_north(`DATA_W'd0),
        .inp_west(east_west_wires[0][0]),
        .weight_in(block_b[2]),
        .outp_south(north_south_wires[0][1]),
        .outp_east(east_west_wires[0][1]),
        .clk(clk),
        .rst(rst),
        .compute(start),
        .weight_en(load)
    );

    // Block P3 (bottom-left corner)
    block P3(
        .inp_north(north_south_wires[0][0]),
        .inp_west(shift_registers[1][0]),
        .weight_in(block_b[1]),
        .outp_south(north_south_wires[1][0]),
        .outp_east(east_west_wires[1][0]),
        .clk(clk),
        .rst(rst),
        .compute(start),
        .weight_en(load)
    );

    // Block P4 (bottom-right corner)
    block P4(
        .inp_north(north_south_wires[0][1]),
        .inp_west(east_west_wires[1][0]),
        .weight_in(block_b[3]),
        .outp_south(north_south_wires[1][1]),
        .outp_east(east_west_wires[1][1]),
        .clk(clk),
        .rst(rst),
        .compute(start),
        .weight_en(load)
    );
 
 
endmodule




module block(inp_north, inp_west, weight_in, outp_south, outp_east,  clk, rst, compute, weight_en);
//we are implementing a weight stationary systolic array. activations are marched in from the activation storage buffer. The activations move horizontally from left to right and the partial sums move vertically from top to bottom.
////activations move horizontally from west to east
    input [`DATA_W-1:0] inp_north, inp_west; // north input is the partial sum and they move from north to south
// weights are the weight matrix that is loaded into the systolic array initially and is not changed during the computation
    input [`DATA_W-1:0] weight_in;
    reg [`DATA_W-1:0] weight;
    output reg [`DATA_W-1:0] outp_south, outp_east;
    input clk, rst, compute, weight_en;
    
    // Instantiate fmul module
    wire [`DATA_W-1:0] mul_result;
    fmul fmul (
        .a_in(inp_west),
        .b_in(weight),
        .result(mul_result)
    );
    wire [`DATA_W-1:0] add_result;
    fadd add_instance (
        .a_in(inp_north),
        .b_in(mul_result),
        .result(add_result)
    );
    always @(posedge clk) begin
        if(!rst ) begin
            outp_east <= `DATA_W'd0;
            outp_south <= `DATA_W'd0;
            weight <= `DATA_W'd0;
        end
        else begin 
            if (weight_en) begin
                weight <= weight_in;
            end
            if (compute) begin
                outp_east <= inp_west;
                //outp_south <= inp_north + inp_west * weight;
                outp_south <= add_result;
            end
        end 
    end
endmodule

//basically a close copy of https://github.com/ReaLLMASIC/nanoGPT/blob/master/HW/SA/verilog/fadd.sv
module fadd(
    input [16-1:0] a_in, b_in, // Inputs in the format of IEEE-`EXP_W-154 Representation.
    output [16-1:0] result // Outputs in the format of IEEE-`EXP_W-154 Representation.
);

wire Exception;
wire output_sign;
wire operation_sub_addBar;

wire [16-1:0] operand_a, operand_b;
wire [`M_W:0] significand_a, significand_b;
wire [`EXP_W-1:0] exponent_diff;

wire [`M_W:0] significand_b_add;
wire [`EXP_W-1:0] exponent_b_add;

wire [`M_W+1:0] significand_add;
wire [16-2:0] add_sum;

wire [`EXP_W-1:0] exp_a, exp_b;


//for operations always operand_a must not be less than b_in
assign {operand_a,operand_b} = (a_in[16-2:0] < b_in[16-2:0]) ? {b_in,a_in} : {a_in,b_in};

assign exp_a = operand_a[16-2:`M_W]; // extract exponent from operand_a
assign exp_b = operand_b[16-2:`M_W]; // extract exponent from operand_b

//Exception flag sets 1 if either one of the exponent is 255.
assign Exception = (&operand_a[16-2:`M_W]) | (&operand_b[16-2:`M_W]);

assign output_sign = operand_a[16-1] ; // since the operand_a is always greater than operand_b, the sign of the result will be same as operand_a.

//operation_sub_addBar is 1 if we are doing subtraction else 0.
assign operation_sub_addBar =  ~(operand_a[16-1] ^ operand_b[16-1]);

//Assigining significand values according to Hidden Bit.
assign significand_a = {1'b1,operand_a[`M_W-1:0]}; // expand the mantissa by 1 bit before multiplication since its always implied
assign significand_b = {1'b1,operand_b[`M_W-1:0]}; // same as above

//Evaluating Exponent Difference
assign exponent_diff = operand_a[16-2:`M_W] - operand_b[16-2:`M_W];

//Shifting significand_b to the right according to exponent_diff. Exapmle: if we have 1.0101 >> 2 = 0.0101 then exponent_diff = 2 and significand_b_add = significand_b >> exponent_diff
assign significand_b_add = significand_b >> exponent_diff;

//Adding exponent_diff to exponent_b. Exapmle: if we have 1.0101 << 2 = 101.01 then exponent_diff = 2 and exponent_b_add = exponent_b + exponent_diff
assign exponent_b_add = operand_b[16-2:`M_W] + exponent_diff; 

//------------------------------------------------ADD BLOCK------------------------------------------//
//if we are adding(operation_sub_addBar=1) need to add significand_b_add to significand_a. 
//Or sets the significand to zero if the signs are different(this means we are doing subtraction), effectively determining the core operation of the floating-point addition based on the sign of the operands.
assign significand_add = ( operation_sub_addBar) ? (significand_a + significand_b_add) : {(`M_W+2){1'b0}}; 

//Taking care of the resulting mantissa. 
//If there is a carry, then the result is normalized by shifting the significand right by one bit(because its implied) and incrementing the exponent by one.
//If there is no carry, we just use the result of the addition, and we have `M_W-1:0 due to the fact that we are using the hidden bit(implied 1).
assign add_sum[`M_W-1:0] = significand_add[`M_W+1] ? significand_add[`M_W:1] : significand_add[`M_W-1:0];

// Taking care of the resulting exponent.
//If carry generates in sum value then exponent must be added with 1 else feed as it is.
assign add_sum[16-2:`M_W] = significand_add[`M_W+1] ? (1'b1 + operand_a[16-2:`M_W]) : operand_a[16-2:`M_W];

assign result = Exception ? {(16){1'b0}} : {output_sign,add_sum};

endmodule


//basically a close copy of https://github.com/ReaLLMASIC/nanoGPT/blob/master/HW/SA/verilog/fmul.sv
module fmul(
    input [16-1:0] a_in,
    input [16-1:0] b_in,
    output [16-1:0] result
);

    reg [`MULT_W-1:0] mul_fix_out;
    reg [`M_W-1:0] M_result;
    wire [`EXP_W-1:0] e_result;
    wire sign;
    reg [`EXP_W:0] e_result0;
    reg overflow;
    reg zero_check;
    
    // Multiplication logic
    always @* begin
        mul_fix_out = {1'b1, a_in[`M_W-1:0]} * {1'b1, b_in[`M_W-1:0]}; //extend the mantissa by 1 bit before multiplication
    end

    // Zero check
    always @* begin
        if (a_in[16-2:`M_W] == 0 || b_in[16-2:`M_W] == 0) begin
            zero_check = 1'b1;
        end else begin
            zero_check = 1'b0;
        end
    end

    // Generate Mantissa. We are only considering the most significat bits of the product to generate the mantissa.
    always @* begin
        //select two MSBs of the product
        case(mul_fix_out[`MULT_W-1:`MULT_W-2])
           //Example: If mul_fix_out is 8 bits wide and represents 01xxxxxx (binary), it extracts xxxxxx, assuming the MSBs are 01
            2'b01: M_result = mul_fix_out[`MULT_W-3:`M_W]; //MSB is dropped(as it is always 1)
            //In 2'b10 or 2'b11 case: 10yyyyyy → Shift → 0yyyyyy (Extract yyyyyy)
            2'b10: M_result = mul_fix_out[`MULT_W-2:`M_W+1]; // Between two and just under 4. product larger than normalized range, so we need to shift right 
            2'b11: M_result = mul_fix_out[`MULT_W-2:`M_W+1]; // same as line above. 
            default: M_result = mul_fix_out[`MULT_W-2:`M_W+1]; // default same as two lines above
        endcase
    end

    // Overflow check
    always @* begin
        //Different cases for overflow:
        //1. If either of the inputs is zero, then the result is zero and there is no overflow.
        //2. Underflow check: If the sum of the exponents is less than the minimum exponent, then the result is zero and there is no overflow. {2'b0,{(EXP_W-1){1'b1}}} is the minimum exponent(001111111 in case of 32bit float)
        //3. Overflow check: If the sum of the exponents is greater than the maximum exponent, then the result is infinity and there is overflow. EXP_MAX is the maximum exponent.
        overflow = (zero_check || ({1'b0, a_in[16-2:`M_W]} + {1'b0, b_in[16-2:`M_W]} + {{`EXP_W{1'b0}}, mul_fix_out[`MULT_W-1]}) < {2'b0,{(`EXP_W-1){1'b1}}} || ({1'b0, a_in[16-2:`M_W]} + {1'b0, b_in[16-2:`M_W]} + {8'd0, mul_fix_out[`MULT_W-1]}) > `EXP_MAX);

        if (~zero_check) begin
            if (overflow) begin
                e_result0 = {(`EXP_W+1){1'b1}};
            end else begin
                //1. We extend the exponent by 1 bit because the result of addition of two exponents can be 1 bit larger than the exponent itself.
                //2. We add the MSB of the mantissa multiplication(before normalization) to the exponent sum to account for the shifting of the mantissa.
                //3. We subtract the bias from the exponent sum to get the final exponent because just adding two exponents would give us exp1 + exp2 + 2 x bias.
                e_result0 = ({1'b0, a_in[16-2:`M_W]} + {1'b0, b_in[16-2:`M_W]} + {{`EXP_W{1'b0}}, mul_fix_out[`MULT_W-1]}) - {2'b0,{(`EXP_W-1){1'b1}}};
            end
        end else begin
            e_result0 = 0;
        end
    end
    assign e_result = e_result0[`EXP_W-1:0];

    // Sign calculation
    assign sign = a_in[16-1] ^ b_in[16-1];

    wire [`M_W-1:0] overflow_mask;
    assign overflow_mask = overflow ? 0 : {(`M_W){1'b1}};

    assign result = {sign, e_result, overflow_mask & M_result};
endmodule
