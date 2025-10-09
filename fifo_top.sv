`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/04/2025 07:56:50 PM
// Design Name: 
// Module Name: fifo_top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module fifo_top(

 input CLK100,
    output [9:0] LED,       // RGB1, RGB0, LED 9..0 placed from left to right
    output [2:0] RGB0,      
    output [2:0] RGB1,
    output [3:0] SS_ANODE,   // Anodes 3..0 placed from left to right
    output [7:0] SS_CATHODE, // Bit order: DP, G, F, E, D, C, B, A
    input [11:0] SW,         // SWs 11..0 placed from left to right
    input [3:0] PB,          // PBs 3..0 placed from left to right
    inout [23:0] GPIO,       // PMODA-C 1P, 1N, ... 3P, 3N order
    output [3:0] SERVO,      // Servo outputs
    output PDM_SPEAKER,      // PDM signals for mic and speaker
    input PDM_MIC_DATA,      
    output PDM_MIC_CLK,
    output ESP32_UART1_TXD,  // WiFi/Bluetooth serial interface 1
    input ESP32_UART1_RXD,
    output IMU_SCLK,         // IMU spi clk
    output IMU_SDI,          // IMU spi data input
    input IMU_SDO_AG,        // IMU spi data output (accel/gyro)
    input IMU_SDO_M,         // IMU spi data output (mag)
    output IMU_CS_AG,        // IMU cs (accel/gyro) 
    output IMU_CS_M,         // IMU cs (mag)
    input IMU_DRDY_M,        // IMU data ready (mag)
    input IMU_INT1_AG,       // IMU interrupt (accel/gyro)
    input IMU_INT_M,         // IMU interrupt (mag)
    output IMU_DEN_AG,       // IMU data enable (accel/gyro)
    inout [14:0]DDR_addr,
    inout [2:0]DDR_ba,
    inout DDR_cas_n,
    inout DDR_ck_n,
    inout DDR_ck_p,
    inout DDR_cke,
    inout DDR_cs_n,
    inout [3:0]DDR_dm,
    inout [31:0]DDR_dq,
    inout [3:0]DDR_dqs_n,
    inout [3:0]DDR_dqs_p,
    inout DDR_odt,
    inout DDR_ras_n,
    inout DDR_reset_n,
    inout DDR_we_n,
    inout FIXED_IO_ddr_vrn,
    inout FIXED_IO_ddr_vrp,
    inout [53:0]FIXED_IO_mio,
    inout FIXED_IO_ps_clk,
    inout FIXED_IO_ps_porb,
    inout FIXED_IO_ps_srstb

    );
    
     // Terminate all of the unused outputs or i/o's
    // assign LED = 10'b0000000000;
    //assign RGB0 = 3'b000;
    assign RGB1 = 3'b000;
    // assign SS_ANODE = 4'b0000;
    // assign SS_CATHODE = 8'b11111111;
    // assign GPIO = 24'bzzzzzzzzzzzzzzzzzzzzzzzz;
    assign SERVO = 4'b0000;
    assign PDM_SPEAKER = 1'b0;
    assign PDM_MIC_CLK = 1'b0;
    assign ESP32_UART1_TXD = 1'b0;
    assign IMU_SCLK = 1'b0;
    assign IMU_SDI = 1'b0;
    assign IMU_CS_AG = 1'b1;
    assign IMU_CS_M = 1'b1;
    assign IMU_DEN_AG = 1'b0;

    // display g (gpio) on left seven segment display
   // assign SS_ANODE = 4'b0111;
   // assign SS_CATHODE = 8'b10010000;
    
    reg pre_reset;
    reg reset;
    
    reg pre_clear;
    reg clear;
    
    reg pre_read;
    reg READ;
    
    reg pre_write;
    reg WRITE;
    
    reg [7:0] pre_DATA;
    reg [7:0] DATA;
    
    reg pre_MODE;
    reg MODE;
    
    reg [7:0] pre_leds;
    reg [7:0] leds;
    
    
    //Metastability blocks 
    always_ff @(posedge CLK100)
    begin 
      pre_reset <= PB[0];
      reset <= pre_reset;
    end
    
    always_ff @(posedge CLK100)
    begin 
      pre_clear <= PB[1];
      clear <= pre_clear;
    end 
    
    always_ff @(posedge CLK100)
    begin
      pre_read <= PB[2];
      READ <= pre_read;
    end 
    
    always_ff @(posedge CLK100)
    begin 
      pre_write <= PB[3];
      WRITE <= pre_write;
    end 
    
    always_ff @(posedge CLK100)
    begin 
      pre_DATA <= SW[7:0];
      DATA <= pre_DATA;
    end 
    
    always_ff @(posedge CLK100) 
    begin 
      pre_MODE <= SW[9];
      MODE <= pre_MODE;
    end
    
    always_ff @(posedge CLK100)
    begin
      pre_leds <= LED[7:0];
      leds <= pre_leds;
    end  
    
    //Debounce the read- PB[2] switche
    
    wire read_to_edgeDetect;
    
    signal_debounce inst_read(
      .slow_clock_input(CLK100),
      .signal_in(READ),
      .single_pulse_out(read_to_edgeDetect)
    );
     
    //edge detector for reading 
    
    reg sig_read;
    reg sig_delay_read;
    wire pe_read;
     
   
   
    always_ff @(posedge CLK100 or posedge reset)
    begin
      if (reset) 
      begin
        sig_read <= 0;
        sig_delay_read <= 0;
      end 
      else 
      begin 
        sig_read <= read_to_edgeDetect;
        sig_delay_read <= sig_read;
      end 
     end
        
    assign pe_read = sig_read && ~sig_delay_read;
    
    
    wire write_to_edgeDetect;
    
    //Debounce the PB[3] write switch 
    signal_debounce inst_write(
      .slow_clock_input(CLK100),
      .signal_in(WRITE),
      .single_pulse_out(write_to_edgeDetect)
    );
    
    //edge detector for writing
    reg sig_write;
    reg sig_delay_write;
    wire pe_write;
    

    always_ff @(posedge CLK100 or posedge reset)
    begin
      if (reset) 
      begin
        sig_write <= 0;
        sig_delay_write <= 0;
      end 
      else 
      begin 
        sig_write <= write_to_edgeDetect;
        sig_delay_write <= sig_write;
      end 
     end
    
    assign pe_write = sig_write && ~sig_delay_write;
    
      
    wire [3:0] write_index;
    wire [3:0] read_index;
    wire [7:0] rd_wr_position;
    wire [7:0] fifo_data_read;
    
    // rd_wr_position [3:0] = read_index 
    // rd_wr_position [7:4] = write_index
    assign rd_wr_position = {write_index, read_index};
    
    //Fifo instantiation for writing and reading 
    fifo fifo_READ_inst(
      .clk(CLK100),
      .reset(reset),
      .wr_data(DATA),
      .wr_request(pe_write),
      .rd_data(fifo_data_read),
      .rd_request(pe_read),
      .empty(RGB0[0]), //Red
      .full(RGB0[1]),  //Green
      .overflow(RGB0[2]), //Blue
      .clear_overflow_request(clear),
      //.mode(MODE),
      .wr_index(write_index),
      .rd_index(read_index)
   );
   
   
   
   //After receiving the data from the fifo module, decide what to output 
   assign LED[7:0] = (~MODE)? fifo_data_read : rd_wr_position;
   
   //Writing Data
//    fifo fifo_WRITE_inst(
//      .clk(CLK100),
//      .reset(reset),
//      .wr_data(DATA),
//      .wr_request(pe_write),
//      .rd_data(),
//      .rd_request(),
//      .empty(RGB0[0]),
//      .full(RGB0[1]),
//      .overflow(RGB0[2]),
//      .clear_overflow_request(clear),
//      .wr_index(LED[3:0]),
//      .rd_index(LED[7:4]),
//      .mode(LED[9])
//  );
    
    
    
endmodule
