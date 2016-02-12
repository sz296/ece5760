// --------------------------------------------------------------------
// --------------------------------------------------------------------
//
// Major Functions: Diffusion limited aggregation 
//  state is in m4k blocks
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
// Bruce R Land, Cornell University, Oct 2009
// Improved top module written by Adam Shapiro Oct 2009
// --------------------------------------------------------------------

module DE2_TOP (
    // Clock Input
    input         CLOCK_27,    // 27 MHz
    input         CLOCK_50,    // 50 MHz
    input         EXT_CLOCK,   // External Clock
    
    // Push Button
    input  [3:0]  KEY,         // Pushbutton[3:0]
    
    // DPDT Switch
    input  [17:0] SW,          // Toggle Switch[17:0]
    
    // 7-SEG Display
    output [6:0]  HEX0,        // Seven Segment Digit 0
    output [6:0]  HEX1,        // Seven Segment Digit 1
    output [6:0]  HEX2,        // Seven Segment Digit 2
    output [6:0]  HEX3,        // Seven Segment Digit 3
    output [6:0]  HEX4,        // Seven Segment Digit 4
    output [6:0]  HEX5,        // Seven Segment Digit 5
    output [6:0]  HEX6,        // Seven Segment Digit 6
    output [6:0]  HEX7,        // Seven Segment Digit 7
    
    // LED
    output [8:0]  LEDG,        // LED Green[8:0]
    output [17:0] LEDR,        // LED Red[17:0]
    
    // UART
    output        UART_TXD,    // UART Transmitter
    input         UART_RXD,    // UART Receiver
    
    // IRDA
    output        IRDA_TXD,    // IRDA Transmitter
    input         IRDA_RXD,    // IRDA Receiver
    
    // SDRAM Interface
    inout  [15:0] DRAM_DQ,     // SDRAM Data bus 16 Bits
    output [11:0] DRAM_ADDR,   // SDRAM Address bus 12 Bits
    output        DRAM_LDQM,   // SDRAM Low-byte Data Mask 
    output        DRAM_UDQM,   // SDRAM High-byte Data Mask
    output        DRAM_WE_N,   // SDRAM Write Enable
    output        DRAM_CAS_N,  // SDRAM Column Address Strobe
    output        DRAM_RAS_N,  // SDRAM Row Address Strobe
    output        DRAM_CS_N,   // SDRAM Chip Select
    output        DRAM_BA_0,   // SDRAM Bank Address 0
    output        DRAM_BA_1,   // SDRAM Bank Address 0
    output        DRAM_CLK,    // SDRAM Clock
    output        DRAM_CKE,    // SDRAM Clock Enable
    
    // Flash Interface
    inout  [7:0]  FL_DQ,       // FLASH Data bus 8 Bits
    output [21:0] FL_ADDR,     // FLASH Address bus 22 Bits
    output        FL_WE_N,     // FLASH Write Enable
    output        FL_RST_N,    // FLASH Reset
    output        FL_OE_N,     // FLASH Output Enable
    output        FL_CE_N,     // FLASH Chip Enable
    
    // SRAM Interface
    inout  [15:0] SRAM_DQ,     // SRAM Data bus 16 Bits
    output [17:0] SRAM_ADDR,   // SRAM Address bus 18 Bits
    output        SRAM_UB_N,   // SRAM High-byte Data Mask 
    output        SRAM_LB_N,   // SRAM Low-byte Data Mask 
    output        SRAM_WE_N,   // SRAM Write Enable
    output        SRAM_CE_N,   // SRAM Chip Enable
    output        SRAM_OE_N,   // SRAM Output Enable
    
    // ISP1362 Interface
    inout  [15:0] OTG_DATA,    // ISP1362 Data bus 16 Bits
    output [1:0]  OTG_ADDR,    // ISP1362 Address 2 Bits
    output        OTG_CS_N,    // ISP1362 Chip Select
    output        OTG_RD_N,    // ISP1362 Write
    output        OTG_WR_N,    // ISP1362 Read
    output        OTG_RST_N,   // ISP1362 Reset
    output        OTG_FSPEED,  // USB Full Speed, 0 = Enable, Z = Disable
    output        OTG_LSPEED,  // USB Low Speed,  0 = Enable, Z = Disable
    input         OTG_INT0,    // ISP1362 Interrupt 0
    input         OTG_INT1,    // ISP1362 Interrupt 1
    input         OTG_DREQ0,   // ISP1362 DMA Request 0
    input         OTG_DREQ1,   // ISP1362 DMA Request 1
    output        OTG_DACK0_N, // ISP1362 DMA Acknowledge 0
    output        OTG_DACK1_N, // ISP1362 DMA Acknowledge 1
    
    // LCD Module 16X2
    inout  [7:0]  LCD_DATA,    // LCD Data bus 8 bits
    output        LCD_ON,      // LCD Power ON/OFF
    output        LCD_BLON,    // LCD Back Light ON/OFF
    output        LCD_RW,      // LCD Read/Write Select, 0 = Write, 1 = Read
    output        LCD_EN,      // LCD Enable
    output        LCD_RS,      // LCD Command/Data Select, 0 = Command, 1 = Data
    
    // SD Card Interface
    inout         SD_DAT,      // SD Card Data
    inout         SD_DAT3,     // SD Card Data 3
    inout         SD_CMD,      // SD Card Command Signal
    output        SD_CLK,      // SD Card Clock
    
    // I2C
    inout         I2C_SDAT,    // I2C Data
    output        I2C_SCLK,    // I2C Clock
    
    // PS2
    input         PS2_DAT,     // PS2 Data
    input         PS2_CLK,     // PS2 Clock
    
    // USB JTAG link
    input         TDI,         // CPLD -> FPGA (data in)
    input         TCK,         // CPLD -> FPGA (clk)
    input         TCS,         // CPLD -> FPGA (CS)
    output        TDO,         // FPGA -> CPLD (data out)
    
    // VGA
    output        VGA_CLK,     // VGA Clock
    output        VGA_HS,      // VGA H_SYNC
    output        VGA_VS,      // VGA V_SYNC
    output        VGA_BLANK,   // VGA BLANK
    output        VGA_SYNC,    // VGA SYNC
    output [9:0]  VGA_R,       // VGA Red[9:0]
    output [9:0]  VGA_G,       // VGA Green[9:0]
    output [9:0]  VGA_B,       // VGA Blue[9:0]
    
    // Ethernet Interface
    inout  [15:0] ENET_DATA,   // DM9000A DATA bus 16Bits
    output        ENET_CMD,    // DM9000A Command/Data Select, 0 = Command, 1 = Data
    output        ENET_CS_N,   // DM9000A Chip Select
    output        ENET_WR_N,   // DM9000A Write
    output        ENET_RD_N,   // DM9000A Read
    output        ENET_RST_N,  // DM9000A Reset
    input         ENET_INT,    // DM9000A Interrupt
    output        ENET_CLK,    // DM9000A Clock 25 MHz
    
    // Audio CODEC
    inout         AUD_ADCLRCK, // Audio CODEC ADC LR Clock
    input         AUD_ADCDAT,  // Audio CODEC ADC Data
    inout         AUD_DACLRCK, // Audio CODEC DAC LR Clock
    output        AUD_DACDAT,  // Audio CODEC DAC Data
    inout         AUD_BCLK,    // Audio CODEC Bit-Stream Clock
    output        AUD_XCK,     // Audio CODEC Chip Clock
    
    // TV Decoder
    input  [7:0]  TD_DATA,     // TV Decoder Data bus 8 bits
    input         TD_HS,       // TV Decoder H_SYNC
    input         TD_VS,       // TV Decoder V_SYNC
    output        TD_RESET,    // TV Decoder Reset
    
    // GPIO
    inout  [35:0] GPIO_0,      // GPIO Connection 0
    inout  [35:0] GPIO_1       // GPIO Connection 1
);

    // Turn off all displays
    assign HEX0 = 7'h7F;
    assign HEX1 = 7'h7F;
    assign HEX2 = 7'h7F;
    assign HEX3 = 7'h7F;
    assign HEX4 = 7'h7F;
    assign HEX5 = 7'h7F;
    assign HEX6 = 7'h7F;
    assign HEX7 = 7'h7F;
    assign LEDR = 18'h0;
    assign LEDG = 9'h0;
   
    //Set all GPIO to tri-state
    assign GPIO_0 = 36'hzzzzzzzzz;
    assign GPIO_1 = 36'hzzzzzzzzz;

    // Disable audio codec
    assign AUD_DACDAT = 1'b0;
    assign AUD_XCK    = 1'b0;

    // Disable DRAM
    assign DRAM_ADDR  = 12'h0;
    assign DRAM_BA_0  = 1'b0;
    assign DRAM_BA_1  = 1'b0;
    assign DRAM_CAS_N = 1'b1;
    assign DRAM_CKE   = 1'b0;
    assign DRAM_CLK   = 1'b0;
    assign DRAM_CS_N  = 1'b1;
    assign DRAM_DQ    = 16'hzzzz;
    assign DRAM_LDQM  = 1'b0;
    assign DRAM_RAS_N = 1'b1;
    assign DRAM_UDQM  = 1'b0;
    assign DRAM_WE_N  = 1'b1;

    // Disable Ethernet
    assign ENET_CLK   = 1'b0;
    assign ENET_CS_N  = 1'b1;
    assign ENET_CMD   = 1'b0;
    assign ENET_DATA  = 16'hzzzz;
    assign ENET_RD_N  = 1'b1;
    assign ENET_RST_N = 1'b1;
    assign ENET_WR_N  = 1'b1; 

    // Disable flash
    assign FL_ADDR  = 22'h0;
    assign FL_CE_N  = 1'b1;
    assign FL_DQ    = 8'hzz;
    assign FL_OE_N  = 1'b1;
    assign FL_RST_N = 1'b1;
    assign FL_WE_N  = 1'b1;

    // Disable LCD
    assign LCD_BLON = 1'b0;
    assign LCD_DATA = 8'hzz;
    assign LCD_EN   = 1'b0;
    assign LCD_ON   = 1'b0;
    assign LCD_RS   = 1'b0;
    assign LCD_RW   = 1'b0;

    // Disable OTG
    assign OTG_ADDR    = 2'h0;
    assign OTG_CS_N    = 1'b1;
    assign OTG_DACK0_N = 1'b1;
    assign OTG_DACK1_N = 1'b1;
    assign OTG_FSPEED  = 1'b1;
    assign OTG_DATA    = 16'hzzzz;
    assign OTG_LSPEED  = 1'b1;
    assign OTG_RD_N    = 1'b1;
    assign OTG_RST_N   = 1'b1;
    assign OTG_WR_N    = 1'b1;

    // Disable SDRAM
    assign SD_DAT = 1'bz;
    assign SD_CLK = 1'b0;

    // Disable SRAM
    assign SRAM_ADDR = 18'h0;
    assign SRAM_CE_N = 1'b1;
    assign SRAM_DQ   = 16'hzzzz;
    assign SRAM_LB_N = 1'b1;
    assign SRAM_OE_N = 1'b1;
    assign SRAM_UB_N = 1'b1;
    assign SRAM_WE_N = 1'b1;

    // Disable all other peripherals
    // assign I2C_SCLK = 1'b0;
    assign IRDA_TXD = 1'b0;
    assign TDO = 1'b0;
    assign UART_TXD = 1'b0;
   
    wire VGA_CTRL_CLK;
    wire AUD_CTRL_CLK;
    wire DLY_RST;

    assign TD_RESET    = 1'b1; // Allow 27 MHz
    assign AUD_ADCLRCK = AUD_DACLRCK;
    assign AUD_XCK     = AUD_CTRL_CLK;

    Reset_Delay r0 ( 
        .iCLK(CLOCK_50),
        .oRESET(DLY_RST)
    );

    VGA_Audio_PLL p1 ( 
        .areset(~DLY_RST),
        .inclk0(CLOCK_27),
        .c0(VGA_CTRL_CLK),
        .c1(AUD_CTRL_CLK),
        .c2(VGA_CLK)
    );

    VGA_Controller u1 ( 
        // Host Side
        .iCursor_RGB_EN(4'b0111),
        .oAddress(mVGA_ADDR),
        .oCoord_X(Coord_X),
        .oCoord_Y(Coord_Y),
        .iRed(mVGA_R),
        .iGreen(mVGA_G),
        .iBlue(mVGA_B),
    
        // VGA Side
        .oVGA_R(VGA_R),
        .oVGA_G(VGA_G),
        .oVGA_B(VGA_B),
        .oVGA_H_SYNC(VGA_HS),
        .oVGA_V_SYNC(VGA_VS),
        .oVGA_SYNC(VGA_SYNC),
        .oVGA_BLANK(VGA_BLANK),
    
        // Control Signal
        .iCLK(VGA_CTRL_CLK),
        .iRST_N(DLY_RST)    
    );

    wire [9:0]  mVGA_R;           // Memory output to VGA
    wire [9:0]  mVGA_G;
    wire [9:0]  mVGA_B;
    wire [18:0] mVGA_ADDR;        // Video memory address
    wire [9:0]  Coord_X, Coord_Y; // Display coods

    ///////////////////////////////////////////////////////////////////////////
    // Cellular Automaton state machine variables                            //
    ///////////////////////////////////////////////////////////////////////////
    wire        RESET;
    wire        state_bit;     // Current data from m4k to state machine
    wire        mem_bit;       // Current data from m4k to VGA
    reg [4:0]   STATE;         // State machine
    reg [4:0]   RETURN;        // Return to this state

    reg         disp_bit;      // Registered data from m4k to VGA
    
    reg         WREN;          // Write enable for a
    reg [18:0]  ADDR;          // for a
    reg         DATA;          // for a
    
    reg  [9:0]  CA_X;          // x-position of cell to be generated
    reg  [8:0]  CA_Y;          // y-position of cell to be generated
    reg  [2:0]  CA_INDEX;      // Index into 3-bit rule
    reg  [7:0]  CA_RULE;       // 8-bit rule read from switches
    reg         MODE;          // Intialize to one cell or a random vector
 
    
    reg  [15:0] LFSR;          // Linear feedback shift register
    reg  [15:0] SEED;          // Random seed for LFSR
    wire        LFSR_LO;       // Least significant bit of shift register
    
    reg         CONTINUE_HOLD; // flag set when continue key is pressed
    
    vga_buffer display(
        .address_a (ADDR) , 
        .address_b ( {Coord_X[9:0], Coord_Y[8:0]} ), // vga current address
        .clock_a (VGA_CTRL_CLK),
        .clock_b (VGA_CTRL_CLK),
        .data_a (DATA),
        .data_b (1'b0), // never write on port b
        .wren_a (WREN),
        .wren_b (1'b0), // never write on port b
        .q_a (state_bit),
        .q_b (mem_bit)
    );

    // Color white
    assign  mVGA_R = {10{disp_bit}} ;
    assign  mVGA_G = {10{disp_bit}} ;
    assign  mVGA_B = {10{disp_bit}} ;

    // Connect reset and continue keys
    assign RESET    = ~KEY[2];
    assign CONTINUE = ~KEY[3];
    
    // Connect feedback loop of shift register
    assign LFSR_LO = LFSR[15] ^ LFSR[14];

    // State names
    parameter INIT   = 5'd0,
              DRAW_1 = 5'd1,
              DRAW_2 = 5'd2,
              CA_1   = 5'd3,
              CA_2   = 5'd4,
              CA_3   = 5'd5,
              CA_4   = 5'd6,
              CA_5   = 5'd7,
              CA_6   = 5'd8,
              WAIT   = 5'd9,
              CONT_1 = 5'd10,
              CONT_2 = 5'd11,
              CONT_3 = 5'd12,
              RAND_1 = 5'd13,
              RAND_2 = 5'd14;

    always @ (negedge VGA_CTRL_CLK)
    begin
        // register the m4k output for better timing on VGA
        // negedge seems to work better than posedge
        disp_bit <= mem_bit;
    end

    always @ (posedge VGA_CTRL_CLK) // VGA_CTRL_CLK
    begin
 
        if ( RESET )  // Sync reset assumes KEY3 is held down 1/60 second
        begin
            // Clear the screen
            ADDR <= {Coord_X[9:0], Coord_Y[8:0]}; // [17:0]
            WREN <= 1'b1;                         // Write some memory
            DATA <= 1'b0;                         // Write all zeros (black) 
            CA_RULE <= SW[7:0];                   // Set CA rule
            MODE <= SW[17];                       // Set mode: initialize with one cell or a random vector
            LFSR <= SEED;                         // Initialize linear feedback shift register

            if ( ~MODE )    
            begin
                STATE <= INIT;                    // Write one cell to top center of screen
            end
            else
            begin
                STATE <= RAND_1;                  // Write a random vector to top of screen
            end
            
        end
 

        else
        
        //
        // Finite state machine: computes CA and writing to VGA
        //
        begin
            case( STATE )
            
                // Intialize state to a single cell centered at top of screen
                INIT:
                begin
                    WREN <= 1'b0;
                    ADDR <= {10'd304, 9'd1};
                    DATA <= 1'b1;
                    
                    CA_X <= 10'd1;
                    CA_Y <=  9'd2;
                    
                    RETURN <= CA_1;
                    
                    STATE <= DRAW_1;
                end
                
                // Set write enable
                DRAW_1:
                begin
                    WREN <= 1'b1;
                    
                    STATE <= DRAW_2;
                end
                
                // Clear write enable
                DRAW_2:
                begin
                    WREN <= 1'b0;
                    
                    STATE <= RETURN;
                end
                
                // Read upper left cell
                CA_1:
                begin
                    WREN <= 1'b0;
                    CA_INDEX <= 3'd0;
                    ADDR <= {CA_X - 10'd1, CA_Y - 9'd1};
                    
                    STATE <= CA_2;
                end
                
                // Read cell directly above
                CA_2:
                begin
                    WREN <= 1'b0;
                    ADDR <= {CA_X        , CA_Y - 9'd1};
                    
                    STATE <= CA_3;
                end
                
                // Read upper right cell; Compute index into rule using result from upper left cell
                CA_3:
                begin
                    WREN <= 1'b0;
                    ADDR <= {CA_X + 10'd1, CA_Y - 9'd1};
                    CA_INDEX <= CA_INDEX + {state_bit, 1'b0, 1'b0};
                    
                    STATE <= CA_4;
                end
                
                // Compute index into rule using result from cell directly above
                CA_4:
                begin
                    WREN <= 1'b0;
                    CA_INDEX <= CA_INDEX + {1'b0, state_bit, 1'b0};
                    
                    STATE <= CA_5;
                end
                
                // Compute index into rule using result from upper right cell
                CA_5:
                begin
                    WREN <= 1'b0;
                    CA_INDEX <= CA_INDEX + {1'b0, 1'b0, state_bit};
                    
                    STATE <= CA_6;
                end
                
                // Write cell to VGA
                CA_6:
                begin
                    WREN <= 1'b0;
                    DATA <= CA_RULE[CA_INDEX];
                    ADDR <= {CA_X, CA_Y};
                    
                    // If end of screen, wait until signaled to continue by key press
                    if (CA_X == 10'd639 && CA_Y == 9'd479)
                    begin 
                        RETURN <= WAIT;
                        STATE <= DRAW_1;
                    end
                    
                    // Otherwise, if end of row, continue to first cell of next row
                    else if (CA_X == 10'd639)
                    begin
                        CA_X <= 10'd1;
                        CA_Y <= CA_Y + 9'd1;
                        
                        RETURN <= CA_1;
                        STATE <= DRAW_1;
                    end
                    
                    // Otherwise, continue generating current row
                    else
                    begin
                        CA_X <= CA_X + 1;
                        
                        RETURN <= CA_1;
                        STATE <= DRAW_1;
                    end
                     
                end
      
                // Wait until signaled to continue by key press
                WAIT:
                begin
                    WREN <= 1'b0;
                    
                    // Modify LFSR to add randomness
                    SEED <= SEED + 16'd1;
        
                    // If continue key pressed, copy bottom line to top of screen
                    if ( CONTINUE && ~CONTINUE_HOLD )
                    begin
                        CA_X <= 10'd1;
                        CONTINUE_HOLD <= 1'b1;
                        
                        STATE <= CONT_1;
                    end
      
                    // Otherwise, if key has been released, clear flag and wait
                    else if ( ~CONTINUE )
                    begin
                        CONTINUE_HOLD <= 1'b0;
                    
                        STATE <= WAIT;
                    end
                    
                    // Otherwise, key is currently still pressed; wait for key to be released
                    else
                    begin
                        STATE <= WAIT;
                    end
                end
      
                // Read cell at bottom of screen
                CONT_1:
                begin
                    WREN <= 1'b0;
                    ADDR <= {CA_X, 9'd479};
                  
                    STATE <= CONT_2;
                end
                
                // Wait one cycle
                CONT_2:
                begin
                    WREN <= 1'b0;
                  
                    STATE <= CONT_3;
                end
                
                // Copy cell to top of screen
                CONT_3:
                begin
                    WREN <= 1'b0;
                    ADDR <= {CA_X, 9'd1};
                    DATA <= state_bit;
                  
                    // If end of row, move onto next row and continue generating rest of screen
                    if (CA_X == 10'd639)
                    begin
                        CA_X <= 10'd1;
                        CA_Y <=  9'd2;
                        RETURN <= CA_1;
                    end
                    
                    // Otherwise, continue copying bottom line to top of screen
                    else
                    begin
                        CA_X <= CA_X + 10'd1;
                        RETURN <= CONT_1;
                    end
                    
                    STATE <= DRAW_1;
                end
                
                // Write output of LSFR to first cell top row
                RAND_1:
                begin
                    WREN <= 1'b0;
                    ADDR <= {10'd1, 9'd1};
                    DATA <= LFSR[15];
                    LFSR <= {LFSR[14:0], LFSR_LO}; // update shift register
                    
                    CA_X <= 10'd2;
                    
                    RETURN <= RAND_2;
                    STATE <= DRAW_1;
                end
                
                // Continue writing output of LSFR to top row
                RAND_2:
                begin
                    WREN <= 1'b0;
                    ADDR <= {CA_X, 9'd1};
                    DATA <= LFSR[15];
                    LFSR <= {LFSR[14:0], LFSR_LO}; // update shift register
                    
                    CA_X <= CA_X + 10'd1; // update x-position
                    
                    // If end of row, then move onto next row and continue generating rest of screen
                    if ( CA_X == 10'd639 )
                    begin
                        CA_Y <= 10'd2;
                        RETURN <= CA_1;
                    end
                    
                    // Otherwise, continue writing random vector to top row
                    else
                    begin
                        RETURN <= RAND_2;
                    end
                    
                    STATE <= DRAW_1;
                end
                
            endcase
        end 
    end

endmodule
