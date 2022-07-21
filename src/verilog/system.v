`timescale 1 ns / 1 ps

module system (
	input            clk,
	input            resetn,
	input enable,
	output           trap,
	input 	irq, //input for interrupts
	output reg [7:0] out_byte,
	input 			 base_sel, //switch to change between decimal to hexadecimal
	output reg       out_byte_en,
	//output reg [7:0] irq_counter, //interrupt counter
	output     [7:0] output_anodo,
	output     [7:0] output_catodo
);
	// set this to 0 for better timing but less performance/MHz
	
	reg [3:0] selector;
	
	parameter FAST_MEMORY = 1;
    
	// 4096 32bit words = 16kB memory
	parameter MEM_SIZE = 4096;

	wire mem_valid;
	wire mem_instr;
	reg mem_ready;
	wire [31:0] mem_addr;
	wire [31:0] mem_wdata;
	wire [3:0] mem_wstrb;
	reg [31:0] mem_rdata;

	wire mem_la_read;
	wire mem_la_write;
	wire [31:0] mem_la_addr;
	wire [31:0] mem_la_wdata;
	wire [3:0] mem_la_wstrb;

	reg [31:0] out_byte32;
	//Instance of the processor with interrupt signals
	//and module parameters
	picorv32  #(
		.ENABLE_IRQ(1),
		.ENABLE_IRQ_QREGS(0),
		.LATCHED_IRQ(0)
	)
	picorv32_core (
		.clk         (clk         ),
		.resetn      (resetn      ),
		.trap        (trap        ),
		.mem_valid   (mem_valid   ),
		.mem_instr   (mem_instr   ),
		.mem_ready   (mem_ready   ),
		.mem_addr    (mem_addr    ),
		.mem_wdata   (mem_wdata   ),
		.mem_wstrb   (mem_wstrb   ),
		.mem_rdata   (mem_rdata   ),
		.mem_la_read (mem_la_read ),
		.mem_la_write(mem_la_write),
		.mem_la_addr (mem_la_addr ),
		.mem_la_wdata(mem_la_wdata),
		.mem_la_wstrb(mem_la_wstrb),
		.irq ({28'b0, irq, 3'b0})
	);
	
	/*text_display t(
	 	.clk 			(clk),
	 	.text_selector 	(selector),
		.enable			(enable),
	 	.resetn 		(resetn),
	 	.output_anodo 	(output_anodo),
	 	.output_catodo	(output_catodo)

    );*/
    
    
  seven_segment_switch ssswitch_inst(
		out_byte,
		clk,
		resetn,
		base_sel,
		output_anodo,
		output_catodo
	);
	
   always @(posedge clk) out_byte <= out_byte32[7:0];
    
	reg [31:0] memory [0:MEM_SIZE-1];

`ifdef SYNTHESIS
    initial $readmemh("../firmware/firmware.hex", memory);
`else
	initial $readmemh("firmware.hex", memory);
`endif

	reg [31:0] m_read_data;
	reg m_read_en;

	generate if (FAST_MEMORY) begin
		always @(posedge clk) begin
			mem_ready <= 1;
			out_byte_en <= 0;
			mem_rdata <= memory[mem_la_addr >> 2];
			if (mem_la_write && (mem_la_addr >> 2) < MEM_SIZE) begin
				if (mem_la_wstrb[0]) memory[mem_la_addr >> 2][ 7: 0] <= mem_la_wdata[ 7: 0];
				if (mem_la_wstrb[1]) memory[mem_la_addr >> 2][15: 8] <= mem_la_wdata[15: 8];
				if (mem_la_wstrb[2]) memory[mem_la_addr >> 2][23:16] <= mem_la_wdata[23:16];
				if (mem_la_wstrb[3]) memory[mem_la_addr >> 2][31:24] <= mem_la_wdata[31:24];
			end
			else
			if (mem_la_write && mem_la_addr == 32'h1000_0000) begin
				out_byte_en <= 1;
				//out_byte <= mem_la_wdata;
			end
			/*if (mem_la_write && mem_la_addr == 32'h1000_0004) begin
				out_byte <= mem_la_wdata; //adds one when it detects an interruption
				out_byte_en <= 1; //enable out?byte signal
			end*/
			if (mem_la_write && mem_la_addr == 32'h1000_0010) begin 
			    out_byte_en <= 1; //enable out?byte signal
				out_byte32 <= mem_la_wdata; //adds one when it detects an interruption
				
			end
		end
		//case not taken in the presence of FAST_MEMORY
	end else begin
		always @(posedge clk) begin
			m_read_en <= 0;
			mem_ready <= mem_valid && !mem_ready && m_read_en;

			m_read_data <= memory[mem_addr >> 2];
			mem_rdata <= m_read_data;

			out_byte_en <= 0;

			(* parallel_case *)
			case (1)
				mem_valid && !mem_ready && !mem_wstrb && (mem_addr >> 2) < MEM_SIZE: begin
					m_read_en <= 1;
				end
				mem_valid && !mem_ready && |mem_wstrb && (mem_addr >> 2) < MEM_SIZE: begin
					if (mem_wstrb[0]) memory[mem_addr >> 2][ 7: 0] <= mem_wdata[ 7: 0];
					if (mem_wstrb[1]) memory[mem_addr >> 2][15: 8] <= mem_wdata[15: 8];
					if (mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
					if (mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
					mem_ready <= 1;
				end
				mem_valid && !mem_ready && |mem_wstrb && mem_addr == 32'h1000_0000: begin
					out_byte_en <= 1;
					out_byte <= mem_wdata;
					mem_ready <= 1;
				end
			endcase
		end
	end endgenerate
endmodule



module seven_segment_dec (
    input clk,
    input resetn,
    input [31:0] num_in,
    output reg [7:0] c_out,
    output reg [7:0] an_out
);

reg [20:0] contador; 
wire [2:0] refresh; 
reg [3:0] numero_BCD;


always @(posedge clk) begin 
    if(!resetn)
        contador <= 0;
    else
        contador <= contador + 1;
end 



    assign refresh = contador[20:18];


// 98765432 / 10000000 = 9
// 98765432 % 10000000 = 8765432 / 1000000 = 8

always @(*) begin
    if (!resetn) begin
            c_out = 8'b11111111; // "0" 
            an_out = 8'b11111110;      
    end
    else begin
        case(refresh)
            3'b000: begin
                an_out = 8'b01111111; 

                numero_BCD = num_in/10000000;            
            end
            3'b001: begin
                an_out = 8'b10111111; 

                numero_BCD = (num_in%10000000)/1000000;            
            end
            3'b010: begin
                an_out = 8'b11011111; 

                numero_BCD = ((num_in%10000000)%1000000)/100000;            
            end
            3'b011: begin
                an_out = 8'b11101111; 

                numero_BCD = (((num_in%10000000)%1000000)%100000)/10000;            
            end
            3'b100: begin
                an_out = 8'b11110111; 

                numero_BCD = ((((num_in%10000000)%1000000)%100000)%10000)/1000;            
            end
            3'b101: begin
                an_out = 8'b11111011; 

                numero_BCD = (((((num_in%10000000)%1000000)%100000)%10000)%1000)/100;            
            end
            3'b110: begin
                an_out = 8'b11111101; 

                numero_BCD = ((((((num_in%10000000)%1000000)%100000)%10000)%1000)%100)/10;            
            end
            3'b111: begin
                an_out = 8'b11111110; 
                
                numero_BCD = ((((((num_in%10000000)%1000000)%100000)%10000)%1000)%100)%10;            
            end
        endcase

        case(numero_BCD)
            4'b0000: 
                c_out = 8'b11000000;      
            4'b0001: 
                c_out = 8'b11111001; 
            4'b0010: 
                c_out = 8'b10100100;  
            4'b0011: 
                c_out = 8'b10110000;  
            4'b0100:    
                c_out = 8'b10011001;  
            4'b0101: 
                c_out = 8'b10010010; 
            4'b0110: 
                c_out = 8'b10000010;  
            4'b0111: 
                c_out = 8'b11111000;  
            4'b1000:    
                c_out = 8'b10000000;      
            4'b1001: 
                c_out = 8'b10011000;  
            default: 
                c_out = 8'b10011000; 
        endcase
    end
end
endmodule

//hexadecimal numbers
module seven_segment_hex(
	input  [31:0] num_in,
	input  clk,
	input  resetn,
	output [7:0] an_out,
	output [7:0] c_out
);

        reg [20:0] big_cnt;
        wire [2:0] cnt;
        reg [3:0] num_bcd;
        wire [7:0] an_outn;

        bin_to_one_hot u_bin_to_one_hot (
        .bin        (cnt),
        .one_hot    (an_outn)
        );

        bcd_to_7seg u_bcd_to_7seg (
        .resetn      (resetn),
        .num_bcd     (num_bcd),
        .num_7seg    (c_out)
        );

        always @(*) begin
                case (cnt)
                        0: num_bcd = num_in[3:0];
                        1: num_bcd = num_in[7:4];
                        2: num_bcd = num_in[11:8];
                        3: num_bcd = num_in[15:12];
                        4: num_bcd = num_in[19:16];
                        5: num_bcd = num_in[23:20];
                        6: num_bcd = num_in[27:24];
                        7: num_bcd = num_in[31:28];
                endcase
        end

        always @(posedge clk) begin 
                if (!resetn) begin
                        big_cnt <= 0;
                end
                else begin
                        big_cnt <= big_cnt + 1; 
                end
        end 

        assign an_out = ~an_outn;
        assign cnt = big_cnt[20:18];
endmodule

module bin_to_one_hot(
        input [2:0] bin,
        output reg [7:0] one_hot
);
        always @(*) begin
                case (bin)
                        0: one_hot = 8'b00000001;
                        1: one_hot = 8'b00000010;
                        2: one_hot = 8'b00000100;
                        3: one_hot = 8'b00001000;
                        4: one_hot = 8'b00010000;
                        5: one_hot = 8'b00100000;
                        6: one_hot = 8'b01000000;
                        7: one_hot = 8'b10000000;
                endcase
        end
endmodule

module bcd_to_7seg(
        input resetn,
	input  [3:0] num_bcd,
	output reg [7:0] num_7seg
);
        always @(*) begin
                if (!resetn) begin
                        num_7seg = 8'b11111111; // "0"     
                end
                else begin
                        case (num_bcd)
                                4'h00: num_7seg = 8'b11000000; // "0"     
                                4'h01: num_7seg = 8'b11111001; // "1" 
                                4'h02: num_7seg = 8'b10100100; // "2" 
                                4'h03: num_7seg = 8'b10110000; // "3" 
                                4'h04: num_7seg = 8'b10011001; // "4" 
                                4'h05: num_7seg = 8'b10010010; // "5" 
                                4'h06: num_7seg = 8'b10000010; // "6" 
                                4'h07: num_7seg = 8'b11111000; // "7" 
                                4'h08: num_7seg = 8'b10000000; // "8" 
                                4'h09: num_7seg = 8'b10011000; // "9" 
                                4'h0a: num_7seg = 8'b10001000; // "a" 
                                4'h0b: num_7seg = 8'b10000011; // "b"  
                                4'h0c: num_7seg = 8'b11000110; // "c" 
                                4'h0d: num_7seg = 8'b10100001; // "d" 
                                4'h0e: num_7seg = 8'b10000110; // "e" 
                                4'h0f: num_7seg = 8'b10001110; // "f" 
                        endcase
                end
        end
endmodule




module seven_segment_switch(
	input  [31:0] num_in,
	input  clk,
	input  resetn,
	input base_sel,
	output reg [7:0] an_out,
	output reg [7:0] c_out
);

        wire [7:0] an_out_dec;
        wire [7:0] an_out_hex;
        wire [7:0] c_out_dec;
        wire [7:0] c_out_hex;

        seven_segment_hex u_seven_segment_hex (
                .num_in    (num_in),
                .clk       (clk),
                .resetn    (resetn),
                .an_out    (an_out_hex),
                .c_out     (c_out_hex)
        );

        seven_segment_dec u_seven_segment_dec (
                .num_in    (num_in),
                .clk       (clk),
                .resetn    (resetn),
                .an_out    (an_out_dec),
                .c_out     (c_out_dec)
        );

       always @(*) begin
                if (base_sel) begin
                        an_out = an_out_dec;
                        c_out = c_out_dec;
                end 
                else begin
                        an_out = an_out_hex;
                        c_out = c_out_hex;
                end
        end
endmodule


module text_display
(
    input      clk,
    input      resetn,
    input      enable,
    input      [3:0] text_selector,
    output reg [7:0] output_anodo,
	output reg [7:0] output_catodo
);
reg [20:0] counter;
wire [2:0] refresh; 
reg [3:0] numero_BCD;


always @(posedge clk) begin 
    if(!resetn)
        counter <= 0;
    else
        counter <= counter + 1;
end 

assign refresh = counter[20:18];





always @(posedge clk) begin 
    if(resetn==0) counter <= 0;
    else counter <= counter + 1;
end 

always @(*) begin 
    if (enable) begin  
        // Se pone start
        output_anodo = 8'b11111011;
        output_catodo= 8'b10001000;
        if (text_selector == 0) begin
            if (counter [20:18] == 7) begin
                output_anodo = 8'b01111111;
                output_catodo= 8'b11111111;  // 
            end 
            else if (counter [20:18] == 6) begin
                output_anodo = 8'b10111111;
                output_catodo= 8'b11111111;  // 
            end  
            else if (counter [20:18] == 5) begin
                output_anodo = 8'b11011111;
                output_catodo= 8'b11111111;  //
            end  
            else if (counter [20:18] == 4) begin
                output_anodo = 8'b11101111;
                output_catodo = 8'b11010010; //palabra s   
            end
            else if (counter [20:18] == 3) begin
                output_anodo = 8'b11110111;
                output_catodo= 8'b10000111;  //palabra t     
            end
            else if (counter [20:18] == 2) begin
                output_anodo = 8'b11111011;
                output_catodo= 8'b10001000;  //palabra a     
            end
            else if (counter [20:18] == 1) begin
                output_anodo = 8'b11111101;
                output_catodo= 8'b10101111; // palabra r 
            end
            else if (counter [20:18] == 0) begin
                output_anodo = 8'b11111110;
                output_catodo= 8'b10000111;  //palabra t  
            end
        end
        // Se pone select
        if (text_selector == 1) begin
            if (counter [20:18] == 7) begin
                output_anodo = 8'b01111111;
                output_catodo= 8'b11111111;  //
            end    
            else if (counter [20:18] == 6) begin
                output_anodo = 8'b10111111;
                output_catodo= 8'b11111111;  // 
            end  
            else if (counter [20:18] == 5) begin
                output_anodo = 8'b11011111;
                output_catodo = 8'b11010010; //palabra s
            end
            else if (counter [20:18] == 4) begin
                output_anodo = 8'b11101111;
                output_catodo= 8'b10000110;  //palabra e   
            end
            else if (counter [20:18] == 3) begin
                output_anodo = 8'b11110111;
                output_catodo= 8'b11000111;  //palabra l   
            end
            else if (counter [20:18] == 2) begin
                output_anodo = 8'b11111011;
                output_catodo= 8'b10000110; //palabra e
            end
            else if (counter [20:18] == 1) begin
                output_anodo = 8'b11111101;
                output_catodo= 8'b11000110; //palabra c
            end
            else if (counter [20:18] == 0) begin
                output_anodo = 8'b11111110;
                output_catodo= 8'b10000111; //palabra t 
            end

        end
        // Se pone paper
        if (text_selector == 2) begin
            if (counter [20:18] == 7) begin
                output_anodo = 8'b01111111;
                output_catodo= 8'b11111111;  //
            end
            else if (counter [20:18] == 6) begin
                output_anodo = 8'b10111111;
                output_catodo= 8'b11111111;  // 
            end  
            else if (counter [20:18] == 5) begin
                output_anodo = 8'b11011111;
                output_catodo= 8'b11111111;  //
            end  
            else if (counter [20:18] == 4) begin
                output_anodo = 8'b11101111;
                output_catodo= 8'b10001100;  //palabra p 
            end
            else if (counter [20:18] == 3) begin
                output_anodo = 8'b11110111;
                output_catodo= 8'b10001000; //palabra a  
            end
            else if (counter [20:18] == 2) begin
                output_anodo = 8'b11111011;
                output_catodo= 8'b10001100; //palabra p
            end
            else if (counter [20:18] == 1) begin
                output_anodo = 8'b11111101;
                output_catodo= 8'b10000110; //palabra e
            end
            if (counter [20:18] == 0) begin
                output_anodo = 8'b11111110;
                output_catodo=  8'b10101111;  //palabra r
            end
        end
        //Se pone Scissors
        if (text_selector == 3) begin
            if (counter [20:18] == 7) begin
                output_anodo = 8'b01111111;
                output_catodo= 8'b11010010;  //palabra s
            end  
            else if (counter [20:18] == 6) begin
                output_anodo = 8'b10111111;
                output_catodo= 8'b11000110;  //palabra c 
            end  
            else if (counter [20:18] == 5) begin
                output_anodo = 8'b11011111;
                output_catodo= 8'b11001111;  //palabra i
            end  
            else if (counter [20:18] == 4) begin
                output_anodo = 8'b11101111;
                output_catodo= 8'b11010010;  //palabra s 
            end   
            else if (counter [20:18] == 3) begin
                output_anodo = 8'b11110111;
                output_catodo= 8'b11010010;  //palabra s 
            end
            else if (counter [20:18] == 2) begin
                output_anodo = 8'b11111011;
                output_catodo= 8'b10100011; //palabra o
            end
            else if (counter [20:18] == 1) begin
                output_anodo = 8'b11111101;
                output_catodo=  8'b10101111; //palabra r
            end
            else if (counter [20:18] == 0) begin
                output_anodo = 8'b11111110;
                output_catodo= 8'b11010010;  //palabra s
            end
        end
        // Se pone rock
        if (text_selector == 4) begin
            if (counter [20:18] == 7) begin
                output_anodo = 8'b01111111;
                output_catodo= 8'b11111111;  //
            end
            else if (counter [20:18] == 6) begin
                output_anodo = 8'b10111111;
                output_catodo= 8'b11111111;  // 
            end  
            else if (counter [20:18] == 5) begin
                output_anodo = 8'b11011111;
                output_catodo= 8'b11111111;  //
            end  
            else if (counter [20:18] == 4) begin
                output_anodo = 8'b11101111;
                output_catodo= 8'b11111111;  //
            end
            else if (counter [20:18] == 3) begin
                output_anodo = 8'b11110111;
                output_catodo=  8'b10101111;//palabra r 
            end   
            else if (counter [20:18] == 2) begin
                output_anodo = 8'b11111011;
                output_catodo= 8'b10100011; //palabra o
            end
            else if (counter [20:18] == 1) begin
                output_anodo = 8'b11111101;
                output_catodo= 8'b11000110; //palabra c
            end
            else if (counter [20:18] == 0) begin
                output_anodo = 8'b11111110;
                output_catodo= 8'b10001010;//palabra k
            end
        end

        // Se pone rival
        if (text_selector == 5) begin
            if (counter [20:18] == 7) begin
                output_anodo = 8'b01111111;
                output_catodo= 8'b11111111;  //
            end
            else if (counter [20:18] == 6) begin
                output_anodo = 8'b10111111;
                output_catodo= 8'b11111111;  // 
            end  
            else if (counter [20:18] == 5) begin
                output_anodo = 8'b11011111;
                output_catodo= 8'b11111111;  //
            end  
            else if (counter [20:18] == 4) begin
                output_anodo = 8'b11101111;
                output_catodo = 8'b10101111; //palabra r   
            end    
            else if (counter [20:18] == 3) begin
                output_anodo = 8'b11110111;
                output_catodo= 8'b11001111;  //palabra i     
            end
            else if (counter [20:18] == 2) begin
                output_anodo = 8'b11111011;
                output_catodo= 8'b11010101;  //palabra v    
            end
            else if (counter [20:18] == 1) begin
                output_anodo = 8'b11111101;
                output_catodo= 8'b10001000;  //palabra a
            end
            else if (counter [20:18] == 0) begin
                output_anodo = 8'b11111110;
                output_catodo=  8'b11000111; //palabra l 
            end
        end
            // Se pone You  won
        if (text_selector == 6) begin
            if (counter [20:18] == 7) begin
                output_anodo = 8'b01111111;
                output_catodo= 8'b10010001;  //palabra y
            end  
            else if (counter [20:18] == 6) begin
                output_anodo = 8'b10111111;
                output_catodo= 8'b10100011;  //palabra o
            end  
            else if (counter [20:18] == 5) begin
                output_anodo =  8'b11011111;
                output_catodo=  8'b11100011; //palabra u
            end  
            else if (counter [20:18] == 4) begin
                output_anodo = 8'b11101111;
                output_catodo= 8'b11111111;  //
            end     
            else if (counter [20:18] == 3) begin
                output_anodo = 8'b1110111;
                output_catodo= 8'b1111111;  //
            end  
            else if (counter [20:18] == 2) begin
                output_anodo = 8'b11111011;
                output_catodo= 8'b10010101;  //palabra w
            end
            else if (counter [20:18] == 1) begin
                output_anodo = 8'b11111101;
                output_catodo= 8'b10100011;  //palabra o
            end
            else if (counter [20:18] == 0) begin
                output_anodo = 8'b11111110;
                output_catodo= 8'b10101011;  //palabra n
            end
        end
        // Se pone You  Lost
        if (text_selector == 7) begin
            if (counter [20:18] == 7) begin
                output_anodo = 8'b01111111;
                output_catodo= 8'b10010001;  //palabra y
            end  
            else if (counter [20:18] == 6) begin
                output_anodo = 8'b10111111;
                output_catodo= 8'b10100011;  //palabra o
            end  
            else if (counter [20:18] == 5) begin
                output_anodo =  8'b11011111;
                output_catodo=  8'b11100011; //palabra u
            end  
            else if (counter [20:18] == 4) begin
                output_anodo = 8'b11101111;
                output_catodo= 8'b11111111;  //
            end     
            else if (counter [20:18] == 3) begin
                output_anodo = 8'b11110111;
                output_catodo= 8'b11000111;   //palabra l 
            end
            else if (counter [20:18] == 2) begin
                output_anodo = 8'b11111011;
                output_catodo= 8'b10100011; //palabra o
            end
            else if (counter [20:18] == 1) begin
                output_anodo = 8'b11111101;
                output_catodo = 8'b11010010; //palabra s
            end
            else if (counter [20:18] == 0) begin
                output_anodo = 8'b11111110;
                output_catodo=  8'b10000111;  //palabra t
            end
        end
        // Se pone tie
        if (text_selector == 8) begin
            if (counter [20:18] == 7) begin
                output_anodo = 8'b01111111;
                output_catodo= 8'b11111111;  //
            end
            else if (counter [20:18] == 6) begin
                output_anodo = 8'b10111111;
                output_catodo= 8'b11111111;  // 
            end  
            else if (counter [20:18] == 5) begin
                output_anodo = 8'b11011111;
                output_catodo= 8'b11111111;  //
            end  
            else if (counter [20:18] == 4) begin
                output_anodo = 8'b11101111;
                output_catodo= 8'b11111111;  //
            end
            else if (counter [20:18] == 3) begin
                output_anodo = 8'b11110111;
                output_catodo= 8'b11111111;   //
            end
            else if (counter [20:18] == 2) begin
                output_anodo = 8'b11111011;
                output_catodo= 8'b10000111;  //palabra t   
            end
            else if (counter [20:18] == 1) begin
                output_anodo = 8'b11111101;
                output_catodo= 8'b11001111;   //palabra i
            end
            else if (counter [20:18] == 0) begin
                output_anodo = 8'b11111110;
                output_catodo= 8'b10000110;   //palabra e 
            end


        end
        
        // Se pone paper
        if (text_selector == 2) begin
            if (counter [20:18] == 7) begin
                output_anodo = 8'b01111111;
                output_catodo= 8'b11111111;  //
            end
            else if (counter [20:18] == 6) begin
                output_anodo = 8'b10111111;
                output_catodo= 8'b11111111;  // 
            end  
            else if (counter [20:18] == 5) begin
                output_anodo = 8'b11011111;
                output_catodo= 8'b11111111;  //
            end  
            else if (counter [20:18] == 4) begin
                output_anodo = 8'b11101111;
                output_catodo= 8'b10001100;  //palabra p 
            end
            else if (counter [20:18] == 3) begin
                output_anodo = 8'b11110111;
                output_catodo= 8'b10001000; //palabra a  
            end
            else if (counter [20:18] == 2) begin
                output_anodo = 8'b11111011;
                output_catodo= 8'b10001100; //palabra p
            end
            else if (counter [20:18] == 1) begin
                output_anodo = 8'b11111101;
                output_catodo= 8'b10000110; //palabra e
            end
            if (counter [20:18] == 0) begin
                output_anodo = 8'b11111110;
                output_catodo=  8'b10101111;  //palabra r
            end
        end
        




    end else begin
        // Si no está en enable se apaga
        if (counter [20:18] == 7) begin
            output_anodo = 8'b01111111;
            output_catodo= 8'b11111111;  //
        end 
        else if (counter [20:18] == 0) begin
            output_anodo = 8'b11111110;
            output_catodo= 8'b11111111; // 
        end
        else if (counter [20:18] == 1) begin
            output_anodo = 8'b11111101;
            output_catodo= 8'b11111111; //
        end
        else if (counter [20:18] == 2) begin
            output_anodo = 8'b11111011;
            output_catodo= 8'b11111111;  //  
        end
        else if (counter [20:18] == 3) begin
            output_anodo = 8'b11110111;
            output_catodo= 8'b11111111; //
        end
        else if (counter [20:18] == 4) begin
            output_anodo = 8'b11101111;
            output_catodo= 8'b11111111; //
         end
        else if (counter [20:18] == 5) begin
            output_anodo = 8'b11011111;
            output_catodo= 8'b11111111; //
        end  
        else if (counter [20:18] == 6) begin
            output_anodo = 8'b10111111;
            output_catodo= 8'b11111111;  //
        end  
    end
end
endmodule