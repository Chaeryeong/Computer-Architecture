`timescale 1ns/1ps
`define mydelay 1

// 1. 바뀐 부분 표시 2. 필요 없는 와이어 지우기

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------
// major modification for milestone#5 denoted as below
  // ##### Chaeryeong Kim: START #####
  // ##### Chaeryoeng Kim: END #####
//--------------------------------------------------------------

// pipeline MIPS processor
module mips(input         clk, reset,
            input  [31:0] instr, memreaddata,
            output [31:0] pc,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata);

  wire        signext, shiftl16, memtoreg, branch;
  wire        zero; 
  wire        alusrc, regdst, regwrite, jump, jr;
  wire [2:0]  alucontrol;
  
  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
	 .op         (instr[31:26]), 
	 .funct      (instr[5:0]),
	 .MEM_memwrite   (memwrite), 
    .zero       (zero),
    .IF_pc         (pc),
    .instr      (instr),
    .MEM_aluout     (memaddr), 
    .MEM_writedata  (memwritedata),
    .readdata   (memreaddata));

endmodule

module maindec(input  [5:0] op, funct,
					     input        IDIF_stall, flush,
               output       signext,
               output       shiftl16,
               output       memtoreg, memwrite,
               output       branch, alusrc,
               output       regdst, regwrite,
               output       jump,jr,
               output [1:0] aluop,
					     output 		 memread); // when LW

  reg [12:0] controls;

  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, memwrite,
          memtoreg, jump, jr, aluop, memread} = controls;

  always @(*)
  begin    // ##### Chaeryeong Kim: START #####
	 if(IDIF_stall == 1 || flush == 1)
		controls <= #`mydelay 13'b0000000000000; //NOP, nullify all controls
	 else   // ##### Chaeryoeng Kim: END #####
    case(op)
      6'b000000: controls <= #`mydelay (funct == 6'b001000) ? 13'b0000000001000 : 13'b0011000000110;//JR, other Rtype
      6'b100011: controls <= #`mydelay 13'b1010100100001;	// LW
      6'b101011: controls <= #`mydelay 13'b1000101000000; // SW
      6'b000100,
		6'b000101: controls <= #`mydelay 13'b1000010000010; // BEQ, BNE
      6'b001000, 
      6'b001001: controls <= #`mydelay 13'b1010100000000; // ADDI, ADDIU
      6'b001101: controls <= #`mydelay 13'b0010100000100; // ORI
      6'b001111: controls <= #`mydelay 13'b0110100000000; // LUI
      6'b000010: controls <= #`mydelay 13'b0000000010000; // J
		6'b000011: controls <= #`mydelay 13'b0010000010000; // JAL
      default:   controls <= #`mydelay 13'bxxxxxxxxxxxxx; // ???
    endcase
  end
  
endmodule

module aludec(input      [5:0] funct,
              input      [1:0] aluop,
              output reg [2:0] alucontrol);

  always @(*)
    case(aluop)
      2'b00: alucontrol <= #`mydelay 3'b010;  // add
      2'b01: alucontrol <= #`mydelay 3'b110;  // sub
      2'b10: alucontrol <= #`mydelay 3'b001;  // or
      default: case(funct)          // RTYPE
          6'b100000,
          6'b100001: alucontrol <= #`mydelay 3'b010; // ADD, ADDU
          6'b100010,
          6'b100011: alucontrol <= #`mydelay 3'b110; // SUB, SUBU
          6'b100100: alucontrol <= #`mydelay 3'b000; // AND
          6'b100101: alucontrol <= #`mydelay 3'b001; // OR
          6'b101010,
			 6'b101011: alucontrol <= #`mydelay 3'b111; // SLT, SLTU
          default:   alucontrol <= #`mydelay 3'bxxx; // ???
        endcase
    endcase
	
	assign ID_alucontrol = alucontrol;
    
endmodule

module datapath(input         clk, reset,
                input [5:0]	  op, funct,
                input [31:0]  instr, readdata,
					      output		    MEM_memwrite,
                output        zero,
					      output [31:0] IF_pc,
                output [31:0] MEM_aluout, MEM_writedata);

  wire [1:0] aluop;
  wire       branch;
  reg		 flush;
	reg	pcsrc;
	wire  [31:0] srcb;
	wire	signext, memtoreg, alusrc, regdst, regwrite, jump, jr; 
	wire  [1:0] Forward_1, Forward_2;
	wire  Forward_1_WBID, Forward_2_WBID;
	wire  [31:0] alu_op1, alu_op2;
	wire  IDIF_stall, PCWrite, IFID_Write;

	wire [31:0] IF_pcplus4;
	wire [31:0]	IF_pcnext;
	wire [31:0] IF_flush_instr;

	wire [31:0] ID_pcplus4;
	wire [31:0] ID_instr;
	wire [31:0] ID_srca, ID_srca_in, ID_writedata, ID_writedata_in;
	wire [31:0] ID_signimm;
	wire [31:0] ID_shiftedimm;
	wire [31:0] ID_signimmsh;
	wire [2:0]  ID_alucontrol;

	wire [31:0] EX_pcplus4;
	wire [31:0] EX_signimmsh;
	wire [31:0] EX_pcbranch;
	wire [31:0] EX_srca, EX_writedata;
	wire [31:0] EX_shiftedimm;
	wire [4:0]	EX_writereg1, EX_writereg;
	wire [31:0]	EX_aluout;
	wire [2:0]  EX_alucontrol;
	wire [31:0] EX_instr;
	wire EX_regdst, EX_alusrc; 					
	wire EX_jr, EX_branch, EX_memwrite, EX_memread; 	
	wire EX_memtoreg, EX_regwrite, EX_jump; 	
	wire [31:0] EX_pcnextbr, EX_pcnextjr;

	wire [31:0] MEM_pcplus4, MEM_pcbranch;
	wire [31:0] MEM_instr;
	wire [31:0]	MEM_srca;
	wire [4:0] MEM_writereg1, MEM_writereg;
	wire MEM_jr, MEM_branch, MEM_memread;
	wire MEM_memtoreg, MEM_regwrite, MEM_jump; 
	wire MEM_pcsrc;
	
	wire [31:0] WB_pcplus4;
	wire [31:0] WB_instr;
	wire [31:0] WB_readdata;
	wire [31:0]	WB_aluout;
	wire [31:0] result1, WB_result;
	wire [4:0] WB_writereg1, WB_writereg;
	wire WB_memtoreg, WB_regwrite, WB_jump; 	
  
  // ######: controller(maindec&aludec) moved into the datapath
  maindec md(
    .op       (ID_instr[31:26]),
	 .funct	  (ID_instr[5:0]),
	 .IDIF_stall (IDIF_stall),
	 .flush	  (flush),
    .signext  (signext),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
    .memwrite (memwrite),
    .branch   (branch),
    .alusrc   (alusrc),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
	 .jr		  (jr),
    .aluop    (aluop),
	 .memread  (memread));

  aludec ad( 
    .funct      (ID_instr[5:0]),
    .aluop      (aluop), 
    .alucontrol (ID_alucontrol));


// ------------------------------------   IF    -------------------------------------------

 // next PC logic

  // ##### Chaeryoeng Kim: START #####
  mux2 #(32) flushmux(
	.d0(instr),
	.d1(32'b0),
	.s	(flush), //when flush -> take all 0's
	.y	(IF_flush_instr)); // => IFID FF */
  // ##### Chaeryoeng Kim: END #####

  flopenr #(32) pcreg( //using enable FF
    .clk   (clk),
    .reset (reset),
	 .en	  (PCWrite), //PCWrite(~stall) => update pc, when not stall fetch new
    .d     (IF_pcnext),
    .q     (IF_pc));

  adder pcadd1(
    .a (IF_pc),
    .b (32'b100),
    .y (IF_pcplus4));

  mux2 #(32) pcbrmux(
    .d0  (IF_pcplus4),
    .d1  (EX_pcbranch),
    .s   (pcsrc),
    .y   (EX_pcnextbr));

  mux2 #(32) pcjrmux(
    .d0  (EX_pcnextbr),
    .d1  (EX_srca),
    .s   (EX_jr),
    .y   (EX_pcnextjr));

  mux2 #(32) pcmux(
    .d0   (EX_pcnextjr),
    .d1   ({EX_pcplus4[31:28], EX_instr[25:0], 2'b00}),
    .s    (EX_jump),
    .y    (IF_pcnext));

// --------------- IF->ID -------------
   
     flopenr #(64) IFID( // 32*2 using enable FF
    .clk   (clk),
    .reset (reset),
	  .en	  (IFID_Write), // ~stall => enabled
    .d     ({IF_flush_instr, IF_pcplus4}),
    .q     ({ID_instr, ID_pcplus4}));

// ------------------------------------   ID    -------------------------------------------

 Forwarding_WBID Forwarding_WBID_1(  // for rs
	 .MEM_regwrite (MEM_regwrite),
	 .WB_regwrite 	(WB_regwrite),
	 .EX_regwrite	(EX_regwrite),
	 .MEM_rd			(MEM_writereg1),
	 .WB_rd			(WB_writereg),
	 .EX_rd			(EX_writereg1),
	 .ID_r			(ID_instr[25:21]), // rs
	 .Forward		(Forward_1_WBID));
  
  mux2 #(32) ID_srcamux(
   .d0	(ID_srca_in), // rd1
	 .d1	(WB_result),
	 .s		(Forward_1_WBID),
	 .y		(ID_srca));

  Forwarding_WBID Forwarding_WBID_2( // for rt
	 .MEM_regwrite (MEM_regwrite),
	 .WB_regwrite 	(WB_regwrite),
	 .EX_regwrite	(EX_regwrite),
	 .MEM_rd			(MEM_writereg1),
	 .WB_rd			(WB_writereg),
	 .EX_rd			(EX_writereg1),
	 .ID_r			(ID_instr[20:16]), // rt
	 .Forward		(Forward_2_WBID));
  
  mux2 #(32) ID_writedatamux(
   .d0	(ID_writedata_in), // rd2
	 .d1	(WB_result),
	 .s		(Forward_2_WBID),
	 .y		(ID_writedata)); // after forwarding rd2

  HazardDetectionUnit HazardDetectionUnit(
	 .EX_memread		(EX_memread),
	 .EX_rt			(EX_instr[20:16]),
	 .ID_rs			(ID_instr[25:21]),
	 .ID_rt			(ID_instr[20:16]),
	 .IDIF_stall	(IDIF_stall), // stall
	 .PCWrite			(PCWrite), // ~stall
	 .IFID_Write		(IFID_Write)); // ~stall
    
    // ##### Chaeryoeng Kim: START #####
  always @ (pcsrc or EX_jump or EX_jr)
   begin
		if(pcsrc || EX_jump || EX_jr)
			flush <= 1'b1;
		else 
			flush <= 1'b0;
	 end
    // ##### Chaeryoeng Kim: END #####

  sl2 immsh(
    .a (ID_signimm),
    .y (ID_signimmsh));  

  sign_zero_ext sze(
    .a       (ID_instr[15:0]),
    .signext (signext),
    .y       (ID_signimm[31:0]));

  shift_left_16 sl16(
    .a         (ID_signimm[31:0]),
    .shiftl16  (shiftl16),
    .y         (ID_shiftedimm[31:0])); // imm op

   // register file logic
  regfile rf(
    .clk     (clk),
    .we      (WB_regwrite),
    .ra1     (ID_instr[25:21]), // rs
    .ra2     (ID_instr[20:16]), // rt
    .wa      (WB_writereg),
    .wd      (WB_result),
    .rd1     (ID_srca_in), // rt1
    .rd2     (ID_writedata_in)); // rd2

// ----------- ID->EX -----------------

	//cf. maindec does nullifying	

  flopr #(192) IDEX( // 32*6
    .clk   (clk),
    .reset (reset),
    .d     ({ID_pcplus4, ID_srca, ID_writedata, ID_shiftedimm, ID_instr, ID_signimmsh}),
    .q     ({EX_pcplus4, EX_srca, EX_writedata, EX_shiftedimm, EX_instr, EX_signimmsh}));
  
  flopr #(9) IDEX_rest( // 1*9 rest control signals implemented into one FF
    .clk   (clk),
    .reset (reset),
    .d     ({regdst, alusrc, jr, branch, memwrite, memread, memtoreg, regwrite, jump}),
    .q     ({EX_regdst, EX_alusrc, EX_jr, EX_branch, EX_memwrite, EX_memread, EX_memtoreg, EX_regwrite, EX_jump}));	  

// ------------------------------------   EX    -------------------------------------------

  mux3 #(32) aluop1_mux( // for 1st operand
   .d0	(EX_srca),
   .d1	(WB_result),
	 .d2	(MEM_aluout),
	 .s		(Forward_1),
	 .y		(alu_op1));

  mux3 #(32) aluop2_mux( // for 2nd operand
	 .d0	(EX_writedata),
	 .d1	(WB_result),
	 .d2	(MEM_aluout),
	 .s		(Forward_2),
	 .y		(alu_op2));

  Forwarding_WBEX_MEMEX Forwarding_WBEX_MEMEX_1( // for rs
	 .MEM_regwrite (MEM_regwrite),
	 .MEM_rd			(MEM_writereg1),
	 .EX_r			(EX_instr[25:21]), // rs
	 .WB_regwrite 	(WB_regwrite),
	 .WB_rd			(WB_writereg1),
	 .Forward		(Forward_1));

  Forwarding_WBEX_MEMEX Forwarding_WBEX_MEMEX_2( // for rt
	 .MEM_regwrite (MEM_regwrite),
	 .MEM_rd			(MEM_writereg1),
	 .EX_r			(EX_instr[20:16]), // rt
	 .WB_regwrite 	(WB_regwrite),
	 .WB_rd			(WB_writereg1),
	 .Forward		(Forward_2));

  adder pcadd2(
    .a (EX_pcplus4),
    .b (EX_signimmsh),
    .y (EX_pcbranch));  // EX.branch.dest

  mux2 #(5) wrmux(
    .d0  (EX_instr[20:16]), // rt
    .d1  (EX_instr[15:11]), // rd
    .s   (EX_regdst),
    .y   (EX_writereg1));
   
   // ##### Chaeryeong Kim: START #####
  mux2 # (5) wrmuxJal( // EX에서 modified
    .d0  (EX_writereg1),
    .d1  ({5'b11111}), // $ra($31)
    .s   (EX_jump),
    .y   (EX_writereg));
    // ##### Chaeryeong Kim: END #####

    /*
      mux2 # (5) wrmuxJal( // WB에서
    .d0  (WB_writereg1),
    .d1  ({5'b11111}), // $ra($31)
    .s   (WB_jump),
    .y   (WB_writereg));
    */

   // ALU logic

  mux2 #(32) srcbmux(
    .d0 (alu_op2),
    .d1 (EX_shiftedimm[31:0]),
    .s  (EX_alusrc),
    .y  (srcb));

  alu alu(
    .a       (alu_op1),
    .b       (srcb),
    .alucont (EX_alucontrol),
    .result  (EX_aluout),
    .zero    (zero));

     // ##### Chaeryoeng Kim: START #####
    // assigning pcsrc to become 1 when BEQ, BNE met their equality, ineqaulity of zero from alu
	always@ (*) 
	 case(EX_instr[31:26]) // EX_instr's opcode, not ID's
		6'b000101: pcsrc <= EX_branch && (~zero);//BNE
		default: pcsrc <= EX_branch && zero;//BEQ
	 endcase
    // ##### Chaeryoeng Kim: END #####

// ------------ EX->MEM ---------------

  flopr #(192) EXMEM( // 32*6 
    .clk   (clk),
    .reset (reset),
    .d     ({EX_pcplus4, EX_instr, EX_pcbranch, EX_srca, EX_aluout, alu_op2}),
    .q     ({MEM_pcplus4, MEM_instr, MEM_pcbranch, MEM_srca, MEM_aluout, MEM_writedata}));

  flopr #(3) EXMEM_alucontrol( // 3
    .clk   (clk),
    .reset (reset),
    .d     (ID_alucontrol),
    .q     (EX_alucontrol));
	 
  flopr #(5) EXMEM_writereg1( //5
    .clk   (clk),
    .reset (reset),
    .d     (EX_writereg1),
    .q     (MEM_writereg1));
  
   // ##### Chaeryeong Kim: START #####
  flopr #(5) EXMEM_writereg( //5 modified
    .clk   (clk),
    .reset (reset),
    .d     (EX_writereg),
    .q     (MEM_writereg));
    // ##### Chaeryeong Kim: END #####
	 
  flopr #(8) EXMEM_rest( // 1*8
    .clk   (clk),
    .reset (reset),
    .d     ({EX_jr, EX_branch, EX_memwrite, EX_memread, EX_memtoreg, EX_regwrite, EX_jump, pcsrc}),
    .q     ({MEM_jr, MEM_branch, MEM_memwrite, MEM_memread, MEM_memtoreg, MEM_regwrite, MEM_jump, MEM_pcsrc}));

// ------------------------------------   MEM   -------------------------------------------

// ------------- MEM->WB --------------

  flopr #(128) MEMWB_pcplus4( // 32*4
    .clk   (clk),
    .reset (reset),
    .d     ({MEM_pcplus4, MEM_instr, readdata, MEM_aluout}),
    .q     ({WB_pcplus4, WB_instr, WB_readdata, WB_aluout}));

  flopr #(5) MEMWB_writereg1( // 5
    .clk   (clk),
    .reset (reset),
    .d     (MEM_writereg1),
    .q     (WB_writereg1));
     
     // ##### Chaeryeong Kim: START #####
  flopr #(5) MEMWB_writereg( //5  modified
    .clk   (clk),
    .reset (reset),
    .d     (MEM_writereg),
    .q     (WB_writereg));
    // ##### Chaeryeong Kim: END #####

  flopr #(3) MEMWB_rest( // 3
    .clk   (clk),
    .reset (reset),
    .d     ({MEM_memtoreg, MEM_regwrite, MEM_jump}),
    .q     ({WB_memtoreg, WB_regwrite, WB_jump}));

// ------------------------------------   WB    -------------------------------------------

  mux2 #(32) resmux(
    .d0 (WB_aluout),
    .d1 (WB_readdata),
    .s  (WB_memtoreg),
    .y  (result1));

  mux2 #(32) resmuxJal(
    .d0 (result1),
    .d1 (WB_pcplus4),
    .s  (WB_jump),
    .y  (WB_result));


endmodule //end of datapath
// Thank you :)

