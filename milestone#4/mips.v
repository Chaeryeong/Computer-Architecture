`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------
// major modification denoted as below
  // ##### Chaeryeong Kim: START #####_comments
  // ##### Chaeryoeng Kim: END #####
//--------------------------------------------------------------

// pipeline MIPS processor
module mips(input         clk, reset,
            input  [31:0] instr,
            input  [31:0] memreaddata,
            output [31:0] pc,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata);

  wire        signext, shiftl16, memtoreg, branch;
  wire        pcsrc, zero;
  wire        alusrc, regdst, regwrite, jump, jr;
  wire [2:0]  alucontrol;

  // Instantiate Datapath
  datapath dp(
    .clk            (clk),
    .reset          (reset),
    .op             (instr[31:26]), 
    .funct          (instr[5:0]),
    .MEM_memwrite   (memwrite), 
    .zero           (zero),
    .IF_pc          (pc),
    .instr          (instr),
    .MEM_aluout     (memaddr), 
    .MEM_writedata  (memwritedata),
    .readdata       (memreaddata));

endmodule

module maindec(input  [5:0] op, funct,
               input        IDIF_stall,
               output       signext,
               output       shiftl16,
               output       memtoreg, memwrite,
               output       branch, alusrc,
               output       regdst, regwrite,
               output       jump,jr,
               output [1:0] aluop,
               output        memread // when LW
               );

  reg [12:0] controls;

  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, memwrite, memtoreg, jump, jr, aluop, memread} = controls;

  always @(*)
  begin
    if(IDIF_stall == 1) //when stall,
      controls <= #`mydelay 13'b0000000000000; // turn all controls to zero => EX: (NOP) SLL ro, ro, ro _ doing nth
    else
    case(op)
      6'b000000: controls <= #`mydelay (funct == 6'b001000) ? 13'b0000000001000 : 13'b0011000000110; // JR, other Rtype
      6'b100011: controls <= #`mydelay 13'b1010100100001;   // LW
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
                input  [5:0]  op,
                input  [5:0]  funct,
                input  [31:0] instr,
                input  [31:0] readdata,
                output        MEM_memwrite,
                output        zero,
                output [31:0] IF_pc,
                output [31:0] MEM_aluout, MEM_writedata
               );

  // ##### Chaeryeong Kim: START #####_adding needed wires for pipelining

   reg   pcsrc;
   wire  [31:0] srcb; 
   wire  [2:0]   alucontrol;
   wire  shift, signext, shift116, memtoreg, alusrc, regdst, regwrite, jump, jr;

   wire  [1:0] Forward_1, Forward_2; 
   wire  Forward_1_WBID, Forward_2_WBID;

   wire  [31:0] alu_op1, alu_op2;

   wire  IDIF_stall, PCWrite, IFID_Write;

   wire [31:0] IF_pcplus4, IF_pcnext;

   wire [2:0]  ID_alucontrol;
   wire [31:0] ID_pcplus4, ID_instr, ID_srca, ID_srca_in, ID_writedata, ID_writedata_in;
   wire [31:0] ID_signimm, ID_shiftedimm, ID_signimmsh;

   wire EX_regdst, EX_alusrc; // in EX
   wire EX_jr, EX_branch, EX_memwrite, EX_memread; // to MEM
   wire EX_memtoreg, EX_regwrite, EX_jump; // to WB
   wire [2:0]   EX_alucontrol;
   wire [4:0]   EX_writereg1;
   wire [31:0]  EX_pcplus4, EX_pcbranch;
   wire [31:0]  EX_signimmsh, EX_shiftedimm;
   wire [31:0]  EX_srca, EX_writedata;
   wire [31:0]  EX_aluout;
   wire [31:0]  EX_instr;
   
   wire MEM_jr, MEM_branch, MEM_memread;  // in MEM
   wire MEM_memtoreg, MEM_regwrite, MEM_jump; // to WB
   wire [4:0] MEM_writereg1;
   wire [31:0] MEM_pcplus4, MEM_pcbranch, MEM_pcnextbr, MEM_pcnextjr;
   wire [31:0] MEM_instr;
   wire [31:0] MEM_srca;
   wire [31:0] MEM_readdata;
   
   wire WB_memtoreg, WB_regwrite, WB_jump;
   wire [4:0] WB_writereg1, WB_writereg;
   wire [31:0] WB_pcplus4;
   wire [31:0] WB_instr;
   wire [31:0] WB_readdata;
   wire [31:0] WB_aluout;
   wire [31:0] result1, WB_result;
   
   wire branch;
   wire [1:0] aluop;
   // ##### Chaeryeong Kim: END #####

  // ######: controller(maindec&aludec) moved into the datapath
  maindec md(
    .op       (ID_instr[31:26]),
    .funct     (ID_instr[5:0]),
    .IDIF_stall (IDIF_stall),
    .signext  (signext),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
    .memwrite (memwrite),
    .branch   (branch),
    .alusrc   (alusrc),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
    .jr        (jr),
    .aluop    (aluop),
    .memread  (memread)
    );

  aludec ad( 
    .funct      (ID_instr[5:0]),
    .aluop      (aluop), 
    .alucontrol (ID_alucontrol));

    // assigning pcsrc to become 1 when BEQ, BNE met their equality, ineqaulity of zero from alu
    always@ (*)
    case(op)
      6'b000101: pcsrc <= branch & ~zero;//BNE
      default: pcsrc <= branch & zero;//BEQ
    endcase
  

  // next PC logic
  flopenr #(32) pcreg( //using enable FF
    .clk   (clk),
    .reset (reset),
    .en     (PCWrite), //PCWrite(~stall) => update pc
    .d     (IF_pcnext),
    .q     (IF_pc));

  adder pcadd1(
    .a (IF_pc),
    .b (32'b100),
    .y (IF_pcplus4));

  sl2 immsh(
    .a (ID_signimm),
    .y (ID_signimmsh));
             
  adder pcadd2(
    .a (EX_pcplus4),
    .b (EX_signimmsh),
    .y (EX_pcbranch));

  mux2 #(32) pcbrmux(
    .d0  (IF_pcplus4),
    .d1  (MEM_pcbranch),
    .s   (pcsrc), // 1 when BEQ, BNE satisfied with equality, inequality
    .y   (MEM_pcnextbr));

  mux2 #(32) pcjrmux(
    .d0  (MEM_pcnextbr),
    .d1  (MEM_srca),
    .s   (jr),
    .y   (MEM_pcnextjr));

  mux2 #(32) pcmux(
    .d0   (MEM_pcnextjr),
    .d1   ({MEM_pcplus4[31:28], MEM_instr[25:0], 2'b00}), // j.dest
    .s    (MEM_jump),
    .y    (IF_pcnext));

  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (WB_regwrite),
    .ra1     (ID_instr[25:21]), // rs
    .ra2     (ID_instr[20:16]), // rt
    .wa      (WB_writereg),
    .wd      (WB_result),
    .rd1     (ID_srca_in),
    .rd2     (ID_writedata_in));

  mux2 #(5) wrmux(
    .d0  (EX_instr[20:16]), // rt
    .d1  (EX_instr[15:11]), // rd
    .s   (EX_regdst),
    .y   (EX_writereg1));

  mux2 # (5) wrmuxJal(
    .d0  (WB_writereg1),
    .d1  ({5'b11111}), // $ra($31)
    .s   (WB_jump),
    .y   (WB_writereg));

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

  sign_zero_ext sze(
    .a       (ID_instr[15:0]),
    .signext (signext),
    .y       (ID_signimm[31:0]));

  shift_left_16 sl16(
    .a         (ID_signimm[31:0]),
    .shiftl16  (shiftl16),
    .y         (ID_shiftedimm[31:0]));

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
    
  // ##### Chaeryoeng Kim: START #####_implementing four pipeline FFs : IF/ID, ID/EX, EX/MEM, MEM/WB

   // 1.IF/ID-------------------------------------
  flopenr #(64) IFID( // using enable FF
    .clk   (clk),
    .reset (reset),
    .en     (IFID_Write), // ~stall => enabled: updating pcplus4, updating next instr
    .d     ({IF_pcplus4, instr}),
    .q     ({ID_pcplus4, ID_instr}));
   
   // 2. ID/EX ------------------------------------
  flopr #(195) IDEX(
    .clk   (clk),
    .reset (reset),
    .d     ({ID_pcplus4, ID_srca, ID_writedata, ID_alucontrol, ID_shiftedimm, ID_instr, ID_signimmsh}),
    .q     ({EX_pcplus4, EX_srca, EX_writedata, EX_alucontrol, EX_shiftedimm, EX_instr, EX_signimmsh}));
  
  flopr #(9) IDEX_rest( //rest control signals implemented into one FF
    .clk   (clk),
    .reset (reset),
    .d     ({regdst,    alusrc,    jr,    branch,    memwrite,    memread,    memtoreg,    regwrite,    jump}),
    .q     ({EX_regdst, EX_alusrc, EX_jr, EX_branch, EX_memwrite, EX_memread, EX_memtoreg, EX_regwrite, EX_jump}));

   // 3. EX/MEM-------------------------------------
  flopr #(197) EXMEM(
    .clk   (clk),
    .reset (reset),
    .d     ({EX_pcplus4,  EX_instr,  EX_pcbranch,  EX_srca,  EX_aluout,  alu_op2,       EX_writereg1}),
    .q     ({MEM_pcplus4, MEM_instr, MEM_pcbranch, MEM_srca, MEM_aluout, MEM_writedata, MEM_writereg1}));
    
  flopr #(7) EXMEM_rest(
    .clk   (clk),
    .reset (reset),
    .d     ({EX_jr,  EX_branch,  EX_memwrite,  EX_memread,  EX_memtoreg,  EX_regwrite,  EX_jump}),
    .q     ({MEM_jr, MEM_branch, MEM_memwrite, MEM_memread, MEM_memtoreg, MEM_regwrite, MEM_jump}));

   // 4. MEM/WB---------------------------------------
  flopr #(133) MEMWB(
    .clk   (clk),
    .reset (reset),
    .d     ({MEM_pcplus4, MEM_instr, readdata,    MEM_aluout, MEM_writereg1}),
    .q     ({WB_pcplus4,  WB_instr,  WB_readdata, WB_aluout,  WB_writereg1}));

  flopr #(3) MEMWB_rest(
    .clk   (clk),
    .reset (reset),
    .d     ({MEM_memtoreg, MEM_regwrite, MEM_jump}),
    .q     ({WB_memtoreg,  WB_regwrite,  WB_jump}));
  // ##### Chaeryeong Kim: END #####

  // ##### Chaeryeong Kim: START #####_instantiating forwarding logics of WB->EX, MEM->EX 
  Forwarding_WBEX_MEMEX Forwarding_WBEX_MEMEX_1( //for rs
    .MEM_regwrite (MEM_regwrite),
    .MEM_rd         (MEM_writereg1),
    .EX_r         (EX_instr[25:21]), // rs
    .WB_regwrite    (WB_regwrite),
    .WB_rd         (WB_writereg1),
    .Forward      (Forward_1)
  );

  Forwarding_WBEX_MEMEX Forwarding_WBEX_MEMEX_2( //for rt
    .MEM_regwrite (MEM_regwrite),
    .MEM_rd         (MEM_writereg1),
    .EX_r         (EX_instr[20:16]), // rt
    .WB_regwrite    (WB_regwrite),
    .WB_rd         (WB_writereg1),
    .Forward      (Forward_2)
  );
  // ##### Chaeryeong Kim: END #####

// ##### Chaeryoeng Kim: START #####_instantiating mux for operands of ALU
  mux3 #(32) alu_op1_mux( // for 1st operand
   .d0   (EX_srca),
   .d1   (WB_result),
   .d2   (MEM_aluout),
   .s      (Forward_1),
   .y      (alu_op1)
  );

  mux3 #(32) alu_op2_mux( // for 2nd operand
   .d0   (EX_writedata),
   .d1   (WB_result),
   .d2   (MEM_aluout),
   .s      (Forward_2),
   .y      (alu_op2)
  );
  // ##### Chaeryeong Kim: END #####

// ##### Chaereyong Kim: START #####_instantiatnig forwarding logics of WB->ID
  Forwarding_WBID Forwarding_WBID_1( // for rs
    .MEM_regwrite (MEM_regwrite),
    .WB_regwrite    (WB_regwrite),
    .EX_regwrite   (EX_regwrite),
    .MEM_rd         (MEM_writereg1),
    .WB_rd         (WB_writereg),
    .EX_rd         (EX_writereg1),
    .ID_r         (ID_instr[25:21]), // rs
    .Forward      (Forward_1_WBID)
  );
  
  Forwarding_WBID Forwarding_WBID_2( // for rt
    .MEM_regwrite (MEM_regwrite),
    .WB_regwrite    (WB_regwrite),
    .EX_regwrite   (EX_regwrite),
    .MEM_rd         (MEM_writereg1),
    .WB_rd         (WB_writereg),
    .EX_rd         (EX_writereg1),
    .ID_r         (ID_instr[20:16]), // rt
    .Forward      (Forward_2_WBID)
  );
  // ##### Chaeryoeng Kim: END #####
  
// ##### Chaeryoeng Kim: START #####_instantiating mux for ID_srca&ID_writedata 
  mux2 #(32) ID_srca_mux(
     .d0   (ID_srca_in),
   .d1   (WB_result),
   .s      (Forward_1_WBID),
   .y      (ID_srca)
  );
  
  mux2 #(32) ID_writedata_mux(
     .d0   (ID_writedata_in),
   .d1   (WB_result),
   .s      (Forward_2_WBID),
   .y      (ID_writedata)
  );
  // ##### Chaeryeong Kim: END #####
  
 // ##### Chaeryoeng Kim: START #####_instantiating hazard detection logic
 HazardDetectionUnit HazardDetectionUnit(
   .EX_memread      (EX_memread),
   .EX_rt         (EX_instr[20:16]),
   .ID_rs         (ID_instr[25:21]),
   .ID_rt         (ID_instr[20:16]),
   .IDIF_stall   (IDIF_stall), // stall
   .PCWrite         (PCWrite), // ~stall
   .IFID_Write      (IFID_Write) // ~stall
 );
// ##### Chaeryoeng Kim: END #####
endmodule //end of datapath
// Thank you :)