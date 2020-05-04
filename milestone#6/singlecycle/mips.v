//mips

`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

// single-cycle MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire        signext, shiftl16;
  wire        branche, branchn;
  wire        pcsrc, zero;
  wire        alusrc;
  wire [1:0]  regdst, memtoreg, jump;
  wire [2:0]  alucontrol;

  // Instantiate Controller
  controller c(
    .op         (instr[31:26]), 
      .funct      (instr[5:0]), 
      .zero       (zero),
      .signext    (signext),
      .shiftl16   (shiftl16),
      .memtoreg   (memtoreg),
      .memwrite   (memwrite),
      .pcsrc      (pcsrc),
      .alusrc     (alusrc),
      .regdst     (regdst),
      .regwrite   (regwrite),
      .jump       (jump),
      .alucontrol (alucontrol));

  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .signext    (signext),
    .shiftl16   (shiftl16),
    .memtoreg   (memtoreg),
    .pcsrc      (pcsrc),
    .alusrc     (alusrc),
    .regdst     (regdst),
    .regwrite   (regwrite),
    .jump       (jump),
    .alucontrol (alucontrol),
    .zero       (zero),
    .pc         (pc),
    .instr      (instr),
    .aluout     (memaddr), 
    .writedata  (memwritedata),
    .readdata   (memreaddata));

endmodule

module controller(input  [5:0] op, funct,
                  input        zero,
                  output       signext,
                  output       shiftl16,
                  output [1:0]  memtoreg, regdst,
                  output       pcsrc, alusrc,
                  output       memwrite, regwrite,
                  output [1:0] jump,
                  output  [2:0] alucontrol); // reg일 필요 없당

  wire [1:0] aluop;
  wire branche, branchn;

  maindec md(
    .op       (op),
    .funct      (funct),
    .signext  (signext),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
    .memwrite (memwrite),
    .branche   (branche),
    .branchn   (branchn),
    .alusrc   (alusrc),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
    .aluop    (aluop)); 

      // ##### Chaeryeong Kim: START #####
  aludec ad( // modified - opcode added
    .op         (op),    // ##### Chaeryeong Kim: END #####
    .funct      (funct),
    .aluop      (aluop), 
    .alucontrol (alucontrol));

  assign pcsrc = (branche & zero) | (branchn & ~zero);

  
endmodule


module maindec(input  [5:0] op, funct,
               output       signext,
               output       shiftl16,
               output [1:0] memtoreg, 
               output       memwrite,
               output       branche, branchn,
               output       alusrc,
               output [1:0] regdst, 
               output       regwrite,
               output [1:0]  jump,
               output [1:0] aluop);  

  reg [14:0] controls;
                                       //2bit
  assign {signext, shiftl16, regwrite, regdst, alusrc, branche, branchn, memwrite,
          memtoreg, jump, aluop} = controls;
           //2bit   //2bit //2bit

   always @(*)
    case(op)
      6'b000000: controls <= #`mydelay (funct == 6'b001000) ? 15'b000000000001011 :15'b001010000000011; // JR & other Rtype
      6'b100011: controls <= #`mydelay 15'b101001000010000; // LW
      6'b101011: controls <= #`mydelay 15'b100001001000000; // SW
      6'b000100: controls <= #`mydelay 15'b100000100000001; // BEQ
      6'b000101: controls <= #`mydelay 15'b100000010000001; // BNE
      6'b001000, 
      6'b001001: controls <= #`mydelay 15'b101001000000000; // ADDI, ADDIU
      6'b001101: controls <= #`mydelay 15'b001001000000010; // ORI
      6'b001111: controls <= #`mydelay 15'b011001000000000; // LUI
      6'b000010: controls <= #`mydelay 15'b000000000000100; // J
      6'b000011: controls <= #`mydelay 15'b001100000100100; // JAL
          // ##### Chaeryeong Kim: START #####
      6'b001010: controls <= #`mydelay 15'b101001000000011; // SLTI // 추가해줌
          // ##### Chaeryeong Kim: END #####
      default:   controls <= #`mydelay 15'bxxxxxxxxxxxxxxx; // ???
    endcase

endmodule
                                // ##### Chaeryeong Kim: START #####
module aludec(input      [5:0] op, // 추가해줌
              input      [5:0] funct,
              input      [1:0] aluop,
              output reg [2:0] alucontrol);  
          
  always @(*) 
  begin
    if (op == 6'b001010) // 추가해줌 
      alucontrol <= #`mydelay 3'b111; // SLTI, STLIU
    else    // ##### Chaeryeong Kim: START #####
      case(aluop)
        2'b00: alucontrol <= #`mydelay 3'b010;  // add
        2'b01: alucontrol <= #`mydelay 3'b110;  // sub
        2'b10: alucontrol <= #`mydelay 3'b001;  // or
        default: case(funct)          // RTYPE
          6'b100000,
          6'b100001: alucontrol <= #`mydelay 3'b010; // ADD, ADDU: only difference is exception
          6'b100010,
          6'b100011: alucontrol <= #`mydelay 3'b110; // SUB, SUBU: only difference is exception
          6'b100100: alucontrol <= #`mydelay 3'b000; // AND
          6'b100101: alucontrol <= #`mydelay 3'b001; // OR
          6'b101011,                                 
          6'b101010: alucontrol <= #`mydelay 3'b111; // SLT, STLU
          default:   alucontrol <= #`mydelay 3'bxxx; // ???
        endcase
    endcase
  end
    /*
  case(op)
    6'b001010: alucontrol <= #`mydelay 3'b111; // SLTI, STLIU
    default:
    */
  //endcase
    
endmodule

module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         pcsrc,
                input         alusrc,
                input         regwrite,
                input  [1:0]  jump, memtoreg, regdst,
                input  [2:0]  alucontrol,
                output        zero,
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata,
                input  [31:0] readdata);

  wire [4:0]  writereg;
  wire [31:0] pcnext, pcnextbr, pcplus4, pcbranch;
  wire [31:0] signimm, signimmsh, shiftedimm;
  wire [31:0] srca, srcb;
  wire [31:0] result;
  wire        shift;

  // next PC logic
  flopr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
    .d     (pcnext),
    .q     (pc));

  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (pcplus4));

  sl2 immsh(
    .a (signimm),
    .y (signimmsh));
             
  adder pcadd2(
    .a (pcplus4),
    .b (signimmsh),
    .y (pcbranch));

  mux2 #(32) pcbrmux(
    .d0  (pcplus4),
    .d1  (pcbranch),
    .s   (pcsrc),
    .y   (pcnextbr));

   mux3 #(32) pcmux(
    .d0   (pcnextbr),
    .d1   ({pcplus4[31:28], instr[25:0], 2'b00}),
    .d2   (srca), //jump = 10, for jr
    .s    (jump),
    .y    (pcnext));
    
    
  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (regwrite),
    .ra1     (instr[25:21]),
    .ra2     (instr[20:16]),
    .wa      (writereg),
    .wd      (result),
    .rd1     (srca),
    .rd2     (writedata));

  mux3 #(5) wrmux(
    .d0  (instr[20:16]), //rt
    .d1  (instr[15:11]), //rd
    .d2  (5'b11111), //$31($ra) for jal when regdst = 10
    .s   (regdst),
    .y   (writereg));

  mux3 #(32) resmux(
    .d0 (aluout),
    .d1 (readdata),
    .d2 (pcplus4), //for jal when memtoreg = 10
    .s  (memtoreg),
    .y  (result));

  sign_zero_ext sze(
    .a       (instr[15:0]),
    .signext (signext),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    .a         (signimm[31:0]),
    .shiftl16  (shiftl16),
    .y         (shiftedimm[31:0]));

  // ALU logic
  mux2 #(32) srcbmux(
    .d0 (writedata),
    .d1 (shiftedimm[31:0]),
    .s  (alusrc),
    .y  (srcb));
    
  alu alu(
    .a       (srca),
    .b       (srcb),
    .alucont (alucontrol),
    .result  (aluout),
    .zero    (zero));
    
endmodule

