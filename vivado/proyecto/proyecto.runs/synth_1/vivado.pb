
�
Command: %s
1870*	planAhead2�
�read_checkpoint -auto_incremental -incremental /home/licit/laboratorio-6-jarvis/vivado/proyecto/proyecto.srcs/utils_1/imports/synth_1/system.dcp2default:defaultZ12-2866h px� 
�
;Read reference checkpoint from %s for incremental synthesis3154*	planAhead2u
a/home/licit/laboratorio-6-jarvis/vivado/proyecto/proyecto.srcs/utils_1/imports/synth_1/system.dcp2default:defaultZ12-5825h px� 
T
-Please ensure there are no constraint changes3725*	planAheadZ12-7989h px� 
�
Command: %s
53*	vivadotcl2�
�synth_design -top system -part xc7a100tcsg324-1 -flatten_hierarchy none -directive RuntimeOptimized -fsm_extraction off -verilog_define SINTESIS2default:defaultZ4-113h px� 
:
Starting synth_design
149*	vivadotclZ4-321h px� 
�
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2
	Synthesis2default:default2
xc7a100t2default:defaultZ17-347h px� 
�
0Got license for feature '%s' and/or device '%s'
310*common2
	Synthesis2default:default2
xc7a100t2default:defaultZ17-349h px� 
W
Loading part %s157*device2$
xc7a100tcsg324-12default:defaultZ21-403h px� 
�
[Reference run did not run incremental synthesis because %s; reverting to default synthesis
2138*designutils2+
the design is too small2default:defaultZ20-4072h px� 
�
�Flow is switching to default flow due to incremental criteria not met. If you would like to alter this behaviour and have the flow terminate instead, please set the following parameter config_implementation {autoIncr.Synth.RejectBehavior Terminate}2229*designutilsZ20-4379h px� 
�
HMultithreading enabled for synth_design using a maximum of %s processes.4828*oasys2
42default:defaultZ8-7079h px� 
a
?Launching helper process for spawning children vivado processes4827*oasysZ8-7078h px� 
`
#Helper process launched with PID %s4824*oasys2
224462default:defaultZ8-7075h px� 
�
%s*synth2�
�Starting RTL Elaboration : Time (s): cpu = 00:00:03 ; elapsed = 00:00:03 . Memory (MB): peak = 2592.781 ; gain = 0.000 ; free physical = 2321 ; free virtual = 23659
2default:defaulth px� 
�
synthesizing module '%s'%s4497*oasys2
system2default:default2
 2default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
32default:default8@Z8-6157h px� 
�
,$readmem data file '%s' is read successfully3426*oasys2,
../firmware/firmware.hex2default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
772default:default8@Z8-3876h px� 
�
synthesizing module '%s'%s4497*oasys2
picorv322default:default2
 2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
572default:default8@Z8-6157h px� 
R
%s
*synth2:
&	Parameter ENABLE_IRQ bound to: 1'b1 
2default:defaulth p
x
� 
X
%s
*synth2@
,	Parameter ENABLE_IRQ_QREGS bound to: 1'b0 
2default:defaulth p
x
� 
s
%s
*synth2[
G	Parameter LATCHED_IRQ bound to: 32'b00000000000000000000000000000000 
2default:defaulth p
x
� 
�
case item %s is unreachable151*oasys2
1'b02default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
12482default:default8@Z8-151h px� 
�
-case statement is not full and has no default155*oasys2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
12942default:default8@Z8-155h px� 
�
-case statement is not full and has no default155*oasys2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
14772default:default8@Z8-155h px� 
�
case item %s is unreachable151*oasys2
1'b02default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
15632default:default8@Z8-151h px� 
�
case item %s is unreachable151*oasys2
1'b02default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
15632default:default8@Z8-151h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2,
mem_la_firstword_reg_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
3762default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2&
last_mem_valid_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
3772default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2(
next_insn_opcode_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
4172default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2)
mem_la_secondword_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
5552default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2,
prefetched_high_word_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
5562default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2%
q_ascii_instr_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
7612default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2"
q_insn_imm_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
7622default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2%
q_insn_opcode_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
7632default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2"
q_insn_rs1_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
7642default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2"
q_insn_rs2_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
7652default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2!
q_insn_rd_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
7662default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2 
dbg_next_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
7672default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2&
dbg_valid_insn_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
7702default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2*
cached_ascii_instr_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
7752default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2'
cached_insn_imm_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
7762default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2*
cached_insn_opcode_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
7782default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2'
cached_insn_rs1_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
7812default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2'
cached_insn_rs2_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
7822default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2&
cached_insn_rd_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
7832default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2%
dbg_insn_addr_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
7872default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys24
 clear_prefetched_high_word_q_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
12722default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2(
set_mem_do_rinst_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
13792default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2(
set_mem_do_rdata_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
13802default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2(
set_mem_do_wdata_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
13812default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2#
alu_out_0_q_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
13832default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2 
alu_wait_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
13862default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2"
alu_wait_2_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
13872default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2"
dbg_rs1val_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
13902default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2"
dbg_rs2val_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
13912default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2(
dbg_rs1val_valid_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
13922default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2(
dbg_rs2val_valid_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
13932default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2(
next_irq_pending_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
14132default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2)
decoder_trigger_q_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
14262default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys20
decoder_pseudo_trigger_q_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
14282default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2%
latched_trace_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
14442default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2"
current_pc_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
14742default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2*
instr_ecall_ebreak_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
10682default:default8@Z8-6014h px� 
�
+Unused sequential element %s was removed. 
4326*oasys2$
pcpi_timeout_reg2default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
14492default:default8@Z8-6014h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
picorv322default:default2
 2default:default2
12default:default2
12default:default2N
8/home/licit/laboratorio-6-jarvis/src/picorv32/picorv32.v2default:default2
572default:default8@Z8-6155h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2

pcpi_valid2default:default2
picorv322default:default2!
picorv32_core2default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
432default:default8@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
	pcpi_insn2default:default2
picorv322default:default2!
picorv32_core2default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
432default:default8@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
pcpi_rs12default:default2
picorv322default:default2!
picorv32_core2default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
432default:default8@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
pcpi_rs22default:default2
picorv322default:default2!
picorv32_core2default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
432default:default8@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
pcpi_wr2default:default2
picorv322default:default2!
picorv32_core2default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
432default:default8@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
pcpi_rd2default:default2
picorv322default:default2!
picorv32_core2default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
432default:default8@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
	pcpi_wait2default:default2
picorv322default:default2!
picorv32_core2default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
432default:default8@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2

pcpi_ready2default:default2
picorv322default:default2!
picorv32_core2default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
432default:default8@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
eoi2default:default2
picorv322default:default2!
picorv32_core2default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
432default:default8@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
trace_valid2default:default2
picorv322default:default2!
picorv32_core2default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
432default:default8@Z8-7071h px� 
�
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2

trace_data2default:default2
picorv322default:default2!
picorv32_core2default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
432default:default8@Z8-7071h px� 
�
Kinstance '%s' of module '%s' has %s connections declared, but only %s given4757*oasys2!
picorv32_core2default:default2
picorv322default:default2
272default:default2
162default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
432default:default8@Z8-7023h px� 
�
0Net %s in module/entity %s does not have driver.3422*oasys2
an_out2default:default2
system2default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
122default:default8@Z8-3848h px� 
�
0Net %s in module/entity %s does not have driver.3422*oasys2
c_out2default:default2
system2default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
132default:default8@Z8-3848h px� 
�
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
system2default:default2
 2default:default2
22default:default2
12default:default2K
5/home/licit/laboratorio-6-jarvis/src/verilog/system.v2default:default2
32default:default8@Z8-6155h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_wr2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[31]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[30]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[29]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[28]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[27]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[26]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[25]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[24]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[23]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[22]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[21]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[20]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[19]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[18]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[17]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[16]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[15]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[14]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[13]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[12]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[11]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
pcpi_rd[10]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2

pcpi_rd[9]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2

pcpi_rd[8]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2

pcpi_rd[7]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2

pcpi_rd[6]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2

pcpi_rd[5]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2

pcpi_rd[4]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2

pcpi_rd[3]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2

pcpi_rd[2]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2

pcpi_rd[1]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2

pcpi_rd[0]2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
	pcpi_wait2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2

pcpi_ready2default:default2
picorv322default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
	an_out[7]2default:default2
system2default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
	an_out[6]2default:default2
system2default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
	an_out[5]2default:default2
system2default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
	an_out[4]2default:default2
system2default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
	an_out[3]2default:default2
system2default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
	an_out[2]2default:default2
system2default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
	an_out[1]2default:default2
system2default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
	an_out[0]2default:default2
system2default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
c_out[7]2default:default2
system2default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
c_out[6]2default:default2
system2default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
c_out[5]2default:default2
system2default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
c_out[4]2default:default2
system2default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
c_out[3]2default:default2
system2default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
c_out[2]2default:default2
system2default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
c_out[1]2default:default2
system2default:defaultZ8-7129h px� 
�
9Port %s in module %s is either unconnected or has no load4866*oasys2
c_out[0]2default:default2
system2default:defaultZ8-7129h px� 
�
%s*synth2�
�Finished RTL Elaboration : Time (s): cpu = 00:00:04 ; elapsed = 00:00:04 . Memory (MB): peak = 2592.781 ; gain = 0.000 ; free physical = 1954 ; free virtual = 23293
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
M
%s
*synth25
!Start Handling Custom Attributes
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Handling Custom Attributes : Time (s): cpu = 00:00:05 ; elapsed = 00:00:05 . Memory (MB): peak = 2592.781 ; gain = 0.000 ; free physical = 2881 ; free virtual = 24223
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished RTL Optimization Phase 1 : Time (s): cpu = 00:00:05 ; elapsed = 00:00:05 . Memory (MB): peak = 2592.781 ; gain = 0.000 ; free physical = 2881 ; free virtual = 24223
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:00.022default:default2
00:00:00.022default:default2
2592.7812default:default2
0.0002default:default2
28752default:default2
242162default:defaultZ17-722h px� 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px� 
>

Processing XDC Constraints
244*projectZ1-262h px� 
=
Initializing timing engine
348*projectZ1-569h px� 
�
Parsing XDC File [%s]
179*designutils2Q
;/home/licit/laboratorio-6-jarvis/src/constraints/system.xdc2default:default8Z20-179h px� 
�
Finished Parsing XDC File [%s]
178*designutils2Q
;/home/licit/laboratorio-6-jarvis/src/constraints/system.xdc2default:default8Z20-178h px� 
�
�Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2O
;/home/licit/laboratorio-6-jarvis/src/constraints/system.xdc2default:default2,
.Xil/system_propImpl.xdc2default:defaultZ1-236h px� 
H
&Completed Processing XDC Constraints

245*projectZ1-263h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2
00:00:002default:default2
2624.7072default:default2
0.0002default:default2
26962default:default2
240352default:defaultZ17-722h px� 
~
!Unisim Transformation Summary:
%s111*project29
%No Unisim elements were transformed.
2default:defaultZ1-111h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common24
 Constraint Validation Runtime : 2default:default2
00:00:002default:default2
00:00:00.012default:default2
2624.7072default:default2
0.0002default:default2
26962default:default2
240352default:defaultZ17-722h px� 
�
[Reference run did not run incremental synthesis because %s; reverting to default synthesis
2138*designutils2+
the design is too small2default:defaultZ20-4072h px� 
�
�Flow is switching to default flow due to incremental criteria not met. If you would like to alter this behaviour and have the flow terminate instead, please set the following parameter config_implementation {autoIncr.Synth.RejectBehavior Terminate}2229*designutilsZ20-4379h px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Constraint Validation : Time (s): cpu = 00:00:09 ; elapsed = 00:00:10 . Memory (MB): peak = 2624.707 ; gain = 31.926 ; free physical = 2858 ; free virtual = 24199
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
V
%s
*synth2>
*Start Loading Part and Timing Information
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
K
%s
*synth23
Loading part: xc7a100tcsg324-1
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Loading Part and Timing Information : Time (s): cpu = 00:00:09 ; elapsed = 00:00:10 . Memory (MB): peak = 2624.707 ; gain = 31.926 ; free physical = 2858 ; free virtual = 24199
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
Z
%s
*synth2B
.Start Applying 'set_property' XDC Constraints
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished applying 'set_property' XDC Constraints : Time (s): cpu = 00:00:09 ; elapsed = 00:00:10 . Memory (MB): peak = 2624.707 ; gain = 31.926 ; free physical = 2858 ; free virtual = 24199
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished RTL Optimization Phase 2 : Time (s): cpu = 00:00:10 ; elapsed = 00:00:10 . Memory (MB): peak = 2624.707 ; gain = 31.926 ; free physical = 2850 ; free virtual = 24193
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
L
%s
*synth24
 Start RTL Component Statistics 
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
� 
:
%s
*synth2"
+---Adders : 
2default:defaulth p
x
� 
X
%s
*synth2@
,	   2 Input   32 Bit       Adders := 4     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   3 Input   32 Bit       Adders := 1     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   2 Input    5 Bit       Adders := 2     
2default:defaulth p
x
� 
8
%s
*synth2 
+---XORs : 
2default:defaulth p
x
� 
Z
%s
*synth2B
.	   2 Input     32 Bit         XORs := 1     
2default:defaulth p
x
� 
=
%s
*synth2%
+---Registers : 
2default:defaulth p
x
� 
Z
%s
*synth2B
.	               36 Bit    Registers := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	               32 Bit    Registers := 17    
2default:defaulth p
x
� 
Z
%s
*synth2B
.	                8 Bit    Registers := 2     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	                5 Bit    Registers := 5     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	                4 Bit    Registers := 1     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	                2 Bit    Registers := 3     
2default:defaulth p
x
� 
Z
%s
*synth2B
.	                1 Bit    Registers := 84    
2default:defaulth p
x
� 
8
%s
*synth2 
+---RAMs : 
2default:defaulth p
x
� 
k
%s
*synth2S
?	             128K Bit	(4096 X 32 bit)          RAMs := 1     
2default:defaulth p
x
� 
i
%s
*synth2Q
=	             1024 Bit	(32 X 32 bit)          RAMs := 1     
2default:defaulth p
x
� 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
� 
X
%s
*synth2@
,	   2 Input   32 Bit        Muxes := 15    
2default:defaulth p
x
� 
X
%s
*synth2@
,	   5 Input   32 Bit        Muxes := 5     
2default:defaulth p
x
� 
X
%s
*synth2@
,	  10 Input   32 Bit        Muxes := 3     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   9 Input   32 Bit        Muxes := 4     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   6 Input   32 Bit        Muxes := 1     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   4 Input   32 Bit        Muxes := 5     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   2 Input   16 Bit        Muxes := 1     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   4 Input    8 Bit        Muxes := 1     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   2 Input    8 Bit        Muxes := 10    
2default:defaulth p
x
� 
X
%s
*synth2@
,	   9 Input    8 Bit        Muxes := 1     
2default:defaulth p
x
� 
X
%s
*synth2@
,	  10 Input    5 Bit        Muxes := 1     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   2 Input    5 Bit        Muxes := 2     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   9 Input    5 Bit        Muxes := 2     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   3 Input    4 Bit        Muxes := 2     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   5 Input    4 Bit        Muxes := 1     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   4 Input    4 Bit        Muxes := 1     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   2 Input    4 Bit        Muxes := 1     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   2 Input    3 Bit        Muxes := 3     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   6 Input    2 Bit        Muxes := 1     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   2 Input    2 Bit        Muxes := 2     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   9 Input    2 Bit        Muxes := 1     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   9 Input    1 Bit        Muxes := 34    
2default:defaulth p
x
� 
X
%s
*synth2@
,	   2 Input    1 Bit        Muxes := 33    
2default:defaulth p
x
� 
X
%s
*synth2@
,	   4 Input    1 Bit        Muxes := 5     
2default:defaulth p
x
� 
X
%s
*synth2@
,	  10 Input    1 Bit        Muxes := 7     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   5 Input    1 Bit        Muxes := 2     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   7 Input    1 Bit        Muxes := 1     
2default:defaulth p
x
� 
X
%s
*synth2@
,	   3 Input    1 Bit        Muxes := 4     
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
O
%s
*synth27
#Finished RTL Component Statistics 
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
H
%s
*synth20
Start Part Resource Summary
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s
*synth2k
WPart Resources:
DSPs: 240 (col length:80)
BRAMs: 270 (col length: RAMB18 80 RAMB36 40)
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
K
%s
*synth23
Finished Part Resource Summary
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
W
%s
*synth2?
+Start Cross Boundary and Area Optimization
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
H
&Parallel synthesis criteria is not met4829*oasysZ8-7080h px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Cross Boundary and Area Optimization : Time (s): cpu = 00:00:14 ; elapsed = 00:00:15 . Memory (MB): peak = 2624.707 ; gain = 31.926 ; free physical = 2822 ; free virtual = 24171
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�---------------------------------------------------------------------------------
Start ROM, RAM, DSP, Shift Register and Retiming Reporting
2default:defaulth px� 
~
%s*synth2f
R---------------------------------------------------------------------------------
2default:defaulth px� 
d
%s*synth2L
8
Block RAM: Preliminary Mapping Report (see note below)
2default:defaulth px� 
�
%s*synth2�
�+------------+------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+
2default:defaulth px� 
�
%s*synth2�
�|Module Name | RTL Object | PORT A (Depth x Width) | W | R | PORT B (Depth x Width) | W | R | Ports driving FF | RAMB18 | RAMB36 | 
2default:defaulth px� 
�
%s*synth2�
�+------------+------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+
2default:defaulth px� 
�
%s*synth2�
�|system      | memory_reg | 4 K x 32(READ_FIRST)   | W |   | 4 K x 32(WRITE_FIRST)  |   | R | Port A and B     | 0      | 4      | 
2default:defaulth px� 
�
%s*synth2�
�+------------+------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+

2default:defaulth px� 
�
%s*synth2�
�Note: The table above is a preliminary report that shows the Block RAMs at the current stage of the synthesis flow. Some Block RAMs may be reimplemented as non Block RAM primitives later in the synthesis flow. Multiple instantiated Block RAMs are reported only once. 
2default:defaulth px� 
j
%s*synth2R
>
Distributed RAM: Preliminary Mapping Report (see note below)
2default:defaulth px� 
}
%s*synth2e
Q+--------------+-------------+-----------+----------------------+--------------+
2default:defaulth px� 
~
%s*synth2f
R|Module Name   | RTL Object  | Inference | Size (Depth x Width) | Primitives   | 
2default:defaulth px� 
}
%s*synth2e
Q+--------------+-------------+-----------+----------------------+--------------+
2default:defaulth px� 
~
%s*synth2f
R|picorv32_core | cpuregs_reg | Implied   | 32 x 32              | RAM32M x 12  | 
2default:defaulth px� 
~
%s*synth2f
R+--------------+-------------+-----------+----------------------+--------------+

2default:defaulth px� 
�
%s*synth2�
�Note: The table above is a preliminary report that shows the Distributed RAMs at the current stage of the synthesis flow. Some Distributed RAMs may be reimplemented as non Distributed RAM primitives later in the synthesis flow. Multiple instantiated RAMs are reported only once.
2default:defaulth px� 
�
%s*synth2�
�---------------------------------------------------------------------------------
Finished ROM, RAM, DSP, Shift Register and Retiming Reporting
2default:defaulth px� 
~
%s*synth2f
R---------------------------------------------------------------------------------
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
R
%s
*synth2:
&Start Applying XDC Timing Constraints
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Applying XDC Timing Constraints : Time (s): cpu = 00:00:19 ; elapsed = 00:00:19 . Memory (MB): peak = 2624.707 ; gain = 31.926 ; free physical = 2707 ; free virtual = 24053
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s
*synth2�
�---------------------------------------------------------------------------------
Start ROM, RAM, DSP, Shift Register and Retiming Reporting
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
M
%s
*synth25
!
Block RAM: Final Mapping Report
2default:defaulth p
x
� 
�
%s
*synth2�
�+------------+------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+
2default:defaulth p
x
� 
�
%s
*synth2�
�|Module Name | RTL Object | PORT A (Depth x Width) | W | R | PORT B (Depth x Width) | W | R | Ports driving FF | RAMB18 | RAMB36 | 
2default:defaulth p
x
� 
�
%s
*synth2�
�+------------+------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+
2default:defaulth p
x
� 
�
%s
*synth2�
�|system      | memory_reg | 4 K x 32(READ_FIRST)   | W |   | 4 K x 32(WRITE_FIRST)  |   | R | Port A and B     | 0      | 4      | 
2default:defaulth p
x
� 
�
%s
*synth2�
�+------------+------------+------------------------+---+---+------------------------+---+---+------------------+--------+--------+

2default:defaulth p
x
� 
S
%s
*synth2;
'
Distributed RAM: Final Mapping Report
2default:defaulth p
x
� 
}
%s
*synth2e
Q+--------------+-------------+-----------+----------------------+--------------+
2default:defaulth p
x
� 
~
%s
*synth2f
R|Module Name   | RTL Object  | Inference | Size (Depth x Width) | Primitives   | 
2default:defaulth p
x
� 
}
%s
*synth2e
Q+--------------+-------------+-----------+----------------------+--------------+
2default:defaulth p
x
� 
~
%s
*synth2f
R|picorv32_core | cpuregs_reg | Implied   | 32 x 32              | RAM32M x 12  | 
2default:defaulth p
x
� 
~
%s
*synth2f
R+--------------+-------------+-----------+----------------------+--------------+

2default:defaulth p
x
� 
�
%s
*synth2�
�---------------------------------------------------------------------------------
Finished ROM, RAM, DSP, Shift Register and Retiming Reporting
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
E
%s
*synth2-
Start Technology Mapping
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
memory_reg_02default:default2
Block2default:defaultZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
memory_reg_12default:default2
Block2default:defaultZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
memory_reg_22default:default2
Block2default:defaultZ8-7052h px� 
�
�The timing for the instance %s (implemented as a %s RAM) might be sub-optimal as no optional output register could be merged into the ram block. Providing additional output register may help in improving timing.
4799*oasys2 
memory_reg_32default:default2
Block2default:defaultZ8-7052h px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Technology Mapping : Time (s): cpu = 00:00:19 ; elapsed = 00:00:20 . Memory (MB): peak = 2624.707 ; gain = 31.926 ; free physical = 2699 ; free virtual = 24047
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
?
%s
*synth2'
Start IO Insertion
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
H
%s
*synth20
Start Final Netlist Cleanup
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
K
%s
*synth23
Finished Final Netlist Cleanup
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished IO Insertion : Time (s): cpu = 00:00:23 ; elapsed = 00:00:23 . Memory (MB): peak = 2624.707 ; gain = 31.926 ; free physical = 2700 ; free virtual = 24046
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
O
%s
*synth27
#Start Renaming Generated Instances
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Renaming Generated Instances : Time (s): cpu = 00:00:23 ; elapsed = 00:00:23 . Memory (MB): peak = 2624.707 ; gain = 31.926 ; free physical = 2700 ; free virtual = 24046
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
M
%s
*synth25
!Start Handling Custom Attributes
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Handling Custom Attributes : Time (s): cpu = 00:00:23 ; elapsed = 00:00:23 . Memory (MB): peak = 2624.707 ; gain = 31.926 ; free physical = 2700 ; free virtual = 24046
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
K
%s
*synth23
Start Writing Synthesis Report
2default:defaulth p
x
� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
A
%s
*synth2)

Report BlackBoxes: 
2default:defaulth p
x
� 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
� 
J
%s
*synth22
| |BlackBox name |Instances |
2default:defaulth p
x
� 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
� 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
� 
A
%s*synth2)

Report Cell Usage: 
2default:defaulth px� 
F
%s*synth2.
+------+---------+------+
2default:defaulth px� 
F
%s*synth2.
|      |Cell     |Count |
2default:defaulth px� 
F
%s*synth2.
+------+---------+------+
2default:defaulth px� 
F
%s*synth2.
|1     |BUFG     |     1|
2default:defaulth px� 
F
%s*synth2.
|2     |CARRY4   |    91|
2default:defaulth px� 
F
%s*synth2.
|3     |LUT1     |    36|
2default:defaulth px� 
F
%s*synth2.
|4     |LUT2     |   140|
2default:defaulth px� 
F
%s*synth2.
|5     |LUT3     |   148|
2default:defaulth px� 
F
%s*synth2.
|6     |LUT4     |   251|
2default:defaulth px� 
F
%s*synth2.
|7     |LUT5     |   252|
2default:defaulth px� 
F
%s*synth2.
|8     |LUT6     |   473|
2default:defaulth px� 
F
%s*synth2.
|9     |MUXF7    |     1|
2default:defaulth px� 
F
%s*synth2.
|10    |RAM32M   |    10|
2default:defaulth px� 
F
%s*synth2.
|11    |RAM32X1D |     4|
2default:defaulth px� 
F
%s*synth2.
|12    |RAMB36E1 |     4|
2default:defaulth px� 
F
%s*synth2.
|16    |FDRE     |   533|
2default:defaulth px� 
F
%s*synth2.
|17    |FDSE     |    55|
2default:defaulth px� 
F
%s*synth2.
|18    |IBUF     |     3|
2default:defaulth px� 
F
%s*synth2.
|19    |OBUF     |    10|
2default:defaulth px� 
F
%s*synth2.
|20    |OBUFT    |    16|
2default:defaulth px� 
F
%s*synth2.
+------+---------+------+
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
�
%s*synth2�
�Finished Writing Synthesis Report : Time (s): cpu = 00:00:23 ; elapsed = 00:00:23 . Memory (MB): peak = 2624.707 ; gain = 31.926 ; free physical = 2700 ; free virtual = 24046
2default:defaulth px� 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
� 
r
%s
*synth2Z
FSynthesis finished with 0 errors, 0 critical warnings and 1 warnings.
2default:defaulth p
x
� 
�
%s
*synth2�
�Synthesis Optimization Runtime : Time (s): cpu = 00:00:21 ; elapsed = 00:00:22 . Memory (MB): peak = 2624.707 ; gain = 0.000 ; free physical = 2752 ; free virtual = 24098
2default:defaulth p
x
� 
�
%s
*synth2�
�Synthesis Optimization Complete : Time (s): cpu = 00:00:23 ; elapsed = 00:00:24 . Memory (MB): peak = 2624.715 ; gain = 31.926 ; free physical = 2752 ; free virtual = 24098
2default:defaulth p
x
� 
B
 Translating synthesized netlist
350*projectZ1-571h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:00.012default:default2
00:00:00.012default:default2
2624.7152default:default2
0.0002default:default2
28492default:default2
241952default:defaultZ17-722h px� 
g
-Analyzing %s Unisim elements for replacement
17*netlist2
1102default:defaultZ29-17h px� 
j
2Unisim Transformation completed in %s CPU seconds
28*netlist2
02default:defaultZ29-28h px� 
�
�Netlist '%s' is not ideal for floorplanning, since the cellview '%s' contains a large number of primitives.  Please consider enabling hierarchy in synthesis if you want to do floorplanning.
310*netlist2
system2default:default2
picorv322default:defaultZ29-101h px� 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px� 
u
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
02default:default2
02default:defaultZ31-138h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2
00:00:002default:default2
2624.7152default:default2
0.0002default:default2
27882default:default2
241342default:defaultZ17-722h px� 
�
!Unisim Transformation Summary:
%s111*project2�
�  A total of 14 instances were transformed.
  RAM32M => RAM32M (RAMD32(x6), RAMS32(x2)): 10 instances
  RAM32X1D => RAM32X1D (RAMD32(x2)): 4 instances
2default:defaultZ1-111h px� 
f
$Synth Design complete, checksum: %s
562*	vivadotcl2
da57ef62default:defaultZ4-1430h px� 
U
Releasing license: %s
83*common2
	Synthesis2default:defaultZ17-83h px� 
�
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
302default:default2
1102default:default2
02default:default2
02default:defaultZ4-41h px� 
^
%s completed successfully
29*	vivadotcl2 
synth_design2default:defaultZ4-42h px� 
�
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2"
synth_design: 2default:default2
00:00:312default:default2
00:00:272default:default2
2624.7152default:default2
32.0232default:default2
28682default:default2
242142default:defaultZ17-722h px� 
�
 The %s '%s' has been generated.
621*common2

checkpoint2default:default2e
Q/home/licit/laboratorio-6-jarvis/vivado/proyecto/proyecto.runs/synth_1/system.dcp2default:defaultZ17-1381h px� 
�
%s4*runtcl2v
bExecuting : report_utilization -file system_utilization_synth.rpt -pb system_utilization_synth.pb
2default:defaulth px� 
�
Exiting %s at %s...
206*common2
Vivado2default:default2,
Thu Jul 21 08:54:11 20222default:defaultZ17-206h px� 


End Record