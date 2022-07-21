#!/bin/bash -f
# ****************************************************************************
# Vivado (TM) v2021.2 (64-bit)
#
# Filename    : simulate.sh
# Simulator   : Xilinx Vivado Simulator
# Description : Script for simulating the design by launching the simulator
#
# Generated by Vivado on Thu Jul 21 13:14:09 CST 2022
# SW Build 3367213 on Tue Oct 19 02:47:39 MDT 2021
#
# IP Build 3369179 on Thu Oct 21 08:25:16 MDT 2021
#
# usage: simulate.sh
#
# ****************************************************************************
set -Eeuo pipefail
# simulate design
echo "xsim system_tb_behav -key {Behavioral:sim_1:Functional:system_tb} -tclbatch system_tb.tcl -view /home/licit/laboratorio-6-jarvis/vivado/prject1/system_tb_behav.wcfg -log simulate.log"
xsim system_tb_behav -key {Behavioral:sim_1:Functional:system_tb} -tclbatch system_tb.tcl -view /home/licit/laboratorio-6-jarvis/vivado/prject1/system_tb_behav.wcfg -log simulate.log

