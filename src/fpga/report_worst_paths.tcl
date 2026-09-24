## report_worst_paths.tcl
##
## Dumps the actual worst setup-timing paths for the clk_sys domain to a
## plain text file so CI captures them as a build artifact.
##
## Why: the standard `quartus_sh --flow compile` run only emits summary
## tables in ap_core.sta.rpt (worst slack + TNS per clock), not the named
## critical path, so a negative-slack summary line alone gives no way to
## root-cause it.
##
## Run standalone after a compile (reuses that compile's database, so it's
## fast - no need to re-run Analysis & Synthesis / Fitter):
##   quartus_sta -t report_worst_paths.tcl ap_core

load_package report
project_open ap_core

create_timing_netlist
read_sdc
update_timing_netlist

set sys_clk {ic|mp1|mf_pllbase_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}

set outdir "output_files"
file mkdir $outdir

report_timing -setup -npaths 20 -detail full_path -multi_corner \
    -to_clock $sys_clk -from_clock $sys_clk \
    -panel_name {Worst Setup Paths (clk_sys)} \
    -file "$outdir/worst_setup_paths.rpt"
