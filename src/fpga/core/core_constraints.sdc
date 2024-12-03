#
# user core constraints
#
# put your clock groups in here as well as any net assignments
derive_pll_clocks
derive_clock_uncertainty


set_clock_groups -asynchronous \
 -group { bridge_spiclk } \
 -group { clk_74a } \
 -group { clk_74b } \
 -group { ic|mp1|mf_pllbase_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk } \
 -group { ic|mp1|mf_pllbase_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk } 

 
set clk_sys {ic|mp1|mf_pllbase_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}
set clk_56m {ic|mp1|mf_pllbase_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}
 
set_multicycle_path -from [get_clocks $clk_56m] -to [get_clocks $clk_sys] -setup 2
set_multicycle_path -from [get_clocks $clk_56m] -to [get_clocks $clk_sys] -hold 1

# Effective clock is only half of the system clock, so allow 2 clock cycles for the paths in the T80 cpu
set_multicycle_path -from {ic|cpu|*} -setup 2
set_multicycle_path -from {ic|cpu|*} -hold 1

set_multicycle_path -to   {ic|cpu|*} -setup 2
set_multicycle_path -to   {ic|cpu|*} -hold 1

# The CE is only active in every 2 clocks, so allow 2 clock cycles
set_multicycle_path -from {ic|tape|*} -setup 2
set_multicycle_path -from {ic|tape|*} -hold 1

set_multicycle_path -from {ic|wd1793|sbuf|*} -setup 2
set_multicycle_path -from {ic|wd1793|sbuf|*} -hold 1
set_multicycle_path -from {ic|wd1793|edsk_rtl_0|*} -setup 2
set_multicycle_path -from {ic|wd1793|edsk_rtl_0|*} -hold 1
set_multicycle_path -from {ic|wd1793|layout_r*} -setup 2
set_multicycle_path -from {ic|wd1793|layout_r*} -hold 1
set_multicycle_path -from {ic|wd1793|disk_track*} -setup 2
set_multicycle_path -from {ic|wd1793|disk_track*} -hold 1
set_multicycle_path -from {ic|wd1793|edsk_addr*} -setup 2
set_multicycle_path -from {ic|wd1793|edsk_addr*} -hold 1
set_multicycle_path -to   {ic|wd1793|state[*]} -setup 2
set_multicycle_path -to   {ic|wd1793|state[*]} -hold 1
set_multicycle_path -to   {ic|wd1793|wait_time[*]} -setup 2
set_multicycle_path -to   {ic|wd1793|wait_time[*]} -hold 1

set_false_path -to {ic|wd1793|s_seekerr}

set_multicycle_path -from {ic|u765|sbuf|*} -setup 2
set_multicycle_path -from {ic|u765|sbuf|*} -hold 1
set_multicycle_path -from {ic|u765|image_track_offsets_rtl_0|*} -setup 2
set_multicycle_path -from {ic|u765|image_track_offsets_rtl_0|*} -hold 1
set_multicycle_path -to   {ic|u765|i_*} -setup 2
set_multicycle_path -to   {ic|u765|i_*} -hold 1
set_multicycle_path -to   {ic|u765|i_*[*]} -setup 2
set_multicycle_path -to   {ic|u765|i_*[*]} -hold 1
set_multicycle_path -to   {ic|u765|pcn[*]} -setup 2
set_multicycle_path -to   {ic|u765|pcn[*]} -hold 1
set_multicycle_path -to   {ic|u765|ncn[*]} -setup 2
set_multicycle_path -to   {ic|u765|ncn[*]} -hold 1
set_multicycle_path -to   {ic|u765|state[*]} -setup 2
set_multicycle_path -to   {ic|u765|state[*]} -hold 1
set_multicycle_path -to   {ic|u765|status[*]} -setup 2
set_multicycle_path -to   {ic|u765|status[*]} -hold 1
set_multicycle_path -to   {ic|u765|i_rpm_time[*][*][*]} -setup 8
set_multicycle_path -to   {ic|u765|i_rpm_time[*][*][*]} -hold 7

set_multicycle_path -from {ic|load} -setup 2
set_multicycle_path -from {ic|load} -hold 1

set_multicycle_path -to   {ic|turbosound|*} -setup 2
set_multicycle_path -to   {ic|turbosound|*} -hold 1
set_multicycle_path -to   {ic|saa1099|*} -setup 2
set_multicycle_path -to   {ic|saa1099|*} -hold 1

set_false_path -from {ic|init_reset}
#set_false_path -from {ic|hps_io|cfg*}
#set_false_path -from {ic|hps_io|status*}
set_false_path -from {ic|arch_reset}
set_false_path -from {ic|snap_loader|snap_reset}
#set_false_path -from {ic|kbd|Fn*}
#set_false_path -from {ic|kbd|mod*}
set_false_path -from {ic|plus3}
set_false_path -from {ic|page_reg_plus3*}
set_false_path -from {ic|page_reg_plus3[*]}
set_false_path -from {ic|zx48}
set_false_path -from {ic|p1024}
set_false_path -from {ic|pf1024}
#set_false_path -from {ic|hps_io|status[*]}

