
# ============================================================
# merged_top_tf_hdmi_audio.sdc
# 用于：TF 图片轮播 + HDMI(视频+音频) 整合工程
#
# 合并原则：
# 1) 保留 top.sdc 里的系统主时钟与 derive_clocks
# 2) 把 timing(2).sdc 里的 pixel/serial 时钟改成整合工程实际名字
# 3) 增加 hdmi_5x_clk / audio_mclk 的命名
# 4) 不再对内部 net 直接 create_clock，避免和 derive_clocks 重复
# ============================================================

# ------------------------------------------------------------
# 1. 板级输入主时钟
#    你的原 top.sdc 里 clk=50MHz，所以这里仍然用 20ns
# ------------------------------------------------------------
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

# ------------------------------------------------------------
# 2. 自动推导 PLL 输出时钟
# ------------------------------------------------------------
derive_clocks

# ------------------------------------------------------------
# 3. 给关键 PLL 输出命名
#    下面这些路径是按你当前工程实例名写的：
#      sys_pll      -> sys_pll_m0
#      video_pll    -> video_pll_m0
#      音频 PLL     -> audio_pll_m0
#
#    如果综合后实例路径略有不同，就按 Timing Analyzer 里的实际 pin 路径改。
# ------------------------------------------------------------
rename_clock -name {sd_card_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {video_clk}   -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {audio_mclk}  -source [get_ports {clk}] -master_clock {clk} [get_pins {audio_pll_m0/pll_inst.clkc[2]}]

# ------------------------------------------------------------
# 4. 从原 timing(2).sdc 迁移过来的 HDMI 像素/串行时钟约束
#
#    原文件是：
#      create_clock -name pixel_clk  ... [get_nets {S_pixel_clk}]
#      create_clock -name serial_clk ... [get_nets {S_serial_clk}]
#      set_clock_groups -exclusive ...
#
#    在整合工程里，这两个时钟已经由 video_pll 产生，并通过 derive_clocks 建出来了，
#    所以这里只保留“组关系”，不再重复 create_clock。
#
#    这里沿用原作者的写法，把 video_clk 和 hdmi_5x_clk 设成 exclusive。
#    如果后面 Timing Analyzer 明确显示两者之间需要做同步时序分析，再把这一条去掉。
# ------------------------------------------------------------
# Pixel and serializer clocks operate simultaneously and are related.
# Do not exclude their real data paths from timing analysis.

# ------------------------------------------------------------
# 5. 音频 I2S 发生器/接收器跨域
#
#    audio_mclk 域里生成 I2S，video_clk 域里做三拍同步采样。
#    这类路径不是普通同步时序路径，直接切掉更合理。
# ------------------------------------------------------------
# No blanket cross-clock exception. Verified synchronizer entry paths are
# constrained individually after CDC audit; downstream stages remain timed.

# Only asynchronous assertion at dedicated reset synchronizers is exempt.
# Locally synchronous reset outputs and ordinary data paths remain timed.
set_false_path -to [get_pins {reset_*_m0/release_pipe_reg*.sr mem_reset_release_reg*.sr hdmi_phy_m0/*/pixel_reset_release_reg*.sr}]
# RTL aliases cover constant-D entry flops packed into other physical cells.
set_false_path -to [get_regs {mem_reset_release[0] reset_*_m0/release_pipe[0]}]

# Do not exempt packed physical-cell reset pins by name: they can contain
# unrelated registers after a fresh synthesis. Recovery/removal on ordinary
# functional registers remains checked; only dedicated reset synchronizers
# above are exempt.

# Vendor dual-clock FIFO reset inputs feed async-assert / local-release
# synchronizers (asy_[rw]_rst0/1). Exempt their assertion pins only.
set_false_path -to [get_pins {wav_audio_fifo_m0/asy_*_rst*_reg*.sr frame_read_write_m0/*_buf/asy_*_rst*_reg*.sr}]
# Select by RTL register alias as well: synthesis can pack the constant-D
# reset-entry flop into an unrelated physical cell name. Only rst0 (the
# asynchronous entry) is selected here, not the local rst0 -> rst1 path.
set_false_path -to [get_regs {wav_audio_fifo_m0/asy_w_rst0 wav_audio_fifo_m0/asy_r_rst0 frame_read_write_m0/write_buf/asy_w_rst0 frame_read_write_m0/write_buf/asy_r_rst0 frame_read_write_m0/read_buf/asy_w_rst0 frame_read_write_m0/read_buf/asy_r_rst0}]

# Audited asynchronous single-bit controls/events: only first sampling stage.
# Preserve a finite routing bound (shorter than the fastest participating
# clock); later synchronizer stages retain normal setup/hold checks.
set_max_delay -datapath_only -to [get_regs {audio_pcm_m0/pause_sync[0] audio_pcm_m0/volume_meta[*] audio_sample_toggle_sys_sync[0] audio_pause_ack_sync[0]}] 19.7
set_false_path -hold -to [get_regs {audio_pcm_m0/pause_sync[0] audio_pcm_m0/volume_meta[*] audio_sample_toggle_sys_sync[0] audio_pause_ack_sync[0]}]
set_max_delay -datapath_only -to [get_regs {audio_sample_toggle_sync[0] audio_receiver_m0/S_i2s_bclk_1d audio_receiver_m0/S_i2s_lrck_1d audio_receiver_m0/S_i2s_dout_1d}] 39.7
set_false_path -hold -to [get_regs {audio_sample_toggle_sync[0] audio_receiver_m0/S_i2s_bclk_1d audio_receiver_m0/S_i2s_lrck_1d audio_receiver_m0/S_i2s_dout_1d}]
# WAV FIFO uses the same verified Gray-pointer crossing as the two frame
# FIFOs. The existing per-IP FIFO constraints are retained for frame buffers.
set_max_delay -datapath_only -from [get_regs {wav_audio_fifo_m0/*/primary_addr_gray_reg[*]}] -to [get_regs {wav_audio_fifo_m0/*/sync_r1[*]}] 19.7
set_false_path -hold -from [get_regs {wav_audio_fifo_m0/*/primary_addr_gray_reg[*]}] -to [get_regs {wav_audio_fifo_m0/*/sync_r1[*]}]

# Frame-completion/owner and display-index handshakes enter explicit
# two/three-stage synchronizers. Payload is held until the returned ack.
set_max_delay -datapath_only -to [get_regs {audio_presented_mode_sync[0] audio_presented_index_meta[*] audio_epoch_sync[0] sd_card_single_bmp_m0/finish_sync[0] sd_card_single_bmp_m0/owner_sync[0] sd_card_single_bmp_m0/write_ack_sync[0] sd_card_single_bmp_m0/display_index_meta[*] video_read_fat32_m0/finish_sync[0] video_read_fat32_m0/owner_sync[0] video_read_fat32_m0/ack_sync[0] video_read_fat32_m0/display_index_meta[*]}] 19.7
set_false_path -hold -to [get_regs {audio_presented_mode_sync[0] audio_presented_index_meta[*] audio_epoch_sync[0] sd_card_single_bmp_m0/finish_sync[0] sd_card_single_bmp_m0/owner_sync[0] sd_card_single_bmp_m0/write_ack_sync[0] sd_card_single_bmp_m0/display_index_meta[*] video_read_fat32_m0/finish_sync[0] video_read_fat32_m0/owner_sync[0] video_read_fat32_m0/ack_sync[0] video_read_fat32_m0/display_index_meta[*]}]

# ------------------------------------------------------------
# 6. 可选：如果你的 frame_read_write 内部异步 FIFO 已经完全处理好了跨域，
#    且报告里总出现 video_clk / sd_card_clk / ext_mem_clk 之间的大量伪违例，
#    可以打开下面这组异步分组。
#
#    建议先不要开，先看报告；
#    只有确认这些跨域都只经过 FIFO / 同步器时再开。
# ------------------------------------------------------------
# set_clock_groups -asynchronous \
#     -group [get_clocks {sd_card_clk}] \
#     -group [get_clocks {ext_mem_clk}] \
#     -group [get_clocks {video_clk}] \
#     -group [get_clocks {hdmi_5x_clk}]

# ------------------------------------------------------------
# 7. 其余 Input/Output delay 先保持空白
#    你当前两个原始 sdc 也都没有写这部分。
# ------------------------------------------------------------
