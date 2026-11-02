module top #(
    parameter integer MAX_BMP_NUM  = 10,
    parameter integer MAX_VIDEO_NUM = 3,
    parameter integer SLIDE_SECONDS = 3,
    // 1: fast 25MHz SPI after card initialization; 0: legacy 12.5MHz.
    parameter integer SD_FAST_SPI = 1,
    // Custom internal HDMI melody.  Values 0..7 map to C4..C5.
    parameter [2:0] MUSIC_NOTE0 = 3'd0,
    parameter [2:0] MUSIC_NOTE1 = 3'd0,
    parameter [2:0] MUSIC_NOTE2 = 3'd4,
    parameter [2:0] MUSIC_NOTE3 = 3'd4,
    parameter [2:0] MUSIC_NOTE4 = 3'd5,
    parameter [2:0] MUSIC_NOTE5 = 3'd5,
    parameter [2:0] MUSIC_NOTE6 = 3'd4,
    parameter [2:0] MUSIC_NOTE7 = 3'd4,
    parameter integer MUSIC_NOTE_HOLD_FRAMES = 24000,
    parameter signed [23:0] MUSIC_VOLUME = 24'sd2000000
)(
    input                       clk,
    input                       key1,
    input                       key2,
    input                       key3,
    input                       key4,
    output [5:0]                seg_sel,
    output [7:0]                seg_data,
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso
);

localparam integer MEM_DATA_BITS = 32;
localparam integer ADDR_BITS     = 21;
localparam [ADDR_BITS-1:0] FRAME_PIXELS = 21'd307200;
localparam [ADDR_BITS-1:0] FRAME_BASE   = 21'd0;
localparam [ADDR_BITS-1:0] FRAME_BASE_1 = FRAME_PIXELS;
localparam [ADDR_BITS-1:0] FRAME_BASE_2 = 21'd614400;
localparam [ADDR_BITS-1:0] FRAME_BASE_3 = 21'd921600;

wire sys_clk_sd;
wire mem_clk;
wire mem_clk_shift;
wire video_clk;
wire hdmi_5x_clk;
wire audio_mclk;
wire audio_pll_lock;
wire mem_pll_lock;
wire rst_mem_request;
wire rst_mem;
reg [15:0] mem_reset_release;
wire rst_core;
wire rst_all;
wire rst_video,rst_audio,rst_serial,rst_audio_stream,rst_subtitle;

wire sdr_init_done;
wire sdr_init_ref_vld;
wire sdr_busy;
wire display_valid;
wire playback_paused;
wire frame_write_finish;
wire [1:0] frame_write_finish_index;
reg  frame_write_toggle_mem;
reg  frame_write_owner_mem;
wire sd_write_req;
wire sd_write_req_ack;
wire sd_write_en;
wire [31:0] sd_write_data;
wire [1:0] sd_write_addr_index;
wire [1:0] video_read_addr_index;
// The SDRAM read side must keep one buffer for a complete HDMI refresh.
// Commit the requested video buffer only at the vertical-blank read request.
reg [1:0] raw_video_buffer_meta, raw_video_buffer_sync;
reg [1:0] raw_video_presented_index;
reg media_mode_meta,media_mode_sync,presented_video_mode;
reg [3:0] media_code_meta,media_code_sync,presented_code;
reg [2:0] raw_video_index_meta,raw_video_index_sync,presented_video_index;
reg video_epoch_meta,video_epoch_sync,presented_video_epoch;
reg [1:0] pause_video_sync;
reg video_read_req_d;

wire video_read_req;
wire video_read_req_ack;
wire video_read_en;
wire [31:0] video_read_data;
wire hs_raw;
wire vs_raw;
wire de_raw;
wire hs;
wire vs;
wire de;
wire [23:0] rgb_raw;
wire hs_effect;
wire vs_effect;
wire de_effect;
wire [23:0] rgb_base;
wire [23:0] rgb;

wire app_rd_en;
wire [ADDR_BITS-1:0] app_rd_addr;
wire sdr_rd_en;
wire [MEM_DATA_BITS-1:0] sdr_rd_data;
wire app_wr_en;
wire [ADDR_BITS-1:0] app_wr_addr;
wire [MEM_DATA_BITS-1:0] app_wr_data;
wire [3:0] app_wr_dm;

wire audio_i2s_bclk;
wire audio_i2s_lrck;
wire audio_i2s_dout;
wire audio_valid;
wire [23:0] audio_left_data;
wire [23:0] audio_right_data;
wire acr_valid;
wire [19:0] acr_cts;
wire [19:0] acr_n;

wire axis_user;
wire axis_valid;
wire axis_last;
wire [23:0] axis_data;
wire edid_read_trig;
wire [9:0] tmds_ch0;
wire [9:0] tmds_ch1;
wire [9:0] tmds_ch2;
wire [9:0] tmds_clk;
reg  [1:0] display_valid_video_sync;
reg selected_display_valid_meta;

wire sd_init_done_card;
wire bmp_sd_read, wav_sd_read, raw_video_sd_read;
wire [31:0] bmp_sd_addr, wav_sd_addr, raw_video_sd_addr;
wire [7:0] sd_shared_data;
wire sd_shared_valid, sd_shared_end;
reg [1:0] sd_owner;
reg last_sd_was_media;
reg [31:0] sd_owner_addr;
wire bmp_sd_valid = sd_shared_valid && (sd_owner==2'd1);
wire bmp_sd_end   = sd_shared_end   && (sd_owner==2'd1);
wire wav_sd_valid = sd_shared_valid && (sd_owner==2'd2);
wire wav_sd_end   = sd_shared_end   && (sd_owner==2'd2);
wire raw_video_sd_valid = sd_shared_valid && (sd_owner==2'd3);
wire raw_video_sd_end   = sd_shared_end   && (sd_owner==2'd3);

wire bmp_display_valid, bmp_playback_paused;
wire [3:0] bmp_state_code;
wire [3:0] bmp_error_status;
wire bmp_write_req, bmp_write_req_ack, bmp_write_en;
wire [31:0] bmp_write_data;
wire [1:0] bmp_write_addr_index, bmp_read_addr_index;
wire raw_video_scan_done, raw_video_found, raw_video_display_valid, raw_video_playback_paused;
wire raw_video_audio_sync_hold;
wire raw_video_stream_epoch;
wire bmp_transport_busy,raw_video_transport_busy;
wire picture_paused,video_paused,requested_video_mode,media_switch_pending;
wire picture_next,video_next,music_next;
// Legacy internal names retained for older simulation force points. They are
// aliases only and are not additional top-level IO.
wire media_paused;
wire music_paused = picture_paused;
wire [1:0] picture_music_index;
wire [2:0] key_function_state;
wire [7:0] brightness_level,volume_level;
wire brightness_ui,volume_ui;
wire system_info_ui,minimal_osd;
wire soft_reset;
wire bmp_enable,raw_video_enable;
wire [3:0] raw_video_status;
wire [2:0] raw_video_index;
wire raw_video_write_req, raw_video_write_req_ack, raw_video_write_en;
wire [31:0] raw_video_write_data;
wire [1:0] raw_video_write_addr_index, raw_video_read_addr_index;
wire raw_video_mode;
reg  active_audio_source_video;
reg  [1:0] active_audio_index;
reg  audio_restart_pending;
reg  audio_restart_pulse;
reg  video_audio_armed;
reg  [1:0] audio_presented_mode_sync;
reg  [2:0] audio_presented_index_meta,audio_presented_index_sync;
reg [1:0] audio_epoch_sync;
reg active_audio_epoch;
reg [31:0] audio_position,picture_audio_bookmark,video_audio_bookmark;
reg [31:0] audio_resume_sample;
reg [1:0] saved_picture_audio_index,saved_video_audio_index;
reg saved_video_audio_epoch;
wire audio_pause_ack;
reg [1:0] audio_pause_ack_sync;
wire [1:0] committed_video_audio_index =
    (audio_presented_index_sync>=3'd1 && audio_presented_index_sync<=3'd3) ?
    audio_presented_index_sync[1:0]-2'd1 : 2'd0;
wire active_audio_paused = active_audio_source_video ? video_paused : picture_paused;
wire desired_audio_source_video = audio_presented_mode_sync[1];
wire [1:0] desired_audio_index = audio_presented_mode_sync[1] ?
                                  committed_video_audio_index : picture_music_index;
wire video_audio_ready_to_start = audio_presented_mode_sync[1] &&
                                  (audio_presented_index_sync>=3'd1) &&
                                  (audio_presented_index_sync<=3'd3);
wire audio_source_valid = active_audio_source_video ?
                          (video_audio_armed && audio_presented_mode_sync[1]) :
                          !raw_video_mode;
wire audio_hold = active_audio_paused || !audio_source_valid || audio_restart_pending || media_switch_pending ||
                  (raw_video_mode!=audio_presented_mode_sync[1]) ||
                  (active_audio_source_video && raw_video_audio_sync_hold);
wire wav_request_allowed=wav_sd_read&&!audio_restart_pending&&!audio_restart_pulse;
wire selected_display_valid=raw_video_mode?raw_video_display_valid:bmp_display_valid;
wire selected_transport_busy=raw_video_mode?raw_video_transport_busy:bmp_transport_busy;
wire selected_read_request=raw_video_mode?raw_video_sd_read:bmp_sd_read;
wire selected_write_request=raw_video_mode?raw_video_write_req:bmp_write_req;
wire media_can_switch=!selected_transport_busy&&!selected_read_request&&
                      !selected_write_request&&!sd_write_req_ack&&
                      audio_pause_ack_sync[1]&&
                      (sd_owner!=2'd1)&&(sd_owner!=2'd3);
assign raw_video_enable=raw_video_mode&&!media_switch_pending;
assign bmp_enable=!raw_video_mode&&!media_switch_pending;
key_function_control #(.DEFAULT_VIDEO_MODE(0)) media_keys(
    .clk(clk),.rst(rst_core),.key1(key1),.key2(key2),.key3(key3),.key4(key4),
    .can_switch(media_can_switch),.video_mode(raw_video_mode),
    .requested_video_mode(requested_video_mode),.switch_pending(media_switch_pending),
    .picture_paused(picture_paused),.video_paused(video_paused),
    .picture_next(picture_next),.video_next(video_next),.music_next(music_next),
    .music_index(picture_music_index),.function_state(key_function_state),
    .brightness_level(brightness_level),.brightness_ui(brightness_ui),
    .volume_level(volume_level),.volume_ui(volume_ui),
    .system_info_ui(system_info_ui),.minimal_osd(minimal_osd),
    .soft_reset(soft_reset));

assign display_valid=selected_display_valid;
assign playback_paused=presented_video_mode?video_paused:picture_paused;
assign media_paused=playback_paused;
assign sd_write_req=raw_video_mode?raw_video_write_req:bmp_write_req;
assign sd_write_en=raw_video_mode?raw_video_write_en:bmp_write_en;
assign sd_write_data=raw_video_mode?raw_video_write_data:bmp_write_data;
assign sd_write_addr_index=raw_video_mode?{1'b1,raw_video_write_addr_index[0]}:bmp_write_addr_index;
assign video_read_addr_index=raw_video_presented_index;
assign raw_video_write_req_ack=sd_write_req_ack&&raw_video_mode;
assign bmp_write_req_ack=sd_write_req_ack&&!raw_video_mode;

wire wav_fifo_wr_en, wav_fifo_full, wav_fifo_empty, wav_fifo_rd_en;
wire [31:0] wav_fifo_wr_data, wav_fifo_rd_data;
wire [8:0] wav_fifo_wrusedw;
wire wav_audio_ready, wav_audio_playing;
wire [3:0] wav_status;
wire [31:0] wav_audio_total_samples;
wire subtitle_wr_en, subtitle_ready, subtitle_pixel, subtitle_visible;
wire [12:0] subtitle_wr_addr;
wire [7:0] subtitle_wr_data;
wire [5:0] subtitle_line_count;
wire [3:0] subtitle_status;
wire [9:0] osd_pixel_x;
wire [8:0] osd_pixel_y;
reg audio_sample_toggle;
reg [2:0] audio_sample_toggle_sync;
reg [2:0] audio_sample_toggle_sys_sync;
wire audio_sample_advance = audio_sample_toggle_sync[2] ^ audio_sample_toggle_sync[1];
wire audio_sample_advance_sys = audio_sample_toggle_sys_sync[2] ^
                                audio_sample_toggle_sys_sync[1];

// KEY4 selects the debounced function state. Only KEY1 in state 4 asserts
// soft_reset; PLL lock remains the independent power-up reset source.
// On memory PLL loss, restart the producers as well as the consumer: resetting
// only the writer could leave a half-transferred frame waiting forever.
assign rst_core = ~audio_pll_lock | ~mem_pll_lock;
assign rst_all = rst_core | soft_reset;
reset_release_sync reset_video_m0(.clk(video_clk),.request(rst_all),.reset_out(rst_video));
reset_release_sync reset_audio_m0(.clk(audio_mclk),.request(rst_all),.reset_out(rst_audio));
reset_release_sync reset_serial_m0(.clk(hdmi_5x_clk),.request(rst_all),.reset_out(rst_serial));
reset_release_sync reset_stream_m0(.clk(audio_mclk),.request(rst_all|audio_restart_pulse),.reset_out(rst_audio_stream));
reset_release_sync reset_subtitle_m0(.clk(video_clk),.request(rst_all|audio_restart_pulse),.reset_out(rst_subtitle));
// Audio lock does not guarantee that the independent SDRAM PLL is stable.
// Assert immediately on lock loss/reset; release only on stable memory clocks.
// KEY4/function/menu changes must never reset the frame store.
assign rst_mem_request = rst_all | ~mem_pll_lock;
always @(posedge mem_clk or posedge rst_mem_request) begin
    if(rst_mem_request) mem_reset_release <= 16'hffff;
    else mem_reset_release <= {mem_reset_release[14:0],1'b0};
end
assign rst_mem = mem_reset_release[15];
assign rgb_base = display_valid_video_sync[1] ? rgb_raw : 24'd0;

sys_pll sys_pll_m0(
    .refclk   (clk),
    .clk0_out (sys_clk_sd),
    .clk1_out (mem_clk),
    .clk2_out (mem_clk_shift),
    .locked   (mem_pll_lock),
    .reset    (1'b0)
);

video_pll video_pll_m0(
    .refclk   (clk),
    .clk0_out (video_clk),
    .clk1_out (hdmi_5x_clk),
    .reset    (1'b0)
);

PLL_HDMI_AUDIO audio_pll_m0(
    .refclk   (clk),
    .reset    (1'b0),
    .extlock  (audio_pll_lock),
    .clk0_out (),
    .clk1_out (),
    .clk2_out (audio_mclk)
);

always @(posedge mem_clk or posedge rst_mem) begin
    if (rst_mem) begin
        frame_write_toggle_mem <= 1'b0;
        frame_write_owner_mem <= 1'b0;
    end else if (frame_write_finish) begin
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
        frame_write_owner_mem <= frame_write_finish_index[1];
    end
end

always @(posedge video_clk or posedge rst_video) begin
    if(rst_video)begin
        raw_video_buffer_meta<=0;raw_video_buffer_sync<=0;
        raw_video_index_meta<=0;raw_video_index_sync<=0;presented_video_index<=0;
        video_epoch_meta<=0;video_epoch_sync<=0;presented_video_epoch<=0;
        raw_video_presented_index<=0;video_read_req_d<=0;
        media_mode_meta<=0;media_mode_sync<=0;presented_video_mode<=0;
        media_code_meta<=0;media_code_sync<=0;presented_code<=0;
        display_valid_video_sync<=0;selected_display_valid_meta<=0;pause_video_sync<=0;
    end else begin
        raw_video_buffer_meta<=raw_video_mode?{1'b1,raw_video_read_addr_index[0]}:bmp_read_addr_index;
        raw_video_buffer_sync<=raw_video_buffer_meta;
        media_mode_meta<=raw_video_mode;media_mode_sync<=media_mode_meta;
        // Keep the image status in this pipeline. Video numbering is latched
        // separately only in the actual HDMI commit branch below.
        media_code_meta<=bmp_state_code;
        media_code_sync<=media_code_meta;
        raw_video_index_meta<=raw_video_index;
        raw_video_index_sync<=raw_video_index_meta;
        video_epoch_meta<=raw_video_stream_epoch;video_epoch_sync<=video_epoch_meta;
        // Qualify validity with the same two-stage latency as mode/index.
        // Otherwise the first valid pulse can expose an old buffer index.
        selected_display_valid_meta<=selected_display_valid;
        display_valid_video_sync[0]<=selected_display_valid_meta;
        pause_video_sync<={pause_video_sync[0],playback_paused};
        video_read_req_d<=video_read_req;
        if(video_read_req&&!video_read_req_d)begin
            // Retain the last complete image while the other mode scans
            // and loads its first frame. Four dedicated SDRAM slots ensure
            // the loaders never overwrite the other mode's displayed data.
            if(display_valid_video_sync[0]&&
               (raw_video_buffer_sync[1]==media_mode_sync))begin
                raw_video_presented_index<=raw_video_buffer_sync;
                presented_video_mode<=media_mode_sync;
                presented_code<=media_mode_sync ? {1'b0,raw_video_index_sync} : media_code_sync;
                presented_video_index<=raw_video_index_sync;
                presented_video_epoch<=video_epoch_sync;
                display_valid_video_sync[1]<=1;
            end
        end
    end
end

// Commit the audio source only after the corresponding picture/video is
// actually presented by HDMI.  A source/index change first waits for any
// outstanding WAV sector to finish; the following one-cycle pulse restarts
// the parser, FIFO, I2S shifter and subtitle timer together.  Consequently
// MUSIC_* and VIDEO_* can never feed the audio path at the same time.
always @(posedge clk or posedge rst_all) begin
    if(rst_all) begin
        active_audio_source_video<=1'b0;
        active_audio_index<=2'd0;
        audio_restart_pending<=1'b0;
        audio_restart_pulse<=1'b0;
        video_audio_armed<=1'b0;
        audio_presented_mode_sync<=2'b00;
        audio_presented_index_meta<=3'd0;
        audio_presented_index_sync<=3'd0;
        audio_epoch_sync<=0;active_audio_epoch<=0;
        audio_position<=0;picture_audio_bookmark<=0;video_audio_bookmark<=0;
        audio_resume_sample<=0;saved_picture_audio_index<=0;saved_video_audio_index<=0;
        saved_video_audio_epoch<=0;audio_pause_ack_sync<=0;
    end else begin
        audio_pause_ack_sync<={audio_pause_ack_sync[0],audio_pause_ack};
        if(audio_sample_advance_sys&&!audio_restart_pulse)begin
            if(wav_audio_total_samples!=0&&audio_position+1'b1>=wav_audio_total_samples)
                audio_position<=0;
            else audio_position<=audio_position+1'b1;
        end
        audio_restart_pulse<=1'b0;
        audio_presented_mode_sync<={audio_presented_mode_sync[0],presented_video_mode};
        audio_presented_index_meta<=presented_video_index;
        audio_presented_index_sync<=audio_presented_index_meta;
        audio_epoch_sync<={audio_epoch_sync[0],presented_video_epoch};
        if(active_audio_source_video && !audio_presented_mode_sync[1])
            video_audio_armed<=1'b0;

        if(!audio_restart_pending &&
           ((active_audio_source_video!=desired_audio_source_video) ||
            (active_audio_index!=desired_audio_index) ||
            (desired_audio_source_video&&(active_audio_epoch!=audio_epoch_sync[1])))) begin
            audio_restart_pending<=1'b1;
        end

        // The acknowledgement is emitted at a complete stereo-pair boundary.
        // By its return, the consumption toggle has also crossed into clk.
        // Drain the SD owner before discarding the shared prefetch FIFO.
        if(audio_restart_pending && (sd_owner!=2'd2) && audio_pause_ack_sync[1]) begin
            if(active_audio_source_video)begin
                video_audio_bookmark<=audio_position;
                saved_video_audio_index<=active_audio_index;
                saved_video_audio_epoch<=active_audio_epoch;
            end else begin
                picture_audio_bookmark<=audio_position;
                saved_picture_audio_index<=active_audio_index;
            end
            // Resume only on a group change to the same remembered item.
            // An explicit next song/video or frame-zero epoch starts at zero.
            if(active_audio_source_video!=desired_audio_source_video)begin
                if(desired_audio_source_video && desired_audio_index==saved_video_audio_index &&
                   audio_epoch_sync[1]==saved_video_audio_epoch)begin
                    audio_resume_sample<=video_audio_bookmark;audio_position<=video_audio_bookmark;
                end else if(!desired_audio_source_video && desired_audio_index==saved_picture_audio_index)begin
                    audio_resume_sample<=picture_audio_bookmark;audio_position<=picture_audio_bookmark;
                end else begin audio_resume_sample<=0;audio_position<=0;end
            end else begin audio_resume_sample<=0;audio_position<=0;end
            active_audio_source_video<=desired_audio_source_video;
            active_audio_index<=desired_audio_index;
            active_audio_epoch<=audio_epoch_sync[1];
            video_audio_armed<=desired_audio_source_video && video_audio_ready_to_start;
            audio_restart_pending<=1'b0;
            audio_restart_pulse<=1'b1;
        end
    end
end

// One SD command remains owned by one client until the complete 512-byte
// response has returned. If both clients request together, alternate them so
// image loading cannot starve audio and audio cannot block image changes.
always @(posedge clk or posedge rst_all) begin
    if(rst_all) begin sd_owner<=0; last_sd_was_media<=0; sd_owner_addr<=0; end
    else begin
        case(sd_owner)
            2'd0: begin
                if((raw_video_sd_read||bmp_sd_read)&&wav_request_allowed) begin
                    if(last_sd_was_media) begin sd_owner<=2'd2; sd_owner_addr<=wav_sd_addr; end
                    else if(raw_video_sd_read) begin sd_owner<=2'd3; sd_owner_addr<=raw_video_sd_addr; end
                    else begin sd_owner<=2'd1; sd_owner_addr<=bmp_sd_addr; end
                end else if(wav_request_allowed) begin sd_owner<=2'd2; sd_owner_addr<=wav_sd_addr; end
                else if(raw_video_sd_read) begin sd_owner<=2'd3; sd_owner_addr<=raw_video_sd_addr; end
                else if(bmp_sd_read) begin sd_owner<=2'd1; sd_owner_addr<=bmp_sd_addr; end
            end
            2'd1: if(sd_shared_end) begin sd_owner<=0; last_sd_was_media<=1; end
            2'd2: if(sd_shared_end) begin sd_owner<=0; last_sd_was_media<=0; end
            2'd3: if(sd_shared_end) begin sd_owner<=0; last_sd_was_media<=1; end
            default:sd_owner<=0;
        endcase
    end
end

sd_card_single_bmp #(
    .CLK_FREQ_HZ       (50_000_000),
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd131071),
    .MAX_BMP_NUM       (MAX_BMP_NUM),
    .SLIDE_SECONDS     (SLIDE_SECONDS)
) sd_card_single_bmp_m0(
    .clk                 (clk),
    .rst                 (rst_all),
    .bmp_width           (16'd640),
    .bmp_height          (16'd480),
    .playback_enable     (bmp_enable),
    .pause_playback      (picture_paused),
    .next_item           (picture_next),
    .display_buffer_index(presented_video_mode?2'd2:{1'b0,raw_video_presented_index[0]}),
    .transport_busy      (bmp_transport_busy),
    .write_finish_toggle (frame_write_toggle_mem),
    .write_finish_owner  (frame_write_owner_mem),
    .display_valid       (bmp_display_valid),
    .playback_paused     (bmp_playback_paused),
    .state_code          (bmp_state_code),
    .error_status        (bmp_error_status),
    .write_addr_index    (bmp_write_addr_index),
    .read_addr_index     (bmp_read_addr_index),
    .write_req           (bmp_write_req),
    .write_req_ack       (bmp_write_req_ack),
    .write_en            (bmp_write_en),
    .write_data          (bmp_write_data),
    .sd_init_done        (sd_init_done_card),
    .sd_sec_read         (bmp_sd_read),
    .sd_sec_read_addr    (bmp_sd_addr),
    .sd_sec_read_data    (sd_shared_data),
    .sd_sec_read_data_valid(bmp_sd_valid),
    .sd_sec_read_end     (bmp_sd_end)
);

// Independent picture/video engines keep their playback position. The
// shared writer is handed over only after the current transport completes.
video_read_fat32 #(
    .CLK_FREQ_HZ(50_000_000),
    .MAX_VIDEO_NUM(MAX_VIDEO_NUM),
    .TIMEOUT_CYCLES(500_000_000)
) video_read_fat32_m0(
    .clk(clk),.rst(rst_all),.start(1'b1),.sd_init_done(sd_init_done_card),
    .playback_enable(raw_video_enable),.pause_playback(video_paused),
    .next_item(video_next),.audio_sample_advance(audio_sample_advance_sys),
    .audio_clock_enable(active_audio_source_video&&!audio_restart_pulse),
    .transport_busy(raw_video_transport_busy),.audio_sync_hold(raw_video_audio_sync_hold),
    .stream_epoch(raw_video_stream_epoch),
    .write_finish_toggle(frame_write_toggle_mem),
    .write_finish_owner(frame_write_owner_mem),
    .display_buffer_index(presented_video_mode?{1'b0,raw_video_presented_index[0]}:2'd2),
    .scan_done(raw_video_scan_done),.video_found(raw_video_found),
    .display_valid(raw_video_display_valid),.playback_paused(raw_video_playback_paused),
    .video_index(raw_video_index),
    .status(raw_video_status),.write_addr_index(raw_video_write_addr_index),
    .read_addr_index(raw_video_read_addr_index),.write_req(raw_video_write_req),
    .write_req_ack(raw_video_write_req_ack),.write_en(raw_video_write_en),
    .write_data(raw_video_write_data),.sd_sec_read(raw_video_sd_read),
    .sd_sec_read_addr(raw_video_sd_addr),.sd_sec_read_data(sd_shared_data),
    .sd_sec_read_data_valid(raw_video_sd_valid),.sd_sec_read_end(raw_video_sd_end)
);

wav_read_fat32 #(
    .MAX_MUSIC_NUM(3)
) wav_read_fat32_m0(
    .clk(clk), .rst(rst_all|audio_restart_pulse), .sd_init_done(sd_init_done_card),
    .audio_source_video(active_audio_source_video),
    .music_index(active_audio_index), .next_item(audio_restart_pulse), .pause_playback(audio_hold),
    .resume_sample(audio_resume_sample),
    .sd_sec_read(wav_sd_read), .sd_sec_read_addr(wav_sd_addr),
    .sd_sec_read_data(sd_shared_data), .sd_sec_read_data_valid(wav_sd_valid), .sd_sec_read_end(wav_sd_end),
    .fifo_wr_en(wav_fifo_wr_en), .fifo_wr_data(wav_fifo_wr_data), .fifo_wrusedw(wav_fifo_wrusedw),
    .audio_ready(wav_audio_ready), .audio_playing(wav_audio_playing), .status(wav_status),
    .audio_total_samples(wav_audio_total_samples),
    .subtitle_wr_en(subtitle_wr_en), .subtitle_wr_addr(subtitle_wr_addr),
    .subtitle_wr_data(subtitle_wr_data), .subtitle_ready(subtitle_ready),
    .subtitle_line_count(subtitle_line_count), .subtitle_status(subtitle_status)
);

sd_card_top #(.FAST_DIV_ZERO(SD_FAST_SPI)) sd_card_top_m0(
    // Preserve SD initialization during a song change; only the WAV parser,
    // Preserve card initialization; only the shared WAV/FIFO/I2S/subtitle
    // stream is restarted when the committed source or index changes.
    .clk(clk), .rst(rst_all), .SD_nCS(sd_ncs), .SD_DCLK(sd_dclk), .SD_MOSI(sd_mosi), .SD_MISO(sd_miso),
    .sd_init_done(sd_init_done_card),
    .sd_sec_read(sd_owner!=0), .sd_sec_read_addr(sd_owner_addr),
    .sd_sec_read_data(sd_shared_data), .sd_sec_read_data_valid(sd_shared_valid), .sd_sec_read_end(sd_shared_end),
    .sd_sec_write(1'b0), .sd_sec_write_addr(32'd0), .sd_sec_write_data(8'd0),
    .sd_sec_write_data_req(), .sd_sec_write_end()
);

// The physical board has two unused leading digits; this design exposes the
// six logical digits that follow them.  The old SD/audio/subtitle diagnostic
// codes are deliberately not routed to the tube anymore.  The six digits are
// always: '-' / function state / '-' / a / b / c.
function automatic [7:0] tube_digit(input [3:0] value);
    begin
        case(value)
            4'd0: tube_digit=8'b1100_0000;
            4'd1: tube_digit=8'b1111_1001;
            4'd2: tube_digit=8'b1010_0100;
            4'd3: tube_digit=8'b1011_0000;
            4'd4: tube_digit=8'b1001_1001;
            4'd5: tube_digit=8'b1001_0010;
            4'd6: tube_digit=8'b1000_0010;
            4'd7: tube_digit=8'b1111_1000;
            4'd8: tube_digit=8'b1000_0000;
            4'd9: tube_digit=8'b1001_0000;
            4'ha: tube_digit=8'b1000_1000; // image 10
            4'hb: tube_digit=8'b1000_0011;
            4'hc: tube_digit=8'b1100_0110;
            4'hd: tube_digit=8'b1010_0001;
            4'he: tube_digit=8'b1000_0110;
            4'hf: tube_digit=8'b1111_1111; // blank
            default: tube_digit=8'b1111_1111;
        endcase
    end
endfunction

function automatic [7:0] tube_dash(input dummy);
    begin tube_dash=8'b1011_1111; end // only the centre segment
endfunction

// Brightness and volume are 17 discrete levels.  Return packed BCD digits;
// 0xF is used for a suppressed leading digit (e.g. 94 -> F94, 6 -> FF6).
function automatic [11:0] level_bcd(input [4:0] level);
    begin
        case(level)
            5'd0:  level_bcd={4'hf,4'hf,4'd0};
            5'd1:  level_bcd={4'hf,4'hf,4'd6};
            5'd2:  level_bcd={4'hf,4'd1,4'd3};
            5'd3:  level_bcd={4'hf,4'd1,4'd9};
            5'd4:  level_bcd={4'hf,4'd2,4'd5};
            5'd5:  level_bcd={4'hf,4'd3,4'd1};
            5'd6:  level_bcd={4'hf,4'd3,4'd8};
            5'd7:  level_bcd={4'hf,4'd4,4'd4};
            5'd8:  level_bcd={4'hf,4'd5,4'd0};
            5'd9:  level_bcd={4'hf,4'd5,4'd6};
            5'd10: level_bcd={4'hf,4'd6,4'd3};
            5'd11: level_bcd={4'hf,4'd6,4'd9};
            5'd12: level_bcd={4'hf,4'd7,4'd5};
            5'd13: level_bcd={4'hf,4'd8,4'd1};
            5'd14: level_bcd={4'hf,4'd8,4'd8};
            5'd15: level_bcd={4'hf,4'd9,4'd4};
            default:level_bcd={4'd1,4'd0,4'd0};
        endcase
    end
endfunction

wire [11:0] brightness_digits=level_bcd(brightness_level[4:0]);
wire [11:0] volume_digits=level_bcd(volume_level[4:0]);
wire [3:0] image_digit=(bmp_display_valid && bmp_error_status==4'd0 &&
                         bmp_state_code>=4'd1 && bmp_state_code<=4'd9) ?
                       bmp_state_code :
                       ((bmp_display_valid && bmp_error_status==4'd0 &&
                         bmp_state_code==4'd10) ? 4'ha : 4'hf);
// Keep the picture-audio number visible even while the video soundtrack is
// active; active_audio_index is intentionally not used here because it then
// refers to VIDEO_0n rather than MUSIC_0n.
wire [3:0] music_digit=(picture_music_index<2'd3) ?
                       (picture_music_index+2'd1) : 4'hf;
wire [3:0] video_digit=(presented_video_index>=3'd1 && presented_video_index<=3'd3) ?
                       {1'b0,presented_video_index} : 4'hf;
wire [3:0] state_digit=(key_function_state>=3'd1 && key_function_state<=3'd4) ?
                       {1'b0,key_function_state} : 4'hf;
wire [7:0] tube_a=(key_function_state==3'd1) ? tube_digit(image_digit) :
                  (key_function_state==3'd2) ? tube_digit(brightness_digits[11:8]) :
                  (key_function_state==3'd3) ? tube_digit(volume_digits[11:8]) :
                  tube_digit(4'hf);
wire [7:0] tube_b=(key_function_state==3'd1) ? tube_digit(music_digit) :
                  (key_function_state==3'd2) ? tube_digit(brightness_digits[7:4]) :
                  (key_function_state==3'd3) ? tube_digit(volume_digits[7:4]) :
                  tube_digit(4'hf);
wire [7:0] tube_c=(key_function_state==3'd1) ? tube_digit(video_digit) :
                  (key_function_state==3'd2) ? tube_digit(brightness_digits[3:0]) :
                  (key_function_state==3'd3) ? tube_digit(volume_digits[3:0]) :
                  tube_digit(4'hf);

seg_scan seg_scan_m0(
    .clk        (clk),
    .rst_n      (~rst_all),
    .seg_sel    (seg_sel),
    .seg_data   (seg_data),
    .seg_data_0 (tube_dash(1'b0)),
    .seg_data_1 (tube_digit(state_digit)),
    .seg_data_2 (tube_dash(1'b0)),
    .seg_data_3 (tube_a),
    .seg_data_4 (tube_b),
    .seg_data_5 (tube_c)
);

video_timing_data video_timing_data_m0(
    .video_clk    (video_clk),
    .rst          (rst_video),
    .read_req     (video_read_req),
    .read_req_ack (video_read_req_ack),
    .hs           (hs_raw),
    .vs           (vs_raw),
    .de           (de_raw)
);

video_delay video_delay_m0(
    .video_clk (video_clk),
    .rst       (rst_video),
    .read_en   (video_read_en),
    .read_data (video_read_data[31:8]),
    .hs        (hs_raw),
    .vs        (vs_raw),
    .de        (de_raw),
    .hs_r      (hs),
    .vs_r      (vs),
    .de_r      (de),
    .vout_data (rgb_raw)
);

video_osd_effects video_osd_effects_m0(
    .clk                 (video_clk),
    .rst                 (rst_video),
    .in_hs               (hs),
    .in_vs               (vs),
    .in_de               (de),
    .in_rgb              (rgb_base),
    .display_valid       (display_valid_video_sync[1]),
    .playback_paused     (pause_video_sync[1]),
    .frame_buffer_select (video_read_addr_index[0]),
    .image_code          (presented_code),
    .video_mode          (presented_video_mode),
    .function_state      (key_function_state),
    .music_index         (picture_music_index),
    .brightness_level    (brightness_level),
    .brightness_ui       (brightness_ui),
    .volume_level        (volume_level),
    .volume_ui           (volume_ui),
    .system_info_ui      (system_info_ui),
    .minimal_osd         (minimal_osd),
    .sd_ready           (sd_init_done_card),
    .media_loading      (selected_transport_busy|media_switch_pending),
    .media_status       (raw_video_mode?raw_video_status:bmp_error_status),
    .audio_loading      (audio_restart_pending|audio_restart_pulse),
    .audio_status       (wav_status),
    .sub_status         (subtitle_status),
    .audio_source_video (active_audio_source_video),
    .active_audio_index (active_audio_index),
    .audio_held         (audio_hold),
    .audio_ready        (wav_audio_ready),
    .sub_ready          (subtitle_ready),
    .audio_valid         (audio_valid),
    .audio_sample        (audio_left_data),
    .subtitle_pixel      (subtitle_pixel),
    .subtitle_visible    (subtitle_visible),
    .current_pixel_x     (osd_pixel_x),
    .current_pixel_y     (osd_pixel_y),
    .out_hs              (hs_effect),
    .out_vs              (vs_effect),
    .out_de              (de_effect),
    .out_rgb             (rgb)
);

subtitle_sync subtitle_sync_m0(
    .wr_clk                 (clk),
    .wr_en                  (subtitle_wr_en),
    .wr_addr                (subtitle_wr_addr),
    .wr_data                (subtitle_wr_data),
    .clk                    (video_clk),
    .rst                    (rst_subtitle),
    .subtitle_ready         (subtitle_ready),
    .subtitle_line_count    (subtitle_line_count),
    .audio_sample_advance   (audio_sample_advance),
    .audio_total_samples    (wav_audio_total_samples),
    .resume_sample          (audio_resume_sample),
    .video_vs               (vs),
    .pixel_x                (osd_pixel_x),
    .pixel_y                (osd_pixel_y),
    .subtitle_pixel         (subtitle_pixel),
    .subtitle_visible       (subtitle_visible)
);

frame_read_write #(
    .ADDR_BITS    (ADDR_BITS),
    .WRITE_V_FLIP (1),
    .FRAME_WIDTH  (640),
    .FRAME_HEIGHT (480)
) frame_read_write_m0(
    .mem_clk          (mem_clk),
    .rst              (rst_mem),
    .Sdr_init_done    (sdr_init_done),
    .Sdr_init_ref_vld (sdr_init_ref_vld),
    .Sdr_busy         (sdr_busy),
    .App_rd_en        (app_rd_en),
    .App_rd_addr      (app_rd_addr),
    .Sdr_rd_en        (sdr_rd_en),
    .Sdr_rd_dout      (sdr_rd_data),
    .read_clk         (video_clk),
    .read_req         (video_read_req),
    .read_req_ack     (video_read_req_ack),
    .read_finish      (),
    .read_addr_0      (FRAME_BASE),
    .read_addr_1      (FRAME_BASE_1),
    .read_addr_2      (FRAME_BASE_2),
    .read_addr_3      (FRAME_BASE_3),
    .read_addr_index  (video_read_addr_index),
    .read_len         (FRAME_PIXELS),
    .read_en          (video_read_en),
    .read_data        (video_read_data),
    .App_wr_en        (app_wr_en),
    .App_wr_addr      (app_wr_addr),
    .App_wr_din       (app_wr_data),
    .App_wr_dm        (app_wr_dm),
    .write_clk        (clk),
    .write_req        (sd_write_req),
    .write_req_ack    (sd_write_req_ack),
    .write_finish     (frame_write_finish),
    .write_finish_index(frame_write_finish_index),
    .write_addr_0     (FRAME_BASE),
    .write_addr_1     (FRAME_BASE_1),
    .write_addr_2     (FRAME_BASE_2),
    .write_addr_3     (FRAME_BASE_3),
    .write_addr_index (sd_write_addr_index),
    .write_len        (FRAME_PIXELS),
    .write_en         (sd_write_en),
    .write_data       (sd_write_data)
);

sdram sdram_m0(
    .Clk              (mem_clk),
    .Clk_sft          (mem_clk_shift),
    .Rst              (rst_mem),
    .Sdr_init_done    (sdr_init_done),
    .Sdr_init_ref_vld (sdr_init_ref_vld),
    .Sdr_busy         (sdr_busy),
    .App_wr_en        (app_wr_en),
    .App_wr_addr      (app_wr_addr),
    .App_wr_dm        (app_wr_dm),
    .App_wr_din       (app_wr_data),
    .App_rd_en        (app_rd_en),
    .App_rd_addr      (app_rd_addr),
    .Sdr_rd_en        (sdr_rd_en),
    .Sdr_rd_dout      (sdr_rd_data)
);

wfifo_32_32_512 wav_audio_fifo_m0(
    .rst(rst_audio_stream), .clkw(clk), .clkr(audio_mclk),
    .we(wav_fifo_wr_en), .di(wav_fifo_wr_data), .re(wav_fifo_rd_en), .dout(wav_fifo_rd_data),
    .valid(), .full_flag(wav_fifo_full), .empty_flag(wav_fifo_empty),
    .afull(), .aempty(), .wrusedw(wav_fifo_wrusedw), .rdusedw()
);

hdmi_audio_pcm_i2s_64fs audio_pcm_m0(
    .I_mclk     (audio_mclk),
    .I_rst      (rst_audio_stream),
    .I_pause    (audio_hold),
    .O_pause_ack(audio_pause_ack),
    .I_volume   (volume_level),
    .I_fifo_data(wav_fifo_rd_data),
    .I_fifo_empty(wav_fifo_empty),
    .O_fifo_rd_en(wav_fifo_rd_en),
    .O_i2s_BCLK (audio_i2s_bclk),
    .O_i2s_LRCK (audio_i2s_lrck),
    .O_i2s_DOUT (audio_i2s_dout)
);

// Cross one event per actual PCM FIFO word into the video clock domain.
// Unlike the continuous HDMI audio-valid cadence, this pauses on underrun.
always @(posedge audio_mclk or posedge rst_audio) begin
    if(rst_audio)
        audio_sample_toggle<=1'b0;
    else if(wav_fifo_rd_en)
        audio_sample_toggle<=~audio_sample_toggle;
end

always @(posedge video_clk or posedge rst_video) begin
    if(rst_video)
        audio_sample_toggle_sync<=3'b000;
    else
        audio_sample_toggle_sync<={audio_sample_toggle_sync[1:0],audio_sample_toggle};
end

// The video file scheduler runs on the 50 MHz system clock, independently
// from the subtitle/OSD video clock.  Synchronize the same sample toggle into
// that domain rather than crossing a one-clock pulse directly.
always @(posedge clk or posedge rst_all) begin
    if(rst_all)
        audio_sample_toggle_sys_sync<=3'b000;
    else
        audio_sample_toggle_sys_sync<={audio_sample_toggle_sys_sync[1:0],audio_sample_toggle};
end

I2S_receiver audio_receiver_m0(
    .I_clk              (video_clk),
    .I_rst              (rst_video),
    .I_i2s_BCLK         (audio_i2s_bclk),
    .I_i2s_LRCK         (audio_i2s_lrck),
    .I_i2s_DOUT         (audio_i2s_dout),
    .O_audio_valid      (audio_valid),
    .O_audio_left_data  (audio_left_data),
    .O_audio_right_data (audio_right_data)
);

audio_arc_calculate #(
    .ACR_N (6144)
) audio_arc_m0(
    .I_clk         (video_clk),
    .I_rst         (rst_video),
    .I_audio_valid (audio_valid),
    .O_acr_valid   (acr_valid),
    .O_acr_cts     (acr_cts),
    .O_acr_n       (acr_n)
);

video_rgb_to_axis_640x480 video_axis_m0(
    .I_clk         (video_clk),
    .I_rst         (rst_video),
    .I_vs          (vs_effect),
    .I_de          (de_effect),
    .I_rgb         (rgb),
    .O_video_user  (axis_user),
    .O_video_valid (axis_valid),
    .O_video_last  (axis_last),
    .O_video_data  (axis_data)
);

startup_pulse #(
    .CNT_MAX (20'd100000)
) edid_start_m0(
    .I_clk   (video_clk),
    .I_rst   (rst_video),
    .O_pulse (edid_read_trig)
);

hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE            ("EG"),
    .HTOTAL            (800),
    .HSA               (96),
    .HFP               (16),
    .HBP               (48),
    .HACTIVE           (640),
    .VTOTAL            (525),
    .VSA               (2),
    .VFP               (10),
    .VBP               (33),
    .VACTIVE           (480),
    .VIDEO_VIC         (1),
    .VIDEO_TPG         ("Disable"),
    .VIDEO_FORMAT      ("RGB"),
    .AUDIO_SAMPLE_RATE ("48K"),
    .IIC_SCL_DIV       (250)
) hdmi_core_m0(
    .I_pixel_clk        (video_clk),
    .I_rst              (rst_video),
    .I_edid_read_trig   (edid_read_trig),
    .O_edid_read_valid  (),
    .O_edid_read_data   (),
    .I_axis_s_user      (axis_user),
    .I_axis_s_valid     (axis_valid),
    .I_axis_s_last      (axis_last),
    .I_axis_s_data      (axis_data),
    .O_axis_s_ready     (),
    .I_audio_valid      (audio_valid),
    .I_audio_left_data  (audio_left_data),
    .I_audio_right_data (audio_right_data),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),
    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),
    .O_ch0_tmds_data    (tmds_ch0),
    .O_ch1_tmds_data    (tmds_ch1),
    .O_ch2_tmds_data    (tmds_ch2),
    .O_clk_tmds_data    (tmds_clk)
);

hdmi_phy_wrapper #(
    .DEVICE ("EG")
) hdmi_phy_m0(
    .I_pixel_clk       (video_clk),
    .I_serial_clk      (hdmi_5x_clk),
    .I_rst             (rst_serial),
    .I_tmds_channel_0  (tmds_ch0),
    .I_tmds_channel_1  (tmds_ch1),
    .I_tmds_channel_2  (tmds_ch2),
    .I_tmds_channel_clk(tmds_clk),
    .O_tmds_ch0_p      (HDMI_D0_P),
    .O_tmds_ch1_p      (HDMI_D1_P),
    .O_tmds_ch2_p      (HDMI_D2_P),
    .O_tmds_clk_p      (HDMI_CLK_P)
);

endmodule

// Reset assertion is asynchronous; deassertion only follows local clock
// edges. Only this first reset synchronizer is exempt from recovery/removal.
module reset_release_sync(input clk,input request,output reset_out);
(* async_reg="true", syn_preserve=1 *) reg [2:0] release_pipe;
always @(posedge clk or posedge request)
 if(request)release_pipe<=3'b111;
 else release_pipe<={release_pipe[1:0],1'b0};
assign reset_out=release_pipe[2];
endmodule

// Central active-low controls. Physical keys are shifted one position:
// KEY1 carries the former KEY4 function, KEY2 the former KEY1 function,
// KEY3 the former KEY2 function, and KEY4 the former KEY3 function.
module key_function_control #(
    parameter integer CLK_FREQ_HZ=50_000_000,
    parameter integer DEFAULT_VIDEO_MODE=0
)(
    input clk,input rst,input key1,input key2,input key3,input key4,input can_switch,
    output reg video_mode,output reg requested_video_mode,
    output switch_pending,output reg picture_paused,output reg video_paused,
    output reg picture_next,output reg video_next,output reg music_next,
    output reg [1:0] music_index,output reg [2:0] function_state,
    output reg [7:0] brightness_level,output reg brightness_ui,
    output reg [7:0] volume_level,output reg volume_ui,
    output reg system_info_ui,output reg minimal_osd,
    output reg soft_reset
);
localparam integer DEBOUNCE_CYCLES=(CLK_FREQ_HZ/50>0)?CLK_FREQ_HZ/50:1;
localparam integer LONG_CYCLES=(CLK_FREQ_HZ/5*4>0)?CLK_FREQ_HZ/5*4:1;
localparam integer REPEAT_DELAY=(CLK_FREQ_HZ/5*3>0)?CLK_FREQ_HZ/5*3:1;
localparam integer REPEAT_CYCLES=(CLK_FREQ_HZ/20*3>0)?CLK_FREQ_HZ/20*3:1;
// Size gesture counters to their existing thresholds, not unconditional 32
// bits. Counts/timing are unchanged; this avoids wasting scarce slice ALUs.
localparam integer DEBOUNCE_WIDTH=(DEBOUNCE_CYCLES>1)?$clog2(DEBOUNCE_CYCLES+1):1;
localparam integer HOLD_WIDTH=(LONG_CYCLES>1)?$clog2(LONG_CYCLES+1):1;
localparam integer REPEAT_WIDTH=(REPEAT_CYCLES>1)?$clog2(REPEAT_CYCLES+1):1;
reg [3:0] key_meta,key_sync,key_state,long_seen;
reg [DEBOUNCE_WIDTH-1:0] debounce_count[0:3];
reg [HOLD_WIDTH-1:0] hold_count[0:3];
reg [REPEAT_WIDTH-1:0] repeat_count[0:3];
reg [2:0] press_state[0:3];
reg [3:0] press_mode,press_ui,press,short_event,long_event,repeat_event;
integer k;
assign switch_pending=(video_mode!=requested_video_mode);
wire state_change=short_event[3]||long_event[3];
wire bright_step=(function_state==2)&&brightness_ui;
wire volume_step=(function_state==3)&&volume_ui;
// Each gesture belongs to the state/mode in which the key was pressed.
// State/menu changes cancel held controls rather than reinterpreting them.
always @(posedge clk or posedge rst) begin
 if(rst) begin
  key_meta<=15;key_sync<=15;key_state<=15;long_seen<=0;
  press_mode<=0;press_ui<=0;press<=0;short_event<=0;long_event<=0;repeat_event<=0;
  for(k=0;k<4;k=k+1)begin
   debounce_count[k]<=0;hold_count[k]<=0;repeat_count[k]<=0;press_state[k]<=0;
  end
  video_mode<=DEFAULT_VIDEO_MODE;requested_video_mode<=DEFAULT_VIDEO_MODE;
  picture_paused<=0;video_paused<=0;picture_next<=0;video_next<=0;music_next<=0;
  music_index<=0;function_state<=1;brightness_level<=16;brightness_ui<=0;
  volume_level<=16;volume_ui<=0;soft_reset<=0;system_info_ui<=0;minimal_osd<=0;
 end else begin
  // key_state[0..3] retain the existing logical KEY1..KEY4 decoder. Feed it
  // from physical KEY2,KEY3,KEY4,KEY1 respectively.
  key_meta<={key1,key4,key3,key2};key_sync<=key_meta;
  press<=0;short_event<=0;long_event<=0;repeat_event<=0;
  picture_next<=0;video_next<=0;music_next<=0;soft_reset<=0;
  for(k=0;k<4;k=k+1)begin
   if(key_sync[k]==key_state[k])debounce_count[k]<=0;
   else if(debounce_count[k]>=DEBOUNCE_CYCLES-1)begin
    debounce_count[k]<=0;key_state[k]<=key_sync[k];
    if(!key_sync[k])begin
     press[k]<=1;hold_count[k]<=0;repeat_count[k]<=0;long_seen[k]<=0;
     press_state[k]<=function_state;press_mode[k]<=video_mode;
     press_ui[k]<=((function_state==2)&&brightness_ui)||((function_state==3)&&volume_ui);
    end else if(!long_seen[k]&&press_state[k]==function_state&&
                (k==3||press_mode[k]==video_mode)) short_event[k]<=1;
   end else debounce_count[k]<=debounce_count[k]+1'b1;
   if(!key_state[k]&&!key_sync[k])begin
    if(hold_count[k]<LONG_CYCLES)hold_count[k]<=hold_count[k]+1'b1;
    if(!long_seen[k]&&hold_count[k]>=LONG_CYCLES-1)begin
     long_seen[k]<=1;
     if(press_state[k]==function_state&&(k==3||press_mode[k]==video_mode))
      long_event[k]<=1;
    end
    if(k<2&&press_state[k]==function_state&&press_ui[k]&&
       (bright_step||volume_step))begin
     if(hold_count[k]>=REPEAT_DELAY-1)begin
      if(repeat_count[k]==0)begin repeat_event[k]<=1;repeat_count[k]<=REPEAT_CYCLES-1;end
      else repeat_count[k]<=repeat_count[k]-1'b1;
     end
    end
   end
  end
  if(switch_pending&&can_switch)video_mode<=requested_video_mode;
  if(state_change)begin
   if(long_event[3])function_state<=(function_state==1)?3'd4:function_state-1'b1;
   else function_state<=(function_state==4)?3'd1:function_state+1'b1;
   brightness_ui<=0;volume_ui<=0;
   system_info_ui<=0;
   // Prevent a held menu/media key from leaking into the next state.
   for(k=0;k<3;k=k+1)begin press_state[k]<=0;press_ui[k]<=0;end
  end else begin
   case(function_state)
    1:begin
     if(short_event[1]&&!switch_pending)requested_video_mode<=~video_mode;
     if(short_event[0]&&!switch_pending)begin
      if(video_mode)video_paused<=~video_paused;else picture_paused<=~picture_paused;
     end
     if(short_event[2]&&!switch_pending)begin
      if(video_mode)video_next<=1;else picture_next<=1;
     end
     if(long_event[2]&&!video_mode&&!switch_pending)begin
      music_next<=1;music_index<=(music_index==2)?2'd0:music_index+1'b1;
     end
    end
    2:begin
     if(short_event[2])begin
      brightness_ui<=~brightness_ui;
      for(k=0;k<2;k=k+1)press_ui[k]<=0;
     end else if(brightness_ui)begin
      if(press_state[0]==2&&(press[0]||(repeat_event[0]&&press_ui[0])))begin
       if(brightness_level<16)brightness_level<=brightness_level+1'b1;
      end else if(press_state[1]==2&&(press[1]||(repeat_event[1]&&press_ui[1])))begin
       if(brightness_level!=0)brightness_level<=brightness_level-1'b1;
      end
     end
    end
    3:begin
     if(short_event[2])begin
      volume_ui<=~volume_ui;
      for(k=0;k<2;k=k+1)press_ui[k]<=0;
     end else if(volume_ui)begin
      if(press_state[0]==3&&(press[0]||(repeat_event[0]&&press_ui[0])))begin
       if(volume_level<16)volume_level<=volume_level+1'b1;
      end else if(press_state[1]==3&&(press[1]||(repeat_event[1]&&press_ui[1])))begin
       if(volume_level!=0)volume_level<=volume_level-1'b1;
      end
     end
    end
    4:begin
     if(short_event[0])soft_reset<=1;
     else begin
      if(short_event[1])system_info_ui<=~system_info_ui;
      if(short_event[2])minimal_osd<=~minimal_osd;
     end
    end
    default:function_state<=1;
   endcase
  end
  if(soft_reset)begin
   video_mode<=DEFAULT_VIDEO_MODE;requested_video_mode<=DEFAULT_VIDEO_MODE;
   picture_paused<=0;video_paused<=0;music_index<=0;function_state<=1;
   brightness_level<=16;brightness_ui<=0;volume_level<=16;volume_ui<=0;
   system_info_ui<=0;minimal_osd<=0;
   press<=0;short_event<=0;long_event<=0;repeat_event<=0;
   for(k=0;k<4;k=k+1)begin press_state[k]<=0;press_ui[k]<=0;long_seen[k]<=1;end
  end
 end
end
endmodule
