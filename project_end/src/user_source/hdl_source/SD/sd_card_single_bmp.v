module sd_card_single_bmp #(
    parameter integer CLK_FREQ_HZ=100_000_000,
    parameter [31:0] SCAN_START_SECTOR=32'd0,
    parameter [31:0] SCAN_MAX_SECTOR=32'd131071,
    parameter integer MAX_BMP_NUM=3,
    parameter integer SLIDE_SECONDS=3
)(
    input clk,input rst,input [15:0] bmp_width,input [15:0] bmp_height,
    input playback_enable,input pause_playback,input next_item,input [1:0] display_buffer_index,
    input write_finish_toggle,input write_finish_owner,output reg display_valid,output playback_paused,
    output transport_busy,output [3:0] state_code,output [3:0] error_status,
    output [1:0] write_addr_index,output [1:0] read_addr_index,
    output write_req,input write_req_ack,output write_en,output [31:0] write_data,
    input sd_init_done,output sd_sec_read,output [31:0] sd_sec_read_addr,
    input [7:0] sd_sec_read_data,input sd_sec_read_data_valid,input sd_sec_read_end
);
localparam [3:0] MAX_BMP_COUNT_CODE=MAX_BMP_NUM;
wire scan_done,load_done,scan_start,load_start;
wire [3:0] fat_status,image_count;
wire bmp_wr;wire [23:0] bmp_rgb;
wire [MAX_BMP_NUM*32-1:0] image_sector,image_cluster,image_size,image_offset;
reg scan_started,load_pending,source_done,frame_done_seen;
reg [3:0] current_image,loading_image;
reg [31:0] slide_timer,init_wait;
reg active_buffer,loading_buffer,wait_display;
reg next_pending;
reg preload_valid,preload_buffer;
reg [3:0] preload_image;
reg [2:0] finish_sync,write_ack_sync;
reg [2:0] owner_sync;
reg finish_seen;
reg [1:0] display_index_meta,display_index_sync;
wire [3:0] next_image=(current_image+1'b1<image_count)?current_image+1'b1:4'd0;
wire [3:0] requested_image=display_valid?next_image:4'd0;
wire advance_due=next_pending||next_item||
                 (!pause_playback&&slide_timer>=CLK_FREQ_HZ*SLIDE_SECONDS);
assign scan_start=sd_init_done&&playback_enable&&!scan_started;
// Fill only the spare buffer as soon as the display has acknowledged the
// current one. A completed preload remains private until KEY3 or the timer.
// Preloading while paused does not advance the presented image.
assign load_start=playback_enable&&scan_done&&(image_count!=0)&&(fat_status==0)&&
                  !load_pending&&!wait_display&&!preload_valid&&
                  (!display_valid||image_count>1||advance_due);
wire present_loading=playback_enable&&load_pending&&source_done&&frame_done_seen&&
                     (!display_valid||advance_due);
wire present_preloaded=playback_enable&&display_valid&&preload_valid&&!wait_display&&advance_due;
assign transport_busy=(fat_status==0)&&((scan_started&&!scan_done)||
                      (load_pending&&!(source_done&&frame_done_seen)));
assign write_en=bmp_wr;assign write_data={bmp_rgb,8'd0};
assign write_addr_index={1'b0,loading_buffer};assign read_addr_index={1'b0,active_buffer};
assign playback_paused=pause_playback;
wire [3:0] progress_status=!sd_init_done?4'd0:!scan_done?4'd1:
                          (load_pending&&!source_done&&!frame_done_seen)?4'd2:
                          (load_pending&&source_done&&!frame_done_seen)?4'd3:4'd6;
// Dedicated read-only error code: state_code A can also mean IMAGE 10.
assign error_status=(fat_status!=0)?fat_status:
                   ((!sd_init_done&&init_wait>=CLK_FREQ_HZ*2)?4'hd:4'd0);
assign state_code=(fat_status!=0)?fat_status:
                  ((!sd_init_done&&init_wait>=CLK_FREQ_HZ*2)?4'hd:
                  (display_valid?current_image+1'b1:progress_status));
always @(posedge clk or posedge rst)begin
 if(rst)begin
  scan_started<=0;current_image<=0;loading_image<=0;active_buffer<=0;loading_buffer<=0;
  preload_valid<=0;preload_buffer<=0;preload_image<=0;
  display_valid<=0;slide_timer<=0;init_wait<=0;load_pending<=0;source_done<=0;
  frame_done_seen<=0;finish_sync<=0;owner_sync<=0;finish_seen<=0;write_ack_sync<=0;wait_display<=0;
  display_index_meta<=0;display_index_sync<=0;
  next_pending<=0;
 end else begin
  if(next_item)next_pending<=1;
  finish_sync<={finish_sync[1:0],write_finish_toggle};
  owner_sync<={owner_sync[1:0],write_finish_owner};
  finish_seen<=finish_sync[2];
  write_ack_sync<={write_ack_sync[1:0],write_req_ack};
  display_index_meta<=display_buffer_index;display_index_sync<=display_index_meta;
  if(!sd_init_done&&init_wait<CLK_FREQ_HZ*2)init_wait<=init_wait+1'b1;
  if(sd_init_done)init_wait<=0;
  if(scan_start)scan_started<=1;
  if(load_start)begin
   load_pending<=1;source_done<=0;frame_done_seen<=0;
   loading_image<=requested_image;loading_buffer<=display_valid?~active_buffer:1'b0;
  end
  if(load_done)source_done<=1;
  // Owner and event are checked at the same synchronized stage. Completion
  // may precede load_done when the last sector contains unused padding.
  if((finish_sync[2]^finish_seen) && load_pending && !load_start && !owner_sync[2])frame_done_seen<=1;
  // A completed background frame is presented immediately only when the
  // user/timer has requested it; otherwise retain it as a ready spare.
  if(playback_enable&&load_pending&&source_done&&frame_done_seen)begin
   if(!display_valid||advance_due)begin
    display_valid<=1;current_image<=loading_image;active_buffer<=loading_buffer;
    slide_timer<=0;load_pending<=0;source_done<=0;frame_done_seen<=0;wait_display<=1;
    preload_valid<=0;next_pending<=0;
   end else begin
    preload_valid<=1;preload_image<=loading_image;preload_buffer<=loading_buffer;
    load_pending<=0;source_done<=0;frame_done_seen<=0;
   end
  end
  // KEY3/timer can commit a frame that was already loaded in the spare bank.
  if(present_preloaded)begin
   current_image<=preload_image;active_buffer<=preload_buffer;
   slide_timer<=0;wait_display<=1;preload_valid<=0;next_pending<=0;
  end
  if(wait_display&&display_index_sync=={1'b0,active_buffer})wait_display<=0;
  // Count the visible slide interval during background I/O as well.
  if(playback_enable&&display_valid&&!pause_playback&&!wait_display&&
     !present_loading&&!present_preloaded&&
     slide_timer<CLK_FREQ_HZ*SLIDE_SECONDS)slide_timer<=slide_timer+1'b1;
  if(!sd_init_done)begin
   scan_started<=0;display_valid<=0;current_image<=0;loading_image<=0;
   active_buffer<=0;loading_buffer<=0;slide_timer<=0;load_pending<=0;
   source_done<=0;frame_done_seen<=0;wait_display<=0;
   preload_valid<=0;preload_buffer<=0;preload_image<=0;
   next_pending<=0;
  end
 end
end
bmp_read_fat32 #(.MAX_BMP_NUM(MAX_BMP_NUM),.TIMEOUT_CYCLES(CLK_FREQ_HZ*10)) reader(
 .clk(clk),.rst(rst),.start_scan(scan_start),.boot_sector(SCAN_START_SECTOR),.target_count(MAX_BMP_COUNT_CODE),
 .scan_done(scan_done),.image_count(image_count),.image_sector(image_sector),.image_cluster(image_cluster),.image_size(image_size),.image_offset(image_offset),
 .load_start(load_start),.load_index(requested_image),.load_done(load_done),.status(fat_status),.write_req(write_req),.write_req_ack(write_ack_sync[2]),
 .sd_sec_read(sd_sec_read),.sd_sec_read_addr(sd_sec_read_addr),.sd_sec_read_data(sd_sec_read_data),.sd_sec_read_data_valid(sd_sec_read_data_valid),.sd_sec_read_end(sd_sec_read_end),
 .bmp_data_wr_en(bmp_wr),.bmp_data(bmp_rgb),.bmp_width(bmp_width),.bmp_height(bmp_height));
endmodule
