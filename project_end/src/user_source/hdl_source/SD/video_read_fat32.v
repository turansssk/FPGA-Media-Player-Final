// FAT32 reader and raw RGB565 video streamer.
// Files: VIDEO_01.VDO .. VIDEO_03.VDO, selected in numeric order.
// Header sector: "VDO1", u16 width, u16 height, u16 fps, u16 reserved,
// u32 frame count, then padding to 512 bytes. Frames are 640x480 RGB565 LE.
// Rows are stored bottom-to-top to match the existing BMP vertical-flip writer.
module video_read_fat32 #(
    parameter integer CLK_FREQ_HZ=50_000_000,
    parameter integer AUDIO_SAMPLE_RATE=48_000,
    parameter integer TIMEOUT_CYCLES=500_000_000,
    parameter integer MAX_VIDEO_NUM=3 // supported range 1..3
)(
    input clk,input rst,input start,input sd_init_done,
    input playback_enable,input pause_playback,input next_item,input audio_sample_advance,
    input audio_clock_enable,
    input write_finish_toggle,input write_finish_owner,
    output transport_busy,output audio_sync_hold,output reg stream_epoch,
    input [1:0] display_buffer_index,
    output reg scan_done,output reg video_found,output reg display_valid,
    output playback_paused,output [2:0] video_index,output reg [3:0] status,
    output [1:0] write_addr_index,output [1:0] read_addr_index,
    output reg write_req,input write_req_ack,output reg write_en,output reg [31:0] write_data,
    output reg sd_sec_read,output reg [31:0] sd_sec_read_addr,
    input [7:0] sd_sec_read_data,input sd_sec_read_data_valid,input sd_sec_read_end
);
localparam [4:0] ST_IDLE=0,ST_MBR=1,ST_BPB=2,ST_DIR=3,ST_DIR_END=4,
                 ST_HDR=5,ST_HDR_CHECK=6,ST_SIZE_CHECK=7,ST_WAIT_REQ=8,
                 ST_FRAME=9,ST_FAT=10,ST_FAT_CHECK=11,ST_WAIT_SDRAM=12,ST_READY=13,
                 ST_WAIT_RELEASE=14,ST_ERROR=15,ST_WAIT_DISPLAY=16,ST_SELECT=17,
                 ST_CATCHUP=18,ST_SKIP=19;
localparam [31:0] FRAME_BYTES=32'd614400,FRAME_SECTORS=32'd1200,FRAME_PIXELS=32'd307200;
reg [4:0] state;
reg [31:0] timeout;
reg [9:0] count;
reg [15:0] bytes_per_sector,reserved;
reg [7:0] sectors_per_cluster,fat_count;
reg [31:0] volume_base,fat_base,fat_size,root_cluster,data_base,root_sector;
reg [7:0] mbr_type;
reg [31:0] partition_lba;
reg [7:0] name[0:10];
reg [7:0] attr;
reg [15:0] cluster_hi,cluster_lo;
reg [31:0] dir_file_size,file_size,file_first_cluster,file_first_sector;
reg [7:0] root_sector_index;
reg dir_match;
reg [31:0] magic;
reg [15:0] video_width,video_height,video_fps;
reg [31:0] frame_count,size_value,size_frames;
reg [31:0] current_frame;
reg [10:0] sector_in_frame;
reg [18:0] pixel_count;
reg [31:0] current_cluster,next_file_sector;
reg [7:0] cluster_sector_pos;
reg need_fat_check,byte_phase;
reg [7:0] pixel_lo;
reg active_buffer,loading_buffer,source_done,frame_done;
reg [2:0] finish_sync;
reg [2:0] owner_sync;
reg finish_seen;
reg [2:0] ack_sync;
reg [1:0] display_index_meta,display_index_sync;
reg start_latched;
reg [9:0] fat_byte_count;
reg [8:0] fat_offset;
reg [7:0] fat_b0,fat_b1,fat_b2,fat_b3;
wire [31:0] fat_next={fat_b3,fat_b2,fat_b1,fat_b0}&32'h0fffffff;
reg [15:0] frame_phase;
// Audio is the master clock. Keep fractional phase AND all elapsed frame
// deadlines, including time spent reading SD and waiting for HDMI vblank.
// 255 deadlines cover over a minute at the normal 4fps. Saturate during
// a terminal I/O error rather than allowing the pending count to wrap.
reg [7:0] frames_due;
reg [10:0] skip_sectors_left;
reg skip_fat,epoch_commit;
reg next_pending,first_file_frame,dir_end_seen,dir_fat;
reg enable_d,restart_pending;
wire sample_tick=display_valid&&audio_clock_enable&&audio_sample_advance;
wire frame_tick=sample_tick&&(frame_phase+video_fps>=AUDIO_SAMPLE_RATE);
wire timed_commit=(state==ST_WAIT_SDRAM)&&playback_enable&&source_done&&frame_done&&
                  !first_file_frame&&display_valid&&!pause_playback&&(frames_due!=0);
wire skip_frame=(state==ST_CATCHUP)&&playback_enable&&!pause_playback&&
                !next_pending&&!restart_pending&&(frames_due!=0);
reg [MAX_VIDEO_NUM-1:0] video_present;
reg [31:0] video_clusters[0:MAX_VIDEO_NUM-1];
reg [31:0] video_sizes[0:MAX_VIDEO_NUM-1];
reg [3:0] video_slot,select_slot,select_attempts;
reg [3:0] presented_video_slot;
reg [31:0] directory_cluster;
reg [12:0] directory_sectors;
wire [3:0] name_number=(name[6]=="1"&&name[7]=="0")?4'd10:
                         (name[6]=="0"&&name[7]>="1"&&name[7]<="9")?
                         (name[7]-8'h30):4'd0;
wire valid_ext=((name[8]=="V")||(name[8]=="v"))&&
               ((name[9]=="D")||(name[9]=="d"))&&
               ((name[10]=="O")||(name[10]=="o"));
wire valid_canonical=((name[0]=="V")||(name[0]=="v"))&&
                     ((name[1]=="I")||(name[1]=="i"))&&
                     ((name[2]=="D")||(name[2]=="d"))&&
                     ((name[3]=="E")||(name[3]=="e"))&&
                     ((name[4]=="O")||(name[4]=="o"))&&name[5]=="_"&&
                     name_number!=0&&name_number<=MAX_VIDEO_NUM;
wire valid_name=(name[0]!=8'h00)&&(name[0]!=8'he5)&&
                !attr[4]&&!attr[3]&&valid_ext&&valid_canonical;
// Keep the previous single-file long-name alias as slot1 fallback only.
// Canonical VIDEO_01 overrides it regardless of directory order.
wire valid_alias=(name[0]=="V")&&(name[1]=="I")&&(name[2]=="D")&&
                 (name[3]=="E")&&(name[4]=="O")&&(name[5]=="_")&&
                 (name[6]=="~")&&(name[7]>="1")&&(name[7]<="9")&&
                 !attr[4]&&!attr[3]&&valid_ext;
// Timeout measures lack of progress, not the duration of a whole video.
// A paused, already-completed frame is an intentional wait, not an error.
wire timeout_active=(state!=ST_IDLE)&&(state!=ST_READY)&&(state!=ST_ERROR)&&
                    (state!=ST_CATCHUP)&&(state!=ST_SKIP)&&
                    ((state!=ST_WAIT_DISPLAY)||playback_enable)&&
                    ((state!=ST_WAIT_SDRAM)||!frame_done);
wire io_progress=sd_sec_read_data_valid||sd_sec_read_end||
                 ((state==ST_WAIT_REQ)&&ack_sync[2])||
                 ((state==ST_WAIT_RELEASE)&&!ack_sync[2])||
                 (finish_sync[2]^finish_seen);
assign write_addr_index={1'b0,loading_buffer};
assign read_addr_index={1'b0,active_buffer};
assign playback_paused=pause_playback;
assign video_index=(presented_video_slot<MAX_VIDEO_NUM)?({1'b0,presented_video_slot}+3'd1):3'd0;
assign transport_busy=(state!=ST_IDLE)&&(state!=ST_READY)&&(state!=ST_ERROR)&&
                      (state!=ST_CATCHUP)&&
                      (state!=ST_WAIT_DISPLAY)&&
                      ((state!=ST_WAIT_SDRAM)||!(source_done&&frame_done));
// Only a FILE transition holds audio. Ordinary frame swaps and slow SD reads
// must not interrupt speech. epoch_commit protects the initial HDMI handshake.
assign audio_sync_hold=display_valid &&
                      ((state==ST_WAIT_DISPLAY&&epoch_commit) || restart_pending ||
                       next_pending || first_file_frame);

always @(posedge clk or posedge rst) begin
  if(rst) begin
    state<=ST_IDLE;timeout<=0;count<=0;sd_sec_read<=0;sd_sec_read_addr<=0;scan_done<=0;video_found<=0;
    display_valid<=0;status<=0;write_req<=0;write_en<=0;write_data<=0;bytes_per_sector<=0;reserved<=0;
    sectors_per_cluster<=0;fat_count<=0;volume_base<=0;fat_base<=0;fat_size<=0;root_cluster<=0;data_base<=0;
    root_sector<=0;mbr_type<=0;partition_lba<=0;attr<=0;cluster_hi<=0;cluster_lo<=0;dir_file_size<=0;file_size<=0;
    file_first_cluster<=0;file_first_sector<=0;root_sector_index<=0;dir_match<=0;magic<=0;video_width<=0;
    video_height<=0;video_fps<=0;frame_count<=0;size_value<=0;size_frames<=0;current_frame<=0;
    sector_in_frame<=0;pixel_count<=0;current_cluster<=0;next_file_sector<=0;cluster_sector_pos<=0;
    need_fat_check<=0;byte_phase<=0;pixel_lo<=0;active_buffer<=0;loading_buffer<=0;source_done<=0;
    frame_done<=0;finish_sync<=0;owner_sync<=0;finish_seen<=0;ack_sync<=0;display_index_meta<=0;display_index_sync<=0;
    start_latched<=0;fat_byte_count<=0;fat_offset<=0;fat_b0<=0;fat_b1<=0;
    fat_b2<=0;fat_b3<=0;frame_phase<=0;frames_due<=0;
    skip_sectors_left<=0;skip_fat<=0;epoch_commit<=0;
    video_present<=0;video_slot<=0;presented_video_slot<=0;select_slot<=0;select_attempts<=0;
    next_pending<=0;first_file_frame<=1;dir_end_seen<=0;dir_fat<=0;
    directory_cluster<=0;directory_sectors<=0;
    enable_d<=0;restart_pending<=0;stream_epoch<=0;
  end else begin
    if(next_item)next_pending<=1;
    enable_d<=playback_enable;
    // Mode disable is a hold: retain file cursor, prefetched frame and phase.
    // Only an explicit next_item selects a new video and starts frame zero.
    write_en<=0;
    finish_sync<={finish_sync[1:0],write_finish_toggle};
    owner_sync<={owner_sync[1:0],write_finish_owner};
    finish_seen<=finish_sync[2];
    ack_sync<={ack_sync[1:0],write_req_ack};
    display_index_meta<=display_buffer_index;display_index_sync<=display_index_meta;
    // The SDRAM writer completion toggle is shared by the BMP and video
    // engines. Detect the fully synchronized edge so its owner is aligned.
    // Latch completion even before the final SD sector ends; presentation
    // still requires both source_done and frame_done.
    if((finish_sync[2]^finish_seen) && owner_sync[2] &&
       ((state==ST_WAIT_RELEASE)||(state==ST_FRAME)||(state==ST_FAT)||
        (state==ST_FAT_CHECK)||(state==ST_WAIT_SDRAM)))frame_done<=1;
    // Never reset phase on an ordinary frame commit: doing so stretched each
    // frame by its read/handshake delay and made audio loop ahead of video.
    if(sample_tick)begin
      if(frame_tick)frame_phase<=frame_phase+video_fps-AUDIO_SAMPLE_RATE;
      else frame_phase<=frame_phase+video_fps;
    end
    case({frame_tick,(timed_commit||skip_frame)})
      2'b10:if(frames_due!=8'hff)frames_due<=frames_due+1'b1;
      2'b01:frames_due<=frames_due-1'b1;
      default:;
    endcase
    if(!timeout_active||io_progress)timeout<=0;
    else if(timeout<TIMEOUT_CYCLES)timeout<=timeout+1'b1;
    if(!sd_init_done)begin
      state<=ST_IDLE;start_latched<=0;scan_done<=0;video_found<=0;display_valid<=0;status<=0;sd_sec_read<=0;
      write_req<=0;active_buffer<=0;frame_phase<=0;frames_due<=0;skip_fat<=0;epoch_commit<=0;
      video_present<=0;next_pending<=0;dir_fat<=0;first_file_frame<=1;
    end else if(timeout_active&&!io_progress&&timeout>=TIMEOUT_CYCLES)begin
      state<=ST_ERROR;status<=(state==ST_WAIT_SDRAM)?4'hd:4'hf;
      scan_done<=1;video_found<=1;sd_sec_read<=0;write_req<=0;
    end else if(playback_enable&&(next_pending||restart_pending)&&video_present!=0&&
                !sd_sec_read&&!write_req&&!ack_sync[2]&&
                ((state==ST_READY)||(state==ST_CATCHUP)||(state==ST_ERROR)||
                 (state==ST_WAIT_SDRAM&&source_done&&frame_done)||
                 (state==ST_WAIT_DISPLAY&&display_index_sync=={1'b0,active_buffer})))begin
      // Drain an in-flight frame, then discard any unpresented old frame.
      // Keep active_buffer/display_valid until the new file's first frame
      // is complete. Paused music and the global pause flag are untouched.
      select_slot<=next_pending ?
                   ((video_slot+1>=MAX_VIDEO_NUM)?4'd0:video_slot+1'b1) : video_slot;
      // Consume one queued KEY3 event here. Keeping next_pending asserted
      // would retrigger selection as soon as the new frame is presented.
      select_attempts<=0;next_pending<=0;restart_pending<=0;first_file_frame<=1;
      source_done<=0;frame_done<=0;skip_fat<=0;timeout<=0;state<=ST_SELECT;
    end else case(state)
      ST_IDLE:begin
        sd_sec_read<=0;write_req<=0;
        if(start&&playback_enable&&!start_latched)begin start_latched<=1;scan_done<=0;video_found<=0;display_valid<=0;status<=0;
          count<=0;dir_match<=0;file_size<=0;root_sector_index<=0;sd_sec_read_addr<=0;state<=ST_MBR;end
      end
      ST_MBR:begin
        sd_sec_read<=1;
        if(sd_sec_read_data_valid)begin
          case(count)450:mbr_type<=sd_sec_read_data;454:partition_lba[7:0]<=sd_sec_read_data;
            455:partition_lba[15:8]<=sd_sec_read_data;456:partition_lba[23:16]<=sd_sec_read_data;
            457:partition_lba[31:24]<=sd_sec_read_data;default:;endcase count<=count+1'b1;end
        if(sd_sec_read_end)begin sd_sec_read<=0;count<=0;
          if(mbr_type==8'h0b||mbr_type==8'h0c||mbr_type==8'h1b||mbr_type==8'h1c)begin volume_base<=partition_lba;sd_sec_read_addr<=partition_lba;end
          else begin volume_base<=0;sd_sec_read_addr<=0;end state<=ST_BPB;end
      end
      ST_BPB:begin
        sd_sec_read<=1;
        if(sd_sec_read_data_valid)begin
          case(count)11:bytes_per_sector[7:0]<=sd_sec_read_data;12:bytes_per_sector[15:8]<=sd_sec_read_data;
            13:sectors_per_cluster<=sd_sec_read_data;14:reserved[7:0]<=sd_sec_read_data;15:reserved[15:8]<=sd_sec_read_data;
            16:fat_count<=sd_sec_read_data;36:fat_size[7:0]<=sd_sec_read_data;37:fat_size[15:8]<=sd_sec_read_data;
            38:fat_size[23:16]<=sd_sec_read_data;39:fat_size[31:24]<=sd_sec_read_data;44:root_cluster[7:0]<=sd_sec_read_data;
            45:root_cluster[15:8]<=sd_sec_read_data;46:root_cluster[23:16]<=sd_sec_read_data;47:root_cluster[31:24]<=sd_sec_read_data;default:;endcase
          count<=count+1'b1;end
        if(sd_sec_read_end)begin sd_sec_read<=0;count<=0;
          if(bytes_per_sector!=512||sectors_per_cluster==0||fat_count==0||fat_size==0||root_cluster<2)begin status<=4'he;scan_done<=1;video_found<=1;state<=ST_ERROR;end
          else begin fat_base<=volume_base+reserved;data_base<=volume_base+reserved+(fat_count*fat_size);
            directory_cluster<=root_cluster;directory_sectors<=0;dir_end_seen<=0;video_present<=0;
            root_sector<=volume_base+reserved+(fat_count*fat_size)+((root_cluster-2)*sectors_per_cluster);
            sd_sec_read_addr<=volume_base+reserved+(fat_count*fat_size)+((root_cluster-2)*sectors_per_cluster);state<=ST_DIR;end end
      end
      ST_DIR:begin
        sd_sec_read<=1;
        if(sd_sec_read_data_valid)begin
          case(count[4:0])0:name[0]<=sd_sec_read_data;1:name[1]<=sd_sec_read_data;2:name[2]<=sd_sec_read_data;
            3:name[3]<=sd_sec_read_data;4:name[4]<=sd_sec_read_data;5:name[5]<=sd_sec_read_data;6:name[6]<=sd_sec_read_data;
            7:name[7]<=sd_sec_read_data;8:name[8]<=sd_sec_read_data;9:name[9]<=sd_sec_read_data;10:name[10]<=sd_sec_read_data;
            11:attr<=sd_sec_read_data;20:cluster_hi[7:0]<=sd_sec_read_data;21:cluster_hi[15:8]<=sd_sec_read_data;
            26:cluster_lo[7:0]<=sd_sec_read_data;27:cluster_lo[15:8]<=sd_sec_read_data;28:dir_file_size[7:0]<=sd_sec_read_data;
            29:dir_file_size[15:8]<=sd_sec_read_data;30:dir_file_size[23:16]<=sd_sec_read_data;
            // Latch all selected-file metadata together. Later directory
            // entries (including empty entries) must not overwrite its size.
            31:begin
              dir_file_size[31:24]<=sd_sec_read_data;
              if(name[0]==0)dir_end_seen<=1;
              if(valid_name&&!dir_end_seen)begin
                video_present[name_number-1'b1]<=1;
                video_sizes[name_number-1'b1]<={sd_sec_read_data,dir_file_size[23:0]};
                video_clusters[name_number-1'b1]<={cluster_hi,cluster_lo};
              end else if(valid_alias&&!dir_end_seen&&!video_present[0])begin
                video_present[0]<=1;video_sizes[0]<={sd_sec_read_data,dir_file_size[23:0]};
                video_clusters[0]<={cluster_hi,cluster_lo};
              end
            end default:;endcase count<=count+1'b1;end
        if(sd_sec_read_end)begin sd_sec_read<=0;count<=0;state<=ST_DIR_END;end
      end
      ST_DIR_END:begin
        if(dir_end_seen)begin select_slot<=0;select_attempts<=0;state<=ST_SELECT;end
        else if(directory_sectors>=4095)begin status<=4'hb;scan_done<=1;state<=ST_ERROR;end
        else if(root_sector_index+1>=sectors_per_cluster)begin
          dir_fat<=1;fat_offset<={directory_cluster[6:0],2'b00};fat_byte_count<=0;
          fat_b0<=0;fat_b1<=0;fat_b2<=0;fat_b3<=0;
          sd_sec_read_addr<=fat_base+(directory_cluster>>7);state<=ST_FAT;
        end else begin directory_sectors<=directory_sectors+1'b1;
          root_sector_index<=root_sector_index+1'b1;sd_sec_read_addr<=root_sector+root_sector_index+1'b1;state<=ST_DIR;end
      end
      ST_SELECT:begin
        if(video_present[select_slot])begin
          video_slot<=select_slot;file_size<=video_sizes[select_slot];
          file_first_cluster<=video_clusters[select_slot];current_cluster<=video_clusters[select_slot];
          file_first_sector<=data_base+((video_clusters[select_slot]-2)*sectors_per_cluster);
          sd_sec_read_addr<=data_base+((video_clusters[select_slot]-2)*sectors_per_cluster);
          video_found<=1;status<=4'h2;count<=0;magic<=0;video_width<=0;
          video_height<=0;video_fps<=0;frame_count<=0;state<=ST_HDR;
        end else if(select_attempts+1>=MAX_VIDEO_NUM)begin
          video_found<=0;status<=4'hc;scan_done<=1;state<=ST_ERROR;
        end else begin select_attempts<=select_attempts+1'b1;
          select_slot<=(select_slot+1>=MAX_VIDEO_NUM)?4'd0:select_slot+1'b1;end
      end
      ST_HDR:begin
        sd_sec_read<=1;
        if(sd_sec_read_data_valid)begin case(count)0:magic[7:0]<=sd_sec_read_data;1:magic[15:8]<=sd_sec_read_data;
          2:magic[23:16]<=sd_sec_read_data;3:magic[31:24]<=sd_sec_read_data;4:video_width[7:0]<=sd_sec_read_data;
          5:video_width[15:8]<=sd_sec_read_data;6:video_height[7:0]<=sd_sec_read_data;7:video_height[15:8]<=sd_sec_read_data;
          8:video_fps[7:0]<=sd_sec_read_data;9:video_fps[15:8]<=sd_sec_read_data;12:frame_count[7:0]<=sd_sec_read_data;
          13:frame_count[15:8]<=sd_sec_read_data;14:frame_count[23:16]<=sd_sec_read_data;15:frame_count[31:24]<=sd_sec_read_data;default:;endcase count<=count+1'b1;end
        if(sd_sec_read_end)begin sd_sec_read<=0;count<=0;state<=ST_HDR_CHECK;end
      end
      ST_HDR_CHECK:begin
        // SD delivers bytes in order. Require the actual VDO1 header;
        // sequential size checking avoids a large combinational divider.
        if(magic==32'h314f4456 && file_first_cluster>=2 &&
                    video_width==640 && video_height==480 && video_fps!=0 && video_fps<=30 &&
                    frame_count!=0 && file_size>=512) begin
          size_value<=512;size_frames<=0;state<=ST_SIZE_CHECK;
        end else begin
          status<=4'ha;scan_done<=1;video_found<=1;state<=ST_ERROR;
        end
      end
      ST_SIZE_CHECK:begin
        if(size_frames<frame_count)begin
          if(size_value>32'hfff69fff||size_value>file_size)begin status<=4'ha;scan_done<=1;video_found<=1;state<=ST_ERROR;end
          else begin size_value<=size_value+FRAME_BYTES;size_frames<=size_frames+1'b1;end
        end else if(size_value!=file_size)begin status<=4'ha;scan_done<=1;video_found<=1;state<=ST_ERROR;end
        else begin scan_done<=1;current_frame<=0;loading_buffer<=display_valid?~active_buffer:1'b0;sector_in_frame<=0;pixel_count<=0;
          byte_phase<=0;source_done<=0;frame_done<=0;
          next_file_sector<=file_first_sector+1'b1;
          if(sectors_per_cluster==1)begin cluster_sector_pos<=0;need_fat_check<=1;end
          else begin cluster_sector_pos<=1;need_fat_check<=0;end
          frame_phase<=0;frames_due<=0;skip_fat<=0;write_req<=1;state<=ST_WAIT_REQ;end
      end
      ST_WAIT_REQ:begin
        if(ack_sync[2])begin write_req<=0;sector_in_frame<=0;pixel_count<=0;byte_phase<=0;frame_done<=0;
          state<=ST_WAIT_RELEASE;end
      end
      ST_WAIT_RELEASE:begin
        // FIFO remains cleared while ACK is high in the SDRAM domain.
        // Do not deliver pixels until the full request/ACK handshake ends.
        if(!ack_sync[2])begin
          if(need_fat_check)begin fat_offset<={current_cluster[6:0],2'b00};fat_byte_count<=0;fat_b0<=0;fat_b1<=0;fat_b2<=0;fat_b3<=0;
            sd_sec_read_addr<=fat_base+(current_cluster>>7);state<=ST_FAT;end
          else begin sd_sec_read_addr<=next_file_sector;state<=ST_FRAME;end end
      end
      ST_FRAME:begin
        sd_sec_read<=1;
        if(sd_sec_read_data_valid&&pixel_count<FRAME_PIXELS)begin
          if(!byte_phase)begin pixel_lo<=sd_sec_read_data;byte_phase<=1;end
          else begin write_data<={
            {sd_sec_read_data[7:3],sd_sec_read_data[7:5]},
            {sd_sec_read_data[2:0],pixel_lo[7:5],sd_sec_read_data[2:1]},
            {pixel_lo[4:0],pixel_lo[4:2]},8'd0};
            write_en<=1;byte_phase<=0;pixel_count<=pixel_count+1'b1;end end
        if(sd_sec_read_end)begin sd_sec_read<=0;next_file_sector<=sd_sec_read_addr+1'b1;
          if(sector_in_frame+1>=FRAME_SECTORS)begin
            if(cluster_sector_pos+1>=sectors_per_cluster)need_fat_check<=1;else cluster_sector_pos<=cluster_sector_pos+1'b1;
            if(pixel_count!=FRAME_PIXELS||byte_phase)begin status<=4'hc;state<=ST_ERROR;end
            else begin source_done<=1;state<=ST_WAIT_SDRAM;end end
          else if(cluster_sector_pos+1>=sectors_per_cluster)begin sector_in_frame<=sector_in_frame+1'b1;need_fat_check<=1;
            fat_offset<={current_cluster[6:0],2'b00};fat_byte_count<=0;fat_b0<=0;fat_b1<=0;fat_b2<=0;fat_b3<=0;
            sd_sec_read_addr<=fat_base+(current_cluster>>7);state<=ST_FAT;end
          else begin sector_in_frame<=sector_in_frame+1'b1;cluster_sector_pos<=cluster_sector_pos+1'b1;sd_sec_read_addr<=sd_sec_read_addr+1'b1;end end
      end
      ST_FAT:begin
        sd_sec_read<=1;
        if(sd_sec_read_data_valid)begin if(fat_byte_count==fat_offset)fat_b0<=sd_sec_read_data;
          if(fat_byte_count==fat_offset+1'b1)fat_b1<=sd_sec_read_data;if(fat_byte_count==fat_offset+2'd2)fat_b2<=sd_sec_read_data;
          if(fat_byte_count==fat_offset+2'd3)fat_b3<=sd_sec_read_data;fat_byte_count<=fat_byte_count+1'b1;end
        if(sd_sec_read_end)begin sd_sec_read<=0;state<=ST_FAT_CHECK;end
      end
      ST_FAT_CHECK:begin
        // Follow the FAT chain even when the file is fragmented.  The
        // previous implementation required fat_next == current_cluster+1,
        // which rejected large VDO files copied onto a non-contiguous card
        // and left the HDMI frame buffer black with status B.
        if(dir_fat)begin
          dir_fat<=0;
          if(fat_next>=32'h0ffffff8)begin select_slot<=0;select_attempts<=0;state<=ST_SELECT;end
          else if(fat_next<2||fat_next>=32'h0ffffff0||fat_next==directory_cluster)begin
            status<=4'hb;scan_done<=1;state<=ST_ERROR;
          end else begin directory_cluster<=fat_next;directory_sectors<=directory_sectors+1'b1;
            root_sector_index<=0;root_sector<=data_base+((fat_next-2)*sectors_per_cluster);
            sd_sec_read_addr<=data_base+((fat_next-2)*sectors_per_cluster);state<=ST_DIR;end
        end else if(fat_next<2 || fat_next>=32'h0ffffff0 || fat_next==current_cluster)begin
          // This state is entered only when more payload is required.
          status<=4'hb;scan_done<=1;video_found<=1;state<=ST_ERROR;
        end else begin
          current_cluster<=fat_next;
          cluster_sector_pos<=0;
          need_fat_check<=0;
          next_file_sector<=data_base+((fat_next-2)*sectors_per_cluster);
          sd_sec_read_addr<=data_base+((fat_next-2)*sectors_per_cluster);
          state<=skip_fat?ST_SKIP:ST_FRAME;
        end
      end
      ST_WAIT_SDRAM:begin
        if(playback_enable&&source_done&&frame_done&&
           ((!display_valid||first_file_frame) ||
            (!pause_playback&&frames_due!=0)))begin
          active_buffer<=loading_buffer;display_valid<=1;status<=4'h1;source_done<=0;frame_done<=0;
          // This is the first point at which the new file has a complete
          // frame and is eligible for HDMI presentation.  Keep the loading
          // slot private until this commit so OSD cannot show a speculative
          // VIDEO_02/VIDEO_03 number.
          presented_video_slot<=video_slot;
          // Only an explicit file selection starts a new audio/subtitle epoch.
          // Natural VDO wraps follow the continuing PCM clock; restarting WAV
          // at a late frame-zero commit would replay the beginning of speech.
          epoch_commit<=first_file_frame;
          if(first_file_frame)begin
            stream_epoch<=~stream_epoch;frame_phase<=0;frames_due<=0;
          end
          first_file_frame<=0;
          if(current_frame+1>=frame_count)begin current_frame<=0;current_cluster<=file_first_cluster;next_file_sector<=file_first_sector+1'b1;
            if(sectors_per_cluster==1)begin cluster_sector_pos<=0;need_fat_check<=1;end else begin cluster_sector_pos<=1;need_fat_check<=0;end end
          else current_frame<=current_frame+1'b1;state<=ST_WAIT_DISPLAY;end
      end
      ST_WAIT_DISPLAY:begin
        // Do not overwrite the old displayed buffer until the HDMI read
        // side has committed the new one at vertical blank.
        if(display_index_sync=={1'b0,active_buffer})begin epoch_commit<=0;state<=ST_READY;end
      end
      ST_READY:begin
        if(playback_enable&&!pause_playback)state<=ST_CATCHUP;
      end
      ST_CATCHUP:begin
        if(playback_enable&&!pause_playback)begin
          if(frames_due!=0)begin
            // Drop overdue frames WITHOUT reading their 614400 payload bytes.
            // Walk the actual FAT chain, so fragmented cards remain supported.
            skip_sectors_left<=11'd1200;state<=ST_SKIP;
          end else begin
            skip_fat<=0;loading_buffer<=~active_buffer;sector_in_frame<=0;pixel_count<=0;byte_phase<=0;
            source_done<=0;frame_done<=0;write_req<=1;state<=ST_WAIT_REQ;
          end
        end
      end
      ST_SKIP:begin
        if(need_fat_check)begin
          skip_fat<=1;fat_offset<={current_cluster[6:0],2'b00};fat_byte_count<=0;
          fat_b0<=0;fat_b1<=0;fat_b2<=0;fat_b3<=0;
          sd_sec_read_addr<=fat_base+(current_cluster>>7);state<=ST_FAT;
        end else begin
          next_file_sector<=next_file_sector+1'b1;
          if(cluster_sector_pos+1>=sectors_per_cluster)need_fat_check<=1;
          else cluster_sector_pos<=cluster_sector_pos+1'b1;
          skip_sectors_left<=skip_sectors_left-1'b1;
          if(skip_sectors_left==1)begin
            if(current_frame+1>=frame_count)begin
              current_frame<=0;current_cluster<=file_first_cluster;next_file_sector<=file_first_sector+1'b1;
              if(sectors_per_cluster==1)begin cluster_sector_pos<=0;need_fat_check<=1;end
              else begin cluster_sector_pos<=1;need_fat_check<=0;end
            end else current_frame<=current_frame+1'b1;
            skip_fat<=0;state<=ST_CATCHUP;
          end
        end
      end
      ST_ERROR:begin sd_sec_read<=0;write_req<=0;end
      default:state<=ST_IDLE;
    endcase
  end
end
endmodule
