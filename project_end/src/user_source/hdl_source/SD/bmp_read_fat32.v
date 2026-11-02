// FAT32 root-directory reader for numbered contiguous BMP files (1.BMP ... N.BMP).
// The module intentionally keeps the existing byte-sector SD interface.
module bmp_read_fat32 #(
    parameter integer MAX_BMP_NUM = 3,
    parameter integer TIMEOUT_CYCLES = 500_000_000,
    parameter integer MAX_SOURCE_WIDTH = 2048
)(
    input clk, input rst, input start_scan,
    input [31:0] boot_sector,
    input [3:0] target_count,
    output reg scan_done, output reg [3:0] image_count,
    output reg [MAX_BMP_NUM*32-1:0] image_sector,
    output reg [MAX_BMP_NUM*32-1:0] image_cluster,
    output reg [MAX_BMP_NUM*32-1:0] image_size,
    output reg [MAX_BMP_NUM*32-1:0] image_offset,
    input load_start, input [3:0] load_index, output reg load_done,
    output reg [3:0] status,
    output reg write_req, input write_req_ack,
    output reg sd_sec_read, output reg [31:0] sd_sec_read_addr,
    input [7:0] sd_sec_read_data, input sd_sec_read_data_valid, input sd_sec_read_end,
    output reg bmp_data_wr_en, output reg [23:0] bmp_data,
    input [15:0] bmp_width, input [15:0] bmp_height
);
localparam ST_IDLE=5'd0, ST_MBR=5'd1, ST_BPB=5'd2, ST_DIR=5'd3, ST_FAT=5'd4,
           ST_HDR=5'd5, ST_SCALE_DIV=5'd6, ST_SCALE_READY=5'd7,
           ST_X_DIV=5'd8, ST_Y_DIV=5'd9, ST_WRITE=5'd10,
           ST_ROW_PREP=5'd11, ST_ROW_READ=5'd12, ST_ROW_OUTPUT=5'd13,
           ST_PIXEL_WAIT=5'd14, ST_PIXEL_EMIT=5'd15, ST_ROW_DONE=5'd16,
           ST_ADVANCE_Y=5'd17, ST_WRITE_RELEASE=5'd18;
// A 640x480x24 BMP has 1920 bytes per row (including the 4-byte alignment
// already accounted for).  The old 10-bit counter wrapped at 1023, so rows
// never advanced and the reader eventually reported timeout (F).
reg [4:0] st; reg [11:0] cnt; reg [31:0] timeout;
reg [4:0] timeout_state;
// Measure a stalled transaction, not the total duration of SD arbitration,
// FAT verification, scaling and an entire 307200-pixel frame combined.
wire io_progress = sd_sec_read_data_valid || sd_sec_read_end ||
                   bmp_data_wr_en || (st != timeout_state);
wire timeout_expired = (st != ST_IDLE) && !io_progress &&
                       (timeout >= TIMEOUT_CYCLES);
reg [15:0] bytes_per_sector, reserved; reg [7:0] sectors_per_cluster, fat_count;
reg [31:0] volume_base, fat_size, root_cluster, fat_base, data_base, dir_sector;
reg [7:0] mbr_type; reg [31:0] mbr_partition_lba;
reg [7:0] dir_sector_no; reg [7:0] name[0:10], attr; reg [15:0] hi,lo; reg [31:0] dir_size;
reg [MAX_BMP_NUM-1:0] seen; reg [3:0] active_index; reg [31:0] verify_cluster, verify_bytes_left, fat_addr;
// FAT32 entries are four bytes and can straddle a 512-byte sector boundary
// when the entry offset is 509, 510 or 511.  The original implementation
// only sampled one sector, which could turn a valid contiguous chain into a
// false ``B`` (fragmented-chain) error.
reg [8:0] fat_off;
reg       fat_cross;
reg       fat_phase;
reg       fat_retry;
reg       fat_eval_pending;
reg       hdr_eval_pending;
reg [7:0] fb0,fb1,fb2,fb3; reg [7:0] bh0,bh1; reg [31:0] bh_size,bh_off,bh_w,bh_h,bh_comp; reg [15:0] bh_bpp;
reg [31:0] row_bytes;

// One source row is buffered in block RAM.  Destination pixels are then
// generated at a fixed 640x480 size with nearest-neighbour coordinate steps.
// This keeps the existing SDRAM writer and HDMI timing completely unchanged.
reg [23:0] source_line [0:MAX_SOURCE_WIDTH-1];
reg [10:0] line_write_addr, line_read_addr;
reg [23:0] line_read_data;
reg [7:0] pixel_b, pixel_g;
reg [1:0] source_byte_index;
reg [15:0] source_pixels_loaded;
reg row_capture_done, line_valid, row_is_content;
reg [31:0] source_row_offset;
reg [8:0] row_sector_skip;

reg scale_width_limited;
reg [31:0] div_work, div_denominator;
reg [15:0] div_quotient;
reg [15:0] scaled_width, scaled_height, x_offset, y_offset;
reg [15:0] x_step_quotient, x_step_remainder;
reg [15:0] y_step_quotient, y_step_remainder;
reg [15:0] output_x, output_y, source_x, source_y;
reg [16:0] x_error, y_error;
reg [15:0] row_advance_left;

localparam [15:0] TARGET_WIDTH = 16'd640;
localparam [15:0] TARGET_HEIGHT = 16'd480;
wire [31:0] cluster_bytes = {24'd0,sectors_per_cluster} << 9;
wire [31:0] fat_next = {fb3,fb2,fb1,fb0} & 32'h0fffffff;
wire [3:0] fname_digit = (name[0]>="1" && name[0]<="9") ? name[0]-"1" : 4'hf;
wire fname_single_ok = (name[1]==" ") && (name[2]==" ") && (name[3]==" ") && (name[4]==" ") && (name[5]==" ") && (name[6]==" ") && (name[7]==" ");
wire fname_ten_ok = (name[0]=="1") && (name[1]=="0") && (name[2]==" ") && (name[3]==" ") && (name[4]==" ") && (name[5]==" ") && (name[6]==" ") && (name[7]==" ");
wire [3:0] fname_slot = fname_ten_ok ? 4'd9 : fname_digit;
wire fname_ok = (attr!=8'h0f) && ((name[8]=="B") || (name[8]=="b")) && ((name[9]=="M") || (name[9]=="m")) && ((name[10]=="P") || (name[10]=="p")) && ((fname_single_ok && (fname_digit<4'd9)) || fname_ten_ok) && (fname_slot<MAX_BMP_NUM);
wire [31:0] width_times_480 = (bh_w << 9) - (bh_w << 5);
wire [31:0] height_times_640 = (bh_h << 9) + (bh_h << 7);
wire [16:0] x_error_sum = x_error + {1'b0,x_step_remainder};
wire [16:0] y_error_sum = y_error + {1'b0,y_step_remainder};
wire [16:0] y_advance = {1'b0,y_step_quotient} +
                         ((y_error_sum >= {1'b0,scaled_height}) ? 17'd1 : 17'd0);
wire bmp_ok = (bh0=="B") && (bh1=="M") &&
              (bmp_width==TARGET_WIDTH) && (bmp_height==TARGET_HEIGHT) &&
              (bh_w>0) && (bh_w<=MAX_SOURCE_WIDTH) && (bh_w[31:16]==0) &&
              (bh_h>0) && (bh_h[31:16]==0) && !bh_h[31] &&
              (bh_bpp==16'd24) && (bh_comp==0) && (bh_off>=54) &&
              (bh_size>=bh_off) && (bh_size<=image_size[active_index*32 +: 32]);
// Registered read port lets the source-line array infer as block RAM.
always @(posedge clk) begin
  line_read_data <= source_line[line_read_addr];
end
always @(posedge clk or posedge rst) begin
  if(rst) begin
    st<=ST_IDLE; cnt<=0; timeout<=0; timeout_state<=ST_IDLE;
    scan_done<=0; image_count<=0; seen<=0; status<=0;
    write_req<=0; sd_sec_read<=0; sd_sec_read_addr<=0; load_done<=0; bmp_data_wr_en<=0; bmp_data<=0;
    dir_sector_no<=0; row_bytes<=0;
    fat_off<=0; fat_cross<=0; fat_phase<=0; fat_retry<=0; fat_eval_pending<=0; hdr_eval_pending<=0;
    image_sector<=0; image_cluster<=0; image_size<=0; image_offset<=0;
    line_write_addr<=0; line_read_addr<=0; pixel_b<=0; pixel_g<=0;
    source_byte_index<=0; source_pixels_loaded<=0; row_capture_done<=0;
    line_valid<=0; row_is_content<=0; source_row_offset<=0; row_sector_skip<=0;
    scale_width_limited<=0; div_work<=0; div_denominator<=0; div_quotient<=0;
    scaled_width<=0; scaled_height<=0; x_offset<=0; y_offset<=0;
    x_step_quotient<=0; x_step_remainder<=0; y_step_quotient<=0; y_step_remainder<=0;
    output_x<=0; output_y<=0; source_x<=0; source_y<=0;
    x_error<=0; y_error<=0; row_advance_left<=0;
  end else begin
    bmp_data_wr_en<=0; load_done<=0;
    timeout_state<=st;
    if(st==ST_IDLE || io_progress) timeout<=0;
    else if(timeout<TIMEOUT_CYCLES) timeout<=timeout+1'b1;
    // Keep timeout recovery above the FSM: case assignments must not revive
    // a failed SD request or overwrite the error exit on the same clock.
    if(timeout_expired) begin
      st<=ST_IDLE; status<=4'hf; sd_sec_read<=0; write_req<=0;
    end else case(st)
      ST_IDLE: begin
        sd_sec_read<=0; write_req<=0;
        if(start_scan) begin scan_done<=0; image_count<=0; seen<=0; status<=0; cnt<=0; sd_sec_read_addr<=boot_sector; st<=ST_MBR; end
        else if(load_start && scan_done && load_index<MAX_BMP_NUM && seen[load_index]) begin
          // Keep a remaining-byte count instead of dividing the file size by
          // the run-time cluster size.  The former combinational divider made
          // this 100 MHz path more than 70 ns long and could make the FAT
          // checker behave unpredictably on hardware even though bitgen
          // completed successfully.
          active_index<=load_index; verify_cluster<=image_cluster[load_index*32 +: 32]; verify_bytes_left<=image_size[load_index*32 +: 32];
          fat_addr<=image_cluster[load_index*32 +: 32]*4;
          fat_off<=((image_cluster[load_index*32 +: 32]*4)&32'h1ff);
          fat_cross<=(((image_cluster[load_index*32 +: 32]*4)&32'h1ff)>9'd508);
          fat_phase<=1'b0;
          fat_retry<=1'b0;
          fat_eval_pending<=1'b0;
          fb0<=0; fb1<=0; fb2<=0; fb3<=0;
          sd_sec_read_addr<=fat_base+((image_cluster[load_index*32 +: 32]*4)>>9); cnt<=0; st<=ST_FAT;
        end
      end
      ST_MBR: begin
        sd_sec_read<=1;
        if(sd_sec_read_data_valid) begin
          case(cnt)
            10'd450: mbr_type<=sd_sec_read_data;
            10'd454: mbr_partition_lba[7:0]<=sd_sec_read_data;
            10'd455: mbr_partition_lba[15:8]<=sd_sec_read_data;
            10'd456: mbr_partition_lba[23:16]<=sd_sec_read_data;
            10'd457: mbr_partition_lba[31:24]<=sd_sec_read_data;
            default:;
          endcase
          cnt<=cnt+1'b1;
        end
        if(sd_sec_read_end) begin
          sd_sec_read<=0; cnt<=0;
          if((mbr_type==8'h0b) || (mbr_type==8'h0c) || (mbr_type==8'h1b) || (mbr_type==8'h1c)) begin
            volume_base<=boot_sector+mbr_partition_lba;
            sd_sec_read_addr<=boot_sector+mbr_partition_lba;
            st<=ST_BPB;
          end else begin
            volume_base<=boot_sector;
            sd_sec_read_addr<=boot_sector;
            st<=ST_BPB;
          end
        end
      end
      ST_BPB: begin
        sd_sec_read<=1;
        if(sd_sec_read_data_valid) begin
          case(cnt)
            11:bytes_per_sector[7:0]<=sd_sec_read_data;12:bytes_per_sector[15:8]<=sd_sec_read_data;13:sectors_per_cluster<=sd_sec_read_data;
            14:reserved[7:0]<=sd_sec_read_data;15:reserved[15:8]<=sd_sec_read_data;16:fat_count<=sd_sec_read_data;
            36:fat_size[7:0]<=sd_sec_read_data;37:fat_size[15:8]<=sd_sec_read_data;38:fat_size[23:16]<=sd_sec_read_data;39:fat_size[31:24]<=sd_sec_read_data;
            44:root_cluster[7:0]<=sd_sec_read_data;45:root_cluster[15:8]<=sd_sec_read_data;46:root_cluster[23:16]<=sd_sec_read_data;47:root_cluster[31:24]<=sd_sec_read_data;
            default:;
          endcase cnt<=cnt+1'b1;
        end
        if(sd_sec_read_end) begin sd_sec_read<=0; cnt<=0;
          if(bytes_per_sector!=512 || sectors_per_cluster==0 || fat_count==0 || fat_size==0 || root_cluster<2) begin status<=4'he; st<=ST_IDLE; end
          else begin fat_base<=volume_base+reserved; data_base<=volume_base+reserved+(fat_count*fat_size); dir_sector<=volume_base+reserved+(fat_count*fat_size)+((root_cluster-2)*sectors_per_cluster); sd_sec_read_addr<=volume_base+reserved+(fat_count*fat_size)+((root_cluster-2)*sectors_per_cluster); st<=ST_DIR; end
        end
      end
      ST_DIR: begin
        sd_sec_read<=1;
        if(sd_sec_read_data_valid) begin
          case(cnt[4:0])
            0:name[0]<=sd_sec_read_data;1:name[1]<=sd_sec_read_data;2:name[2]<=sd_sec_read_data;3:name[3]<=sd_sec_read_data;4:name[4]<=sd_sec_read_data;5:name[5]<=sd_sec_read_data;6:name[6]<=sd_sec_read_data;7:name[7]<=sd_sec_read_data;8:name[8]<=sd_sec_read_data;9:name[9]<=sd_sec_read_data;10:name[10]<=sd_sec_read_data;11:attr<=sd_sec_read_data;20:hi[7:0]<=sd_sec_read_data;21:hi[15:8]<=sd_sec_read_data;26:lo[7:0]<=sd_sec_read_data;27:lo[15:8]<=sd_sec_read_data;28:dir_size[7:0]<=sd_sec_read_data;29:dir_size[15:8]<=sd_sec_read_data;30:dir_size[23:16]<=sd_sec_read_data;
            31:begin dir_size[31:24]<=sd_sec_read_data; if(fname_ok && !seen[fname_slot]) begin image_cluster[fname_slot*32 +: 32]<={hi,lo}; image_sector[fname_slot*32 +: 32]<=data_base+(({hi,lo}-2)*sectors_per_cluster); image_size[fname_slot*32 +: 32]<={sd_sec_read_data,dir_size[23:0]}; image_offset[fname_slot*32 +: 32]<=54; seen[fname_slot]<=1; image_count<=image_count+1'b1; end end
            default:;
          endcase cnt<=cnt+1'b1;
        end
        if(sd_sec_read_end) begin sd_sec_read<=0; cnt<=0; if(dir_sector_no>=sectors_per_cluster-1 || image_count>=target_count) begin scan_done<=1; if(image_count==0) status<=4'hc; st<=ST_IDLE; end else begin dir_sector_no<=dir_sector_no+1'b1; dir_sector<=dir_sector+1'b1; sd_sec_read_addr<=dir_sector+1'b1; end end
      end
      ST_FAT: begin
        if(fat_eval_pending) begin
          // Evaluate only after the clock edge which latched the last FAT
          // byte.  Comparing in the sd_sec_read_end cycle sees stale fb*.
          fat_eval_pending<=1'b0;
          sd_sec_read<=0;
          if(verify_bytes_left<=cluster_bytes) begin
            sd_sec_read_addr<=image_sector[active_index*32 +: 32]; st<=ST_HDR;
          end else if((fat_next!=verify_cluster+1) || (fat_next<32'd2) || (fat_next>=32'h0ffffff8)) begin
            if(!fat_retry) begin
              fat_retry<=1'b1;
              fat_phase<=1'b0;
              fat_off<=(verify_cluster*4)&32'h1ff;
              fat_cross<=(((verify_cluster*4)&32'h1ff)>9'd508);
              fb0<=0; fb1<=0; fb2<=0; fb3<=0;
              sd_sec_read_addr<=fat_base+((verify_cluster*4)>>9);
              cnt<=0;
            end else begin
              status<=4'hb; st<=ST_IDLE;
            end
          end else begin
            verify_cluster<=fat_next;
            verify_bytes_left<=verify_bytes_left-cluster_bytes;
            fat_addr<=fat_next*4;
            fat_off<=(fat_next*4)&32'h1ff;
            fat_cross<=(((fat_next*4)&32'h1ff)>9'd508);
            fat_phase<=1'b0;
            fat_retry<=1'b0;
            fb0<=0; fb1<=0; fb2<=0; fb3<=0;
            sd_sec_read_addr<=fat_base+((fat_next*4)>>9);
          end
        end else begin
        sd_sec_read<=1;
        if(sd_sec_read_data_valid) begin
          if(!fat_cross) begin
            if(cnt==fat_off)   fb0<=sd_sec_read_data;
            if(cnt==fat_off+1) fb1<=sd_sec_read_data;
            if(cnt==fat_off+2) fb2<=sd_sec_read_data;
            if(cnt==fat_off+3) fb3<=sd_sec_read_data;
          end else if(fat_phase==1'b0) begin
            // First part of an entry split at the end of this sector.
            if(cnt==fat_off)   fb0<=sd_sec_read_data;
            if(cnt==fat_off+1) fb1<=sd_sec_read_data;
            if(cnt==fat_off+2) fb2<=sd_sec_read_data;
          end else begin
            // Remaining bytes are at offsets 0, 1 and/or 2 of next sector.
            if(fat_off==9'd509) begin
              if(cnt==0) fb3<=sd_sec_read_data;
            end else if(fat_off==9'd510) begin
              if(cnt==0) fb2<=sd_sec_read_data;
              if(cnt==1) fb3<=sd_sec_read_data;
            end else begin // fat_off == 511
              if(cnt==0) fb1<=sd_sec_read_data;
              if(cnt==1) fb2<=sd_sec_read_data;
              if(cnt==2) fb3<=sd_sec_read_data;
            end
          end
          cnt<=cnt+1'b1;
        end
        if(sd_sec_read_end) begin
          sd_sec_read<=0;
          if(fat_cross && fat_phase==1'b0) begin
            // Fetch the next FAT sector before evaluating fat_next.
            fat_phase<=1'b1;
            cnt<=0;
            sd_sec_read_addr<=sd_sec_read_addr+1'b1;
          end else begin
            cnt<=0;
            fat_eval_pending<=1'b1;
          end
        end
        end
      end
      ST_HDR: begin
        if(hdr_eval_pending) begin
          hdr_eval_pending<=1'b0; sd_sec_read<=0; cnt<=0;
          if(!bmp_ok) begin status<=4'ha; st<=ST_IDLE; end
          else begin
            image_offset[active_index*32 +: 32]<=bh_off;
            row_bytes<=((bh_w+(bh_w<<1))+3)&32'hfffffffc;
            div_quotient<=0;
            if(width_times_480>=height_times_640) begin
              // Source is wider than 4:3: fill the output width and letterbox.
              scale_width_limited<=1'b1;
              div_work<=height_times_640;
              div_denominator<=bh_w;
            end else begin
              // Source is taller than 4:3: fill the output height and pillarbox.
              scale_width_limited<=1'b0;
              div_work<=width_times_480;
              div_denominator<=bh_h;
            end
            st<=ST_SCALE_DIV;
          end
        end else begin
        sd_sec_read<=1;
        if(sd_sec_read_data_valid) begin case(cnt)0:bh0<=sd_sec_read_data;1:bh1<=sd_sec_read_data;2:bh_size[7:0]<=sd_sec_read_data;3:bh_size[15:8]<=sd_sec_read_data;4:bh_size[23:16]<=sd_sec_read_data;5:bh_size[31:24]<=sd_sec_read_data;10:bh_off[7:0]<=sd_sec_read_data;11:bh_off[15:8]<=sd_sec_read_data;12:bh_off[23:16]<=sd_sec_read_data;13:bh_off[31:24]<=sd_sec_read_data;18:bh_w[7:0]<=sd_sec_read_data;19:bh_w[15:8]<=sd_sec_read_data;20:bh_w[23:16]<=sd_sec_read_data;21:bh_w[31:24]<=sd_sec_read_data;22:bh_h[7:0]<=sd_sec_read_data;23:bh_h[15:8]<=sd_sec_read_data;24:bh_h[23:16]<=sd_sec_read_data;25:bh_h[31:24]<=sd_sec_read_data;28:bh_bpp[7:0]<=sd_sec_read_data;29:bh_bpp[15:8]<=sd_sec_read_data;30:bh_comp[7:0]<=sd_sec_read_data;31:bh_comp[15:8]<=sd_sec_read_data;32:bh_comp[23:16]<=sd_sec_read_data;33:bh_comp[31:24]<=sd_sec_read_data;default:;endcase cnt<=cnt+1'b1; end
        if(sd_sec_read_end) begin sd_sec_read<=0; cnt<=0; hdr_eval_pending<=1'b1; end
        end
      end
      // The divider performs one subtraction per clock.  It is only active
      // while preparing a new image, so it avoids a long divider path without
      // affecting pixel throughput.
      ST_SCALE_DIV: begin
        if(div_work>=div_denominator) begin
          div_work<=div_work-div_denominator;
          div_quotient<=div_quotient+1'b1;
        end else begin
          if(scale_width_limited) begin
            scaled_width<=TARGET_WIDTH;
            scaled_height<=(div_quotient==0)?16'd1:div_quotient;
          end else begin
            scaled_width<=(div_quotient==0)?16'd1:div_quotient;
            scaled_height<=TARGET_HEIGHT;
          end
          st<=ST_SCALE_READY;
        end
      end
      ST_SCALE_READY: begin
        x_offset<=(TARGET_WIDTH-scaled_width)>>1;
        y_offset<=(TARGET_HEIGHT-scaled_height)>>1;
        div_work<=bh_w;
        div_denominator<=scaled_width;
        div_quotient<=0;
        st<=ST_X_DIV;
      end
      ST_X_DIV: begin
        if(div_work>=div_denominator) begin
          div_work<=div_work-div_denominator;
          div_quotient<=div_quotient+1'b1;
        end else begin
          x_step_quotient<=div_quotient;
          x_step_remainder<=div_work[15:0];
          div_work<=bh_h;
          div_denominator<=scaled_height;
          div_quotient<=0;
          st<=ST_Y_DIV;
        end
      end
      ST_Y_DIV: begin
        if(div_work>=div_denominator) begin
          div_work<=div_work-div_denominator;
          div_quotient<=div_quotient+1'b1;
        end else begin
          y_step_quotient<=div_quotient;
          y_step_remainder<=div_work[15:0];
          write_req<=1'b1;
          st<=ST_WRITE;
        end
      end
      ST_WRITE: begin
        if(write_req_ack) begin
          write_req<=0;
          // ACK high means frame_fifo_write is still clearing the FIFO.
          // Black padding starts without an SD read, so it must not be sent
          // until the synchronized ACK has returned low.
          st<=ST_WRITE_RELEASE;
        end
      end
      ST_WRITE_RELEASE: begin
        if(!write_req_ack) begin
          output_x<=0; output_y<=0; source_x<=0; source_y<=0;
          x_error<=0; y_error<=0;
          source_row_offset<=bh_off;
          line_valid<=0;
          st<=ST_ROW_PREP;
        end
      end
      ST_ROW_PREP: begin
        output_x<=0; source_x<=0; x_error<=0;
        if((output_y<y_offset) || (output_y>=y_offset+scaled_height)) begin
          row_is_content<=0;
          st<=ST_ROW_OUTPUT;
        end else begin
          row_is_content<=1;
          if(line_valid) begin
            st<=ST_ROW_OUTPUT;
          end else begin
            sd_sec_read_addr<=image_sector[active_index*32 +: 32]+(source_row_offset>>9);
            row_sector_skip<=source_row_offset[8:0];
            cnt<=0; source_byte_index<=0; source_pixels_loaded<=0;
            line_write_addr<=0; row_capture_done<=0;
            st<=ST_ROW_READ;
          end
        end
      end
      ST_ROW_READ: begin
        sd_sec_read<=1;
        if(sd_sec_read_data_valid) begin
          cnt<=cnt+1'b1;
          if((cnt>=row_sector_skip) && (source_pixels_loaded<bh_w[15:0])) begin
            if(source_byte_index==0) begin
              pixel_b<=sd_sec_read_data;
              source_byte_index<=1;
            end else if(source_byte_index==1) begin
              pixel_g<=sd_sec_read_data;
              source_byte_index<=2;
            end else begin
              source_line[line_write_addr]<={sd_sec_read_data,pixel_g,pixel_b};
              line_write_addr<=line_write_addr+1'b1;
              source_pixels_loaded<=source_pixels_loaded+1'b1;
              source_byte_index<=0;
              if(source_pixels_loaded+1'b1>=bh_w[15:0]) row_capture_done<=1'b1;
            end
          end
        end
        if(sd_sec_read_end) begin
          sd_sec_read<=0; cnt<=0;
          if(row_capture_done) begin
            line_valid<=1;
            st<=ST_ROW_OUTPUT;
          end else begin
            row_sector_skip<=0;
            sd_sec_read_addr<=sd_sec_read_addr+1'b1;
          end
        end
      end
      ST_ROW_OUTPUT: begin
        // Padding uses the same cadence as content; do not burst black
        // rows at three times the normal rate into the unbackpressured FIFO.
        line_read_addr<=source_x[10:0];
        st<=ST_PIXEL_WAIT;
      end
      ST_PIXEL_WAIT: begin
        // source_line has a synchronous block-RAM read port.
        st<=ST_PIXEL_EMIT;
      end
      ST_PIXEL_EMIT: begin
        bmp_data<=(!row_is_content || (output_x<x_offset) ||
                   (output_x>=x_offset+scaled_width)) ? 24'h000000 : line_read_data;
        bmp_data_wr_en<=1;
        if(output_x==TARGET_WIDTH-1'b1) begin
          st<=ST_ROW_DONE;
        end else begin
          output_x<=output_x+1'b1;
          if(row_is_content && (output_x>=x_offset) &&
             (output_x<x_offset+scaled_width)) begin
            source_x<=source_x+x_step_quotient+
                    ((x_error_sum>={1'b0,scaled_width})?16'd1:16'd0);
            if(x_error_sum>={1'b0,scaled_width}) x_error<=x_error_sum-{1'b0,scaled_width};
            else x_error<=x_error_sum;
          end
          st<=ST_ROW_OUTPUT;
        end
      end
      ST_ROW_DONE: begin
        if(output_y==TARGET_HEIGHT-1'b1) begin
          load_done<=1;
          st<=ST_IDLE;
        end else begin
          output_y<=output_y+1'b1;
          if(row_is_content) begin
            source_y<=source_y+y_advance[15:0];
            row_advance_left<=y_advance[15:0];
            if(y_error_sum>={1'b0,scaled_height}) y_error<=y_error_sum-{1'b0,scaled_height};
            else y_error<=y_error_sum;
            if(y_advance!=0) begin
              line_valid<=0;
              st<=ST_ADVANCE_Y;
            end else begin
              st<=ST_ROW_PREP;
            end
          end else begin
            st<=ST_ROW_PREP;
          end
        end
      end
      ST_ADVANCE_Y: begin
        if(row_advance_left!=0) begin
          source_row_offset<=source_row_offset+row_bytes;
          row_advance_left<=row_advance_left-1'b1;
        end else begin
          st<=ST_ROW_PREP;
        end
      end
      default:st<=ST_IDLE;
    endcase
  end
end
endmodule
