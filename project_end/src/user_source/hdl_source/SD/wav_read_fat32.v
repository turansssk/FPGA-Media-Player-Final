// FAT32 WAV/SUB reader shared by the picture-music and video-sound groups.
// Supported payload: uncompressed PCM, 48 kHz, 16-bit, mono or stereo.
// WAV and SUB files may occupy a non-contiguous FAT32 cluster chain.
module wav_read_fat32 #(
    parameter integer HEADER_MAX_SECTORS = 16,
    parameter integer TIMEOUT_CYCLES = 500_000_000,
    parameter integer MAX_MUSIC_NUM = 3
)(
    input  wire clk,
    input  wire rst,
    input  wire sd_init_done,
    output reg  sd_sec_read,
    output reg  [31:0] sd_sec_read_addr,
    input  wire [7:0] sd_sec_read_data,
    input  wire sd_sec_read_data_valid,
    input  wire sd_sec_read_end,
    // 0: MUSIC_01..03.WAV/SUB, 1: VIDEO_01..03.WAV/SUB.
    // The top level changes this only while this reader is held in reset.
    input  wire audio_source_video,
    input  wire [1:0] music_index,
    input  wire next_item,
    input  wire pause_playback,
    input  wire [31:0] resume_sample,
    output reg  fifo_wr_en,
    output reg  [31:0] fifo_wr_data,
    input  wire [8:0] fifo_wrusedw,
    output reg  audio_ready,
    output reg  audio_playing,
    output reg  [3:0] status,
    output reg  [31:0] audio_total_samples,
    output reg  subtitle_wr_en,
    output reg  [12:0] subtitle_wr_addr,
    output reg  [7:0] subtitle_wr_data,
    output reg  subtitle_ready,
    output reg  [5:0] subtitle_line_count,
    output reg  [3:0] subtitle_status
);
    localparam ST_WAIT=4'd0, ST_MBR=4'd1, ST_BPB=4'd2, ST_DIR=4'd3,
               ST_HDR=4'd4, ST_HDR_EVAL=4'd5, ST_STREAM_WAIT=4'd6,
               ST_STREAM=4'd7, ST_FAT=4'd8, ST_FAT_EVAL=4'd9,
               ST_RESTART=4'd10, ST_SUB=4'd11, ST_SUB_EVAL=4'd12,
               ST_DIR_EVAL=4'd13, ST_SEEK=4'd14, ST_ERROR=4'd15;
    localparam [31:0] CHUNK_FMT  = 32'h20746d66; // "fmt " after byte reversal
    localparam [31:0] CHUNK_DATA = 32'h61746164; // "data"

    reg [3:0] state;
    reg [9:0] byte_count;
    reg [31:0] timeout_count;
    reg [7:0] mbr_type;
    reg [31:0] partition_lba, volume_base;
    reg [15:0] bytes_per_sector, reserved_sectors;
    reg [7:0] sectors_per_cluster, fat_count;
    reg [31:0] fat_size, root_cluster, fat_base, data_base;
    reg [31:0] root_sector;
    reg [7:0] root_sector_index;

    reg [7:0] dir_name [0:10];
    reg [7:0] dir_attr;
    reg [15:0] dir_cluster_hi, dir_cluster_lo;
    reg [31:0] dir_file_size;
    reg file_found;
    reg [31:0] file_cluster, file_size, file_first_sector;
    reg subtitle_file_found;
    reg [31:0] subtitle_file_size, subtitle_first_sector, subtitle_file_cluster;
    reg [3:0] subtitle_sector_index;
    reg [7:0] subtitle_sector_in_cluster;
    reg [31:0] subtitle_sector_count;
    reg [31:0] subtitle_magic, subtitle_sample_rate;
    reg [7:0] subtitle_glyph_count, subtitle_max_chars;

    reg [2:0] header_phase;
    reg [1:0] id_count, size_count;
    reg [31:0] chunk_id, chunk_size, chunk_remaining, chunk_payload_pos;
    reg [31:0] header_sector_index;
    reg [31:0] data_offset, data_size;
    reg fmt_found, data_found;
    reg [15:0] wav_format, wav_channels, wav_bits;
    reg [31:0] wav_sample_rate;

    // A normal PCM WAV places "fmt " at byte 12.  Keep an independent
    // absolute-offset decoder for that common layout.  The chunk walker
    // below is still used for files containing arbitrary RIFF chunks.
    reg [31:0] fixed_fmt_id, fixed_fmt_size, fixed_data_id, fixed_data_size;
    reg fixed_fmt_found, fixed_data_found;
    reg header_signature_bad;
    reg [31:0] riff_id, wave_id;
    reg [31:0] direct_data_id, direct_data_size;

    reg [31:0] stream_sector, stream_skip, bytes_left;
    reg [7:0] sector_in_cluster;
    reg [31:0] current_cluster;
    reg [1:0] sample_byte_index;
    reg [15:0] left_sample, right_sample;
    reg [8:0] fat_offset;
    reg [7:0] fat_b0, fat_b1, fat_b2, fat_b3;
    reg fat_read_for_subtitle;
    reg seek_active;
    reg [31:0] seek_sectors;
    wire [31:0] resume_frames = (resume_sample < ((wav_channels==1)?(data_size>>1):(data_size>>2))) ? resume_sample : 32'd0;
    wire [31:0] resume_bytes = (wav_channels==1) ? (resume_frames<<1) : (resume_frames<<2);
    wire [31:0] resume_offset = data_offset+resume_bytes;
    wire [31:0] fat_next = {fat_b3,fat_b2,fat_b1,fat_b0} & 32'h0fffffff;

    // music_index is a zero-based selector (0,1,2), while the FAT name is
    // one-based (01,02,03).  Compare the ASCII tens/ones fields explicitly
    // so no unsized arithmetic or character-width conversion can turn 02/03
    // into a false match.
    wire music_basename = ((dir_name[0]=="M")||(dir_name[0]=="m")) &&
                          ((dir_name[1]=="U")||(dir_name[1]=="u")) &&
                          ((dir_name[2]=="S")||(dir_name[2]=="s")) &&
                          ((dir_name[3]=="I")||(dir_name[3]=="i")) &&
                          ((dir_name[4]=="C")||(dir_name[4]=="c"));
    wire video_basename = ((dir_name[0]=="V")||(dir_name[0]=="v")) &&
                          ((dir_name[1]=="I")||(dir_name[1]=="i")) &&
                          ((dir_name[2]=="D")||(dir_name[2]=="d")) &&
                          ((dir_name[3]=="E")||(dir_name[3]=="e")) &&
                          ((dir_name[4]=="O")||(dir_name[4]=="o"));
    wire target_basename = (music_index<MAX_MUSIC_NUM) &&
                           (audio_source_video ? video_basename : music_basename) &&
                           dir_name[5]=="_" && dir_name[6]==8'h30 &&
                           ((music_index==2'd0 && dir_name[7]==8'h31) ||
                            (music_index==2'd1 && dir_name[7]==8'h32) ||
                            (music_index==2'd2 && dir_name[7]==8'h33));
    // FAT short names are normally upper-case, but cards created by some
    // host tools preserve lower-case characters.  Accept both spellings;
    // the basename comparison below already does the same.
    wire wav_extension = ((dir_name[8]=="W")||(dir_name[8]=="w")) &&
                         ((dir_name[9]=="A")||(dir_name[9]=="a")) &&
                         ((dir_name[10]=="V")||(dir_name[10]=="v"));
    wire sub_extension = ((dir_name[8]=="S")||(dir_name[8]=="s")) &&
                         ((dir_name[9]=="U")||(dir_name[9]=="u")) &&
                         ((dir_name[10]=="B")||(dir_name[10]=="b"));
    wire valid_dir_entry = (dir_attr!=8'h0f) && !dir_attr[3] && !dir_attr[4] &&
                           (dir_name[0]!=8'h00) && (dir_name[0]!=8'he5);
    wire valid_wav_entry = valid_dir_entry && target_basename && wav_extension;
    wire valid_sub_entry = valid_dir_entry && target_basename && sub_extension;
    wire fifo_has_sector_space = (fifo_wrusedw <= 9'd240);
    integer i;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state<=ST_WAIT; byte_count<=0; timeout_count<=0;
            sd_sec_read<=0; sd_sec_read_addr<=0; fifo_wr_en<=0; fifo_wr_data<=0;
            audio_ready<=0; audio_playing<=0; status<=0; audio_total_samples<=0;
            subtitle_wr_en<=0; subtitle_wr_addr<=0; subtitle_wr_data<=0;
            subtitle_ready<=0; subtitle_line_count<=0; subtitle_status<=0;
            mbr_type<=0; partition_lba<=0; volume_base<=0;
            bytes_per_sector<=0; reserved_sectors<=0; sectors_per_cluster<=0;
            fat_count<=0; fat_size<=0; root_cluster<=0; fat_base<=0; data_base<=0;
            root_sector<=0; root_sector_index<=0; dir_attr<=0;
            dir_cluster_hi<=0; dir_cluster_lo<=0; dir_file_size<=0; file_found<=0;
            file_cluster<=0; file_size<=0; file_first_sector<=0;
            subtitle_file_found<=0; subtitle_file_size<=0; subtitle_first_sector<=0; subtitle_file_cluster<=0;
            subtitle_sector_index<=0; subtitle_sector_in_cluster<=0; subtitle_sector_count<=0;
            subtitle_magic<=0; subtitle_sample_rate<=0;
            subtitle_glyph_count<=0; subtitle_max_chars<=0;
            header_phase<=0; id_count<=0; size_count<=0; chunk_id<=0; chunk_size<=0;
            chunk_remaining<=0; chunk_payload_pos<=0; header_sector_index<=0;
            data_offset<=0; data_size<=0; fmt_found<=0; data_found<=0;
            wav_format<=0; wav_channels<=0; wav_bits<=0; wav_sample_rate<=0;
            fixed_fmt_id<=0; fixed_fmt_size<=0; fixed_data_id<=0; fixed_data_size<=0;
            fixed_fmt_found<=0; fixed_data_found<=0;
            header_signature_bad<=0;
            riff_id<=0; wave_id<=0;
            direct_data_id<=0; direct_data_size<=0;
            stream_sector<=0; stream_skip<=0; bytes_left<=0; sector_in_cluster<=0;
            current_cluster<=0; sample_byte_index<=0; left_sample<=0; right_sample<=0;
            fat_offset<=0; fat_b0<=0; fat_b1<=0; fat_b2<=0; fat_b3<=0; fat_read_for_subtitle<=0;
            seek_active<=0;seek_sectors<=0;
            for(i=0;i<11;i=i+1) dir_name[i]<=0;
        end else begin
            fifo_wr_en <= 1'b0;
            subtitle_wr_en <= 1'b0;
            if (!sd_init_done) begin
                state<=ST_WAIT; sd_sec_read<=0; audio_ready<=0; audio_playing<=0;
                status<=0; timeout_count<=0; file_found<=0; audio_total_samples<=0;
                subtitle_file_found<=0; subtitle_ready<=0; subtitle_status<=0;
            end else begin
                if (state!=ST_WAIT && state!=ST_STREAM_WAIT && state!=ST_RESTART && state!=ST_ERROR) begin
                    if (timeout_count<TIMEOUT_CYCLES) timeout_count<=timeout_count+1'b1;
                    else begin state<=ST_ERROR; status<=4'hf; sd_sec_read<=0; audio_playing<=0; end
                end else timeout_count<=0;

                case(state)
                    ST_WAIT: begin
                        sd_sec_read<=0; byte_count<=0; status<=0;
                        partition_lba<=0; mbr_type<=0;
                        sd_sec_read_addr<=0; state<=ST_MBR;
                    end
                    ST_MBR: begin
                        sd_sec_read<=1;
                        if(sd_sec_read_data_valid) begin
                            case(byte_count)
                                10'd450:mbr_type<=sd_sec_read_data;
                                10'd454:partition_lba[7:0]<=sd_sec_read_data;
                                10'd455:partition_lba[15:8]<=sd_sec_read_data;
                                10'd456:partition_lba[23:16]<=sd_sec_read_data;
                                10'd457:partition_lba[31:24]<=sd_sec_read_data;
                                default:;
                            endcase
                            byte_count<=byte_count+1'b1;
                        end
                        if(sd_sec_read_end) begin
                            sd_sec_read<=0; byte_count<=0;
                            if(mbr_type==8'h0b || mbr_type==8'h0c || mbr_type==8'h1b || mbr_type==8'h1c) begin
                                volume_base<=partition_lba; sd_sec_read_addr<=partition_lba;
                            end else begin volume_base<=0; sd_sec_read_addr<=0; end
                            state<=ST_BPB;
                        end
                    end
                    ST_BPB: begin
                        sd_sec_read<=1;
                        if(sd_sec_read_data_valid) begin
                            case(byte_count)
                                11:bytes_per_sector[7:0]<=sd_sec_read_data;
                                12:bytes_per_sector[15:8]<=sd_sec_read_data;
                                13:sectors_per_cluster<=sd_sec_read_data;
                                14:reserved_sectors[7:0]<=sd_sec_read_data;
                                15:reserved_sectors[15:8]<=sd_sec_read_data;
                                16:fat_count<=sd_sec_read_data;
                                36:fat_size[7:0]<=sd_sec_read_data;
                                37:fat_size[15:8]<=sd_sec_read_data;
                                38:fat_size[23:16]<=sd_sec_read_data;
                                39:fat_size[31:24]<=sd_sec_read_data;
                                44:root_cluster[7:0]<=sd_sec_read_data;
                                45:root_cluster[15:8]<=sd_sec_read_data;
                                46:root_cluster[23:16]<=sd_sec_read_data;
                                47:root_cluster[31:24]<=sd_sec_read_data;
                                default:;
                            endcase
                            byte_count<=byte_count+1'b1;
                        end
                        if(sd_sec_read_end) begin
                            sd_sec_read<=0; byte_count<=0;
                            if(bytes_per_sector!=512 || sectors_per_cluster==0 || fat_count==0 || fat_size==0 || root_cluster<2) begin
                                status<=4'he; state<=ST_ERROR;
                            end else begin
                                fat_base<=volume_base+reserved_sectors;
                                data_base<=volume_base+reserved_sectors+(fat_count*fat_size);
                                root_sector<=volume_base+reserved_sectors+(fat_count*fat_size)+((root_cluster-2)*sectors_per_cluster);
                                sd_sec_read_addr<=volume_base+reserved_sectors+(fat_count*fat_size)+((root_cluster-2)*sectors_per_cluster);
                                root_sector_index<=0; file_found<=0; subtitle_file_found<=0;
                                subtitle_ready<=0; subtitle_status<=0; state<=ST_DIR;
                            end
                        end
                    end
                    ST_DIR: begin
                        sd_sec_read<=1;
                        if(sd_sec_read_data_valid) begin
                            case(byte_count[4:0])
                                0:dir_name[0]<=sd_sec_read_data;1:dir_name[1]<=sd_sec_read_data;
                                2:dir_name[2]<=sd_sec_read_data;3:dir_name[3]<=sd_sec_read_data;
                                4:dir_name[4]<=sd_sec_read_data;5:dir_name[5]<=sd_sec_read_data;
                                6:dir_name[6]<=sd_sec_read_data;7:dir_name[7]<=sd_sec_read_data;
                                8:dir_name[8]<=sd_sec_read_data;9:dir_name[9]<=sd_sec_read_data;
                                10:dir_name[10]<=sd_sec_read_data;11:dir_attr<=sd_sec_read_data;
                                20:dir_cluster_hi[7:0]<=sd_sec_read_data;
                                21:dir_cluster_hi[15:8]<=sd_sec_read_data;
                                26:dir_cluster_lo[7:0]<=sd_sec_read_data;
                                27:dir_cluster_lo[15:8]<=sd_sec_read_data;
                                28:dir_file_size[7:0]<=sd_sec_read_data;
                                29:dir_file_size[15:8]<=sd_sec_read_data;
                                30:dir_file_size[23:16]<=sd_sec_read_data;
                                31:begin
                                    dir_file_size[31:24]<=sd_sec_read_data;
                                    if(valid_wav_entry && !file_found) begin
                                        file_found<=1;
                                        file_cluster<={dir_cluster_hi,dir_cluster_lo};
                                        file_size<={sd_sec_read_data,dir_file_size[23:0]};
                                        file_first_sector<=data_base+(({dir_cluster_hi,dir_cluster_lo}-2)*sectors_per_cluster);
                                    end
                                    if(valid_sub_entry && !subtitle_file_found) begin
                                        subtitle_file_found<=1;
                                        subtitle_file_size<={sd_sec_read_data,dir_file_size[23:0]};
                                        subtitle_file_cluster<={dir_cluster_hi,dir_cluster_lo};
                                        subtitle_first_sector<=data_base+(({dir_cluster_hi,dir_cluster_lo}-2)*sectors_per_cluster);
                                    end
                                end
                                default:;
                            endcase
                            byte_count<=byte_count+1'b1;
                        end
                        if(sd_sec_read_end) begin
                            sd_sec_read<=0; byte_count<=0;
                            state<=ST_DIR_EVAL;
                        end
                    end
                    ST_DIR_EVAL: begin
                        sd_sec_read<=0;
                        if(file_found && subtitle_file_found) begin
                            if(subtitle_file_size!=0 && subtitle_file_size<=32'd8192) begin
                                subtitle_wr_addr<=0; subtitle_sector_index<=0;
                                subtitle_sector_in_cluster<=0;
                                subtitle_sector_count<=((subtitle_file_size+32'd511)>>9);
                                subtitle_magic<=0; subtitle_sample_rate<=0;
                                subtitle_line_count<=0; subtitle_glyph_count<=0;
                                subtitle_max_chars<=0; sd_sec_read_addr<=subtitle_first_sector;
                                state<=ST_SUB;
                            end else begin
                                subtitle_status<=4'he;
                                sd_sec_read_addr<=file_first_sector; header_sector_index<=0;
                                header_phase<=0; id_count<=0; size_count<=0; chunk_id<=0;
                                fmt_found<=0; data_found<=0;
                                fixed_fmt_id<=0; fixed_fmt_size<=0;
                                fixed_data_id<=0; fixed_data_size<=0;
                                fixed_fmt_found<=0; fixed_data_found<=0;
                                header_signature_bad<=0; status<=0;
                                riff_id<=0; wave_id<=0;
                                direct_data_id<=0; direct_data_size<=0;
                                state<=ST_HDR;
                            end
                        end else if(root_sector_index>=sectors_per_cluster-1) begin
                            if(!file_found) begin status<=4'hc; state<=ST_ERROR; end
                            else begin
                                subtitle_status<=4'hd;
                                sd_sec_read_addr<=file_first_sector; header_sector_index<=0;
                                header_phase<=0; id_count<=0; size_count<=0; chunk_id<=0;
                                fmt_found<=0; data_found<=0;
                                fixed_fmt_id<=0; fixed_fmt_size<=0;
                                fixed_data_id<=0; fixed_data_size<=0;
                                fixed_fmt_found<=0; fixed_data_found<=0;
                                header_signature_bad<=0; status<=0;
                                riff_id<=0; wave_id<=0;
                                direct_data_id<=0; direct_data_size<=0;
                                state<=ST_HDR;
                            end
                        end else begin
                            root_sector_index<=root_sector_index+1'b1;
                            sd_sec_read_addr<=root_sector+root_sector_index+1'b1;
                            state<=ST_DIR;
                        end
                    end
                    ST_SUB: begin
                        sd_sec_read<=1;
                        if(sd_sec_read_data_valid) begin
                            subtitle_wr_en<=1'b1;
                            subtitle_wr_data<=sd_sec_read_data;
                            case(subtitle_wr_addr)
                                13'd0:subtitle_magic[7:0]<=sd_sec_read_data;
                                13'd1:subtitle_magic[15:8]<=sd_sec_read_data;
                                13'd2:subtitle_magic[23:16]<=sd_sec_read_data;
                                13'd3:subtitle_magic[31:24]<=sd_sec_read_data;
                                13'd4:subtitle_line_count<=sd_sec_read_data[5:0];
                                13'd5:subtitle_glyph_count<=sd_sec_read_data;
                                13'd6:subtitle_max_chars<=sd_sec_read_data;
                                13'd8:subtitle_sample_rate[7:0]<=sd_sec_read_data;
                                13'd9:subtitle_sample_rate[15:8]<=sd_sec_read_data;
                                13'd10:subtitle_sample_rate[23:16]<=sd_sec_read_data;
                                13'd11:subtitle_sample_rate[31:24]<=sd_sec_read_data;
                                default:;
                            endcase
                            subtitle_wr_addr<=subtitle_wr_addr+1'b1;
                        end
                        if(sd_sec_read_end) begin
                            sd_sec_read<=0;
                            // subtitle_sector_index is 4 bits (the RAM holds at
                            // most 16 sectors).  Compare against the 32-bit
                            // count without allowing 15+1 to wrap to zero.
                            if(({28'd0,subtitle_sector_index}+32'd1)>=subtitle_sector_count)
                                state<=ST_SUB_EVAL;
                            else begin
                                subtitle_sector_index<=subtitle_sector_index+1'b1;
                                if(subtitle_sector_in_cluster+1'b1>=sectors_per_cluster) begin
                                    // Follow the subtitle file's FAT chain before
                                    // requesting the next sector.
                                    fat_read_for_subtitle<=1'b1;
                                    fat_offset<=(subtitle_file_cluster*4)&9'h1ff;
                                    fat_b0<=0; fat_b1<=0; fat_b2<=0; fat_b3<=0;
                                    byte_count<=0;
                                    sd_sec_read_addr<=fat_base+((subtitle_file_cluster*4)>>9);
                                    state<=ST_FAT;
                                end else begin
                                    subtitle_sector_in_cluster<=subtitle_sector_in_cluster+1'b1;
                                    sd_sec_read_addr<=sd_sec_read_addr+1'b1;
                                end
                            end
                        end
                    end
                    ST_SUB_EVAL: begin
                        sd_sec_read<=0;
                        fat_read_for_subtitle<=1'b0;
                        if(subtitle_magic==32'h31425553 && subtitle_line_count!=0 &&
                           subtitle_line_count<=6'd60 && subtitle_glyph_count<=8'd192 &&
                           subtitle_max_chars==8'd20 && subtitle_sample_rate==32'd48000) begin
                            subtitle_ready<=1'b1;
                            subtitle_status<=4'd1;
                        end else begin
                            subtitle_ready<=1'b0;
                            subtitle_status<=4'he;
                        end
                        sd_sec_read_addr<=file_first_sector; header_sector_index<=0;
                        header_phase<=0; id_count<=0; size_count<=0; chunk_id<=0;
                        fmt_found<=0; data_found<=0;
                        fixed_fmt_id<=0; fixed_fmt_size<=0;
                        fixed_data_id<=0; fixed_data_size<=0;
                        fixed_fmt_found<=0; fixed_data_found<=0;
                        header_signature_bad<=0; status<=0;
                        riff_id<=0; wave_id<=0;
                        direct_data_id<=0; direct_data_size<=0;
                        state<=ST_HDR;
                    end
                    ST_HDR: begin
                        sd_sec_read<=1;
                        if(sd_sec_read_data_valid) begin
                            // Fixed RIFF/WAVE identification bytes.
                            if(header_sector_index==0) begin
                                // Check the RIFF/WAVE signature numerically.
                                // Do not drive the public status register while
                                // bytes are still arriving: the old code could
                                // leave a transient false status A latched into
                                // ST_HDR_EVAL when the SD response boundary
                                // coincided with the last header byte.
                                case(byte_count)
                                    0: riff_id[7:0]<=sd_sec_read_data;
                                    1: riff_id[15:8]<=sd_sec_read_data;
                                    2: riff_id[23:16]<=sd_sec_read_data;
                                    3: riff_id[31:24]<=sd_sec_read_data;
                                    8: wave_id[7:0]<=sd_sec_read_data;
                                    9: wave_id[15:8]<=sd_sec_read_data;
                                    10: wave_id[23:16]<=sd_sec_read_data;
                                    11: wave_id[31:24]<=sd_sec_read_data;
                                    // Standard PCM WAVs used by the card have
                                    // either RIFF/fmt/data (data id at byte 36,
                                    // size at 40) or RIFF/fmt/LIST/data (data
                                    // id at byte 70, size at 74).  The parser's
                                    // streaming chunk walker remains active,
                                    // but these explicit probes make data
                                    // discovery independent of byte-counter
                                    // state across SD response timing.
                                    36: direct_data_id[7:0]<=sd_sec_read_data;
                                    37: direct_data_id[15:8]<=sd_sec_read_data;
                                    38: direct_data_id[23:16]<=sd_sec_read_data;
                                    39: direct_data_id[31:24]<=sd_sec_read_data;
                                    40: direct_data_size[7:0]<=sd_sec_read_data;
                                    41: direct_data_size[15:8]<=sd_sec_read_data;
                                    42: direct_data_size[23:16]<=sd_sec_read_data;
                                    43: begin
                                        direct_data_size[31:24]<=sd_sec_read_data;
                                        if(direct_data_id==CHUNK_DATA) begin
                                            data_found<=1'b1;
                                            data_size<={sd_sec_read_data,direct_data_size[23:0]};
                                            data_offset<=32'd44;
                                        end
                                    end
                                    70: direct_data_id[7:0]<=sd_sec_read_data;
                                    71: direct_data_id[15:8]<=sd_sec_read_data;
                                    72: direct_data_id[23:16]<=sd_sec_read_data;
                                    73: direct_data_id[31:24]<=sd_sec_read_data;
                                    74: direct_data_size[7:0]<=sd_sec_read_data;
                                    75: direct_data_size[15:8]<=sd_sec_read_data;
                                    76: direct_data_size[23:16]<=sd_sec_read_data;
                                    77: begin
                                        direct_data_size[31:24]<=sd_sec_read_data;
                                        if(direct_data_id==CHUNK_DATA) begin
                                            data_found<=1'b1;
                                            data_size<={sd_sec_read_data,direct_data_size[23:0]};
                                            data_offset<=32'd78;
                                        end
                                    end
                                    default:;
                                endcase

                                // Independent decoder for the conventional
                                // RIFF -> fmt -> optional LIST -> data layout.
                                case(byte_count)
                                    12: fixed_fmt_id[7:0]<=sd_sec_read_data;
                                    13: fixed_fmt_id[15:8]<=sd_sec_read_data;
                                    14: fixed_fmt_id[23:16]<=sd_sec_read_data;
                                    15: fixed_fmt_id[31:24]<=sd_sec_read_data;
                                    16: fixed_fmt_size[7:0]<=sd_sec_read_data;
                                    17: fixed_fmt_size[15:8]<=sd_sec_read_data;
                                    18: fixed_fmt_size[23:16]<=sd_sec_read_data;
                                    19: fixed_fmt_size[31:24]<=sd_sec_read_data;
                                    20: wav_format[7:0]<=sd_sec_read_data;
                                    21: wav_format[15:8]<=sd_sec_read_data;
                                    22: wav_channels[7:0]<=sd_sec_read_data;
                                    23: wav_channels[15:8]<=sd_sec_read_data;
                                    24: wav_sample_rate[7:0]<=sd_sec_read_data;
                                    25: wav_sample_rate[15:8]<=sd_sec_read_data;
                                    26: wav_sample_rate[23:16]<=sd_sec_read_data;
                                    27: wav_sample_rate[31:24]<=sd_sec_read_data;
                                    34: wav_bits[7:0]<=sd_sec_read_data;
                                    35: begin
                                        wav_bits[15:8]<=sd_sec_read_data;
                                        if(fixed_fmt_id==CHUNK_FMT && fixed_fmt_size>=16)
                                            fixed_fmt_found<=1'b1;
                                    end
                                    // Do not decode the data chunk at a fixed
                                    // offset here. MUSIC_01 contains a LIST
                                    // chunk and therefore starts its payload at
                                    // byte 78, while MUSIC_02/MUSIC_03 have a
                                    // conventional data chunk at byte 44.
                                    // The chunk walker below is format-aware and
                                    // records the actual data offset/size for
                                    // both layouts. Keeping the old 70..77
                                    // check caused the no-LIST files to be
                                    // falsely marked with status A.
                                    default:;
                                endcase
                            end
                            if(header_sector_index!=0 || byte_count>=12) begin
                                case(header_phase)
                                    0: begin
                                        chunk_id<={sd_sec_read_data,chunk_id[31:8]};
                                        if(id_count==3) begin id_count<=0; size_count<=0; chunk_size<=0; header_phase<=1; end
                                        else id_count<=id_count+1'b1;
                                    end
                                    1: begin
                                        chunk_size<={sd_sec_read_data,chunk_size[31:8]};
                                        if(size_count==3) begin
                                            size_count<=0; chunk_payload_pos<=0;
                                            chunk_remaining<={sd_sec_read_data,chunk_size[31:8]};
                                            if(chunk_id==CHUNK_DATA) begin
                                                data_found<=1; data_size<={sd_sec_read_data,chunk_size[31:8]};
                                                data_offset<=(header_sector_index<<9)+byte_count+1'b1;
                                            end
                                            header_phase<=({sd_sec_read_data,chunk_size[31:8]}==0) ? 0 : 2;
                                        end else size_count<=size_count+1'b1;
                                    end
                                    2: begin
                                        if(chunk_id==CHUNK_FMT) begin
                                            case(chunk_payload_pos)
                                                0:wav_format[7:0]<=sd_sec_read_data;
                                                1:wav_format[15:8]<=sd_sec_read_data;
                                                2:wav_channels[7:0]<=sd_sec_read_data;
                                                3:wav_channels[15:8]<=sd_sec_read_data;
                                                4:wav_sample_rate[7:0]<=sd_sec_read_data;
                                                5:wav_sample_rate[15:8]<=sd_sec_read_data;
                                                6:wav_sample_rate[23:16]<=sd_sec_read_data;
                                                7:wav_sample_rate[31:24]<=sd_sec_read_data;
                                                14:wav_bits[7:0]<=sd_sec_read_data;
                                                15:begin wav_bits[15:8]<=sd_sec_read_data; fmt_found<=1; end
                                                default:;
                                            endcase
                                        end
                                        chunk_payload_pos<=chunk_payload_pos+1'b1;
                                        if(chunk_remaining<=1) begin
                                            // RIFF chunks with odd payload size have one pad byte.
                                            header_phase<=chunk_size[0] ? 3 : 0;
                                            id_count<=0; chunk_id<=0;
                                        end else chunk_remaining<=chunk_remaining-1'b1;
                                    end
                                    3: header_phase<=0;
                                    default:header_phase<=0;
                                endcase
                            end
                            byte_count<=byte_count+1'b1;
                        end
                        if(sd_sec_read_end) begin
                            sd_sec_read<=0; byte_count<=0;
                            if(data_found) state<=ST_HDR_EVAL;
                            else if(header_sector_index>=HEADER_MAX_SECTORS-1) begin status<=4'ha; state<=ST_ERROR; end
                            else begin header_sector_index<=header_sector_index+1'b1; sd_sec_read_addr<=sd_sec_read_addr+1'b1; end
                        end
                    end
                    ST_HDR_EVAL: begin
                        // Detailed format status makes a board-only failure
                        // diagnosable from the audio status digit.
                        if(riff_id!=32'h46464952 || wave_id!=32'h45564157) begin
                            status<=4'h2; state<=ST_ERROR; // RIFF/WAVE signature
                        end else if(!(fmt_found || fixed_fmt_found)) begin
                            status<=4'h3; state<=ST_ERROR; // no usable fmt chunk
                        end else if(!(data_found || fixed_data_found)) begin
                            status<=4'h4; state<=ST_ERROR; // no data chunk
                        end else if(wav_format!=1) begin
                            status<=4'h5; state<=ST_ERROR; // not integer PCM
                        end else if(wav_channels!=1 && wav_channels!=2) begin
                            status<=4'h6; state<=ST_ERROR; // channel count
                        end else if(wav_sample_rate!=48000) begin
                            status<=4'h7; state<=ST_ERROR; // sample rate
                        end else if(wav_bits!=16) begin
                            status<=4'h8; state<=ST_ERROR; // sample width
                        end else if((wav_channels==1 && data_size[0]!=0) ||
                                    (wav_channels==2 && data_size[1:0]!=0) ||
                                    data_size==0 || data_offset>=file_size ||
                                    data_size>file_size-data_offset) begin
                            status<=4'h9; state<=ST_ERROR; // size/alignment
                        end else begin
                            audio_ready<=1; audio_playing<=1; status<=4'd1;
                            audio_total_samples <= (wav_channels==1) ? (data_size>>1) : (data_size>>2);
                            stream_sector<=file_first_sector; sd_sec_read_addr<=file_first_sector;
                            stream_skip<={23'd0,resume_offset[8:0]}; bytes_left<=data_size-resume_bytes;
                            seek_sectors<=resume_offset>>9;seek_active<=1;
                            current_cluster<=file_cluster; sector_in_cluster<=0;
                            sample_byte_index<=0; state<=ST_SEEK;
                        end
                    end
                    ST_SEEK: begin
                        // Skip whole clusters through FAT, not by reading and
                        // discarding every PCM byte. Works for fragmented files.
                        sd_sec_read<=0;
                        if(seek_sectors>=sectors_per_cluster)begin
                            seek_sectors<=seek_sectors-sectors_per_cluster;
                            fat_read_for_subtitle<=0;
                            fat_offset<=(current_cluster*4)&9'h1ff;
                            fat_b0<=0;fat_b1<=0;fat_b2<=0;fat_b3<=0;byte_count<=0;
                            sd_sec_read_addr<=fat_base+((current_cluster*4)>>9);state<=ST_FAT;
                        end else begin
                            stream_sector<=stream_sector+seek_sectors;
                            sector_in_cluster<=seek_sectors[7:0];seek_active<=0;
                            state<=ST_STREAM_WAIT;
                        end
                    end
                    ST_STREAM_WAIT: begin
                        sd_sec_read<=0;
                        // A pause is a transport hold, not a parser reset.  Do
                        // not issue another sector request while paused; the
                        // current FIFO contents and stream position remain
                        // intact and resume continues at the same sample.
                        if(!pause_playback && fifo_has_sector_space) begin
                            sd_sec_read_addr<=stream_sector;
                            state<=ST_STREAM;
                        end
                    end
                    ST_STREAM: begin
                        // A requested sector is indivisible.  Finish accepting
                        // all its bytes even when pause/restart arrives midway;
                        // ST_STREAM_WAIT prevents the NEXT sector being issued.
                        // Dropping bytes here and rereading this sector on resume
                        // corrupts the PCM position and the partial sample word.
                        sd_sec_read<=1'b1;
                        if(sd_sec_read_data_valid) begin
                            if(stream_skip!=0) stream_skip<=stream_skip-1'b1;
                            else if(bytes_left!=0) begin
                                if(wav_channels==1) begin
                                    if(sample_byte_index==0) begin left_sample[7:0]<=sd_sec_read_data; sample_byte_index<=1; end
                                    else begin
                                        left_sample[15:8]<=sd_sec_read_data; sample_byte_index<=0;
                                        fifo_wr_data<={{sd_sec_read_data,left_sample[7:0]},{sd_sec_read_data,left_sample[7:0]}};
                                        fifo_wr_en<=1; bytes_left<=bytes_left-2;
                                    end
                                end else begin
                                    case(sample_byte_index)
                                        0:begin left_sample[7:0]<=sd_sec_read_data; sample_byte_index<=1; end
                                        1:begin left_sample[15:8]<=sd_sec_read_data; sample_byte_index<=2; end
                                        2:begin right_sample[7:0]<=sd_sec_read_data; sample_byte_index<=3; end
                                        default:begin
                                            right_sample[15:8]<=sd_sec_read_data; sample_byte_index<=0;
                                            fifo_wr_data<={left_sample,{sd_sec_read_data,right_sample[7:0]}};
                                            fifo_wr_en<=1; bytes_left<=bytes_left-4;
                                        end
                                    endcase
                                end
                            end
                        end
                        if(sd_sec_read_end) begin
                            sd_sec_read<=0;
                            if(bytes_left==0) state<=ST_RESTART;
                            else if(sector_in_cluster+1>=sectors_per_cluster) begin
                                fat_read_for_subtitle<=1'b0;
                                fat_offset<=(current_cluster*4)&9'h1ff;
                                fat_b0<=0;fat_b1<=0;fat_b2<=0;fat_b3<=0;byte_count<=0;
                                sd_sec_read_addr<=fat_base+((current_cluster*4)>>9); state<=ST_FAT;
                            end else begin
                                sector_in_cluster<=sector_in_cluster+1'b1;
                                stream_sector<=stream_sector+1'b1; state<=ST_STREAM_WAIT;
                            end
                        end
                    end
                    ST_FAT: begin
                        sd_sec_read<=1;
                        if(sd_sec_read_data_valid) begin
                            if(byte_count==fat_offset) fat_b0<=sd_sec_read_data;
                            if(byte_count==fat_offset+1) fat_b1<=sd_sec_read_data;
                            if(byte_count==fat_offset+2) fat_b2<=sd_sec_read_data;
                            if(byte_count==fat_offset+3) fat_b3<=sd_sec_read_data;
                            byte_count<=byte_count+1'b1;
                        end
                        if(sd_sec_read_end) begin sd_sec_read<=0; byte_count<=0; state<=ST_FAT_EVAL; end
                    end
                    ST_FAT_EVAL: begin
                        if(fat_next<2 || fat_next>=32'h0ffffff8) begin
                            status<=4'hb; audio_playing<=0; state<=ST_ERROR;
                        end else if(fat_read_for_subtitle) begin
                            subtitle_file_cluster<=fat_next;
                            subtitle_sector_in_cluster<=0;
                            sd_sec_read_addr<=data_base+((fat_next-2)*sectors_per_cluster);
                            state<=ST_SUB;
                        end else begin
                            current_cluster<=fat_next; sector_in_cluster<=0;
                            // Do not assume adjacent clusters.  Recompute the
                            // first data sector from the FAT successor.
                            stream_sector<=data_base+((fat_next-2)*sectors_per_cluster);
                            sd_sec_read_addr<=data_base+((fat_next-2)*sectors_per_cluster);
                            state<=seek_active ? ST_SEEK : ST_STREAM_WAIT;
                        end
                    end
                    ST_RESTART: begin
                        // Loop the selected WAV file continuously.
                        stream_sector<=file_first_sector; sd_sec_read_addr<=file_first_sector;
                        stream_skip<=data_offset; bytes_left<=data_size;
                        current_cluster<=file_cluster; sector_in_cluster<=0;
                        sample_byte_index<=0; state<=ST_STREAM_WAIT;
                    end
                    ST_ERROR: begin sd_sec_read<=0; audio_playing<=0; end
                    default:state<=ST_WAIT;
                endcase
            end
        end
    end
endmodule

// I2S transmitter compatible with the existing HDMI audio receiver. FIFO
// words are {left[15:0], right[15:0]} at 48 kHz.
module hdmi_audio_pcm_i2s_64fs(
    input wire I_mclk, input wire I_rst,
    input wire I_pause,
    output reg O_pause_ack,
    input wire [7:0] I_volume,
    input wire [31:0] I_fifo_data, input wire I_fifo_empty,
    output reg O_fifo_rd_en,
    output reg O_i2s_BCLK, output reg O_i2s_LRCK, output reg O_i2s_DOUT
);
    reg [5:0] bit_count;
    reg [63:0] shift_reg;
    reg [31:0] sample_word;
    reg [1:0] pause_sync;
    reg [7:0] volume_meta,volume_sync,volume_previous,volume_stable,volume_pair;
    // HDMI samples are 24-bit signed PCM.  Converting 16-bit PCM by sign
    // extension alone would leave it 48 dB below the 24-bit full scale.
    // Preserve the waveform and its original dBFS level by shifting left 8.
    // 0..16 represents 0..100%. Shift/add avoids consuming scarce DSPs.
    // Scaling signed 16-bit PCM by gain, then shifting 4, yields 24-bit PCM
    // with exactly unity gain at 16 and no overflow at either signed rail.
    function [23:0] scale_pcm;
        input [15:0] value;
        input [7:0] gain;
        reg signed [23:0] extended,product;
        integer bit_index;
        begin
            extended={{8{value[15]}},value}; product=0;
            for(bit_index=0;bit_index<5;bit_index=bit_index+1)
                if(gain[bit_index]) product=product+(extended<<<bit_index);
            scale_pcm=product<<<4;
        end
    endfunction
    wire [23:0] fifo_left = scale_pcm(I_fifo_data[31:16],volume_stable);
    wire [23:0] held_right = scale_pcm(sample_word[15:0],volume_pair);
    always @(posedge I_mclk or posedge I_rst) begin
        if(I_rst) begin
            O_fifo_rd_en<=0; O_i2s_BCLK<=0; O_i2s_LRCK<=0; O_i2s_DOUT<=0;
            bit_count<=0; shift_reg<=0; sample_word<=0;pause_sync<=0;
            O_pause_ack<=0;
            volume_meta<=16;volume_sync<=16;volume_previous<=16;volume_stable<=16;volume_pair<=16;
        end else begin
            pause_sync<={pause_sync[0],I_pause};
            if(!pause_sync[1])O_pause_ack<=0;
            volume_meta<=I_volume;volume_sync<=volume_meta;volume_previous<=volume_sync;
            if(volume_sync==volume_previous&&volume_sync<=16) volume_stable<=volume_sync;
            O_fifo_rd_en<=0; O_i2s_BCLK<=~O_i2s_BCLK;
            if(O_i2s_BCLK) begin
                O_i2s_DOUT<=shift_reg[63]; shift_reg<={shift_reg[62:0],1'b0};
                if(bit_count==63) begin
                    bit_count<=0;
                    if(!O_i2s_LRCK) begin
                        O_i2s_LRCK<=1; shift_reg<={held_right,40'd0};
                    end else begin
                        O_i2s_LRCK<=0;
                        // Keep I2S clocks/HDMI audio packets running with
                        // silence, but do not consume PCM or advance lyrics.
                        if(!pause_sync[1]&&!I_fifo_empty) begin
                            volume_pair<=volume_stable;
                            sample_word<=I_fifo_data; shift_reg<={fifo_left,40'd0}; O_fifo_rd_en<=1;
                        end else begin sample_word<=0; shift_reg<=0;O_pause_ack<=pause_sync[1];end
                    end
                end else bit_count<=bit_count+1'b1;
            end
        end
    end
endmodule
