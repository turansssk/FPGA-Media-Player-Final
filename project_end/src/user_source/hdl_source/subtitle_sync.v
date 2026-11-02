// Runtime subtitle synchronizer and 16x16 glyph renderer.
// The selected MUSIC_nn.SUB or VIDEO_nn.SUB is loaded by wav_read_fat32
// before its matching audio starts.  Timing is
// driven by actual FIFO sample consumption, so SD-card stalls cannot
// desynchronise text.
module subtitle_sync #(
    parameter integer MAX_LINES = 60,
    parameter integer MAX_CHARS = 20
)(
    input  wire        wr_clk,
    input  wire        wr_en,
    input  wire [12:0] wr_addr,
    input  wire [7:0]  wr_data,
    input  wire        clk,
    input  wire        rst,
    input  wire        subtitle_ready,
    input  wire [5:0]  subtitle_line_count,
    input  wire        audio_sample_advance,
    input  wire [31:0] audio_total_samples,
    input  wire [31:0] resume_sample,
    input  wire        video_vs,
    input  wire [9:0]  pixel_x,
    input  wire [8:0]  pixel_y,
    output wire        subtitle_pixel,
    output wire        subtitle_visible
);

reg [31:0] cue_start [0:MAX_LINES-1];
reg [31:0] cue_end   [0:MAX_LINES-1];
reg [4:0]  cue_len   [0:MAX_LINES-1];
reg [7:0]  cue_char  [0:MAX_LINES*MAX_CHARS-1];
reg [7:0]  glyph_hi  [0:3071];
reg [7:0]  glyph_lo  [0:3071];

// wav_read_fat32 updates subtitle_wr_data and increments subtitle_wr_addr in
// the same clocked process.  Because both modules use non-blocking assignments,
// this write port observes the previous byte together with the already
// incremented address.  Compensate by one address here.  The 13-bit subtraction
// intentionally wraps 0 to 8191, which also aligns the final byte of the 8 KiB
// subtitle file after the producer address counter wraps.
wire [12:0] wr_addr_aligned = wr_addr-13'd1;
wire [12:0] cue_offset = wr_addr_aligned-13'd32;
wire [5:0]  wr_cue_index = cue_offset[10:5];
wire [4:0]  wr_cue_field = cue_offset[4:0];
wire [10:0] wr_char_index = (wr_cue_index*MAX_CHARS)+(wr_cue_field-5'd9);
wire [11:0] wr_glyph_row = (wr_addr_aligned-13'd2048)>>1;

always @(posedge wr_clk) begin
    if(wr_en) begin
        if(wr_addr_aligned>=13'd32 && wr_addr_aligned<13'd1952 && wr_cue_index<MAX_LINES) begin
            case(wr_cue_field)
                0:cue_start[wr_cue_index][7:0]<=wr_data;
                1:cue_start[wr_cue_index][15:8]<=wr_data;
                2:cue_start[wr_cue_index][23:16]<=wr_data;
                3:cue_start[wr_cue_index][31:24]<=wr_data;
                4:cue_end[wr_cue_index][7:0]<=wr_data;
                5:cue_end[wr_cue_index][15:8]<=wr_data;
                6:cue_end[wr_cue_index][23:16]<=wr_data;
                7:cue_end[wr_cue_index][31:24]<=wr_data;
                8:cue_len[wr_cue_index]<=wr_data[4:0];
                default:if(wr_cue_field>=9 && wr_cue_field<29)
                    cue_char[wr_char_index]<=wr_data;
            endcase
        end
        if(wr_addr_aligned>=13'd2048) begin
            if(!wr_addr_aligned[0]) glyph_hi[wr_glyph_row]<=wr_data;
            else             glyph_lo[wr_glyph_row]<=wr_data;
        end
    end
end

reg [1:0] ready_sync;
reg [31:0] total_meta, total_sync;
reg [31:0] resume_meta,resume_sync;
reg [5:0] line_count_meta, line_count_sync;
reg [31:0] sample_count;
reg [5:0] current_line, displayed_line;
reg       displayed_active;
reg       vs_d;

wire current_active = ready_sync[1] && current_line<line_count_sync &&
                      sample_count>=cue_start[current_line] &&
                      sample_count<cue_end[current_line];

always @(posedge clk or posedge rst) begin
    if(rst) begin
        ready_sync<=0; total_meta<=0; total_sync<=0;resume_meta<=0;resume_sync<=0;
        line_count_meta<=0; line_count_sync<=0;
        sample_count<=0; current_line<=0; displayed_line<=0;
        displayed_active<=0; vs_d<=0;
    end else begin
        ready_sync<={ready_sync[0],subtitle_ready};
        total_meta<=audio_total_samples;
        total_sync<=total_meta;
        resume_meta<=resume_sample;resume_sync<=resume_meta;
        line_count_meta<=subtitle_line_count;
        line_count_sync<=line_count_meta;
        vs_d<=video_vs;

        if(!ready_sync[1] || total_sync==0) begin
            sample_count<=resume_sync;
            current_line<=0;
        end else if(audio_sample_advance) begin
            if(sample_count+1'b1>=total_sync) begin
                sample_count<=0;
                current_line<=0;
            end else begin
                sample_count<=sample_count+1'b1;
                if(current_line+1'b1<line_count_sync &&
                   sample_count+1'b1>=cue_end[current_line])
                    current_line<=current_line+1'b1;
            end
        end
        // Resume may land several cues into the song. Catch up without
        // consuming audio events; at most MAX_LINES pixel clocks are needed.
        if(ready_sync[1]&&current_line+1'b1<line_count_sync&&
           sample_count>=cue_end[current_line])current_line<=current_line+1'b1;
        if(audio_sample_advance&&total_sync!=0&&sample_count+1'b1>=total_sync)
            current_line<=0;

        // Subtitle changes become visible only at a video frame boundary.
        if(video_vs!=vs_d) begin
            displayed_line<=current_line;
            displayed_active<=current_active;
        end
    end
end

wire [4:0] displayed_len = cue_len[displayed_line];
wire [9:0] subtitle_start_x = 10'd320-({5'd0,displayed_len}<<3);
// Position only: cue timing, BRAM lookup and pixel-pipeline latency unchanged.
wire in_subtitle_box = displayed_active && pixel_y>=9'd464 && pixel_y<9'd480 &&
                       pixel_x>=subtitle_start_x &&
                       pixel_x<(subtitle_start_x+({5'd0,displayed_len}<<4));
wire [4:0] render_char = (pixel_x-subtitle_start_x)>>4;
wire [3:0] render_row = pixel_y-9'd464;
wire [3:0] render_col = pixel_x-subtitle_start_x;
wire [10:0] render_char_addr = (displayed_line*MAX_CHARS)+render_char;

// Both cue_char and the glyph arrays infer synchronous memories on EG4.
// Pipeline the two dependent reads explicitly.  The previous implementation
// fed the cue RAM output straight into the glyph BRAM address in one pixel
// cycle; after synthesis this left the renderer reading an invalid/old glyph
// even though the subtitle timing and background band were correct.
// The EG4 BRAM read ports have registered outputs.  Keep the complete
// request (character address, row, column and active flag) in a first stage,
// then explicitly pipeline the character lookup and glyph-row lookup.  This
// avoids relying on simulator/asynchronous-array semantics, which previously
// resulted in a valid subtitle band but no visible glyph pixels in hardware.
reg [10:0] cue_char_addr_q;
reg [3:0]  render_row_q, render_col_q;
reg        render_active_q;
reg [7:0]  glyph_id_q;
reg [3:0]  glyph_row_index_q, glyph_col_index_q;
reg        glyph_active_q;
wire [11:0] glyph_memory_addr = {glyph_id_q,4'b0000}+glyph_row_index_q;
reg [15:0] glyph_bitmap_q;
reg [3:0]  glyph_bit_index_q;
reg        glyph_pixel_active_q;
always @(posedge clk) begin
    // Stage 0: register the request presented by the current video pixel.
    cue_char_addr_q<=render_char_addr;
    render_row_q<=render_row;
    render_col_q<=render_col;
    render_active_q<=in_subtitle_box;

    // Stage 1: synchronous character-index lookup.
    glyph_id_q<=cue_char[cue_char_addr_q];
    glyph_row_index_q<=render_row_q;
    glyph_col_index_q<=render_col_q;
    glyph_active_q<=render_active_q;

    // Stage 2: synchronous 16-bit bitmap row lookup.
    glyph_bitmap_q[15:8]<=glyph_hi[glyph_memory_addr];
    glyph_bitmap_q[7:0]<=glyph_lo[glyph_memory_addr];
    glyph_bit_index_q<=4'd15-glyph_col_index_q;
    glyph_pixel_active_q<=glyph_active_q;
end

assign subtitle_pixel = glyph_pixel_active_q && glyph_bitmap_q[glyph_bit_index_q];
assign subtitle_visible = displayed_active;

endmodule
