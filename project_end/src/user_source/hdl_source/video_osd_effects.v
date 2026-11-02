// Lightweight HDMI post-processing for the 640x480 multimedia player.
// Adds a short, non-black fade-in for still images, a title/logo overlay, the
// committed media number, and an audio level meter without touching the
// SDRAM data path.
module video_osd_effects (
    input  wire        clk,
    input  wire        rst,
    input  wire        in_hs,
    input  wire        in_vs,
    input  wire        in_de,
    input  wire [23:0] in_rgb,
    input  wire        display_valid,
    input  wire        playback_paused,
    input  wire        frame_buffer_select,
    input  wire [3:0]  image_code,
    input  wire        video_mode,
    input  wire [2:0]  function_state,
    input  wire [1:0]  music_index,
    input  wire [7:0]  brightness_level,
    input  wire        brightness_ui,
    input  wire [7:0]  volume_level,
    input  wire        volume_ui,
    input  wire        system_info_ui,
    input  wire        minimal_osd,
    input  wire        sd_ready,
    input  wire        media_loading,
    input  wire [3:0]  media_status,
    input  wire        audio_loading,
    input  wire [3:0]  audio_status,
    input  wire [3:0]  sub_status,
    input  wire        audio_source_video,
    input  wire [1:0]  active_audio_index,
    input  wire        audio_held,
    input  wire        audio_ready,
    input  wire        sub_ready,
    input  wire        audio_valid,
    input  wire [23:0] audio_sample,
    input  wire        subtitle_pixel,
    input  wire        subtitle_visible,
    output wire [9:0]  current_pixel_x,
    output wire [8:0]  current_pixel_y,
    output reg         out_hs,
    output reg         out_vs,
    output reg         out_de,
    output reg  [23:0] out_rgb
);

reg        de_d, vs_d, frame_arm;
reg [9:0]  pixel_x;
reg [8:0]  pixel_y;
reg [1:0]  buffer_sync;
reg        displayed_buffer;
reg        display_valid_d;
reg [3:0]  fade_level;
reg [3:0]  image_code_meta, image_code_sync, displayed_image;
reg [1:0]  video_mode_sync;
reg        displayed_video_mode;
reg [7:0]  audio_peak;
reg [3:0]  audio_level;
// Slow key controls are synchronized and required to settle, then committed
// together at the first active line. No setting changes halfway down a frame.
reg [23:0] control_meta,control_sync,control_previous,control_stable,control_frame;
wire [2:0] shown_function=control_frame[23:21];
wire [1:0] shown_music=control_frame[20:19];
wire [7:0] shown_brightness=control_frame[18:11];
wire [7:0] shown_volume=control_frame[10:3];
wire shown_brightness_ui=control_frame[2];
wire shown_volume_ui=control_frame[1];
wire shown_paused=control_frame[0];
// Read-only diagnostics. Settle asynchronous buses before frame-atomic commit.
// No diagnostic or visibility signal feeds the media/reset controllers.
reg [22:0] diag_meta,diag_sync,diag_previous,diag_stable,diag_frame;
wire shown_info,shown_minimal,shown_sd,shown_loading,shown_audio_loading;
wire [3:0] shown_media_status,shown_audio_status,shown_sub_status;
wire shown_audio_source,shown_audio_held,shown_audio_ready,shown_sub_ready;
wire [1:0] shown_audio_index;
assign {shown_info,shown_minimal,shown_sd,shown_loading,shown_media_status,
        shown_audio_loading,shown_audio_status,shown_sub_status,
        shown_audio_source,shown_audio_index,shown_audio_held,
        shown_audio_ready,shown_sub_ready}=diag_frame;
wire info_open=shown_info&&(shown_function==4);
wire normal_osd=!shown_minimal;
wire media_error=(shown_media_status>=4'ha);
wire audio_error=(shown_audio_status>=2)&&!shown_audio_loading;
wire sub_error=(shown_sub_status>=4'hd)&&!shown_audio_loading;
wire any_error=media_error||audio_error||sub_error;

wire [7:0] audio_abs = audio_sample[23] ? (~audio_sample[23:16] + 1'b1) : audio_sample[23:16];
assign current_pixel_x = pixel_x;
assign current_pixel_y = pixel_y;

function [7:0] fade_channel;
    input [7:0] value;
    input [3:0] level;
    reg [11:0] product;
    begin
        if (level >= 4'd8)
            fade_channel = value;
        else begin
            product = value * level;
            fade_channel = product[10:3];
        end
    end
endfunction

function [7:0] brightness_channel;
    input [7:0] value;
    input [7:0] level;
    reg [12:0] extended,product;
    integer bit_index;
    begin
        extended={5'd0,value};product=0;
        for(bit_index=0;bit_index<5;bit_index=bit_index+1)
            if(level[bit_index]) product=product+(extended<<bit_index);
        brightness_channel=product[11:4];
    end
endfunction

// Rounded percentages for 16 steps; a tiny LUT replaces decimal division.
function [11:0] percent_bcd;
    input [7:0] level;
    begin
        case(level)
          0:percent_bcd=12'h000;1:percent_bcd=12'h006;2:percent_bcd=12'h013;
          3:percent_bcd=12'h019;4:percent_bcd=12'h025;5:percent_bcd=12'h031;
          6:percent_bcd=12'h038;7:percent_bcd=12'h044;8:percent_bcd=12'h050;
          9:percent_bcd=12'h056;10:percent_bcd=12'h063;11:percent_bcd=12'h069;
          12:percent_bcd=12'h075;13:percent_bcd=12'h081;14:percent_bcd=12'h088;
          15:percent_bcd=12'h094;default:percent_bcd=12'h100;
        endcase
    end
endfunction

// 8x8 glyphs, MSB is the left-most pixel.  Only characters used by the OSD
// are implemented, which keeps the decoder much smaller than a full font ROM.
function [7:0] glyph_row;
    input [7:0] ch;
    input [2:0] row;
    begin
        glyph_row = 8'h00;
        case (ch)
            "0": case(row) 0:glyph_row=8'h3c;1:glyph_row=8'h66;2:glyph_row=8'h6e;3:glyph_row=8'h76;4:glyph_row=8'h66;5:glyph_row=8'h66;6:glyph_row=8'h3c;default:glyph_row=0;endcase
            "1": case(row) 0:glyph_row=8'h18;1:glyph_row=8'h38;2:glyph_row=8'h18;3:glyph_row=8'h18;4:glyph_row=8'h18;5:glyph_row=8'h18;6:glyph_row=8'h7e;default:glyph_row=0;endcase
            "2": case(row) 0:glyph_row=8'h3c;1:glyph_row=8'h66;2:glyph_row=8'h06;3:glyph_row=8'h0c;4:glyph_row=8'h30;5:glyph_row=8'h60;6:glyph_row=8'h7e;default:glyph_row=0;endcase
            "3": case(row) 0:glyph_row=8'h3c;1:glyph_row=8'h66;2:glyph_row=8'h06;3:glyph_row=8'h1c;4:glyph_row=8'h06;5:glyph_row=8'h66;6:glyph_row=8'h3c;default:glyph_row=0;endcase
            "4": case(row) 0:glyph_row=8'h0c;1:glyph_row=8'h1c;2:glyph_row=8'h3c;3:glyph_row=8'h6c;4:glyph_row=8'h7e;5:glyph_row=8'h0c;6:glyph_row=8'h0c;default:glyph_row=0;endcase
            "5": case(row) 0:glyph_row=8'h7e;1:glyph_row=8'h60;2:glyph_row=8'h7c;3:glyph_row=8'h06;4:glyph_row=8'h06;5:glyph_row=8'h66;6:glyph_row=8'h3c;default:glyph_row=0;endcase
            "6": case(row) 0:glyph_row=8'h1c;1:glyph_row=8'h30;2:glyph_row=8'h60;3:glyph_row=8'h7c;4:glyph_row=8'h66;5:glyph_row=8'h66;6:glyph_row=8'h3c;default:glyph_row=0;endcase
            "7": case(row) 0:glyph_row=8'h7e;1:glyph_row=8'h06;2:glyph_row=8'h0c;3:glyph_row=8'h18;4:glyph_row=8'h30;5:glyph_row=8'h30;6:glyph_row=8'h30;default:glyph_row=0;endcase
            "8": case(row) 0:glyph_row=8'h3c;1:glyph_row=8'h66;2:glyph_row=8'h66;3:glyph_row=8'h3c;4:glyph_row=8'h66;5:glyph_row=8'h66;6:glyph_row=8'h3c;default:glyph_row=0;endcase
            "9": case(row) 0:glyph_row=8'h3c;1:glyph_row=8'h66;2:glyph_row=8'h66;3:glyph_row=8'h3e;4:glyph_row=8'h06;5:glyph_row=8'h0c;6:glyph_row=8'h38;default:glyph_row=0;endcase
            "A": case(row) 0:glyph_row=8'h18;1:glyph_row=8'h3c;2:glyph_row=8'h66;3:glyph_row=8'h66;4:glyph_row=8'h7e;5:glyph_row=8'h66;6:glyph_row=8'h66;default:glyph_row=0;endcase
            "C": case(row) 0:glyph_row=8'h3c;1:glyph_row=8'h66;2:glyph_row=8'h60;3:glyph_row=8'h60;4:glyph_row=8'h60;5:glyph_row=8'h66;6:glyph_row=8'h3c;default:glyph_row=0;endcase
            "D": case(row) 0:glyph_row=8'h78;1:glyph_row=8'h6c;2:glyph_row=8'h66;3:glyph_row=8'h66;4:glyph_row=8'h66;5:glyph_row=8'h6c;6:glyph_row=8'h78;default:glyph_row=0;endcase
            "E": case(row) 0:glyph_row=8'h7e;1:glyph_row=8'h60;2:glyph_row=8'h60;3:glyph_row=8'h7c;4:glyph_row=8'h60;5:glyph_row=8'h60;6:glyph_row=8'h7e;default:glyph_row=0;endcase
            "F": case(row) 0:glyph_row=8'h7e;1:glyph_row=8'h60;2:glyph_row=8'h60;3:glyph_row=8'h7c;4:glyph_row=8'h60;5:glyph_row=8'h60;6:glyph_row=8'h60;default:glyph_row=0;endcase
            "G": case(row) 0:glyph_row=8'h3c;1:glyph_row=8'h66;2:glyph_row=8'h60;3:glyph_row=8'h6e;4:glyph_row=8'h66;5:glyph_row=8'h66;6:glyph_row=8'h3e;default:glyph_row=0;endcase
            "H": case(row) 0:glyph_row=8'h66;1:glyph_row=8'h66;2:glyph_row=8'h66;3:glyph_row=8'h7e;4:glyph_row=8'h66;5:glyph_row=8'h66;6:glyph_row=8'h66;default:glyph_row=0;endcase
            "I": case(row) 0:glyph_row=8'h3c;1:glyph_row=8'h18;2:glyph_row=8'h18;3:glyph_row=8'h18;4:glyph_row=8'h18;5:glyph_row=8'h18;6:glyph_row=8'h3c;default:glyph_row=0;endcase
            "L": case(row) 0:glyph_row=8'h60;1:glyph_row=8'h60;2:glyph_row=8'h60;3:glyph_row=8'h60;4:glyph_row=8'h60;5:glyph_row=8'h60;6:glyph_row=8'h7e;default:glyph_row=0;endcase
            "M": case(row) 0:glyph_row=8'h63;1:glyph_row=8'h77;2:glyph_row=8'h7f;3:glyph_row=8'h6b;4:glyph_row=8'h63;5:glyph_row=8'h63;6:glyph_row=8'h63;default:glyph_row=0;endcase
            "N": case(row) 0:glyph_row=8'h66;1:glyph_row=8'h76;2:glyph_row=8'h7e;3:glyph_row=8'h7e;4:glyph_row=8'h6e;5:glyph_row=8'h66;6:glyph_row=8'h66;default:glyph_row=0;endcase
            "O": case(row) 0:glyph_row=8'h3c;1:glyph_row=8'h66;2:glyph_row=8'h66;3:glyph_row=8'h66;4:glyph_row=8'h66;5:glyph_row=8'h66;6:glyph_row=8'h3c;default:glyph_row=0;endcase
            // Keep P visually distinct from the seven-segment-style 1 used
            // for the function-state indicator.  The full-width top/bowl
            // strokes also remain legible after the 2x OSD scaling.
            "P": case(row) 0:glyph_row=8'h7e;1:glyph_row=8'h66;2:glyph_row=8'h66;3:glyph_row=8'h7e;4:glyph_row=8'h60;5:glyph_row=8'h60;6:glyph_row=8'h60;default:glyph_row=0;endcase
            "R": case(row) 0:glyph_row=8'h7c;1:glyph_row=8'h66;2:glyph_row=8'h66;3:glyph_row=8'h7c;4:glyph_row=8'h6c;5:glyph_row=8'h66;6:glyph_row=8'h66;default:glyph_row=0;endcase
            "S": case(row) 0:glyph_row=8'h3c;1:glyph_row=8'h60;2:glyph_row=8'h60;3:glyph_row=8'h3c;4:glyph_row=8'h06;5:glyph_row=8'h06;6:glyph_row=8'h7c;default:glyph_row=0;endcase
            "T": case(row) 0:glyph_row=8'h7e;1:glyph_row=8'h18;2:glyph_row=8'h18;3:glyph_row=8'h18;4:glyph_row=8'h18;5:glyph_row=8'h18;6:glyph_row=8'h18;default:glyph_row=0;endcase
            "U": case(row) 0:glyph_row=8'h66;1:glyph_row=8'h66;2:glyph_row=8'h66;3:glyph_row=8'h66;4:glyph_row=8'h66;5:glyph_row=8'h66;6:glyph_row=8'h3c;default:glyph_row=0;endcase
            "Y": case(row) 0:glyph_row=8'h66;1:glyph_row=8'h66;2:glyph_row=8'h3c;3:glyph_row=8'h18;4:glyph_row=8'h18;5:glyph_row=8'h18;6:glyph_row=8'h18;default:glyph_row=0;endcase
            "V": case(row) 0:glyph_row=8'h66;1:glyph_row=8'h66;2:glyph_row=8'h66;3:glyph_row=8'h66;4:glyph_row=8'h66;5:glyph_row=8'h3c;6:glyph_row=8'h18;default:glyph_row=0;endcase
            "B": case(row) 0:glyph_row=8'h7c;1:glyph_row=8'h66;2:glyph_row=8'h66;3:glyph_row=8'h7c;4:glyph_row=8'h66;5:glyph_row=8'h66;6:glyph_row=8'h7c;default:glyph_row=0;endcase
            "K": case(row) 0:glyph_row=8'h66;1:glyph_row=8'h6c;2:glyph_row=8'h78;3:glyph_row=8'h70;4:glyph_row=8'h78;5:glyph_row=8'h6c;6:glyph_row=8'h66;default:glyph_row=0;endcase
            "W": case(row) 0:glyph_row=8'h66;1:glyph_row=8'h66;2:glyph_row=8'h66;3:glyph_row=8'h6e;4:glyph_row=8'h7f;5:glyph_row=8'h77;6:glyph_row=8'h63;default:glyph_row=0;endcase
            "X": case(row) 0:glyph_row=8'h66;1:glyph_row=8'h66;2:glyph_row=8'h3c;3:glyph_row=8'h18;4:glyph_row=8'h3c;5:glyph_row=8'h66;6:glyph_row=8'h66;default:glyph_row=0;endcase
            "Z": case(row) 0:glyph_row=8'h7e;1:glyph_row=8'h06;2:glyph_row=8'h0c;3:glyph_row=8'h18;4:glyph_row=8'h30;5:glyph_row=8'h60;6:glyph_row=8'h7e;default:glyph_row=0;endcase
            "+": case(row) 0:glyph_row=8'h00;1:glyph_row=8'h18;2:glyph_row=8'h18;3:glyph_row=8'h7e;4:glyph_row=8'h18;5:glyph_row=8'h18;6:glyph_row=8'h00;default:glyph_row=0;endcase
            "/": case(row) 0:glyph_row=8'h02;1:glyph_row=8'h06;2:glyph_row=8'h0c;3:glyph_row=8'h18;4:glyph_row=8'h30;5:glyph_row=8'h60;6:glyph_row=8'h40;default:glyph_row=0;endcase
            "%": case(row) 0:glyph_row=8'h62;1:glyph_row=8'h64;2:glyph_row=8'h08;3:glyph_row=8'h10;4:glyph_row=8'h20;5:glyph_row=8'h46;6:glyph_row=8'h86;default:glyph_row=0;endcase
            default: glyph_row=8'h00;
        endcase
    end
endfunction

// The compact key-help panel: only the active state is displayed.
// Finite SimHei 16x16 bitmap subset, generated by tools/generate_key_help_font.py.
// ASCII shares the existing glyph decoder; all text uses 16-pixel cells.
function [7:0] panel_title_char;
    input [2:0] state;
    input [4:0] index;
    begin
        case(state)
          1: case(index) 0:panel_title_char=8'h9a;1:panel_title_char=8'h93;2:panel_title_char="1";3:panel_title_char=" ";4:panel_title_char=8'ha9;5:panel_title_char=8'ha8;6:panel_title_char=8'h95;7:panel_title_char=8'h8a;default:panel_title_char=" ";endcase
          2: case(index) 0:panel_title_char=8'h9a;1:panel_title_char=8'h93;2:panel_title_char="2";3:panel_title_char=" ";4:panel_title_char=8'h83;5:panel_title_char=8'h91;6:panel_title_char=8'h95;7:panel_title_char=8'h8a;default:panel_title_char=" ";endcase
          3: case(index) 0:panel_title_char=8'h9a;1:panel_title_char=8'h93;2:panel_title_char="3";3:panel_title_char=" ";4:panel_title_char=8'ha6;5:panel_title_char=8'ha2;6:panel_title_char=8'h95;7:panel_title_char=8'h8a;default:panel_title_char=" ";endcase
          4: case(index) 0:panel_title_char=8'h9a;1:panel_title_char=8'h93;2:panel_title_char="4";3:panel_title_char=" ";4:panel_title_char=8'h9c;5:panel_title_char=8'h9d;6:panel_title_char=8'h95;7:panel_title_char=8'h8a;default:panel_title_char=" ";endcase
          default: panel_title_char=" ";
        endcase
    end
endfunction

function [7:0] panel_key_char;
    input [2:0] state;
    input [2:0] key_number;
    input [4:0] index;
    input is_video, bright_open, vol_open;
    reg [7:0] physical_key_label;
    begin
        case(key_number)
          3'd1: physical_key_label="2";
          3'd2: physical_key_label="3";
          3'd3: physical_key_label="4";
          default: physical_key_label="1";
        endcase
        case({state,key_number})
          6'd9: begin
            case(index) 0:panel_key_char="K";1:panel_key_char="E";2:panel_key_char="Y";3:panel_key_char="1";4:panel_key_char=" ";5:panel_key_char=8'h96;6:panel_key_char=8'h86;7:panel_key_char="/";8:panel_key_char=8'h9e;9:panel_key_char=8'h9f;default:panel_key_char=" ";endcase
          end
          6'd10: begin
            case(index) 0:panel_key_char="K";1:panel_key_char="E";2:panel_key_char="Y";3:panel_key_char="2";4:panel_key_char=" ";5:panel_key_char=8'h8d;6:panel_key_char=8'h99;7:panel_key_char="/";8:panel_key_char=8'ha0;9:panel_key_char=8'ha7;default:panel_key_char=" ";endcase
          end
          6'd11: begin
            if (is_video) begin
              case(index) 0:panel_key_char="K";1:panel_key_char="E";2:panel_key_char="Y";3:panel_key_char="3";4:panel_key_char=" ";5:panel_key_char=8'h81;6:panel_key_char=8'h80;7:panel_key_char=8'ha0;8:panel_key_char=8'ha7;default:panel_key_char=" ";endcase
            end else begin
              case(index) 0:panel_key_char="K";1:panel_key_char="E";2:panel_key_char="Y";3:panel_key_char="3";4:panel_key_char=" ";5:panel_key_char=8'h81;6:panel_key_char=8'h80;7:panel_key_char=8'h8d;8:panel_key_char=8'h99;9:panel_key_char=" ";10:panel_key_char=8'h81;11:panel_key_char=8'h80;12:panel_key_char=8'ha6;13:panel_key_char=8'ha7;default:panel_key_char=" ";endcase
            end
          end
          6'd17: begin
            case(index) 0:panel_key_char="K";1:panel_key_char="E";2:panel_key_char="Y";3:panel_key_char="1";4:panel_key_char=" ";5:panel_key_char=8'h8e;6:panel_key_char=8'h8b;default:panel_key_char=" ";endcase
          end
          6'd18: begin
            case(index) 0:panel_key_char="K";1:panel_key_char="E";2:panel_key_char="Y";3:panel_key_char="2";4:panel_key_char=" ";5:panel_key_char=8'ha4;6:panel_key_char=8'h85;default:panel_key_char=" ";endcase
          end
          6'd19: begin
            if (bright_open) begin
              case(index) 0:panel_key_char="K";1:panel_key_char="E";2:panel_key_char="Y";3:panel_key_char="3";4:panel_key_char=" ";5:panel_key_char=8'h87;6:panel_key_char=8'ha3;7:panel_key_char=8'h9b;8:panel_key_char=8'ha5;default:panel_key_char=" ";endcase
            end else begin
              case(index) 0:panel_key_char="K";1:panel_key_char="E";2:panel_key_char="Y";3:panel_key_char="3";4:panel_key_char=" ";5:panel_key_char=8'h94;6:panel_key_char=8'h92;7:panel_key_char=8'h9b;8:panel_key_char=8'ha5;default:panel_key_char=" ";endcase
            end
          end
          6'd25: begin
            case(index) 0:panel_key_char="K";1:panel_key_char="E";2:panel_key_char="Y";3:panel_key_char="1";4:panel_key_char=" ";5:panel_key_char=8'h8e;6:panel_key_char=8'h8b;default:panel_key_char=" ";endcase
          end
          6'd26: begin
            case(index) 0:panel_key_char="K";1:panel_key_char="E";2:panel_key_char="Y";3:panel_key_char="2";4:panel_key_char=" ";5:panel_key_char=8'ha4;6:panel_key_char=8'h85;default:panel_key_char=" ";endcase
          end
          6'd27: begin
            if (vol_open) begin
              case(index) 0:panel_key_char="K";1:panel_key_char="E";2:panel_key_char="Y";3:panel_key_char="3";4:panel_key_char=" ";5:panel_key_char=8'h87;6:panel_key_char=8'ha3;7:panel_key_char=8'h9b;8:panel_key_char=8'ha5;default:panel_key_char=" ";endcase
            end else begin
              case(index) 0:panel_key_char="K";1:panel_key_char="E";2:panel_key_char="Y";3:panel_key_char="3";4:panel_key_char=" ";5:panel_key_char=8'h94;6:panel_key_char=8'h92;7:panel_key_char=8'h9b;8:panel_key_char=8'ha5;default:panel_key_char=" ";endcase
            end
          end
          6'd33: begin
            case(index) 0:panel_key_char="K";1:panel_key_char="E";2:panel_key_char="Y";3:panel_key_char="1";4:panel_key_char=" ";5:panel_key_char=8'h89;6:panel_key_char=8'h90;7:panel_key_char=8'h8c;default:panel_key_char=" ";endcase
          end
          6'd34: begin
            case(index) 0:panel_key_char="K";1:panel_key_char="E";2:panel_key_char="Y";3:panel_key_char="2";4:panel_key_char=" ";5:panel_key_char=8'h9c;6:panel_key_char=8'h9d;7:panel_key_char=8'had;8:panel_key_char=8'hae;default:panel_key_char=" ";endcase
          end
          6'd35: begin
            case(index) 0:panel_key_char="K";1:panel_key_char="E";2:panel_key_char="Y";3:panel_key_char="3";4:panel_key_char=" ";5:panel_key_char=8'hb2;6:panel_key_char=8'hb1;7:panel_key_char=8'haf;8:panel_key_char=8'hb0;default:panel_key_char=" ";endcase
          end
          default: panel_key_char=" ";
        endcase
        if(index==3) panel_key_char=physical_key_label;
    end
endfunction

function [15:0] panel_glyph_row;
    input [7:0] ch;
    input [3:0] row;
    reg [255:0] bitmap;
    reg [7:0] latin;
    integer bit_number;
    begin
        bitmap=0;
        case(ch)
          8'h80: bitmap=256'h00000000000000000000000000007ffe7ffe0000000000000000000000000000; // 一
          8'h81: bitmap=256'h000000007ffe0100010001000100016001300118010c01000100010001000000; // 下
          8'h82: bitmap=256'h000000081ff810001080108030803ffe208000800c901898308c208403800200; // 乐
          8'h83: bitmap=256'h01007ffe00000ff008100ff000007ffe7ffe400207e004600c62387e20000000; // 亮
          8'h84: bitmap=256'h000008401860100013fe3000310851185118119010901090103017fe10001000; // 位
          8'h85: bitmap=256'h080018fc13a012203220322053fe52201220123012901392138a124e10000000; // 低
          8'h86: bitmap=256'h186017fe100033f8320853f8500017fe140213fc10201020102010e000800000; // 停
          8'h87: bitmap=256'h00000420042006603ffc3ffc0180018001807ffe0180034006700c1c700e0000; // 关
          8'h88: bitmap=256'h0000044004600c2008301818300e6ffe02100210021006100410183030602000; // 分
          8'h89: bitmap=256'h180009fe01fe7c26046608641b641e443e445a4418c4188419841b1c18000000; // 初
          8'h8a: bitmap=256'h00000c062c066c367fb64c367fb60c360c367fb66cb66cb66c866d860c0e0000; // 制
          8'h8b: bitmap=256'h00000800080008007f7c0944094409441944114411441144317c234466440000; // 加
          8'h8c: bitmap=256'h000008c00cc008c418c418cc38d878e018c019c01fc018c018c218c2187e0000; // 化
          8'h8d: bitmap=256'h00003ffc230423e426642f4421842674299420c4230420c420043ffc20060000; // 图
          8'h8e: bitmap=256'h018c108813fe122212aa7eaa122213fe100011fc1d0471fc010401fc01040000; // 增
          8'h8f: bitmap=256'h000004000ffc18003ff868080ff808080ff804000ff81c30336001c07f7e2002; // 复
          8'h90: bitmap=256'h1060104010407c88258425fe2400240069fc39841984158431fc618401840000; // 始
          8'h91: bitmap=256'h008000803ffc221022102ffc221023f020002ff8231021a060c0473e08020000; // 度
          8'h92: bitmap=256'h00003ffc06200620062006207ffe7ffe0620042004200c201820702000000000; // 开
          8'h93: bitmap=256'h010001007ffe0240066004201b18310e60000080248c2406641247f000000000; // 态
          8'h94: bitmap=256'h1800180019fe7e105810181018101e1078105810181018101810387000400000; // 打
          8'h95: bitmap=256'h00001060102013fe12027ada118c1306180073fc102010201020102073fe0000; // 控
          8'h96: bitmap=256'h00000804087c3740347c3f5804587fd8049800001ff810081ff810081ff80000; // 暂
          8'h97: bitmap=256'h00000660066006607ffe6666666666667ffe66666666666666667ffe60060000; // 曲
          8'h98: bitmap=256'h0100010001003ff8010001007ffe038007400d601930311c6106010000000000; // 未
          8'h99: bitmap=256'h18201820182018201ffe1800180018001ff01010101030102010601000100000; // 片
          8'h9a: bitmap=256'h00000c600c284c2c6c242c200ffe0c601c603c706c504cd00c880d8c0f060c00; // 状
          8'h9b: bitmap=256'h00003ff8318831883ff831883ff8018003800c70743e04220420082018201020; // 界
          8'h9c: bitmap=256'h0000001c3fe0018002100c601fc003100c0c3ffe00800890108c208643800000; // 系
          8'h9d: bitmap=256'h1060102033fe24404c407888118c11fe245078d000d004907992431e02000000; // 统
          8'h9e: bitmap=256'h00000010101023d623544f107b1013fe3330633c7b5603d003107b0043fe0000; // 继
          8'h9f: bitmap=256'h00001020102021fe2c2049fe782410f0313025b0782003fe0c2078dc03860000; // 续
          8'ha0: bitmap=256'h100011fc01047d240924092411243d2475645070107010b211b2131e10000000; // 视
          8'ha1: bitmap=256'h00007f7c7f7c14047f0455045504557c5540634041407f4041407f42417e0000; // 配
          8'ha2: bitmap=256'h00001ff81ff810081ff87ffe00001ff81ff811881ff83ff801807ffe00000000; // 量
          8'ha3: bitmap=256'h100019fe0806604660466ff660c66146634666466c46784661c6610c00000000; // 闭
          8'ha4: bitmap=256'h00407cfc498c49c8587050784bae4a204dfc4c205a2043fe4020402040300000; // 降
          8'ha5: bitmap=256'h00007ffe010003003ffc2644264427c4264427c4264426443ffc200400000000; // 面
          8'ha6: bitmap=256'h010001803ffc0c30042004607ffe00001ff818181ff8181818181ff818180000; // 音
          8'ha7: bitmap=256'h0000080028fe2f102820287e7f46085628562b566a560e560c20186c61c60000; // 频
          8'ha8: bitmap=256'h000000401840104017fe30e030d051d05158134816461dfa1040104010400000; // 体
          8'ha9: bitmap=256'h110813fe110811087df8250825f828204bfe387018b01da82326462200200000; // 媒
          8'haa: bitmap=256'h1060102013fe7b021362104010401ffe7088518811d01070107c378600000000; // 按
          8'hab: bitmap=256'h0000200021fe60007e0048fc088408847e8418fc188c1ccc3648201841fe0000; // 短
          8'hac: bitmap=256'h00000c000c300c600cc00d800d007ffe0c800cc00c400c200db80e1e0c040000; // 长
          8'had: bitmap=256'h00000840186017fe100033fc3000500053fc100013fc1304130413fc13041000; // 信
          8'hae: bitmap=256'h030002001ff810081ff810081ff810081ff80080248c2486441207f000000000; // 息
          8'haf: bitmap=256'h00001ff810081ff81ff810081ff806402640364416481e5806407ffe00000000; // 显
          8'hb0: bitmap=256'h000000001ff80000000000007ffe008200800c90189810883084608403800200; // 示
          8'hb1: bitmap=256'h08201f7e3458668808800bfc200427e4242427e42424242427e4201c20180000; // 简
          8'hb2: bitmap=256'h00001020102055fe742011fc7c2011fe35fc358473fcd18411fc1184119c1188; // 精
          default: bitmap=0;
        endcase
        panel_glyph_row=bitmap[255-{row,4'b0000} -: 16];
        if (!ch[7]) begin
            latin=glyph_row(ch,row[3:1]);
            for(bit_number=0;bit_number<8;bit_number=bit_number+1)
                panel_glyph_row[15-bit_number*2 -: 2]={2{latin[7-bit_number]}};
        end
    end
endfunction

function [7:0] status_char;
    input [2:0] index;
    input       paused;
    begin
        if (paused) begin
            case(index)
                0:status_char="P"; 1:status_char="A"; 2:status_char="U";
                3:status_char="S"; 4:status_char="E"; default:status_char=" ";
            endcase
        end else begin
            case(index)
                0:status_char="P"; 1:status_char="L"; 2:status_char="A";
                3:status_char="Y"; 4:status_char=" "; default:status_char=" ";
            endcase
        end
    end
endfunction

function [7:0] title_char;
    input [4:0] index;
    begin
        case(index)
            0:title_char="A"; 1:title_char="N"; 2:title_char="L"; 3:title_char="O";
            4:title_char="G"; 5:title_char="I"; 6:title_char="C"; 7:title_char=" ";
            8:title_char="F"; 9:title_char="P"; 10:title_char="G"; 11:title_char="A";
            12:title_char=" "; 13:title_char="M"; 14:title_char="E"; 15:title_char="D";
            16:title_char="I"; 17:title_char="A"; default:title_char=" ";
        endcase
    end
endfunction

function [7:0] info_char;
    input [3:0] index;
    input [3:0] code;
    input       is_video;
    input       is_music;
    begin
      if(is_music)begin
        case(index)
            0:info_char="M";1:info_char="U";2:info_char="S";
            3:info_char="I";4:info_char="C";5:info_char=" ";
            6:info_char="0";7:info_char=(8'h31+code);
            default:info_char=" ";
        endcase
      end else if(is_video)begin
        case(index)
            0:info_char="V";1:info_char="I";2:info_char="D";
            3:info_char="E";4:info_char="O";5:info_char=" ";
            // Video labels are always two digits (VIDEO 01..VIDEO 03).
            // `code` is the committed/presented video number, not a
            // preloading or requested slot.  Leave IMAGE and MUSIC labels
            // unchanged below.
            6:info_char="0";
            7:info_char=(code<10)?("0"+code):("A"+code-10);
            default:info_char=" ";
        endcase
      end else begin
        case(index)
            0:info_char="I"; 1:info_char="M"; 2:info_char="A"; 3:info_char="G";
            4:info_char="E"; 5:info_char=" ";
            6:info_char=(code==4'ha) ? "1" : "0";
            7:info_char=(code==4'ha) ? "0" : ("0"+code);
            default:info_char=" ";
        endcase
      end
    end
endfunction

// One decoder for the whole bottom row avoids duplicating the IMAGE/VIDEO
// decoder and MUSIC decoder behind a subtractor/mux on a crowded device.
function [7:0] bottom_info_char;
    input [4:0] index;
    input [3:0] image;
    input [1:0] music;
    input is_video;
    begin
        case(index)
            0:bottom_info_char=is_video?"V":"I";
            1:bottom_info_char=is_video?"I":"M";
            2:bottom_info_char=is_video?"D":"A";
            3:bottom_info_char=is_video?"E":"G";
            4:bottom_info_char=is_video?"O":"E";
            6:bottom_info_char=(!is_video&&image==10)?"1":"0";
            7:bottom_info_char=(!is_video&&image==10)?"0":(8'h30+image);
            10:bottom_info_char=is_video?" ":"M";
            11:bottom_info_char=is_video?" ":"U";
            12:bottom_info_char=is_video?" ":"S";
            13:bottom_info_char=is_video?" ":"I";
            14:bottom_info_char=is_video?" ":"C";
            16:bottom_info_char=is_video?" ":"0";
            17:bottom_info_char=is_video?" ":(8'h31+music);
            default:bottom_info_char=" ";
        endcase
    end
endfunction

reg [23:0] faded_rgb;
reg [23:0] bright_rgb;
reg [23:0] composed_rgb;
reg [7:0]  selected_char;
reg [7:0]  selected_row;
reg [4:0]  char_index;
reg [2:0]  glyph_x, glyph_y;
reg        text_pixel;
reg        logo_pixel;
reg [4:0]  meter_index;
reg [4:0]  panel_index;
reg [2:0]  panel_key;
reg [9:0]  panel_x;
reg [3:0]  panel_y;
reg [15:0] panel_row;
reg        panel_text;
// Only PLAY/PAUSE is drawn here; the state number remains in the panel title.
reg [2:0]  status_index;
// Picture mode shows both presented image and background MUSIC selection,
// independently of the current key-function state. Video retains VIDEO nn.
wire adjust_ui = ((shown_function==3'd2)&&shown_brightness_ui) ||
                 ((shown_function==3'd3)&&shown_volume_ui);
wire [7:0] adjust_value = (shown_function==3'd2) ? shown_brightness : shown_volume;
wire [11:0] adjust_bcd=percent_bcd(adjust_value);
wire [9:0] adjust_width=({2'd0,adjust_value}<<3)+({2'd0,adjust_value}<<2)+{2'd0,adjust_value};

// One small diagnostic text decoder shared by the information page and the
// always-visible error banner. Raw hex codes retain their original domains.
function [7:0] hex_char;
    input [3:0] code;
    begin hex_char=(code<10)?(8'h30+code):(8'h41+code-4'd10); end
endfunction
// Character-wise table avoids a wide 224-bit string mux followed by a
// variable bit-select. All original text, locations and raw codes remain.
function [7:0] diagnostic_char;
    input [2:0] line_number;
    input [4:0] index;
    reg [3:0] message_id,code;
    begin
        code=media_error?shown_media_status:(audio_error?shown_audio_status:shown_sub_status);
        message_id={1'b0,line_number};
        if(line_number==4)begin
            if(any_error)begin
                message_id=6;
                if((!media_error&&audio_error&&code==12)||(!media_error&&!audio_error&&code==13))message_id=7;
                else if(code==15||(media_error&&code==13))message_id=8;
                else if(code==11||code==14)message_id=9;
                else if(media_error&&code==12)message_id=10;
            end else if(shown_loading||shown_audio_loading||!shown_sd||!shown_audio_ready)message_id=11;
        end
        diagnostic_char=" ";
        case(message_id)
          4'd0:case(index) 0:diagnostic_char="S";1:diagnostic_char="Y";2:diagnostic_char="S";3:diagnostic_char="T";4:diagnostic_char="E";5:diagnostic_char="M";7:diagnostic_char="I";8:diagnostic_char="N";9:diagnostic_char="F";10:diagnostic_char="O"; default:diagnostic_char=" ";endcase
          4'd1:case(index) 0:diagnostic_char="I";1:diagnostic_char="M";2:diagnostic_char="A";3:diagnostic_char="G";4:diagnostic_char="E";6:diagnostic_char="0";7:diagnostic_char="0";10:diagnostic_char="M";11:diagnostic_char="U";12:diagnostic_char="S";13:diagnostic_char="I";14:diagnostic_char="C";16:diagnostic_char="0";17:diagnostic_char="0"; default:diagnostic_char=" ";endcase
          4'd2:case(index) 0:diagnostic_char="T";1:diagnostic_char="F";3:diagnostic_char="R";4:diagnostic_char="E";5:diagnostic_char="A";6:diagnostic_char="D";7:diagnostic_char="Y";10:diagnostic_char="A";11:diagnostic_char="U";12:diagnostic_char="D";13:diagnostic_char="I";14:diagnostic_char="O";16:diagnostic_char="R";17:diagnostic_char="E";18:diagnostic_char="A";19:diagnostic_char="D";20:diagnostic_char="Y"; default:diagnostic_char=" ";endcase
          4'd3:case(index) 0:diagnostic_char="S";1:diagnostic_char="U";2:diagnostic_char="B";4:diagnostic_char="R";5:diagnostic_char="E";6:diagnostic_char="A";7:diagnostic_char="D";8:diagnostic_char="Y";12:diagnostic_char="S";13:diagnostic_char="O";14:diagnostic_char="U";15:diagnostic_char="R";16:diagnostic_char="C";17:diagnostic_char="E";19:diagnostic_char="M";20:diagnostic_char="U";21:diagnostic_char="S";22:diagnostic_char="I";23:diagnostic_char="C"; default:diagnostic_char=" ";endcase
          4'd4:case(index) 0:diagnostic_char="N";1:diagnostic_char="O";3:diagnostic_char="R";4:diagnostic_char="E";5:diagnostic_char="P";6:diagnostic_char="O";7:diagnostic_char="R";8:diagnostic_char="T";9:diagnostic_char="E";10:diagnostic_char="D";12:diagnostic_char="E";13:diagnostic_char="R";14:diagnostic_char="R";15:diagnostic_char="O";16:diagnostic_char="R"; default:diagnostic_char=" ";endcase
          4'd5:case(index) 0:diagnostic_char="M";1:diagnostic_char="E";2:diagnostic_char="D";3:diagnostic_char="I";4:diagnostic_char="A";6:diagnostic_char="0";9:diagnostic_char="W";10:diagnostic_char="A";11:diagnostic_char="V";13:diagnostic_char="0";16:diagnostic_char="S";17:diagnostic_char="U";18:diagnostic_char="B";20:diagnostic_char="0"; default:diagnostic_char=" ";endcase
          4'd6:case(index) 4:diagnostic_char="F";5:diagnostic_char="O";6:diagnostic_char="R";7:diagnostic_char="M";8:diagnostic_char="A";9:diagnostic_char="T";11:diagnostic_char="E";12:diagnostic_char="R";13:diagnostic_char="R";14:diagnostic_char="O";15:diagnostic_char="R"; default:diagnostic_char=" ";endcase
          4'd7:case(index) 4:diagnostic_char="F";5:diagnostic_char="I";6:diagnostic_char="L";7:diagnostic_char="E";9:diagnostic_char="N";10:diagnostic_char="O";11:diagnostic_char="T";13:diagnostic_char="F";14:diagnostic_char="O";15:diagnostic_char="U";16:diagnostic_char="N";17:diagnostic_char="D"; default:diagnostic_char=" ";endcase
          4'd8:case(index) 4:diagnostic_char="R";5:diagnostic_char="E";6:diagnostic_char="A";7:diagnostic_char="D";9:diagnostic_char="T";10:diagnostic_char="I";11:diagnostic_char="M";12:diagnostic_char="E";13:diagnostic_char="O";14:diagnostic_char="U";15:diagnostic_char="T"; default:diagnostic_char=" ";endcase
          4'd9:case(index) 4:diagnostic_char="F";5:diagnostic_char="A";6:diagnostic_char="T";8:diagnostic_char="R";9:diagnostic_char="E";10:diagnostic_char="A";11:diagnostic_char="D";13:diagnostic_char="E";14:diagnostic_char="R";15:diagnostic_char="R";16:diagnostic_char="O";17:diagnostic_char="R"; default:diagnostic_char=" ";endcase
          4'd10:case(index) 4:diagnostic_char="F";5:diagnostic_char="I";6:diagnostic_char="L";7:diagnostic_char="E";8:diagnostic_char="/";9:diagnostic_char="D";10:diagnostic_char="A";11:diagnostic_char="T";12:diagnostic_char="A";14:diagnostic_char="E";15:diagnostic_char="R";16:diagnostic_char="R";17:diagnostic_char="O";18:diagnostic_char="R"; default:diagnostic_char=" ";endcase
          4'd11:case(index) 0:diagnostic_char="L";1:diagnostic_char="O";2:diagnostic_char="A";3:diagnostic_char="D";4:diagnostic_char="I";5:diagnostic_char="N";6:diagnostic_char="G"; default:diagnostic_char=" ";endcase
          default:diagnostic_char=" ";
        endcase
        case(line_number)
          1:begin
            if(index<5&&displayed_video_mode)case(index)
                0:diagnostic_char="V";1:diagnostic_char="I";2:diagnostic_char="D";3:diagnostic_char="E";4:diagnostic_char="O";endcase
            if(index==6)diagnostic_char=(!displayed_video_mode&&displayed_image==10)?"1":"0";
            if(index==7)diagnostic_char=(!displayed_video_mode&&displayed_image==10)?"0":(8'h30+displayed_image);
            if(index==17)diagnostic_char=8'h31+shown_audio_index;
            if(displayed_video_mode&&index>=10&&index<15)case(index)
                10:diagnostic_char="A";11:diagnostic_char="U";12:diagnostic_char="D";13:diagnostic_char="I";14:diagnostic_char="O";endcase
          end
          2:begin
            if(!shown_sd&&index>=3&&index<8)case(index)
                3:diagnostic_char="W";4:diagnostic_char="A";5:diagnostic_char="I";6:diagnostic_char="T";7:diagnostic_char=" ";endcase
            if(shown_audio_loading||!shown_audio_ready)begin
                if(index>=16&&index<23)case(index)
                    16:diagnostic_char="L";17:diagnostic_char="O";18:diagnostic_char="A";19:diagnostic_char="D";20:diagnostic_char="I";21:diagnostic_char="N";22:diagnostic_char="G";endcase
            end else if(shown_audio_held&&index>=16&&index<21)case(index)
                16:diagnostic_char="H";17:diagnostic_char="O";18:diagnostic_char="L";19:diagnostic_char="D";20:diagnostic_char=" ";endcase
          end
          3:begin
            if(!shown_sub_ready&&index>=4&&index<11)case(index)
                4:diagnostic_char="L";5:diagnostic_char="O";6:diagnostic_char="A";7:diagnostic_char="D";8:diagnostic_char="I";9:diagnostic_char="N";10:diagnostic_char="G";endcase
            if(shown_audio_source&&index>=19&&index<24)case(index)
                19:diagnostic_char="V";20:diagnostic_char="I";21:diagnostic_char="D";22:diagnostic_char="E";23:diagnostic_char="O";endcase
          end
          4:if(any_error)begin
            case(index)
                0:diagnostic_char=media_error?"M":(audio_error?"W":"S");
                1:diagnostic_char=media_error?"E":(audio_error?"A":"U");
                2:diagnostic_char=media_error?"D":(audio_error?"V":"B");
                26:diagnostic_char="/";27:diagnostic_char=hex_char(code);
                default:begin end
            endcase
          end
          5:case(index)6:diagnostic_char=hex_char(shown_media_status);13:diagnostic_char=hex_char(shown_audio_status);20:diagnostic_char=hex_char(shown_sub_status);endcase
          default:begin end
        endcase
    end
endfunction
reg [2:0] diagnostic_line;
reg diagnostic_text;
reg heading_text;
// All text regions are spatially disjoint. Select character/position/color
// first, then use one shared 16x16 font decoder for the final pixel.
reg font_enable;
reg [7:0] font_char;
reg [3:0] font_x,font_y;
reg [23:0] font_color;
reg [15:0] font_row;
reg [23:0] background_rgb;
reg pipe_hs,pipe_vs,pipe_de,pipe_font_enable;
reg [7:0] pipe_font_char;
reg [3:0] pipe_font_x,pipe_font_y;
reg [23:0] pipe_font_color,pipe_background;
wire [15:0] pipe_font_row=panel_glyph_row(pipe_font_char,pipe_font_y);

always @* begin
    // Each raw-video frame alternates SDRAM buffers. That is not a slide
    // transition: restarting the fade on every frame causes periodic flashes.
    faded_rgb = video_mode_sync[1] ? in_rgb :
                {fade_channel(in_rgb[23:16],fade_level),
                 fade_channel(in_rgb[15:8],fade_level),
                 fade_channel(in_rgb[7:0],fade_level)};
    bright_rgb = {brightness_channel(faded_rgb[23:16],shown_brightness),
                  brightness_channel(faded_rgb[15:8],shown_brightness),
                  brightness_channel(faded_rgb[7:0],shown_brightness)};
    composed_rgb = bright_rgb;
    selected_char = " ";
    selected_row = 8'h00;
    char_index = 0;
    glyph_x = 0;
    glyph_y = 0;
    text_pixel = 1'b0;
    logo_pixel = 1'b0;
    meter_index = 0;
    panel_index = 0;
    panel_key = 0;
    panel_x = 0;
    panel_y = 0;
    panel_row = 0;
    panel_text = 0;
    status_index = 0;
    diagnostic_line=0;
    diagnostic_text=0;
    heading_text=0;
    font_enable=0;font_char=" ";font_x=0;font_y=0;
    font_color=24'hf5f7fa;font_row=0;

    // Restrained dark bands keep text readable over arbitrary photographs.
    // Total overlay height: 56 top lines and 36 bottom lines (was 100+56).
    if (normal_osd && ((pixel_y<9'd56) || (pixel_y>=9'd444)))
        composed_rgb = {2'b00,bright_rgb[23:18],2'b00,bright_rgb[15:10],2'b00,bright_rgb[7:2]};

    // Runtime 16x16 subtitles occupy the bottom-most active lines.
    if(subtitle_visible && pixel_y>=9'd464 && pixel_y<9'd480)
        composed_rgb = {1'b0,bright_rgb[23:17],1'b0,bright_rgb[15:9],1'b0,bright_rgb[7:1]};
    // Subtitle glyphs are rendered in red for better contrast with the
    // grey lower-band background and the displayed photographs.
    if(subtitle_pixel)
        composed_rgb = 24'hff0000;

    // Compact red/cyan AL mark at the upper left.
    if (normal_osd && (pixel_x>=10'd8) && (pixel_x<10'd24) && (pixel_y>=9'd12) && (pixel_y<9'd28)) begin
        logo_pixel = ((pixel_x==10'd9) || (pixel_y==9'd13) ||
                      ((pixel_x>=10'd9) && (pixel_x<=10'd16) && (pixel_y==9'd20)) ||
                      (pixel_x==10'd17) || ((pixel_y==9'd26) && pixel_x>=10'd17));
        if (logo_pixel)
            composed_rgb = (pixel_x<10'd17) ? 24'hff4b45 : 24'h37d7c4;
    end

    // Match the key-help font: 2x 8x8 glyphs in 16x16 cells.
    // Title, playback status and state title share y=12..27; no taller band.
    if (normal_osd && (pixel_x>=10'd40) && (pixel_x<10'd328) && (pixel_y>=9'd12) && (pixel_y<9'd28)) begin
        char_index = (pixel_x-10'd40) >> 4;
        glyph_x = ((pixel_x-10'd40) >> 1) & 3'b111;
        glyph_y = (pixel_y-9'd12) >> 1;
        selected_char = title_char(char_index);
        heading_text=1;
    end
    // Share the same Latin font decoder for disjoint title/status regions.
    if (normal_osd && (pixel_x>=10'd352) && (pixel_x<10'd432) && (pixel_y>=9'd12) && (pixel_y<9'd28)) begin
        status_index = (pixel_x-10'd352) >> 4;
        glyph_x = ((pixel_x-10'd352) >> 1) & 3'b111;
        glyph_y = (pixel_y-9'd12) >> 1;
        selected_char = status_char(status_index,shown_paused);
        heading_text=1;
    end
    if(heading_text)begin
        font_enable=1;font_char=selected_char;
        font_x={glyph_x,pixel_x[0]};font_y={glyph_y,pixel_y[0]};
        font_color=(pixel_x>=352) ? (shown_paused?24'hffd166:24'h37d7c4) : 24'hf5f7fa;
    end

    // Compact full-width key-help panel with remapped physical key labels.
    // Text has an 8-pixel safe margin; the background has no side gaps.
    // The physical KEY1 state/menu key remains omitted by this compact row.
    if (normal_osd && (pixel_x<10'd640) && (pixel_y>=9'd32) && (pixel_y<9'd56)) begin
        composed_rgb = {2'b00,bright_rgb[23:18],2'b00,bright_rgb[15:10],2'b00,bright_rgb[7:2]};
        // State 1 reserves fourteen cells for both KEY3 gesture labels.
        // Keep 16x16 glyphs, one row, and the existing full-width background.
        if (shown_function==3'd1 && pixel_y>=9'd36 && pixel_y<9'd52 &&
            ((pixel_x>=10'd8 && pixel_x<10'd232) ||
             (pixel_x>=10'd248 && pixel_x<10'd408) ||
             (pixel_x>=10'd424 && pixel_x<10'd632))) begin
            panel_key = (pixel_x<10'd232) ? 3'd3 : ((pixel_x<10'd408)?3'd2:3'd1);
            panel_x = (pixel_x<10'd232) ? (pixel_x-10'd8) :
                      ((pixel_x<10'd408) ? (pixel_x-10'd248) : (pixel_x-10'd424));
            panel_text = 1;
        end else if (shown_function!=3'd1 &&
            ((pixel_x>=10'd8 && pixel_x<10'd200) ||
             (pixel_x>=10'd208 && pixel_x<10'd400) ||
             (pixel_x>=10'd408 && pixel_x<10'd632)) &&
            (pixel_y>=9'd36 && pixel_y<9'd52)) begin
            panel_key = (pixel_x<10'd200) ? 3'd3 : ((pixel_x<10'd400)?3'd2:3'd1);
            panel_x = (pixel_x<10'd200) ? (pixel_x-10'd8) :
                      ((pixel_x<10'd400) ? (pixel_x-10'd208) : (pixel_x-10'd408));
            panel_text = 1;
        end
        if(panel_text)begin
            panel_index = panel_x>>4;
            panel_y = pixel_y-9'd36;
            selected_char = panel_key_char(shown_function,panel_key,panel_index,
                                          displayed_video_mode,shown_brightness_ui,shown_volume_ui);
        end
        if (panel_text) begin
            font_enable=1;font_char=selected_char;font_x=panel_x[3:0];font_y=panel_y;
            if (shown_function==3'd1 && !displayed_video_mode &&
                panel_key==3 && panel_index>=10 && panel_index<14)
                font_color = 24'hffd166; // Long-press soundtrack action only.
            else if (panel_key==0) font_color = 24'hf5f7fa;
            else font_color =
                ((shown_function==3'd2 && !shown_brightness_ui) ||
                 (shown_function==3'd3 && !shown_volume_ui)) && panel_key<=2 ? 24'h89939c : 24'h37d7c4;
        end
    end

    // Top-right state title; the longest label is 11 cells (176 pixels).
    if (normal_osd && (pixel_x>=10'd448) && (pixel_x<10'd624) &&
        (pixel_y>=9'd12) && (pixel_y<9'd28)) begin
        panel_index = (pixel_x-10'd448) >> 4;
        panel_x = pixel_x-10'd448;
        panel_y = pixel_y-9'd12;
        selected_char = panel_title_char(shown_function,panel_index);
        font_enable=1;font_char=selected_char;font_x=panel_x[3:0];font_y=panel_y;
        font_color=24'hf5f7fa;
    end

    // IMAGE nn  MUSIC nn: 18 existing 16-pixel cells, x=16..303.
    // Keep the subtitle row and the right-hand level meter untouched.
    // Share the existing bottom-row font decoder with diagnostics. Their
    // screen regions are disjoint, so no additional font mux/ROM is needed.
    if(info_open && pixel_x>=80 && pixel_x<560 && pixel_y>=144 && pixel_y<352)begin
        composed_rgb={2'b00,bright_rgb[23:18],2'b00,bright_rgb[15:10],2'b00,bright_rgb[7:2]};
        if(pixel_x>=96 && pixel_x<544 && pixel_y>=152 && pixel_y<344)begin
            diagnostic_line=(pixel_y-152)>>5;
            diagnostic_text=(((pixel_y-152)&31)<16);
            char_index=(pixel_x-96)>>4;
            glyph_x=((pixel_x-96)>>1)&7;
            glyph_y=((pixel_y-152)>>1)&7;
        end
    end
    if(any_error && pixel_x>=16 && pixel_x<464 && pixel_y>=420 && pixel_y<436)begin
        composed_rgb=24'h301818;
        diagnostic_line=4;diagnostic_text=1;
        char_index=(pixel_x-16)>>4;glyph_x=((pixel_x-16)>>1)&7;
        glyph_y=(pixel_y-420)>>1;
    end
    selected_char=diagnostic_char(diagnostic_line,char_index);
    if (normal_osd && (pixel_x>=10'd16) &&
        (pixel_x<(displayed_video_mode ? 10'd144 : 10'd304)) &&
        (pixel_y>=9'd446) && (pixel_y<9'd462)) begin
        char_index = (pixel_x-10'd16) >> 4;
        glyph_x = ((pixel_x-10'd16) >> 1) & 3'b111;
        glyph_y = (pixel_y-9'd446) >> 1;
        selected_char = bottom_info_char(char_index,displayed_image,shown_music,displayed_video_mode);
        diagnostic_text=1;
    end
    if(diagnostic_text)begin
        font_enable=1;font_char=selected_char;
        font_x={glyph_x,pixel_x[0]};font_y={glyph_y,pixel_y[0]};
        font_color=(diagnostic_line==4&&any_error)?24'hff7068:24'hf5f7fa;
    end

    // Sixteen-segment audio level meter driven by the transmitted samples.
    if (normal_osd && (pixel_x>=10'd480) && (pixel_x<10'd624) && (pixel_y>=9'd446) && (pixel_y<9'd462)) begin
        meter_index = (pixel_x-10'd480) >> 3;
        if ((((pixel_x-10'd480) & 10'd7)<10'd6) && (meter_index<audio_level))
            composed_rgb = (meter_index<5'd11) ? 24'h37d7c4 : 24'hff4b45;
        else if ((((pixel_x-10'd480) & 10'd7)<10'd6) && (pixel_y>=9'd459))
            composed_rgb = 24'h46505c;
    end

    // FPGA-local brightness/volume adjustment panel. It is drawn above the
    // media image and leaves the existing title, media label and subtitles.
    if (adjust_ui && pixel_x>=10'd176 && pixel_x<10'd464 && pixel_y>=9'd170 && pixel_y<9'd310) begin
        if (pixel_x<10'd184 || pixel_x>=10'd456 || pixel_y<9'd178 || pixel_y>=9'd302)
            composed_rgb = 24'h2b3945;
        else
            composed_rgb = 24'he0e8ed;
    end
    if (adjust_ui && pixel_x>=10'd208 && pixel_x<10'd400 && pixel_y>=9'd194 && pixel_y<9'd210) begin
        char_index=(pixel_x-10'd208)>>4;
        glyph_x=((pixel_x-10'd208)>>1)&3'b111;
        glyph_y=(pixel_y-9'd194)>>1;
        if(shown_function==3'd2) begin
            case(char_index)
              0:selected_char="B";1:selected_char="R";2:selected_char="I";3:selected_char="G";
              4:selected_char="H";5:selected_char="T";6:selected_char="N";7:selected_char="E";
              8:selected_char="S";9:selected_char="S";default:selected_char=" ";endcase
        end else begin
            case(char_index) 0:selected_char="V";1:selected_char="O";2:selected_char="L";3:selected_char="U";4:selected_char="M";5:selected_char="E";default:selected_char=" ";endcase
        end
        font_enable=1;font_char=selected_char;
        font_x={glyph_x,pixel_x[0]};font_y={glyph_y,pixel_y[0]};font_color=24'h163c5a;
    end
    if (adjust_ui && pixel_x>=10'd208 && pixel_x<10'd432 && pixel_y>=9'd232 && pixel_y<9'd256) begin
        if (pixel_x<10'd216 || pixel_x>=10'd424 || pixel_y<9'd236 || pixel_y>=9'd252)
            composed_rgb=24'h536775;
        else if ((pixel_x-10'd216)<adjust_width)
            composed_rgb=24'h168bd0;
        else composed_rgb=24'hedf3f6;
    end
    if (adjust_ui && pixel_x>=10'd288 && pixel_x<10'd352 && pixel_y>=9'd270 && pixel_y<9'd286) begin
        char_index=(pixel_x-10'd288)>>4;
        glyph_x=((pixel_x-10'd288)>>1)&3'b111;glyph_y=(pixel_y-9'd270)>>1;
        case(char_index)
          0:selected_char=adjust_bcd[11:8]!=0 ? (8'h30+{4'd0,adjust_bcd[11:8]}) : " ";
          1:selected_char=8'h30+{4'd0,adjust_bcd[7:4]};
          2:selected_char=8'h30+{4'd0,adjust_bcd[3:0]};
          default:selected_char="%";
        endcase
        font_enable=1;font_char=selected_char;
        font_x={glyph_x,pixel_x[0]};font_y={glyph_y,pixel_y[0]};font_color=24'h163c5a;
    end
    background_rgb=composed_rgb;
    font_row=panel_glyph_row(font_char,font_y);
    if(font_enable&&font_row[15-font_x])composed_rgb=font_color;
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        de_d <= 0; vs_d <= 0; frame_arm <= 1;
        pixel_x <= 0; pixel_y <= 0;
        buffer_sync <= 0; displayed_buffer <= 0; display_valid_d <= 0;
        fade_level <= 0;
        image_code_meta <= 0; image_code_sync <= 0; displayed_image <= 0;
        video_mode_sync <= 0; displayed_video_mode <= 0;
        audio_peak <= 0; audio_level <= 0;
        control_meta<={3'd1,2'd0,8'd16,8'd16,3'b000};
        control_sync<={3'd1,2'd0,8'd16,8'd16,3'b000};
        control_previous<={3'd1,2'd0,8'd16,8'd16,3'b000};
        control_stable<={3'd1,2'd0,8'd16,8'd16,3'b000};
        control_frame<={3'd1,2'd0,8'd16,8'd16,3'b000};
        diag_meta<=0;diag_sync<=0;diag_previous<=0;diag_stable<=0;diag_frame<=0;
        out_hs <= 0; out_vs <= 0; out_de <= 0; out_rgb <= 0;
        pipe_hs<=0;pipe_vs<=0;pipe_de<=0;pipe_font_enable<=0;
        pipe_font_char<=0;pipe_font_x<=0;pipe_font_y<=0;
        pipe_font_color<=0;pipe_background<=0;
    end else begin
        diag_meta<={system_info_ui,minimal_osd,sd_ready,media_loading,media_status,
                    audio_loading,audio_status,sub_status,audio_source_video,
                    active_audio_index,audio_held,audio_ready,sub_ready};
        diag_sync<=diag_meta;diag_previous<=diag_sync;
        if(diag_sync==diag_previous)diag_stable<=diag_sync;
        control_meta<={function_state,music_index,brightness_level,volume_level,brightness_ui,volume_ui,playback_paused};
        control_sync<=control_meta;control_previous<=control_sync;
        if(control_sync==control_previous&&control_sync[18:11]<=16&&control_sync[10:3]<=16)
            control_stable<=control_sync;
        buffer_sync <= {buffer_sync[0],frame_buffer_select};
        image_code_meta <= image_code;
        image_code_sync <= image_code_meta;
        video_mode_sync <= {video_mode_sync[0],video_mode};
        de_d <= in_de;
        vs_d <= in_vs;
        // First stage: region/character selection, brightness and subtitle
        // composition. Second stage: shared font decoder and final RGB mux.
        pipe_hs<=in_hs;pipe_vs<=in_vs;pipe_de<=in_de;
        pipe_font_enable<=font_enable;pipe_font_char<=font_char;
        pipe_font_x<=font_x;pipe_font_y<=font_y;
        pipe_font_color<=font_color;pipe_background<=background_rgb;
        out_hs <= pipe_hs;
        out_vs <= pipe_vs;
        out_de <= pipe_de;
        out_rgb <= !pipe_de?24'd0:
                   (pipe_font_enable&&pipe_font_row[15-pipe_font_x])?pipe_font_color:pipe_background;

        if (audio_valid && audio_abs>audio_peak)
            audio_peak <= audio_abs;

        if (vs_d!=in_vs) begin
            frame_arm <= 1'b1;
            pixel_x <= 0;
            pixel_y <= 0;
        end else if (in_de) begin
            pixel_x <= de_d ? pixel_x+1'b1 : 0;
        end else if (de_d) begin
            pixel_x <= 0;
            pixel_y <= pixel_y+1'b1;
        end

        if (in_de && !de_d && frame_arm) begin
            control_frame<=control_stable;
            diag_frame<=diag_stable;
            frame_arm <= 1'b0;
            displayed_image <= image_code_sync;
            displayed_video_mode <= video_mode_sync[1];
            audio_level <= audio_peak[7:4];
            audio_peak <= audio_valid ? audio_abs : 0;
            display_valid_d <= display_valid;
            if (video_mode_sync[1]) begin
                displayed_buffer <= buffer_sync[1];
                fade_level <= 8;
            end else if (!display_valid)
                fade_level <= 0;
            else if (!display_valid_d || (buffer_sync[1]!=displayed_buffer)) begin
                displayed_buffer <= buffer_sync[1];
                // Start at half brightness rather than a near-black frame.
                // Two subsequent refreshes reach full brightness (~33 ms
                // at 60 Hz); overlays remain at their original brightness.
                fade_level <= 4;
            end else if (fade_level<4'd8)
                fade_level <= fade_level+2'd2;
        end
    end
end

endmodule
