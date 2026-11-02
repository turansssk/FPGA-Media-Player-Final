// Two active-low keys, qualified for 20 ms on both press and release.
// The mode change drains an owned SD sector before restarting the media
// pipeline. WAV and HDMI timing are not reset by a mode change.
module playback_control #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer RESET_CYCLES = 32
)(
    input wire clk, input wire rst,
    input wire key_mode, input wire key_pause,
    input wire media_sd_busy,
    output reg mode_video,
    output reg global_pause,
    output reg media_reset
);
localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ/50 > 0) ? CLK_FREQ_HZ/50 : 1;
reg [1:0] key_meta, key_sync, key_state;
reg [31:0] mode_count, pause_count;
reg [1:0] press;
reg requested_video;
reg [31:0] reset_count;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        key_meta<=2'b11; key_sync<=2'b11; key_state<=2'b11;
        mode_count<=0; pause_count<=0; press<=0;
    end else begin
        key_meta<={key_pause,key_mode}; key_sync<=key_meta; press<=0;
        if(key_sync[0]==key_state[0]) mode_count<=0;
        else if(mode_count>=DEBOUNCE_CYCLES-1) begin
            key_state[0]<=key_sync[0]; mode_count<=0;
            if(!key_sync[0]) press[0]<=1;
        end else mode_count<=mode_count+1'b1;
        if(key_sync[1]==key_state[1]) pause_count<=0;
        else if(pause_count>=DEBOUNCE_CYCLES-1) begin
            key_state[1]<=key_sync[1]; pause_count<=0;
            if(!key_sync[1]) press[1]<=1;
        end else pause_count<=pause_count+1'b1;
    end
end

always @(posedge clk or posedge rst) begin
    if(rst) begin
        mode_video<=0; requested_video<=0; global_pause<=0;
        media_reset<=0; reset_count<=0;
    end else begin
        if(press[1]) global_pause<=~global_pause;
        if(press[0]) begin
            requested_video<=~requested_video;
            media_reset<=1; reset_count<=0;
        end else if(media_reset) begin
            if(media_sd_busy) reset_count<=0;
            else if(reset_count>=RESET_CYCLES-1) begin
                mode_video<=requested_video; media_reset<=0; reset_count<=0;
            end else reset_count<=reset_count+1'b1;
        end
    end
end
endmodule
