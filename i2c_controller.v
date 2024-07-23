///////////////////////////////////////////////////////////////////////////////
// Description: I2C (Inter-Integrated Circuit) Controller(Master)
//              Creates master based on input configuration.
//
// 			 	(Start)-[ADDR]-[RW]-[ACK]-[DATA]-[ACK]-(END)
//
//              I2C uses only two signals: serial data line (SDA) and serial clock line (SCL).
//				To kick-off transaction, user must set i_enable = 1.
//				To restart transaction, user must wait for o_tx_ready = 1
//				and set i_enable = 1.
//
// Note:        i_clk must be at least 2x faster than i2c_scl
//
// Parameters:  ADDR_BW - Set address bit width.
//				DATA_BW - Set data bit width.
//              CLK_PER_HALF_BIT - Sets frequency of i2c_scl.  i2c_scl is
//              derived from i_clk.  Set to integer number of clocks for each
//              half-bit of i2c_data data.  E.g. 100 MHz i_clk, CLK_PER_HALF_BIT = 2
//              would create i2c_scl of 25 MHz.  Must be >= 2
//
// Auther:		Roger
///////////////////////////////////////////////////////////////////////////////

module i2c_controller
#(
	parameter ADDR_BW = 7,
	parameter DATA_BW = 8,
	parameter CLK_PER_HALF_BIT = 2
)
(
	input i_clk,
	input i_rstn,
	input i_enable,
	input i_tx_rw,
	input [ADDR_BW-1:0] i_tx_addr,
	input [DATA_BW-1:0] i_tx_data,
	output o_tx_ready,
	output [DATA_BW-1:0] o_rx_data,
	
	output i2c_scl,
	inout i2c_sda
);

localparam CNT_BW = $clog2(DATA_BW);

localparam S_IDLE = 0;
localparam S_START = 1;
localparam S_ADDRESS = 2;
localparam S_ADDR_ACK = 3;
localparam S_WRITE_DATA = 4;
localparam S_READ_DATA = 5;
localparam S_LOAD_ACK = 6;
localparam S_SEND_ACK = 7;
localparam S_END = 8;

reg [8:0] state;

wire clk, rstn, enable, tx_ready;
wire tx_rw;
wire [ADDR_BW-1:0] tx_addr;
wire [DATA_BW-1:0] tx_data;
wire sda_in;

reg [$clog2(CLK_PER_HALF_BIT*2)-1:0] i2c_clk_cnt;
reg i2c_clk;
reg i2c_clk_p_edge, i2c_clk_n_edge;
reg i2c_scl_enable;
reg [CNT_BW-1:0] sft_cnt;
reg [DATA_BW-1:0] data_mem;
reg [(ADDR_BW+1)-1:0] addr_mem;
reg sda_out;
reg sda_en;

//=======================================================================
// internal signal declaration
//=======================================================================

assign clk = i_clk;
assign rstn = i_rstn;
assign enable = i_enable;
assign tx_rw = i_tx_rw;
assign tx_addr = i_tx_addr;
assign tx_data = i_tx_data;
assign o_rx_data = data_mem;
assign o_tx_ready = tx_ready;

assign tx_ready = (state[S_IDLE] && rstn) ? 1'b1 : 1'b0;
assign i2c_scl = (~i2c_scl_enable) ? 1'b1 : i2c_clk;

// i2c_sda tristate buffer
io_pad u_i2c_io_pad
(
	.tx_data(sda_out),
	.tx_en(sda_en),
	.rx_data(sda_in),
	.pad_data(i2c_sda)
);

//=======================================================================
// RTL begin
//=======================================================================

//-- i2c_scl generation
always @(posedge clk or negedge rstn) begin
	if (~rstn) begin
		i2c_clk_cnt <= 0;
		i2c_clk <= 1'b0;
		i2c_clk_p_edge <= 1'b0;
		i2c_clk_n_edge <= 1'b0;
	end
	else begin
		i2c_clk_p_edge <= 1'b0;
		i2c_clk_n_edge <= 1'b0;
		if (i2c_clk_cnt == CLK_PER_HALF_BIT*2-1) begin
			i2c_clk_n_edge <= 1'b1;
			i2c_clk_cnt <= 0;
			i2c_clk <= ~i2c_clk;
		end
		else if (i2c_clk_cnt == CLK_PER_HALF_BIT-1) begin
			i2c_clk_p_edge <= 1'b1;
			i2c_clk_cnt <= i2c_clk_cnt + 1'b1;
			i2c_clk <= ~i2c_clk;
		end
		else begin
			i2c_clk_cnt <= i2c_clk_cnt + 1'b1;
		end
	end
end

//--FSM logic
always @(posedge clk or negedge rstn) begin
	if (~rstn) begin
		state <= 9'b0;
		state[S_IDLE] <= 1'b1;
		addr_mem <= {(ADDR_BW+1){1'b0}};
		data_mem <= {DATA_BW{1'b0}};
		sda_out <= 1'b1;
		sft_cnt <= 0;
		i2c_scl_enable <= 1'b0;
		sda_en <= 1'b0;
	end
	else begin
		state <= 9'b0;
		case(1'b1)
			state[S_IDLE]: begin
				if (i2c_clk_p_edge) begin
					if (enable) begin
						state[S_START] <= 1'b1;
						addr_mem <= {tx_addr, tx_rw};
						data_mem <= tx_data;
					end
					else begin
						state[S_IDLE] <= 1'b1;
					end
				end
				else begin
					state[S_IDLE] <= 1'b1;
				end
			end
			state[S_START]: begin
				if (i2c_clk_p_edge) begin
					sda_out <= 1'b0;
					state[S_ADDRESS] <= 1'b1;
					sft_cnt <= DATA_BW-1;
				end
				else begin
					state[S_START] <= 1'b1;
				end
				if (i2c_clk_n_edge) begin
					i2c_scl_enable <= 1'b0;
					sda_en <= 1'b1;
				end
			end
			state[S_ADDRESS]: begin
				if (i2c_clk_p_edge) begin
					if (sft_cnt == 0) begin
						state[S_ADDR_ACK] <= 1'b1;
					end
					else begin
						sft_cnt <= sft_cnt - 1'b1;
						state[S_ADDRESS] <= 1'b1;
					end
				end
				else begin
					state[S_ADDRESS] <= 1'b1;
				end
				if (i2c_clk_n_edge) begin
					sda_out <= addr_mem[sft_cnt];
					i2c_scl_enable <= 1'b1;
				end
			end
			state[S_ADDR_ACK]: begin
				if (i2c_clk_p_edge) begin
					if (sda_in == 1'b0) begin
						sft_cnt <= DATA_BW-1;
						if (addr_mem[0] == 1'b0) begin
							state[S_WRITE_DATA] <= 1'b1;
						end
						else begin
							state[S_READ_DATA] <= 1'b1;
						end
					end
					else begin
						state[S_END] <= 1'b1;
					end
				end
				else begin
					state[S_ADDR_ACK] <= 1'b1;
				end
				if (i2c_clk_n_edge) begin
					sda_en <= 1'b0;
					i2c_scl_enable <= 1'b1;
				end
			end
			state[S_WRITE_DATA]: begin
				if (i2c_clk_p_edge) begin
					if (sft_cnt == 0) begin
						state[S_LOAD_ACK] <= 1'b1;
					end
					else begin
						sft_cnt <= sft_cnt - 1'b1;
						state[S_WRITE_DATA] <= 1'b1;
					end
				end
				else begin
					state[S_WRITE_DATA] <= 1'b1;
				end
				if (i2c_clk_n_edge) begin
					sda_out <= data_mem[sft_cnt];
					sda_en <= 1'b1;
					i2c_scl_enable <= 1'b1;
				end
			end
			state[S_READ_DATA]: begin
				if (i2c_clk_p_edge) begin
					data_mem[sft_cnt] <= sda_in;
					if (sft_cnt == 0) begin
						state[S_SEND_ACK] <= 1'b1;
					end
					else begin
						sft_cnt <= sft_cnt - 1'b1;
						state[S_READ_DATA] <= 1'b1;
					end
				end
				else begin
					state[S_READ_DATA] <= 1'b1;
				end
				if (i2c_clk_n_edge) begin
					sda_en <= 1'b0;
					i2c_scl_enable <= 1'b1;
				end
			end
			state[S_LOAD_ACK]: begin
				if (i2c_clk_p_edge) begin
					if (sda_in == 1'b0 && enable == 1'b1) begin
						state[S_IDLE] <= 1'b1;
					end
					else begin
						state[S_END] <= 1'b1;
					end
				end
				else begin
					state[S_LOAD_ACK] <= 1'b1;
				end
				if (i2c_clk_n_edge) begin
					sda_en <= 1'b0;
					i2c_scl_enable <= 1'b1;
				end
			end
			state[S_SEND_ACK]: begin
				if (i2c_clk_p_edge) begin
					state[S_END] <= 1'b1;
				end
				else begin
					state[S_SEND_ACK] <= 1'b1;
				end
				if (i2c_clk_n_edge) begin
					sda_out <= 1'b0;
					sda_en <= 1'b1;
					i2c_scl_enable <= 1'b1;
				end
			end
			state[S_END]: begin
				if (i2c_clk_p_edge) begin
					state[S_IDLE] <= 1'b1;
				end
				else begin
					state[S_END] <= 1'b1;
				end
				if (i2c_clk_n_edge) begin
					sda_out <= 1'b1;
					sda_en <= 1'b1;
					i2c_scl_enable <= 1'b0;
				end
			end
			default: begin
				sda_out <= 1'b1;
				data_mem <= {DATA_BW{1'b0}};
				addr_mem <= {(ADDR_BW+1){1'b0}};
				state <= 9'b0;
				state[S_IDLE] <= 1'b1;
				sft_cnt <= 0;
				i2c_scl_enable <= 1'b0;
				sda_en <= 1'b1;
			end
		endcase
	end
end

endmodule
		





