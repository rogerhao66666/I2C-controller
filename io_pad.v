module io_pad
(
	input tx_data,
	input tx_en,
	input rx_data,
	inout pad_data
);

assign pad_data = tx_en ? tx_data : 1'bz;
assign rx_data = pad_data;

endmodule