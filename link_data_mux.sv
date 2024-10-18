// comb, only muxes valid
module link_data_demux(
	axis_nb_if.slave muxed,
	axis_nb_if.master event_stream,
	axis_nb_if.master config_stream
);
	assign event_stream.data = muxed.data.data;
	assign config_stream.data = muxed.data.data;
	assign event_stream.data = muxed.valid && (muxed.data.ty == EVENT);
	assign config_stream.data = muxed.valid && (muxed.data.ty == DATA);
endmodule

// comb, priority on event data
module link_data_mux(
	axis_nb_if.master muxed,
	axis_nb_if.slave event_stream,
	axis_if.slave config_stream
);
	assign muxed.data.data = event_stream.valid ? event_stream.data : config_stream.data;
	assign muxed.data.valid = event_stream.valid ? event_stream.valid : config_stream.valid;
	assign config_stream.ready = ~event_stream.valid;
	assign muxed.data.ty = event_stream.valid ? EVENT : DATA;
endmodule
