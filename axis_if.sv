interface axis_nb_if #(parameter type data_t);
	logic valid;
	data_t data;

	modport master(output valid, data);
	modport slave(input valid, data);
endinterface

interface axis_if #(parameter type data_t);
	logic valid;
	logic ready;
	data_t data;

	modport master(output valid, data, input ready);
	modport slave(input valid, data, output ready);
endinterface


module axis_master_port_properties(
	input wire clk, rst,
	axis_if.slave slave
);
	// this really should be a liveness property, but bad oss support for those
	// always assert (s_eventually valid)
	cover property (slave.valid & ~slave.ready);

	always_ff @(posedge clk) begin
		if (rst) begin
			assert (~slave.valid);
		end else begin
			if ($past(slave.valid && ~slave.ready)) begin
				assert(slave.valid);
				assert($stable(slave.data));
			end
			if ($fell(slave.valid)) begin
				assert($past(slave.ready));
			end
		end
	end
endmodule

module axis_passthrough(
	axis_if.slave in,
	axis_if.master out,
);
	assign in.ready = out.ready;
	assign out.valid = in.valid;
	assign out.data = in.data;
endmodule
