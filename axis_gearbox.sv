module axis_gearbox(
	input wire clk, rst,
	axis_if.slave in,
	axis_if.master out
);
	typedef in.data_t in_data_t;
	typedef out.data_t out_data_t;

	localparam IN_SIZE = $size(in_data_t);
	localparam OUT_SIZE = $size(out_data_t);

	if (IN_SIZE > OUT_SIZE) begin
		assert (IN_SIZE % OUT_SIZE == 0) $error("IN_SIZE must be evenly divisible by OUT_SIZE");

		localparam LAST = (IN_SIZE / OUT_SIZE) - 1;
		int reading;
		assign out.valid = in.valid;
		assign out.data = in.data[OUT_SIZE * reading +: OUT_SIZE];
		assign in.ready = out.ready & (reading == LAST);

		always_ff @(posedge clk) begin
			if (rst) begin
				reading <= 0;
			end else begin
				if (out.valid & out.ready) begin
					if (reading == LAST) begin
						reading <= 0;
					end else begin
						reading <= reading + 1;
					end
				end
			end
		end
	end else if (IN_SIZE < OUT_SIZE) begin // for simplicitly sake this always has latency one / holds one full out_data_t register
		localparam NUM_WORDS = OUT_SIZE / IN_SIZE;
		localparam LAST = NUM_WORDS - 1;

		assert (OUT_SIZE % IN_SIZE == 0) $error("IN_SIZE must be evenly divisible by OUT_SIZE");

		in_data_t data_buffers[NUM_WORDS];

		// state encoding:
		// [0, LAST] ⇔ reading word `state`, NUM_WORDS ⇔ have a full word to present
		int state;
		assign out.data = {<<data_buffers};

		assign in.ready = (reading == NUM_WORDS) : out.ready : '1;
		assign out.valid = state == NUM_WORDS;
		always_ff @(posedge clk) begin
			if (~rst) begin
				if (in.ready & in.valid) begin
					if (reading == NUM_WORDS) begin
						data_buffers[0] <= in.data;
					end else begin
						data_buffers[reading] <= in.data;
					end
				end
			end
		end

		always_ff @(posedge clk) begin
			if (rst) begin
				reading <= 0;
			end else begin
				if (reading <= LAST) begin
					if (in.valid & in.ready) begin
						reading <= reading + 1;
					end
				end else begin
					if (out.valid & out.ready) begin
						reading <= 0;
					end
				end
			end
		end
	end else begin
		axis_passthrough passthrough_inst(.in, .out);
	end

endmodule
