`default_nettype none

import memory_mapped_router_config::*;

package fatmeshy_pkg;
	localparam EVENT_WIDTH    = 16;
	localparam SEQ_WIDTH    = 8;
	// localparam LINK_WORD_SIZE = EVENT_WIDTH;
	localparam LINK_WORD_SIZE = memory_mapped_router_config::ENCODED_FLIT_SIZE + SEQ_WIDTH;
	localparam NUM_PORT = 4;

	typedef enum {
		EVENT,
		CONFIG
	} link_data_type;

	parameter	 CREDIT_WIDTH = SEQ_WIDTH;
	typedef logic [CREDIT_WIDTH -1 : 0]	credit_t;
endpackage

// this ignores gearbox, etc
module top(
	input wire clk, rst,
	input wire [fatmeshy_pkg::LINK_WORD_SIZE - 1: 0] rx_link_data_north,
	input wire [fatmeshy_pkg::LINK_WORD_SIZE - 1: 0] rx_link_data_south,
	input wire [fatmeshy_pkg::LINK_WORD_SIZE - 1: 0] rx_link_data_east,
	input wire [fatmeshy_pkg::LINK_WORD_SIZE - 1: 0] rx_link_data_west,
	input wire rx_link_valid_north,
	input wire rx_link_valid_south,
	input wire rx_link_valid_east,
	input wire rx_link_valid_west,
	// input wire rx_link_type [4],

	output wire [fatmeshy_pkg::LINK_WORD_SIZE - 1: 0] tx_link_data_north,
	output wire [fatmeshy_pkg::LINK_WORD_SIZE - 1: 0] tx_link_data_south,
	output wire [fatmeshy_pkg::LINK_WORD_SIZE - 1: 0] tx_link_data_east,
	output wire [fatmeshy_pkg::LINK_WORD_SIZE - 1: 0] tx_link_data_west,
	output wire tx_link_valid_north,
	output wire tx_link_valid_south,
	output wire tx_link_valid_east,
	output wire tx_link_valid_west,
	// output wire tx_link_type,
	input wire tx_accept_north,
	input wire tx_accept_south,
	input wire tx_accept_east,
	input wire tx_accept_west,

	input wire tx_reject_north,
	input wire tx_reject_south,
	input wire tx_reject_east,
	input wire tx_reject_west,

	output wire tx_prio_north,
	output wire tx_prio_south,
	output wire tx_prio_east,
	output wire tx_prio_west,

	input coordinate route_computer_position,

	input flit in_flit,
	input wire in_valid,
	output wire in_ready,

	output flit out_flit,
	output wire out_valid,
	input wire out_ready
);
	typedef enum {
		NORTH,
		SOUTH,
		EAST,
		WEST
	} cardinal_ports;
	typedef enum {
		LOCAL =	WEST + 1
	} local_port;

	cfg_if cfg();
	assign cfg.route_computer_cfg_position = route_computer_position;

	flit_stream_if  in[LOCAL + 1]();
	flit_stream_if out[LOCAL + 1]();

	localparam L = LOCAL;
	assign in[L].valid = in_valid;
	assign in[L].payload = in_flit;
	assign in_ready = in[L].ready;

	assign out_valid = out[L].valid;
	assign out_flit = out[L].payload;
	assign out[L].ready = out_ready;

	wire [fatmeshy_pkg::LINK_WORD_SIZE - 1 : 0] rx_link_data[4];
	assign rx_link_data[NORTH] = rx_link_data_north;
	assign rx_link_data[SOUTH] = rx_link_data_south;
	assign rx_link_data[EAST] = rx_link_data_east;
	assign rx_link_data[WEST] = rx_link_data_west;

	wire rx_link_valid[4];
	assign rx_link_valid[NORTH] = rx_link_valid_north;
	assign rx_link_valid[SOUTH] = rx_link_valid_south;
	assign rx_link_valid[EAST] = rx_link_valid_east;
	assign rx_link_valid[WEST] = rx_link_valid_west;

	wire [fatmeshy_pkg::LINK_WORD_SIZE - 1 : 0] tx_link_data[4];
	assign tx_link_data_north = tx_link_data[NORTH];
	assign tx_link_data_south = tx_link_data[SOUTH];
	assign tx_link_data_east = tx_link_data[EAST];
	assign tx_link_data_west = tx_link_data[WEST];

	wire tx_link_valid[4];
	assign tx_link_valid_north = tx_link_valid[NORTH];
	assign tx_link_valid_south = tx_link_valid[SOUTH];
	assign tx_link_valid_east = tx_link_valid[EAST];
	assign tx_link_valid_west = tx_link_valid[WEST];

	wire tx_prio[4];
	assign tx_prio_north = tx_prio[NORTH];
	assign tx_prio_south = tx_prio[SOUTH];
	assign tx_prio_east = tx_prio[EAST];
	assign tx_prio_west = tx_prio[WEST];

	wire tx_accept[4];
	assign tx_accept[NORTH] = tx_accept_north;
	assign tx_accept[SOUTH] = tx_accept_south;
	assign tx_accept[EAST] = tx_accept_east;
	assign tx_accept[WEST] = tx_accept_west;

	wire tx_reject[4];
	assign tx_reject[NORTH] = tx_reject_north;
	assign tx_reject[SOUTH] = tx_reject_south;
	assign tx_reject[EAST] = tx_reject_east;
	assign tx_reject[WEST] = tx_reject_west;

	memory_mapped_router router_i(
		.clk,
		.rst,
		.cfg(cfg.slave),
		.local_in(in[LOCAL].slave),
		.local_out(out[LOCAL].master),
		.north_in(in[NORTH].slave),
		.north_out(out[NORTH].master),
		.south_in(in[SOUTH].slave),
		.south_out(out[SOUTH].master),
		.east_in(in[EAST].slave),
		.east_out(out[EAST].master),
		.west_in(in[WEST].slave),
		.west_out(out[WEST].master)
	);

	for (genvar port_idx = 0; port_idx < LOCAL; port_idx++) begin : genblk_ports
	// for (genvar port_idx = 0; port_idx < LOCAL; port_idx++) begin
		// TODO(robin):	extract this into a module <-> arq packing / unpacking
		flit rx_unpacked_flit;
		assign rx_unpacked_flit = rx_link_data[port_idx][fatmeshy_pkg::SEQ_WIDTH +: $bits(flit)];
		flit_tag rx_flit_tag;
		assign rx_flit_tag = rx_unpacked_flit.tag;

		wire tx_idx;
		wire tx_nack;
		wire [fatmeshy_pkg::LINK_WORD_SIZE - 1: 0] tx_data;
		flit tx_packed_flit, tx_packed_flit_tag_fixed;

		assign tx_packed_flit = tx_data[fatmeshy_pkg::SEQ_WIDTH +: $bits(flit)];

		assign tx_packed_flit_tag_fixed.data = tx_packed_flit.data;
		assign tx_packed_flit_tag_fixed.tag = (tx_idx == 1) ? (tx_nack ? ARQ_NACK : ARQ_ACK) : tx_packed_flit.tag;


		assign tx_link_data[port_idx][fatmeshy_pkg::SEQ_WIDTH +: $bits(flit)] = tx_packed_flit_tag_fixed;
		assign tx_link_data[port_idx][0 +: fatmeshy_pkg::SEQ_WIDTH] = tx_data[0 +: fatmeshy_pkg::SEQ_WIDTH];

		// credit_counter credit

		arq_wrap arq (
			.clock(clk),
			.reset(rst),

			// to router
			.write_valid(out[port_idx].valid),
			.write_data(out[port_idx].payload),
			.write_next(out[port_idx].ready),
			.write_idx('0),

			.read_valid(in[port_idx].valid),
			.read_data(in[port_idx].payload),
			.read_next(in[port_idx].ready),
			.read_idx(),

			// to link
			.rx_valid(rx_link_valid[port_idx]),
			.rx_data(rx_link_data[port_idx]),
			.rx_idx((rx_unpacked_flit.tag == ARQ_ACK) || (rx_unpacked_flit.tag == ARQ_NACK)),
			.rx_nack(rx_unpacked_flit.tag == ARQ_NACK),

			.tx_idx,
			.tx_valid(tx_link_valid[port_idx]),
			.tx_nack,
			.tx_data,

			.tx_accept(tx_accept[port_idx]),
			.tx_prio(tx_prio[port_idx]),
			.tx_reject(tx_reject[port_idx])
		);
	end
endmodule

module credit_counter(
	input wire clk, rst,
	input fatmeshy_pkg::credit_t credit_in,
	input wire credit_valid,
	input tx_valid,
	output tx_accept
);
	fatmeshy_pkg::credit_t credit_amount;
	always_ff @(posedge clk) begin
		if (rst) begin
			credit_amount <= 0;
		end else begin
			if (credit_valid) begin
				credit_amount <= credit_in - (tx_accept & tx_valid);
			end else begin
				credit_amount <= credit_amount - (tx_accept & tx_valid);
			end
		end
	end

	assign tx_accept = (credit_amount > 0) & tx_valid;
endmodule // credit_counter
