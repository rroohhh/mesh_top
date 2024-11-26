#include <boost/program_options.hpp>
#include "mesh.cpp"
#include <print>

// namespace std {
//     [[noreturn]] inline void unreachable()
// {
//     // Uses compiler specific extensions if possible.
//     // Even if no extension is used, undefined behavior is still raised by
//     // an empty function body and the noreturn attribute.
// #if defined(_MSC_VER) && !defined(__clang__) // MSVC
//     __assume(false);
// #else // GCC, Clang
//     __builtin_unreachable();
// #endif
// }
// }

// template<size_t bits>
// auto operator<=>(const value<bits>& a, const value<bits>& b) {
//     if (a == b) return std::strong_ordering::equal;
//     if (a.ucmp(b)) return std::strong_ordering::less;
//     return std::strong_ordering::greater;
// }

typedef uint16_t payload_t;

// generates traffic and can verify it
class TrafficVerification {
    coordinate                      self;
    std::map<coordinate, payload_t> send_state;
    std::map<coordinate, payload_t> receive_state;

    payload_t next_state(payload_t this_state) {
        this_state ^= this_state << 7;
        this_state ^= this_state >> 9;
        this_state ^= this_state << 8;
        return this_state;
    }
    // uint32_t prsg31(uint32_t lfsr) {
    //     lfsr = lfsr << 16 | (lfsr << 1 ^ lfsr << 4) >> 16;
    //     lfsr = lfsr << 16 | (lfsr << 1 ^ lfsr << 4) >> 16;
    //     return lfsr;
    // }

    payload_t seed_payload(coordinate target) {
        value<16> a = target;
        value<16> b = self;
        uint16_t a_int = a.get<uint16_t>();
        uint16_t b_int = b.get<uint16_t>();
        return a_int << 4 ^ b_int;
    }

public:
    TrafficVerification(coordinate self) : self(self) {}

    bool receive_from(coordinate sender, payload_t payload) {
        auto state = receive_state.find(sender);
        if(state == receive_state.end()) {
            receive_state.insert({sender, payload});
            return true;
        } else {
            auto old      = state->second;
            auto expected = next_state(old);
            state->second = payload;
            return expected == payload;
        };
    }

    payload_t send_to_peek(coordinate target) {
        auto state = send_state.find(target);
        if(state == send_state.end()) {
            auto payload = seed_payload(target);
            send_state.insert({target, payload});
            return payload;
        } else {
            return next_state(state->second);
        }
    }
    payload_t send_to_pop(coordinate target) {
        auto state = send_state.find(target);
        if(state == send_state.end()) {
            auto payload = seed_payload(target);
            send_state.insert({target, payload});
            return payload;
        } else {
            state->second = next_state(state->second);
            return state->second;
        }
    }
};

template<typename T>
struct is_value {
    static constexpr bool value = false;
};

template<size_t bits>
struct is_value<value<bits>> {
    static constexpr bool value = true;
};


template<class T, class IntT>
auto payload_from_int(IntT v) {
    using res_type = decltype(T{}.payload);
    value<std::numeric_limits<IntT>::digits> as_value;
    as_value.set(v);
    res_type res;
    return res.bit_or(as_value.template zcast<res_type::bits>());
}


class TraceFPGABandwidth {
    NodeInfo i;
public:
  using Params = TraceFPGABandwidthParams;
private:
    Params params;

    // state
    uint32_t      sent_flits  = 0;
    uint32_t      sent        = 0;
    uint32_t      to_send     = 0;
    bool          is_fpga;

    TrafficVerification traffic_gen;

  // shared ptr to have stable address for fst_writer
    std::unique_ptr<value<32>> flit_latency = std::make_unique<value<32>>();
    std::unique_ptr<value<32>> flits_received = std::make_unique<value<32>>();
    std::unique_ptr<value<32>> flits_sent = std::make_unique<value<32>>();
    std::unique_ptr<value<32>> packets_sent = std::make_unique<value<32>>();
    std::unique_ptr<value<32>> packets_to_send = std::make_unique<value<32>>();
public:
    // lfsr cannot be started with 0
    TraceFPGABandwidth(const NodeInfo & node_info, const Params & params) : i(node_info), params(params), is_fpga(i.y == 0), traffic_gen({.x = i.x, .y = i.y}) {
        fstWriterSetComment(i.fst_ctx, node_role_attr({.is_fpga = is_fpga}).c_str());
        i.fst_writer.add(i.hier_prefix + "flit_latency", *flit_latency, {});
        i.fst_writer.add(i.hier_prefix + "flits_received", *flits_received, {});
        i.fst_writer.add(i.hier_prefix + "flits_sent", *flits_sent, {});
        i.fst_writer.add(i.hier_prefix + "packets_sent", *packets_sent, {});
        i.fst_writer.add(i.hier_prefix + "packets_to_send", *packets_to_send, {});
    }

    bool step() {
        if(is_fpga) {
            // assume infinite FPGA bandwidth
            i.node.p_out__ready.set(1);

            if(i.node.p_out__valid and i.node.p_out__ready) {
                flits_received->set(flits_received->get<uint32_t>() + 1);
                auto decoded = flit::decode(i.node.p_out__flit);
                uint64_t payload = std::visit([](auto v){
                    if constexpr (is_value<decltype(v.payload)>::value) {
                        // TODO(robin): can we replace this hardcoded 64 anytime?
                        return v.payload.template zcast<64>().template get<uint64_t>();
                    } else {
                        return v.payload;
                    }
                }, decoded);
                auto timestamp = payload >> 32;
                u8 x = (payload >> 24) & 0xFF;
                u8 y = (payload >> 16) & 0xFF;
                payload = payload & 0xFFFF;

                flit_latency->set(i.timestamp - timestamp);
                // std::println("[{}, {}]@{: 6}: got [{}, {}]@{: 6} {:#06x}", i.x, i.y, i.timestamp, x, y, timestamp, payload);
                if (not traffic_gen.receive_from(coordinate{.x = x, .y = y}, payload)) {
                    std::println("mismatch");
                    return true;
                }
                // std::println("{:#16x}", payload);
            }
        } else {
            // non poissonian to emulate constant ADC sampling
            to_send = i.timestamp * params.p;
            if((sent < to_send) && sent_flits == 0) { sent_flits = params.packet_len; }

            coordinate closest_fpga{.x = i.x, .y = 0};
            uint64_t payload = ((uint64_t)i.timestamp << 32) | ((u32) i.x) << 24 | ((u32) i.y) << 16 | traffic_gen.send_to_peek(closest_fpga);
            if(params.packet_len == 1) {
                i.node.p_in__flit =
                    flit{flit_start_and_end{.target = closest_fpga, .payload{payload_from_int<flit_start_and_end>(payload)}}};
            } else {
                if(sent_flits == params.packet_len) {
                    i.node.p_in__flit =
                        flit{flit_start{.target = closest_fpga, .payload{payload_from_int<flit_start>(payload)}}};
                } else if(sent_flits > 1) {
                    i.node.p_in__flit = flit{flit_payload{payload_from_int<flit_payload>(payload)}};
                } else if(sent_flits == 1) {
                    i.node.p_in__flit = flit{flit_tail{payload_from_int<flit_tail>(payload)}};
                } else {
                    i.node.p_in__flit = flit{flit_start{}};
                }
            }

            i.node.p_in__valid.set(sent_flits != 0);

            if(i.node.p_in__ready and i.node.p_in__valid) {
                flits_sent->set(flits_sent->get<uint32_t>() + 1);

                sent_flits--;
                traffic_gen.send_to_pop(closest_fpga);

                if(sent_flits == 0) { sent++; }
            }

            // std::println("[{},{}] sent {}, to_send {}", i.x, i.y, sent, to_send);
            packets_sent->set(sent);
            packets_to_send->set(to_send);


        }
        return false;
    }
};

namespace po = boost::program_options;
int main(int ac, char ** av) {
    po::options_description desc("Allowed options");
    double e, p;
    uint64_t rng_seed;
    uint32_t width, height;
    std::string filename;
    uint32_t steps;
    uint32_t packet_len;

    // TODO(robin): configure link latency
    desc.add_options()
        ("help", "produce help message")
        ("event_rate", po::value<double>(&e)->required(), "event rate")
        ("config_rate", po::value<double>(&p)->required(), "config rate")
        ("rng_seed", po::value<uint64_t>(&rng_seed)->required(), "rng seed")
        ("width", po::value<uint32_t>(&width)->required(), "mesh width")
        ("height", po::value<uint32_t>(&height)->required(), "mesh height")
        ("out", po::value<std::string>(&filename)->required(), "out filename")
        ("packet_len", po::value<uint32_t>(&packet_len)->required(), "number of flits per packet")
        ("steps", po::value<uint32_t>(&steps)->required(), "number of steps")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);


    std::println("height {}, width {}", height, width);

    auto mesh = Mesh<cxxrtl_design::p_top, TraceFPGABandwidth, PoissonEventTraffic>(filename, height, width, true, 5, 1, rng_seed, {.e = e}, {.packet_len = packet_len, .p = p});
    // auto mesh = Mesh<TraceFPGABandwidth>(3, true, 5, 1);

    for(size_t i = 0; i < steps; i++) {
        ZoneScopedN("main_step");
        if(mesh.step()) {
            break;
        }
        if(i % 1000 == 0) { std::println("step {}", i); }
    }

    return 0;
}

// baseline 4.6 seconds  for 10'000 cycles w/o sample
//
// 1 threads 5.6 seconds  for 10'000 cycles w/o sample?
// 4 threads 2.4 seconds  for 10'000 cycles w/o sample?
// 8 threads 1.2 seconds  for 10'000 cycles w/o sample?

// hartmut / nauth : async irregular spiking
// link data utilization ()
