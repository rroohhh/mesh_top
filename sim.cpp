#include <variant>
#include <utility>
#include <cinttypes>

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

inline constexpr uint8_t operator ""_u8( unsigned long long arg ) noexcept
{
    return static_cast<uint8_t>( arg );
}

#include "router_top.h"
#include "flit.cpp"
#include <vector>
#include <fstream>
#include "../cxxrtl_fst/cxxrtl_fst.h"


typedef decltype(cxxrtl_design::p_top().p_rx__link__data__north)  LinkWord;
typedef decltype(cxxrtl_design::p_top().p_rx__link__valid__north) LinkScalar;

class LinkConnection {
    std::vector<LinkWord>  words;
    std::vector<LinkScalar> valids;

    LinkWord & sender_data;
    LinkWord & receiver_data;

    LinkScalar & sender_valid;
    LinkScalar & receiver_valid;

    LinkScalar & sender_accept;
    LinkScalar & sender_reject;
    LinkScalar & sender_prio;

    float    error_rate;
    unsigned reader, writer;

public:
    LinkConnection(LinkWord & sender_data, LinkScalar & sender_valid, LinkScalar & sender_accept, LinkScalar & sender_reject, LinkScalar & sender_prio, LinkWord & receiver_data,
                   LinkScalar & receiver_valid, unsigned delay, float error_rate)
        : sender_data(sender_data), receiver_data(receiver_data), sender_valid(sender_valid),
          receiver_valid(receiver_valid), sender_accept(sender_accept), sender_reject(sender_reject), sender_prio(sender_prio), error_rate(error_rate) {
        words  = std::vector<LinkWord>(delay);
        valids = std::vector<LinkScalar>(delay);
        reader = 0;
        writer = delay - 1;

        // TODO(robin): model error rate using these
    }

    void step() {
        sender_accept.set(1);
        sender_reject.set(0);

        words[writer]  = sender_data;
        valids[writer] = sender_valid;

        receiver_data  = words[reader];
        receiver_valid = valids[reader];

        reader = (reader + 1) % words.size();
        writer = (writer + 1) % words.size();
    }
};

struct NodeInfo {
        cxxrtl_design::p_top & node;
        uint64_t & timestamp;
        uint8_t               x, y;
        uint8_t               size_x, size_y;
};


template <class T>
concept NodeObserverC = requires(T a, const NodeInfo & info) {
    { T(info) };
    { a.step() };
};

template <NodeObserverC Obs>
class Mesh {
    // std::vector<std::vector<cxxrtl_design::p_top>> nodes;
    cxxrtl_design::p_top ** nodes;
    std::vector<Obs>                               node_observers;
    std::vector<LinkConnection>                    links;
    unsigned                                       size;
    cxxrtl::fst_writer fst_writer{"out.fst"};
    size_t steps = 0;
    cxxrtl::debug_items all_debug_items;
    bool sample = true;

public:
    Mesh(uint8_t size, bool sample, unsigned link_delay) : size(size), sample(sample) {
        assert(link_delay > 1 && "link delay > 1 needed for multithreading, etc");
        // never ever put some calls to the design in this loop, the data of the vectors still moves around and shit will be fucked
        nodes = new cxxrtl_design::p_top*[size];
        for(int x = 0; x < size; x++) {
            nodes[x] = new cxxrtl_design::p_top[size];
        }


        for(int x = 0; x < size; x++) {
            for(int y = 0; y < size; y++) {
                nodes[x][y].debug_info(&all_debug_items, nullptr, "node[" + std::to_string(x) + "][" + std::to_string(y) + "] ");
            }
        }
        if (sample)
        fst_writer.add_without_memories(all_debug_items);



        for(int x = 0; x < size; x++) { for(int y = 0; y < size; y++) { nodes[x][y].step(); } }
        if (sample)
        fst_writer.sample(0);

        for(int x = 0; x < size; x++) { for(int y = 0; y < size; y++) { nodes[x][y].p_rst.set(true); } }
        for(int i = 0; i < 5; i++) { step(); }

        // wait for quite some time after reset to allow for stuff to perculate
        for(int x = 0; x < size; x++) { for(int y = 0; y < size; y++) { nodes[x][y].p_rst.set(false); } }
        for(int i = 0; i < 64; i++) { step(); }

        for(uint8_t x = 0; x < size; x++) {
            for(uint8_t y = 0; y < size; y++) {
                nodes[x][y].p_route__computer__position = coordinate{.x = x, .y = y};
                node_observers.emplace_back(NodeInfo{nodes[x][y], steps, x, y, size, size});

                if(x > 0) {
                    links.emplace_back(nodes[x][y].p_tx__link__data__west,
                                       nodes[x][y].p_tx__link__valid__west,
                                       nodes[x][y].p_tx__accept__west,
                                       nodes[x][y].p_tx__reject__west,
                                       nodes[x][y].p_tx__prio__west,
                                       nodes[x - 1][y].p_rx__link__data__east,
                                       nodes[x - 1][y].p_rx__link__valid__east,
                                       link_delay, 0.0);
                }
                if(x < (size - 1)) {
                    links.emplace_back(nodes[x][y].p_tx__link__data__east,
                                       nodes[x][y].p_tx__link__valid__east,
                                       nodes[x][y].p_tx__accept__east,
                                       nodes[x][y].p_tx__reject__east,
                                       nodes[x][y].p_tx__prio__east,
                                       nodes[x + 1][y].p_rx__link__data__west,
                                       nodes[x + 1][y].p_rx__link__valid__west, link_delay, 0.0);
                }
                if(y > 0) {
                    links.emplace_back(nodes[x][y].p_tx__link__data__north, nodes[x][y].p_tx__link__valid__north,
                                       nodes[x][y].p_tx__accept__north,
                                       nodes[x][y].p_tx__reject__north,
                                       nodes[x][y].p_tx__prio__north,
                                       nodes[x][y - 1].p_rx__link__data__south,
                                       nodes[x][y - 1].p_rx__link__valid__south, link_delay, 0.0);
                }
                if(y < (size - 1)) {
                    links.emplace_back(nodes[x][y].p_tx__link__data__south, nodes[x][y].p_tx__link__valid__south,
                                       nodes[x][y].p_tx__accept__south,
                                       nodes[x][y].p_tx__reject__south,
                                       nodes[x][y].p_tx__prio__south,
                                       nodes[x][y + 1].p_rx__link__data__north,
                                       nodes[x][y + 1].p_rx__link__valid__north, link_delay, 0.0);
                }
            }
        }
    }

    void step() {
        // #pragma omp parallel for
        for(auto & obs : node_observers) { obs.step(); }

        // #pragma omp parallel for
        for(auto & link : links) { link.step(); }

        // #pragma omp parallel for collapse(2)
        for(int x = 0; x < size; x++) {
            for(int y = 0; y < size; y++) {
                nodes[x][y].p_clk.set(false);
                nodes[x][y].step();
            }
        }

        if(sample)
        fst_writer.sample(2 * steps + 0);

        // #pragma omp parallel for collapse(2)
        for(int x = 0; x < size; x++) {
            for(int y = 0; y < size; y++) {
                nodes[x][y].p_clk.set(true);
                nodes[x][y].step();
            }
        }

        if(sample)
        fst_writer.sample(2 * steps + 1);

        steps++;
    }
};

uint32_t prsg31(uint32_t lfsr) {
    lfsr = lfsr << 16 | (lfsr<<1 ^ lfsr<<4) >> 16;
    lfsr = lfsr << 16 | (lfsr<<1 ^ lfsr<<4) >> 16;
    return lfsr;
}

template<int bits>
struct std::formatter<value<bits>, char>
{
    constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

    auto format(const auto & s, auto & ctx) const
    {
        ostringstream ss;
        ss << s;
        return std::format_to(ctx.out(), "{}", s.str());
    }
};

int main() {
    class Node {
        NodeInfo i;
        int                   sent = 0;
        int                   maxsent = 1024;
        uint32_t lfsr_send;

    public:
        // lfsr cannot be started with 0
        Node(const NodeInfo & node_info) : i(node_info), lfsr_send(((uint32_t) i.y) << 8 | i.x + 1) {
            // TODO(robin): do we ever expect backpressure here?
            i.node.p_out__ready.set(1);
        }

        void step() {
            if(i.node.p_out__valid and i.node.p_out__ready) {
                auto decoded = flit::decode(i.node.p_out__flit);
                // std::println("{}", flit_start{});
                // std::visit([this](auto v){ std::println("[{}, {}]@0x{:x}: {}", i.x, i.y, i.timestamp, v); }, decoded);
            }

            if(i.y == 0 or true) {
                i.node.p_in__valid.set(sent != maxsent);

                // chunk_t payload = (i.timestamp & 0xff'ffff) | (lfsr_send << 24);
                if(sent != maxsent) {
                    // i.node.p_in__flit = flit{flit_start_and_end{.target = {.x = (uint8_t) (i.size_x - i.x  - 1_u8), .y = (uint8_t) (i.size_y - i.y - 1_u8)}, .payload{(uint32_t) i.timestamp, lfsr_send}}};
                    i.node.p_in__flit = flit{flit_start_and_end{.target = {.x = (uint8_t) (0), .y = (uint8_t) (0)}, .payload{(uint32_t) i.timestamp, lfsr_send}}};
                    // i.node.p_in__flit = flit{flit_start_and_end{.target = {.x = not i.x, .y = i.y}, .payload{-1u, -1u}}};

                    if(i.node.p_in__ready and i.node.p_in__valid) {
                        sent++;
                        lfsr_send = prsg31(lfsr_send);
                    }
                }
            }
        }
    };

    auto mesh = Mesh<Node>(8, true, 5);

    for(size_t i = 0; i < 10000; i++) { mesh.step(); }


    return 0;
}
