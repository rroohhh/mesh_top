#include <functional>
#include <variant>
#include <vector>
#include <thread>
#include <atomic>
#include <cinttypes>
#include <print>
#include <tracy/Tracy.hpp>
#include <random>
#include "memory_mapped_router_pkg.cpp"
#include "mesh_utils.cpp"

struct bar_t {
    unsigned const        count;
    std::atomic<size_t> spaces;
    std::atomic<size_t> generation;
    std::function<void(void)> completion;
    bar_t(size_t count, std::function<void(void)> completion) : count(count), spaces(count), generation(0), completion(completion) {}
    void arrive_and_wait() {
        unsigned const my_generation = generation;
        if(!--spaces) {
            spaces = count;

            completion();

            ++generation;
        } else {
            while(generation == my_generation) { asm volatile("pause"); }; // { std::this_thread::yield(); };
        }
    }
};

using u8 = uint8_t;
using u32 = uint32_t;
inline constexpr u8 operator""_u8(unsigned long long arg) noexcept { return static_cast<u8>(arg); }

#include "../cxxrtl_fst/cxxrtl_fst.h"
using namespace cxxrtl;

template<size_t bits>
auto operator<=>(const value<bits>& a, const value<bits>& b) {
    if (a == b) return std::strong_ordering::equal;
    if (a.ucmp(b)) return std::strong_ordering::less;
    return std::strong_ordering::greater;
}

// TODO(robin): currently just ignores the format specifiers
template <int bits>
struct std::formatter<value<bits>, char> : std::formatter<unsigned int> {
    auto format(const auto & s, auto & ctx) const {
        ostringstream ss;
        ss << s;
        return std::format_to(ctx.out(), "{}", s.str());
    }
};


#include "flit.cpp"
#include "router_top.h"

typedef decltype(cxxrtl_design::p_top().p_rx__link__data__north)  LinkWord;
typedef decltype(cxxrtl_design::p_top().p_rx__link__valid__north) LinkScalar;

uint64_t splitmix64(uint64_t &x) {
    uint64_t z = (x += 0x9e3779b97f4a7c15);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9;
    z = (z ^ (z >> 27)) * 0x94d049bb133111eb;
    return z ^ (z >> 31);
}

class xoshiro256pp {
public:
    using result_type = uint64_t;

    xoshiro256pp(uint64_t seed) {
        s[0] = splitmix64(seed);
        s[1] = splitmix64(seed);
        s[2] = splitmix64(seed);
        s[3] = splitmix64(seed);
    }

    static constexpr auto max = std::numeric_limits<result_type>::max;
    static constexpr auto min = std::numeric_limits<result_type>::min;

    result_type operator()() {
        const result_type result = rotl(s[0] + s[3], 23) + s[0];
        const uint64_t t = s[1] << 17;

        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];

        s[2] ^= t;

        s[3] = rotl(s[3], 45);

        return result;
    }
private:
    uint64_t s[4];

    inline uint64_t rotl(const uint64_t x, int k) {
        return (x << k) | (x >> (64 - k));
    }

};


template <class T>
concept EventTrafficModelC = requires(T a, bool did_send, T::Params params, uint64_t seed) {
    { did_send = a.did_send() };
    { T(seed, params) };
};

class PoissonEventTraffic {
    xoshiro256pp rng;
    std::discrete_distribution<int> dist;

public:
    using Params = double;

    PoissonEventTraffic(uint64_t seed, Params rate) : rng(seed), dist({1.0 - rate, rate}) {}

    bool did_send() {
        return dist(rng) == 1;
    }
};



// this models a link connections, which is several parts
// 1. the gearboxing (we need n actual link words for one traffic word)
// 2. the multiplexing with event traffic: we can only send (sub) traffic words if no event traffic is going in that instance,
//    the `EventTrafficModel` decides when this is the case
//
template <class EventTrafficModel>
class LinkConnection {
    std::vector<LinkWord>   words;
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

    EventTrafficModel event_traffic_model;

    uint64_t link_words_to_send = 0;

    static constexpr auto MUX_COUNT = memory_mapped_router_config::MUX_COUNT;
public:
    LinkConnection(uint64_t rng_seed, EventTrafficModel::Params params, LinkWord & sender_data, LinkScalar & sender_valid, LinkScalar & sender_accept,
                   LinkScalar & sender_reject, LinkScalar & sender_prio, LinkWord & receiver_data,
                   LinkScalar & receiver_valid, unsigned delay, float error_rate)
        : sender_data(sender_data), receiver_data(receiver_data), sender_valid(sender_valid),
          receiver_valid(receiver_valid), sender_accept(sender_accept), sender_reject(sender_reject),
          sender_prio(sender_prio), error_rate(error_rate), event_traffic_model(rng_seed, params)
    {
        words  = std::vector<LinkWord>(delay);
        valids = std::vector<LinkScalar>(delay);

        reader = 0;
        writer = delay - 1;

        // TODO(robin): model error rate maybe
    }

    void step() {
        bool accept = link_words_to_send == 0;
        sender_accept.set(accept);
        if (sender_valid and accept) {
            link_words_to_send += MUX_COUNT;
        }
        for (int i = 0; i < MUX_COUNT; i++) {
            if (not event_traffic_model.did_send()) {
                if (link_words_to_send > 0) {
                    link_words_to_send -= 1;
                }
            }
        }

        // sender_reject.set(0);

        words[writer]  = sender_data;
        valids[writer] = sender_valid.bit_and(value<1>{accept});

        receiver_data  = words[reader];
        receiver_valid = valids[reader];

        reader = (reader + 1) % words.size();
        writer = (writer + 1) % words.size();
    }
};

struct NodeInfo {
    cxxrtl_design::p_top & node;
    uint64_t &             timestamp;
    u8                     x, y;
    u8                     size_x, size_y;
};

template <class T>
concept NodeObserverC = requires(T a, const NodeInfo & info) {
    { T(info) };
    { a.step() };
};

template <class Design, NodeObserverC Obs, EventTrafficModelC EventTrafficModel>
class Mesh {
    struct Node {
        Design d;
        uint64_t rng_seed;
    };

    Node * nodes;
    EventTrafficModel::Params event_model_params;

    std::vector<size_t> node_indices;

    cxxrtl::fstCtx fst_ctx;
    std::mutex fst_ctx_mutex;

    std::atomic<unsigned> stepping_phase;

    size_t width, height;
    size_t num_workers;
    size_t link_delay;
    float  error_rate;

    size_t              steps = 0;
    bool                sample = true;

    std::vector<std::thread> workers;
    // std::barrier<> worker_sync_point;
    bar_t worker_sync_point;

    // TODO(robin): does this have to be a atomic? Probably not as the barrier as already a sync point
    std::atomic<bool> stop;
    std::atomic<bool> mesh_should_stop;

    mutable std::mutex worker_lock;

    void work(size_t worker) {
        tracy::SetThreadName(std::format("worker {}", worker).c_str());

        std::vector<Obs>            node_observers;
        std::vector<LinkConnection<EventTrafficModel>> links;
        size_t                      per_worker = (width * height) / num_workers;

        size_t base_node = per_worker * worker;
        size_t end_node  = per_worker * (worker + 1);

        cxxrtl::fst_writer fst_writer{fst_ctx, &fst_ctx_mutex};
        std::vector<cxxrtl::debug_items> debug_items(per_worker);

        {
            ZoneScopedN("init");

            if (sample) {
                for(int i = base_node; i < end_node; i++) {
                    auto node = node_indices[i];
                    u8 x = node / width;
                    u8 y = node % width;
                    nodes[node].d.debug_info(&debug_items[i - base_node], nullptr, "node[" + std::to_string(x) + "][" + std::to_string(y) + "] ");
                    {
                        auto lock = std::lock_guard{worker_lock};
                        // TODO(robin): rather crude hack to separate fst_writer and mesh
                        fstWriterSetComment(fst_ctx, node_attr({.x = x, .y = y}).c_str());
                        node_observers.emplace_back(NodeInfo{nodes[x * width + y].d, steps, x, y, (u8)height, (u8)width});
                        fst_writer.add_without_memories(debug_items[i - base_node]);
                    }
                }
            }


            // init step
            for(int i = base_node; i < end_node; i++) {
                auto node = node_indices[i];
                nodes[node].d.step();
            }

            // init sample
            // if (sample) fst_writer.sample(0);

            // TODO(robin): sample here
            for(int n = base_node; n < end_node; n++) {
                auto node = node_indices[n];

                nodes[node].d.p_rst.set(true);
                for(int i = 0; i < 5; i++) {
                    nodes[node].d.p_clk.set(false);
                    nodes[node].d.step();
                    nodes[node].d.p_clk.set(true);
                    nodes[node].d.step();
                }
                nodes[node].d.p_rst.set(false);

                for(int i = 0; i < 64; i++) {
                    nodes[node].d.p_clk.set(false);
                    nodes[node].d.step();
                    nodes[node].d.p_clk.set(true);
                    nodes[node].d.step();
                }
            }

            for(u8 n = base_node; n < end_node; n++) {
                auto node_idx = node_indices[n];
                u8 x = node_idx / width;
                u8 y = node_idx % width;

                auto & node = nodes[x * width + y];
                node.d.p_route__computer__position = coordinate{.x = x, .y = y};
                if (not sample) {
                    node_observers.emplace_back(NodeInfo{nodes[x * width + y].d, steps, x, y, (u8)height, (u8)width});
                }

                if(x > 0) {
                    links.emplace_back(splitmix64(node.rng_seed), event_model_params,
                        nodes[x * width + y].d.p_tx__link__data__west, nodes[x * width + y].d.p_tx__link__valid__west,
                        nodes[x * width + y].d.p_tx__accept__west, nodes[x * width + y].d.p_tx__reject__west,
                        nodes[x * width + y].d.p_tx__prio__west, nodes[(x - 1) * width + y].d.p_rx__link__data__east,
                        nodes[(x - 1) * width + y].d.p_rx__link__valid__east, link_delay, error_rate);
                }
                if(x < (height - 1)) {
                    links.emplace_back(splitmix64(node.rng_seed), event_model_params,
                        nodes[x * width + y].d.p_tx__link__data__east, nodes[x * width + y].d.p_tx__link__valid__east,
                        nodes[x * width + y].d.p_tx__accept__east, nodes[x * width + y].d.p_tx__reject__east,
                        nodes[x * width + y].d.p_tx__prio__east, nodes[(x + 1) * width + y].d.p_rx__link__data__west,
                        nodes[(x + 1) * width + y].d.p_rx__link__valid__west, link_delay, error_rate);
                }
                if(y > 0) {
                    links.emplace_back(splitmix64(node.rng_seed), event_model_params,
                        nodes[x * width + y].d.p_tx__link__data__north, nodes[x * width + y].d.p_tx__link__valid__north,
                        nodes[x * width + y].d.p_tx__accept__north, nodes[x * width + y].d.p_tx__reject__north,
                        nodes[x * width + y].d.p_tx__prio__north, nodes[x * width + y - 1].d.p_rx__link__data__south,
                        nodes[x * width + y - 1].d.p_rx__link__valid__south, link_delay, error_rate);
                }
                if(y < (width - 1)) {
                    links.emplace_back(splitmix64(node.rng_seed), event_model_params,
                        nodes[x * width + y].d.p_tx__link__data__south, nodes[x * width + y].d.p_tx__link__valid__south,
                        nodes[x * width + y].d.p_tx__accept__south, nodes[x * width + y].d.p_tx__reject__south,
                        nodes[x * width + y].d.p_tx__prio__south, nodes[x * width + y + 1].d.p_rx__link__data__north,
                        nodes[x * width + y + 1].d.p_rx__link__valid__north, link_delay, error_rate);
                }
            }
        }

        // wait for everybody to be done
        worker_sync_point.arrive_and_wait();

        while(true) {
            bool should_stop = false;

            // wait for a new step to be scheduled whenever we step
            {
                ZoneScopedN("next_step_wait");
                worker_sync_point.arrive_and_wait();
            }

            if(stop.load()) { break; }

            {
                ZoneScopedN("step");

                {
                    ZoneScopedN("step_observers");
                    for(auto & obs : node_observers) {
                      should_stop = obs.step() || should_stop;
                    }
                }

                {

                    ZoneScopedN("step_links");
                    for(auto & link : links) { link.step(); }
                }

                {
                    ZoneScopedN("user_logic_wait");
                    worker_sync_point.arrive_and_wait();
                }

                {
                    ZoneScopedN("step_router");
                    for(int i = base_node; i < end_node; i++) {
                        auto node = node_indices[i];
                        nodes[node].d.p_clk.set(false);
                        nodes[node].d.step();
                    }


                    if(sample) {
                        ZoneScopedN("fst_sample");
                        fst_writer.sample(2 * steps + 0, false);
                    }

                    {
                        ZoneScopedN("sync_half");
                        worker_sync_point.arrive_and_wait();
                    }

                    for(int i = base_node; i < end_node; i++) {
                        auto node = node_indices[i];
                        nodes[node].d.p_clk.set(true);
                        nodes[node].d.step();
                    }

                    if(sample) {
                        ZoneScopedN("fst_sample");
                        // std::println("sample");
                        fst_writer.sample(2 * steps + 1, false);
                    }
                }
            }

            if (should_stop) {
              mesh_should_stop = true;
            }

            {
                ZoneScopedN("end_of_step_wait");
                worker_sync_point.arrive_and_wait();
            }
        }
    }

    xoshiro256pp rng;
public:
    ~Mesh() {
        stop.store(true);

        worker_sync_point.arrive_and_wait();
        for(auto & worker : workers) { worker.join(); }

        delete[] nodes;

        fstWriterClose(fst_ctx);
    }
    Mesh(u8 width, u8 height, bool sample, unsigned link_delay, size_t num_workers, uint64_t rng_seed, EventTrafficModel::Params event_model_params)
        : event_model_params(event_model_params), width(width), height(height), num_workers(num_workers), link_delay(link_delay), error_rate(0.0), sample(sample), worker_sync_point{num_workers + 1, [&]() {
            if (this->sample) {
                cxxrtl::fst_writer fst_writer{fst_ctx, &fst_ctx_mutex};

                if(stepping_phase == 3) {
                    stepping_phase--;
                } else if(stepping_phase == 2) {
                    fst_writer.sample(2 * steps + 0);
                    stepping_phase--;
                } else if (stepping_phase == 1) {
                    fst_writer.sample(2 * steps + 1);
                    stepping_phase--;
                } else {
                    /* nothing */
                }
            }
        }}, rng(rng_seed) {
        assert(link_delay > 1 && "link delay > 1 needed for multithreading, etc");
        assert(((width * height) % num_workers) == 0 && "num workers must evenly divide number of nodes");


        // allocate the nodes
        nodes = new Node[width * height];
        for (size_t i = 0; i < width * height; i++) {
            nodes[i].rng_seed = rng();
        }


        // shuffle nodes across worker threads as a crude form of load balancing
        // adaptive or workstealing would probably be a lot better, but for
        // big meshes this seems to work reasonably well
        node_indices = std::vector<size_t>(width * height, 0);
        std::iota(node_indices.begin(), node_indices.end(), 0);
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(node_indices.begin(), node_indices.end(), g);

        fst_ctx = fstWriterCreate("out.fst", 0);
        fstWriterSetParallelMode(fst_ctx, true);
        fstWriterSetPackType(fst_ctx, FST_WR_PT_LZ4);
        fstWriterSetComment(fst_ctx, system_attr({.rng_seed = rng_seed}).c_str());
        // fstWriterSetRepackOnClose(fst_ctx, 1);

        // std::println("{}", node_indices);
        for(size_t worker = 0; worker < num_workers; worker++) { workers.emplace_back(&Mesh::work, this, worker); }

        worker_sync_point.arrive_and_wait();
    }

    bool step() {
        stepping_phase = 3;

        // sync up to user logic
        worker_sync_point.arrive_and_wait();

        // allow the threads to run one half step
        worker_sync_point.arrive_and_wait();

        // allow the threads to run the other half step
        worker_sync_point.arrive_and_wait();

        // wait for the step to be completed, workers are blocked until next trigger
        worker_sync_point.arrive_and_wait();

        steps++;

        return mesh_should_stop;
    }
};
