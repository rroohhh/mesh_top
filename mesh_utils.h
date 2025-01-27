#pragma once

#include <istream>

#include "../nlohmann/json.hpp"
using json = nlohmann::json;

struct TraceFPGABandwidthParams {
  uint32_t packet_len;
  double p;
};

struct PoissonEventTrafficParams {
  double e;
};

enum class LinkDirection
{
	North,
	South,
	East,
	West
};

std::istream& operator>> (std::istream &in, LinkDirection &dir);

void from_json(const json& j, LinkDirection& n);
void to_json(json& j, const LinkDirection& n);

struct FixedErrorModelParams {
  uint32_t first;
  uint32_t interval;
  uint8_t x;
  uint8_t y;
  LinkDirection direction;
};

struct PoissonErrorModelParams {
  double bit_error_rate;
};

template <class Node, class Event, class Error>
struct SystemAttr {
  static const constexpr char * tag = "system";
  uint64_t rng_seed;
  uint8_t width, height;
  uint8_t link_delay;
  Node::Params node_params;
  Event::Params event_params;
  Error::Params error_params;
};

struct NodeAttr {
  static const constexpr char * tag = "node";
  int x, y;
};

struct NodeRoleAttr {
  static const constexpr char * tag = "node_role";
  bool is_fpga;
};


void from_json(const json& j, TraceFPGABandwidthParams& n);
void to_json(json& j, const TraceFPGABandwidthParams& n);
void from_json(const json& j, PoissonEventTrafficParams& n);
void to_json(json& j, const PoissonEventTrafficParams& n);
void from_json(const json& j, FixedErrorModelParams& n);
void to_json(json& j, const FixedErrorModelParams& n);
void from_json(const json& j, PoissonErrorModelParams& n);
void to_json(json& j, const PoissonErrorModelParams& n);

void from_json(const json& j, NodeAttr& n);
void to_json(json& j, const NodeAttr& n);
void from_json(const json& j, NodeRoleAttr& n);
void to_json(json& j, const NodeRoleAttr& n);

template<class A, class B, class C>
void from_json(const json& j, SystemAttr<A, B, C>& n) {
    j.at("width").get_to(n.width);
    j.at("height").get_to(n.height);
    j.at("rng_seed").get_to(n.rng_seed);
    // TODO(robin): remove, this is only to read the old simulations
    try {
      j.at("link_delay").get_to(n.link_delay);
    } catch(...) {}

    j.at("node_params").get_to(n.node_params);
    j.at("event_params").get_to(n.event_params);

    // TODO(robin): remove, this is only to read the old simulations
		try {
			j.at("error_params").get_to(n.error_params);
		} catch(...) {}
}

template <class A, class B, class C>
void to_json(json& j, const SystemAttr<A, B, C>& n) {
    j = json{{"type", SystemAttr<A, B, C>::tag},
             {"width", n.width},
             {"height", n.height},
             {"rng_seed", n.rng_seed},
             {"link_delay", n.link_delay},
             {"event_params", n.event_params},
             {"error_params", n.error_params},
             {"node_params", n.node_params}};
}


template <class T>
struct json_formatter {
    constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }
    auto format(const T & v, auto & ctx) const {
      json j = v;
      return std::format_to(ctx.out(), "{}", j.dump());
    }
};

template<>
struct std::formatter<NodeAttr> : json_formatter<NodeAttr> {};
template<>
struct std::formatter<NodeRoleAttr> : json_formatter<NodeRoleAttr> {};

template<class A, class B, class C>
struct std::formatter<SystemAttr<A, B, C>> : json_formatter<NodeRoleAttr> {};

// // TODO(robin): currently just ignores the format specifiers
// template <std::enable_if>
// struct std::formatter<value<bits>, char> : std::formatter<unsigned int> {
//     auto format(const auto & s, auto & ctx) const {
//         ostringstream ss;
//         ss << s;
//         return std::format_to(ctx.out(), "{}", s.str());
//     }
// };

std::string node_attr(NodeAttr attr);
std::string node_role_attr(NodeRoleAttr attr);

template<class A, class B, class C>
std::string system_attr(SystemAttr<A, B, C> attr) {
  json j = attr;
  return j.dump();
}

// TODO(robin): support for more than just string attrs
struct SignalAttr {
  std::string name;
  std::variant<int64_t, uint64_t, double, std::string> value;
};

const auto ATTR_PREFIX = std::string_view{"attr:"};

// keep in sync with
enum attr_value_type {
	MISSING = 0,
	UINT   	= 1,
	SINT   	= 2,
	STRING 	= 3,
	DOUBLE 	= 4,
};

template<class A, class B, class C>
std::variant<NodeAttr, NodeRoleAttr, SystemAttr<A, B, C>, SignalAttr> parse_attr(const char * attr) {
  auto attr_view = std::string_view{attr};
  if (attr_view.starts_with(ATTR_PREFIX)) {
    auto name_value = attr_view.substr(ATTR_PREFIX.size());
    auto pos = name_value.find_first_of(':');

    // TODO(robin): this is only for legacy files
    if (name_value.size() > (pos + 2)) {
      if (name_value[pos + 2] != ':') {
        return SignalAttr{.name = std::string{name_value.substr(0, pos)}, .value = std::string{name_value.substr(pos + 1)}};
      } else {
        auto name = std::string{name_value.substr(0, pos)};
        auto type_value = std::string{name_value.substr(pos + 1)};
        auto pos_type = type_value.find_first_of(':');
        auto type = std::stoi(type_value.substr(0, pos_type));
        auto value = std::string{type_value.substr(pos_type + 1)};

        switch (type) {
          case attr_value_type::UINT: {
            return SignalAttr{.name = name, .value = std::stoull(value)};
          }
          case attr_value_type::SINT: {
            return SignalAttr{.name = name, .value = std::stoll(value)};
          }
          case attr_value_type::STRING: {
            return SignalAttr{.name = name, .value = value};
          }
          case attr_value_type::DOUBLE: {
            return SignalAttr{.name = name, .value = std::stod(value)};
          }
        }

        return {};
      }
    } else {
      return SignalAttr{.name = std::string{name_value.substr(0, pos)}, .value = std::string{name_value.substr(pos + 1)}};
    }
  } else {
    auto data = json::parse(attr);
    if (data.at("type") == NodeAttr::tag) {
      return {data.template get<NodeAttr>()};
    } else if (data.at("type") == NodeRoleAttr::tag) {
      return {data.template get<NodeRoleAttr>()};
    } else if (data.at("type") == SystemAttr<A, B, C>::tag) {
      return {data.template get<SystemAttr<A, B, C>>()};
    }
    return {};
  }
}

template <class T>
struct ParamsWrap
{
	using Params = T;
};
