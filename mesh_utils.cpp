#include "../nlohmann/json.hpp"
using json = nlohmann::json;

struct SystemAttr {
  static const constexpr char * tag = "system";
  uint64_t rng_seed;
};

struct NodeAttr {
  static const constexpr char * tag = "node";
  int x, y;
};

struct NodeRoleAttr {
  static const constexpr char * tag = "node_role";
  bool is_fpga;
  float e, p; // event and config rate
  std::string traffic_type; // only poisson currently
};

void to_json(json& j, const NodeAttr& n) {
    j = json{{"type", NodeAttr::tag}, {"x", n.x}, {"y", n.y}};
}

void from_json(const json& j, NodeAttr& n) {
    j.at("x").get_to(n.x);
    j.at("y").get_to(n.y);
}

void to_json(json& j, const NodeRoleAttr& n) {
    j = json{{"type", NodeAttr::tag}, {"is_fpga", n.is_fpga}, {"e", n.e}, {"p", n.p}, {"traffic_type", n.traffic_type}};
}

void from_json(const json& j, NodeRoleAttr& n) {
    j.at("is_fpga").get_to(n.is_fpga);
    j.at("e").get_to(n.e);
    j.at("p").get_to(n.p);
    j.at("traffic_type").get_to(n.traffic_type);
}

void to_json(json& j, const SystemAttr& n) {
    j = json{{"type", SystemAttr::tag}, {"rng_seed", n.rng_seed}};
}

void from_json(const json& j, SystemAttr& n) {
    j.at("rng_seed").get_to(n.rng_seed);
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
struct std::formatter<NodeRoleAttr> : json_formatter<NodeRoleAttr> {};

// // TODO(robin): currently just ignores the format specifiers
// template <std::enable_if>
// struct std::formatter<value<bits>, char> : std::formatter<unsigned int> {
//     auto format(const auto & s, auto & ctx) const {
//         ostringstream ss;
//         ss << s;
//         return std::format_to(ctx.out(), "{}", s.str());
//     }
// };

std::string node_attr(NodeAttr attr) {
  json j = attr;
  return j.dump();
}

std::string node_role_attr(NodeRoleAttr attr) {
  json j = attr;
  return j.dump();
}

// TODO(robin): support for more than just string attrs
struct SignalAttr {
  std::string name;
  std::string value;
};

const auto ATTR_PREFIX = std::string_view{"attr:"};

std::variant<NodeAttr, NodeRoleAttr, SignalAttr> parse_attr(const char * attr) {
  auto attr_view = std::string_view{attr};
  if (attr_view.starts_with(ATTR_PREFIX)) {
    auto name_value = attr_view.substr(ATTR_PREFIX.size());
    auto pos = name_value.find_first_of(':');
    return SignalAttr{.name = std::string{name_value.substr(0, pos)}, .value = std::string{name_value.substr(pos + 1)}};
  } else {
    auto data = json::parse(attr);
    if (data.at("type") == NodeAttr::tag) {
      return {data.template get<NodeAttr>()};
    } else if (data.at("type") == NodeRoleAttr::tag) {
      return {data.template get<NodeRoleAttr>()};
    }
    return {};
  }
}
