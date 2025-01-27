#include "mesh_utils.h"


void from_json(const json& j, LinkDirection& n) {
  std::stringstream ss(j.get<std::string>());
  ss >> n;
}

void to_json(json& j, const LinkDirection& n) {
  switch(n) {
    case LinkDirection::North: {
      j = "north";
      break;
    }
    case LinkDirection::South: {
      j = "south";
      break;
    }
    case LinkDirection::East: {
      j = "east";
      break;
    }
    case LinkDirection::West: {
      j = "west";
      break;
    }
  }
}

std::istream& operator>> (std::istream &in, LinkDirection &target) {
  std::string dir;
  in >> dir;
  if (dir == "north") {
    target = LinkDirection::North;
  } else if (dir == "south") {
    target = LinkDirection::South;
  } else if (dir == "east") {
    target = LinkDirection::East;
  } else if (dir == "west") {
    target = LinkDirection::West;
  } else {
    assert(false);
  }
  return in;
}

void from_json(const json& j, TraceFPGABandwidthParams& n) {
    j.at("p").get_to(n.p);
    j.at("packet_len").get_to(n.packet_len);
}

void to_json(json& j, const TraceFPGABandwidthParams& n) {
    j = json{{"p", n.p}, {"packet_len", n.packet_len}};
}

void from_json(const json& j, PoissonEventTrafficParams& n) {
    j.at("e").get_to(n.e);
}

void to_json(json& j, const PoissonEventTrafficParams& n) {
    j = json{{"e", n.e}};
}

void from_json(const json& j, FixedErrorModelParams& n) {
    j.at("first").get_to(n.first);
    j.at("interval").get_to(n.interval);
    j.at("x").get_to(n.x);
    j.at("y").get_to(n.y);
    j.at("direction").get_to(n.direction);
}

void to_json(json& j, const FixedErrorModelParams& n) {
    j = json{
      {"first", n.first},
      {"interval", n.interval},
      {"x", n.x},
      {"y", n.y},
      {"direction", n.direction},
    };
}

void from_json(const json& j, PoissonErrorModelParams& n) {
    j.at("bit_error_rate").get_to(n.bit_error_rate);
}

void to_json(json& j, const PoissonErrorModelParams& n) {
    j = json{
      {"bit_error_rate", n.bit_error_rate},
    };
}

void to_json(json& j, const NodeAttr& n) {
    j = json{{"type", NodeAttr::tag}, {"x", n.x}, {"y", n.y}};
}

void from_json(const json& j, NodeAttr& n) {
    j.at("x").get_to(n.x);
    j.at("y").get_to(n.y);
}

void to_json(json& j, const NodeRoleAttr& n) {
    j = json{{"type", NodeRoleAttr::tag}, {"is_fpga", n.is_fpga}};
}

void from_json(const json& j, NodeRoleAttr& n) {
    j.at("is_fpga").get_to(n.is_fpga);
}

std::string node_attr(NodeAttr attr) {
  json j = attr;
  return j.dump();
}

std::string node_role_attr(NodeRoleAttr attr) {
  json j = attr;
  return j.dump();
}
