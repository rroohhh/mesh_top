#include "mesh_utils.h"

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
