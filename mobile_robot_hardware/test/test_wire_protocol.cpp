#include "mobile_robot_hardware/wire_protocol.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

using mobile_robot_hardware::parseHandshakeReply;

namespace {

// @return: true if `line` parses as a reply with `prefix`
bool parses(const std::string &line, const std::string &prefix,
            uint32_t *id = nullptr, std::string *rest = nullptr) {
  uint32_t parsedId = 0;
  std::string parsedRest;
  const bool ok = parseHandshakeReply(line, prefix, parsedId, parsedRest);
  if (id) {
    *id = parsedId;
  }
  if (rest) {
    *rest = parsedRest;
  }
  return ok;
}

} // namespace

TEST(ParseHandshakeReply, Ack) {
  uint32_t id = 0;
  std::string rest = "x";
  ASSERT_TRUE(parses("S,ACK,12345", "S,ACK,", &id, &rest));
  EXPECT_EQ(id, 12345u);
  EXPECT_EQ(rest, "");
}

TEST(ParseHandshakeReply, NackWithField) {
  uint32_t id = 0;
  std::string rest;
  ASSERT_TRUE(parses("S,NACK,42,slope_left", "S,NACK,", &id, &rest));
  EXPECT_EQ(id, 42u);
  EXPECT_EQ(rest, "slope_left");
}

TEST(ParseHandshakeReply, RejectsOtherPrefix) {
  EXPECT_FALSE(parses("S,NACK,42,kp", "S,ACK,"));
  EXPECT_FALSE(parses("S,SETUP", "S,ACK,"));
  EXPECT_FALSE(parses("L,S,ACK,1", "S,ACK,"));
}

TEST(ParseHandshakeReply, RejectsMissingOrMalformedId) {
  EXPECT_FALSE(parses("S,ACK,", "S,ACK,"));
  EXPECT_FALSE(parses("S,ACK,,kp", "S,ACK,"));
  EXPECT_FALSE(parses("S,ACK,abc", "S,ACK,"));
  EXPECT_FALSE(parses("S,ACK,12x", "S,ACK,"));
  EXPECT_FALSE(parses("S,ACK,1.5", "S,ACK,"));
}
