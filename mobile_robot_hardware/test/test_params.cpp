#include "mobile_robot_hardware/params.hpp"

#include <gtest/gtest.h>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

using hardware_interface::ComponentInfo;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::InterfaceInfo;
using mobile_robot_hardware::detail::ParsedParams;
using mobile_robot_hardware::detail::parseParam;
using mobile_robot_hardware::detail::parseParams;
using mobile_robot_hardware::detail::ticksToRad;
using mobile_robot_hardware::detail::validateJoints;

namespace {

using ParamMap = std::unordered_map<std::string, std::string>;

const std::vector<std::string> k_expectedJoints = {"left_wheel_joint",
                                                   "right_wheel_joint"};

// the <param> block of mobile_robot_nodes/description/ros2_control.xacro
ParamMap validParams() {
  return {
      {"serial_port", "/dev/esp32"},
      {"baud_rate", "115200"},
      {"pid_kp", "0.5"},
      {"pid_ki", "2.0"},
      {"pid_integral_limit", "0.3"},
      {"ff_slope_left", "0.1"},
      {"ff_slope_right", "0.2"},
      {"ff_min_duty_left", "0.05"},
      {"ff_min_duty_right", "0.0"},
      {"ticks_per_wheel_rev", "480"},
      {"watchdog_timeout_ms", "200"},
      {"control_rate_hz", "200"},
      {"report_rate_hz", "50"},
      {"invert_left", "true"},
      {"invert_right", "false"},
  };
}

InterfaceInfo interface(const std::string &name) {
  InterfaceInfo info;
  info.name = name;
  return info;
}

// a joint as the ros2_control URDF declares it
ComponentInfo wheelJoint(const std::string &name) {
  ComponentInfo joint;
  joint.name = name;
  joint.type = "joint";
  joint.command_interfaces = {interface(HW_IF_VELOCITY)};
  joint.state_interfaces = {interface(HW_IF_VELOCITY),
                            interface(HW_IF_POSITION)};
  return joint;
}

std::vector<ComponentInfo> validJoints() {
  return {wheelJoint("left_wheel_joint"), wheelJoint("right_wheel_joint")};
}

} // namespace

///////////////////////////////////////////////////////////////////////
// parseParam
///////////////////////////////////////////////////////////////////////

TEST(ParseParam, MissingKeyThrowsNamingTheParameter) {
  const ParamMap params = {{"other", "1"}};
  try {
    parseParam<float>(params, "pid_kp");
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("pid_kp"), std::string::npos);
  }
}

TEST(ParseParam, StringIsReturnedVerbatim) {
  EXPECT_EQ(parseParam<std::string>({{"p", "/dev/esp32"}}, "p"), "/dev/esp32");
}

TEST(ParseParam, EmptyStringIsAValidString) {
  EXPECT_EQ(parseParam<std::string>({{"p", ""}}, "p"), "");
}

TEST(ParseParam, EmptyNumberThrows) {
  EXPECT_THROW(parseParam<float>({{"p", ""}}, "p"), std::runtime_error);
  EXPECT_THROW(parseParam<uint32_t>({{"p", ""}}, "p"), std::runtime_error);
}

TEST(ParseParam, MalformedNumberThrows) {
  EXPECT_THROW(parseParam<float>({{"p", "abc"}}, "p"), std::runtime_error);
  EXPECT_THROW(parseParam<uint16_t>({{"p", "abc"}}, "p"), std::runtime_error);
}

// "0.5x" must not silently become 0.5, a typo in the URDF has to fail loudly
TEST(ParseParam, TrailingGarbageThrows) {
  EXPECT_THROW(parseParam<float>({{"p", "0.5x"}}, "p"), std::runtime_error);
  EXPECT_THROW(parseParam<uint32_t>({{"p", "115200baud"}}, "p"),
               std::runtime_error);
}

TEST(ParseParam, FloatParsesDecimalAndExponent) {
  EXPECT_NEAR(parseParam<float>({{"p", "0.5"}}, "p"), 0.5f, 1e-7f);
  EXPECT_NEAR(parseParam<float>({{"p", "-2.25"}}, "p"), -2.25f, 1e-7f);
  EXPECT_NEAR(parseParam<float>({{"p", "1e-3"}}, "p"), 1e-3f, 1e-10f);
  EXPECT_NEAR(parseParam<float>({{"p", "480"}}, "p"), 480.0f, 1e-7f);
}

// stod() accepts it, but a float can't hold it and would become inf
TEST(ParseParam, FloatOutOfRangeThrows) {
  EXPECT_THROW(parseParam<float>({{"p", "1e39"}}, "p"), std::runtime_error);
  EXPECT_THROW(parseParam<float>({{"p", "-1e39"}}, "p"), std::runtime_error);
}

TEST(ParseParam, NonFiniteFloatThrows) {
  EXPECT_THROW(parseParam<float>({{"p", "nan"}}, "p"), std::runtime_error);
  EXPECT_THROW(parseParam<float>({{"p", "inf"}}, "p"), std::runtime_error);
}

TEST(ParseParam, UnsignedBoundariesAreAccepted) {
  EXPECT_EQ(parseParam<uint32_t>({{"p", "0"}}, "p"), 0u);
  EXPECT_EQ(parseParam<uint32_t>({{"p", "4294967295"}}, "p"), UINT32_MAX);
  EXPECT_EQ(parseParam<uint16_t>({{"p", "65535"}}, "p"), UINT16_MAX);
}

TEST(ParseParam, NegativeUnsignedThrows) {
  EXPECT_THROW(parseParam<uint32_t>({{"p", "-1"}}, "p"), std::runtime_error);
  EXPECT_THROW(parseParam<uint16_t>({{"p", "-1"}}, "p"), std::runtime_error);
}

TEST(ParseParam, AboveIntegerMaxThrows) {
  EXPECT_THROW(parseParam<uint32_t>({{"p", "4294967296"}}, "p"),
               std::runtime_error);
  EXPECT_THROW(parseParam<uint16_t>({{"p", "65536"}}, "p"), std::runtime_error);
}

// "200.5" for control_rate_hz must not silently run the loop at 200 Hz
TEST(ParseParam, FractionalIntegerThrows) {
  EXPECT_THROW(parseParam<uint16_t>({{"p", "200.5"}}, "p"), std::runtime_error);
  EXPECT_THROW(parseParam<uint32_t>({{"p", "0.5"}}, "p"), std::runtime_error);
}

TEST(ParseParam, IntegerWrittenAsWholeFloatIsAccepted) {
  EXPECT_EQ(parseParam<uint16_t>({{"p", "200.0"}}, "p"), 200u);
}

TEST(ParseParam, BoolParsesTrueAndFalse) {
  EXPECT_TRUE(parseParam<bool>({{"p", "true"}}, "p"));
  EXPECT_FALSE(parseParam<bool>({{"p", "false"}}, "p"));
  // xacro and hand-written URDFs both use capitalized spellings
  EXPECT_TRUE(parseParam<bool>({{"p", "True"}}, "p"));
  EXPECT_FALSE(parseParam<bool>({{"p", "FALSE"}}, "p"));
}

TEST(ParseParam, BoolOtherValueThrows) {
  EXPECT_THROW(parseParam<bool>({{"p", "1"}}, "p"), std::runtime_error);
  EXPECT_THROW(parseParam<bool>({{"p", "yes"}}, "p"), std::runtime_error);
  EXPECT_THROW(parseParam<bool>({{"p", ""}}, "p"), std::runtime_error);
}

///////////////////////////////////////////////////////////////////////
// parseParams
///////////////////////////////////////////////////////////////////////

TEST(ParseParams, ValidMapGivesExpectedStruct) {
  const ParsedParams parsed = parseParams(validParams());

  EXPECT_EQ(parsed.serialPort, "/dev/esp32");
  EXPECT_EQ(parsed.baudRate, 115200u);
  EXPECT_NEAR(parsed.config.kp, 0.5f, 1e-6f);
  EXPECT_NEAR(parsed.config.ki, 2.0f, 1e-6f);
  EXPECT_NEAR(parsed.config.integral_limit, 0.3f, 1e-6f);
  EXPECT_NEAR(parsed.config.slope_left, 0.1f, 1e-6f);
  EXPECT_NEAR(parsed.config.slope_right, 0.2f, 1e-6f);
  EXPECT_NEAR(parsed.config.min_duty_left, 0.05f, 1e-6f);
  EXPECT_NEAR(parsed.config.min_duty_right, 0.0f, 1e-6f);
  EXPECT_NEAR(parsed.config.ticks_per_wheel_rev, 480.0f, 1e-6f);
  EXPECT_EQ(parsed.config.watchdog_timeout_ms, 200u);
  EXPECT_EQ(parsed.config.control_rate_hz, 200u);
  EXPECT_EQ(parsed.config.report_rate_hz, 50u);
  EXPECT_TRUE(parsed.config.invert_left);
  EXPECT_FALSE(parsed.config.invert_right);
}

TEST(ParseParams, BaudRateZeroThrows) {
  ParamMap params = validParams();
  params["baud_rate"] = "0";
  EXPECT_THROW(parseParams(params), std::runtime_error);
}

// handshake_timeout_ms is optional, validParams() leaves it out

TEST(ParseParams, HandshakeTimeoutMissingLeavesItUnset) {
  const ParsedParams parsed = parseParams(validParams());
  // on_init() falls back to its 10 s default
  EXPECT_FALSE(parsed.handshakeTimeout.has_value());
}

TEST(ParseParams, HandshakeTimeoutIsReadInMilliseconds) {
  ParamMap params = validParams();
  params["handshake_timeout_ms"] = "5000";
  const ParsedParams parsed = parseParams(params);
  ASSERT_TRUE(parsed.handshakeTimeout.has_value());
  EXPECT_EQ(*parsed.handshakeTimeout, std::chrono::milliseconds(5000));
}

TEST(ParseParams, HandshakeTimeoutBoundariesAreAccepted) {
  ParamMap params = validParams();
  params["handshake_timeout_ms"] = "1";
  EXPECT_EQ(parseParams(params).handshakeTimeout, std::chrono::milliseconds(1));
  params["handshake_timeout_ms"] = "4294967295";
  EXPECT_EQ(parseParams(params).handshakeTimeout,
            std::chrono::milliseconds(UINT32_MAX));
}

TEST(ParseParams, HandshakeTimeoutWrittenAsWholeFloatIsAccepted) {
  ParamMap params = validParams();
  params["handshake_timeout_ms"] = "200.0";
  EXPECT_EQ(parseParams(params).handshakeTimeout,
            std::chrono::milliseconds(200));
  params["handshake_timeout_ms"] = "1e3";
  EXPECT_EQ(parseParams(params).handshakeTimeout,
            std::chrono::milliseconds(1000));
}

// a 0 ms timeout would give up before the first Config is sent
TEST(ParseParams, HandshakeTimeoutZeroThrowsNamingTheParameter) {
  ParamMap params = validParams();
  params["handshake_timeout_ms"] = "0";
  try {
    parseParams(params);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("handshake_timeout_ms"),
              std::string::npos)
        << e.what();
  }
}

TEST(ParseParams, HandshakeTimeoutNegativeThrows) {
  ParamMap params = validParams();
  for (const std::string value : {"-1", "-5000"}) {
    params["handshake_timeout_ms"] = value;
    EXPECT_THROW(parseParams(params), std::runtime_error) << value;
  }
}

TEST(ParseParams, HandshakeTimeoutAboveUint32MaxThrows) {
  ParamMap params = validParams();
  for (const std::string value : {"4294967296", "1e20"}) {
    params["handshake_timeout_ms"] = value;
    EXPECT_THROW(parseParams(params), std::runtime_error) << value;
  }
}

// "200.5" must not silently become 200 ms
TEST(ParseParams, HandshakeTimeoutFractionalThrows) {
  ParamMap params = validParams();
  for (const std::string value : {"0.5", "200.5"}) {
    params["handshake_timeout_ms"] = value;
    EXPECT_THROW(parseParams(params), std::runtime_error) << value;
  }
}

// stod() doesn't trim, a stray space in the URDF value is rejected
TEST(ParseParams, HandshakeTimeoutSurroundingWhitespaceThrows) {
  ParamMap params = validParams();
  for (const std::string value : {" 200", "200 ", "\t200"}) {
    params["handshake_timeout_ms"] = value;
    EXPECT_THROW(parseParams(params), std::runtime_error) << value;
  }
}

TEST(ParseParams, HandshakeTimeoutMalformedThrowsNamingTheParameter) {
  ParamMap params = validParams();
  for (const std::string value : {"", "soon", "200ms", "nan", "inf"}) {
    params["handshake_timeout_ms"] = value;
    try {
      parseParams(params);
      FAIL() << "expected std::runtime_error for '" << value << "'";
    } catch (const std::runtime_error &e) {
      EXPECT_NE(std::string(e.what()).find("handshake_timeout_ms"),
                std::string::npos)
          << e.what();
    }
  }
}

TEST(ParseParams, EveryMissingParameterThrows) {
  for (const auto &[name, value] : validParams()) {
    ParamMap params = validParams();
    params.erase(name);
    EXPECT_THROW(parseParams(params), std::runtime_error) << name;
  }
}

// one row per field validateConfig() checks: a value just outside the
// field's limit must make parseParams fail and name the field
struct RejectedField {
  std::string param;
  std::string value;
  std::string fieldName; // as configFieldName() reports it
};

class ParseParamsRejects : public ::testing::TestWithParam<RejectedField> {};

TEST_P(ParseParamsRejects, FieldOutOfRange) {
  const RejectedField &row = GetParam();
  ParamMap params = validParams();
  params[row.param] = row.value;
  try {
    parseParams(params);
    FAIL() << "expected std::runtime_error";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("'" + row.fieldName + "'"),
              std::string::npos)
        << e.what();
  }
}

INSTANTIATE_TEST_SUITE_P(
    ValidateConfig, ParseParamsRejects,
    ::testing::Values(
        RejectedField{"pid_kp", "-0.001", "kp"},
        RejectedField{"pid_ki", "-0.001", "ki"},
        RejectedField{"pid_integral_limit", "-0.001", "integral_limit"},
        // the slope is a divisor in the feedforward, 0 is excluded
        RejectedField{"ff_slope_left", "0", "slope_left"},
        RejectedField{"ff_slope_right", "0", "slope_right"},
        RejectedField{"ff_min_duty_left", "1.0", "min_duty_left"},
        RejectedField{"ff_min_duty_right", "-0.001", "min_duty_right"},
        RejectedField{"ticks_per_wheel_rev", "0", "ticks_per_wheel_rev"},
        RejectedField{"watchdog_timeout_ms", "5001", "watchdog_timeout_ms"},
        RejectedField{"control_rate_hz", "0", "control_rate_hz"},
        // faster than control_rate_hz (200) would report duplicate samples
        RejectedField{"report_rate_hz", "201", "report_rate_hz"}),
    [](const ::testing::TestParamInfo<RejectedField> &info) {
      return info.param.param;
    });

///////////////////////////////////////////////////////////////////////
// validateJoints
///////////////////////////////////////////////////////////////////////

TEST(ValidateJoints, ExpectedJointsAndInterfacesPass) {
  EXPECT_NO_THROW(validateJoints(validJoints(), k_expectedJoints));
}

// the URDF order is up to the author, read()/write() look joints up by name
TEST(ValidateJoints, JointOrderDoesNotMatter) {
  const std::vector<ComponentInfo> joints = {wheelJoint("right_wheel_joint"),
                                             wheelJoint("left_wheel_joint")};
  EXPECT_NO_THROW(validateJoints(joints, k_expectedJoints));
}

TEST(ValidateJoints, StateInterfaceOrderDoesNotMatter) {
  std::vector<ComponentInfo> joints = validJoints();
  joints[0].state_interfaces = {interface(HW_IF_POSITION),
                                interface(HW_IF_VELOCITY)};
  EXPECT_NO_THROW(validateJoints(joints, k_expectedJoints));
}

TEST(ValidateJoints, NoJointsThrows) {
  EXPECT_THROW(validateJoints({}, k_expectedJoints), std::runtime_error);
}

TEST(ValidateJoints, TooFewJointsThrows) {
  const std::vector<ComponentInfo> joints = {wheelJoint("left_wheel_joint")};
  EXPECT_THROW(validateJoints(joints, k_expectedJoints), std::runtime_error);
}

TEST(ValidateJoints, TooManyJointsThrows) {
  std::vector<ComponentInfo> joints = validJoints();
  joints.push_back(wheelJoint("caster_joint"));
  EXPECT_THROW(validateJoints(joints, k_expectedJoints), std::runtime_error);
}

TEST(ValidateJoints, WrongJointNameThrows) {
  const std::vector<ComponentInfo> joints = {wheelJoint("left_wheel_joint"),
                                             wheelJoint("rear_wheel_joint")};
  EXPECT_THROW(validateJoints(joints, k_expectedJoints), std::runtime_error);
}

// right count, but one wheel declared twice and the other one missing
TEST(ValidateJoints, DuplicateJointThrows) {
  const std::vector<ComponentInfo> joints = {wheelJoint("left_wheel_joint"),
                                             wheelJoint("left_wheel_joint")};
  EXPECT_THROW(validateJoints(joints, k_expectedJoints), std::runtime_error);
}

TEST(ValidateJoints, MissingCommandInterfaceThrows) {
  std::vector<ComponentInfo> joints = validJoints();
  joints[1].command_interfaces.clear();
  EXPECT_THROW(validateJoints(joints, k_expectedJoints), std::runtime_error);
}

TEST(ValidateJoints, WrongCommandInterfaceThrows) {
  std::vector<ComponentInfo> joints = validJoints();
  joints[0].command_interfaces = {interface(HW_IF_POSITION)};
  EXPECT_THROW(validateJoints(joints, k_expectedJoints), std::runtime_error);
}

TEST(ValidateJoints, ExtraCommandInterfaceThrows) {
  std::vector<ComponentInfo> joints = validJoints();
  joints[0].command_interfaces.push_back(interface(HW_IF_POSITION));
  EXPECT_THROW(validateJoints(joints, k_expectedJoints), std::runtime_error);
}

TEST(ValidateJoints, MissingPositionStateInterfaceThrows) {
  std::vector<ComponentInfo> joints = validJoints();
  joints[0].state_interfaces = {interface(HW_IF_VELOCITY)};
  EXPECT_THROW(validateJoints(joints, k_expectedJoints), std::runtime_error);
}

TEST(ValidateJoints, MissingVelocityStateInterfaceThrows) {
  std::vector<ComponentInfo> joints = validJoints();
  joints[1].state_interfaces = {interface(HW_IF_POSITION)};
  EXPECT_THROW(validateJoints(joints, k_expectedJoints), std::runtime_error);
}

TEST(ValidateJoints, ExtraStateInterfaceThrows) {
  std::vector<ComponentInfo> joints = validJoints();
  joints[1].state_interfaces.push_back(
      interface(hardware_interface::HW_IF_EFFORT));
  EXPECT_THROW(validateJoints(joints, k_expectedJoints), std::runtime_error);
}

// two entries, but the same one twice: the count alone isn't enough
TEST(ValidateJoints, DuplicateStateInterfaceThrows) {
  std::vector<ComponentInfo> joints = validJoints();
  joints[0].state_interfaces = {interface(HW_IF_VELOCITY),
                                interface(HW_IF_VELOCITY)};
  EXPECT_THROW(validateJoints(joints, k_expectedJoints), std::runtime_error);
}

///////////////////////////////////////////////////////////////////////
// ticksToRad
///////////////////////////////////////////////////////////////////////

TEST(TicksToRad, ZeroTicksIsZero) {
  EXPECT_NEAR(ticksToRad(0, 480.0f), 0.0, 1e-12);
}

TEST(TicksToRad, OneRevolutionIsTwoPi) {
  EXPECT_NEAR(ticksToRad(480, 480.0f), 2.0 * M_PI, 1e-12);
}

TEST(TicksToRad, SingleTick) {
  EXPECT_NEAR(ticksToRad(1, 480.0f), 2.0 * M_PI / 480.0, 1e-12);
}

// the wheel turning backwards counts down
TEST(TicksToRad, NegativeTicksGiveNegativeAngle) {
  EXPECT_NEAR(ticksToRad(-480, 480.0f), -2.0 * M_PI, 1e-12);
  EXPECT_NEAR(ticksToRad(-240, 480.0f), -M_PI, 1e-12);
}

// the totals are accumulated over a whole run, the multiplication must not
// overflow in int32 before the division
TEST(TicksToRad, LargeTickCountsDoNotOverflow) {
  const int32_t max = std::numeric_limits<int32_t>::max();
  const int32_t min = std::numeric_limits<int32_t>::min();
  EXPECT_NEAR(ticksToRad(max, 480.0f), max * 2.0 * M_PI / 480.0, 1e-6);
  EXPECT_NEAR(ticksToRad(min, 480.0f), min * 2.0 * M_PI / 480.0, 1e-6);
}

TEST(TicksToRad, FractionalTicksPerRev) {
  EXPECT_NEAR(ticksToRad(3, 1.5f), 4.0 * M_PI, 1e-12);
}
