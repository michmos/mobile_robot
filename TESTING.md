# Testing Standards

Shared rules for every test in this repo. Anyone adding tests, human or agent, reads this first.
Issues labeled `testing` reference this file; if an issue and this file disagree, ask in the issue.

## 1. Ground rules

1. **No hardware, ever.** Every test must pass on a development machine (or the docker image /
   CI runner) with no ESP32, motors, encoders, lidar, joystick or Raspberry Pi attached.
   Never open a real device (`/dev/esp32`, `/dev/ttyAMA0`, `/dev/input/*`, GPIO). Fake it (see §4).
2. **No network.** No internet and no other hosts. Local DDS traffic inside a test process or
   launch_test is fine.
3. **Deterministic.** The same code must always give the same result. No dependence on wall-clock
   timing, scheduling order, random seeds without a fixed seed, or test execution order.
4. **Fast.** A unit test file runs in well under a second. An integration test (pty / launch_testing)
   should finish within ~30 s; mark anything slower and explain why in a comment.
5. **Independent.** Each test sets up its own state. No shared mutable globals between tests and no
   files left behind (use `tmp_path` / `std::filesystem::temp_directory_path()` + cleanup).
6. **Runs via colcon.** All tests are registered in the package's `CMakeLists.txt` under
   `if(BUILD_TESTING)`, so `colcon test` picks them up. No test that only runs by hand.

## 2. Test levels and tools

| Level | What | Tool | Location |
|---|---|---|---|
| Unit (C++) | Pure functions, classes, state machines, math | `ament_add_gtest` (gtest / gmock) | `<pkg>/test/test_<unit>.cpp` |
| Unit (Python) | Pure Python modules | `ament_add_pytest_test` (pytest) | `<pkg>/test/test_<unit>.py` |
| Firmware (host) | Hardware-independent logic in `esp32_firmware/inc/` | gtest in `mobile_robot_hardware`, compiled against `test/arduino_stub/` | `mobile_robot_hardware/test/firmware/test_<unit>.cpp` |
| Contract | Pi side ↔ firmware agree on the wire format | gtest that links both sides | `mobile_robot_hardware/test/` |
| Component | ros2_control plugin / controller driven through their lifecycle | gtest + fakes (pty, hand-made interfaces) | `<pkg>/test/` |
| Smoke | xacro/URDF, YAML, launch files, plugin.xml load and are consistent | pytest | `mobile_robot_nodes/test/` |
| Integration | Full launch with `mock_components` hardware | `launch_testing` (`add_launch_test`) | `mobile_robot_nodes/test/` |

Prefer the lowest level that can catch the bug. Most tests should be unit tests.

## 3. What a good test looks like

- **One behavior per test.** The name says what it checks:
  `TEST(MotorControl, ZeroCommandResetsIntegral)`, `def test_raises_on_wheel_radius_mismatch()`.
- **Arrange / act / assert**, with readable literal inputs. Avoid loops that hide which case failed;
  use parameterized tests (`TEST_P`, `pytest.mark.parametrize`) for tables of cases.
- **Test behavior, not implementation.** Assert on outputs, published messages, return values and
  state that is visible from outside. Don't assert on private members unless there is no other way.
- **Cover the edges.** Every unit test file covers: the normal case, boundaries (0, min, max,
  ±1 around limits), invalid input (malformed, out of range, empty) and, where it applies, overflow
  or wraparound (e.g. `uint32` µs timestamps, encoder ticks), dt == 0 and first-sample behavior.
- **Floating point:** use `EXPECT_NEAR` / `pytest.approx` with an explicit tolerance. Never `==`.
- **No sleeps for synchronization.** Wait on a condition with a bounded timeout (poll + deadline),
  never `sleep(1)` and hope. Where time matters, inject it (controllable `millis()`/`micros()` in
  the Arduino stub, explicit `time`/`period` arguments to `update()`/`read()`).
- **Comments** explain *why* a case matters (e.g. "ESP32 reboot resets the tick counter"), not what
  the assert does. Match the style of the surrounding code (see `CLAUDE.md` conventions).

## 4. Faking hardware

| Real thing | Fake |
|---|---|
| ESP32 serial link (hardware plugin) | **pty pair** (`openpty()`): pass the slave path (`/dev/pts/N`) as the `serial_port` hardware parameter and let a test thread on the master side play the ESP32 (send `S,SETUP`, answer `C,...` with `S,ACK,<id>`/`S,NACK,...`, stream `E,...` lines, read `M,...` lines). The plugin's production serial code stays unchanged. Reuse `mobile_robot_hardware/test/fake_esp32.hpp` (the fake esp32) and `test/hardware_fixture.hpp` (a gtest fixture that runs the plugin through its lifecycle against it), used by the plugin's component tests. |
| ros2_control hardware for launch/integration tests | `mock_components/GenericSystem`, selected with the `use_mock_hardware` xacro/launch argument |
| Controller's hardware | Hand-made `hardware_interface::CommandInterface` / `StateInterface` objects (the pattern used by ros2_controllers' `diff_drive_controller` tests) |
| Arduino core (`millis`, `micros`, `Serial`, GPIO, LEDC, interrupts) | `mobile_robot_hardware/test/arduino_stub/Arduino.h`: a controllable clock, a fake `Serial` with an input queue and output capture, and recorders for GPIO/LEDC calls (esp32 core 3.x API: per-pin `ledcAttach`/`ledcAttachChannel`/`ledcWrite`). Extend this stub instead of creating a second one. |
| `ESP32Encoder` library (PCNT encoders) | `mobile_robot_hardware/test/arduino_stub/ESP32Encoder.h`: records how each encoder was attached (pins, quadrature mode, pull resistors, filter) and returns a count the test sets with `arduino_stub::setEncoderCount()`; cleared by `arduino_stub::reset()`. |
| Lidar, joystick, foxglove | Not started in tests. The launch file has arguments to disable them. |

## 5. Changing production code for testability

Allowed, but keep it small and behavior-neutral:

- Moving pure helpers out of an anonymous namespace or a private section into a header
  (e.g. `include/<pkg>/<name>.hpp` or a `detail` namespace) so they can be tested directly.
- Moving logic out of `esp32_firmware.ino` into `esp32_firmware/inc/*.hpp` so it compiles on the host.
- Making a hard-coded constant (e.g. a timeout) configurable, **with the current value as the default**.

Rules:
- Put the refactor in its **own commit** (`refactor(<scope>): ...`) before the commit with the tests.
- No functional change. The firmware must still compile:
  `arduino-cli compile --fqbn esp32:esp32:esp32 src/mobile_robot/esp32_firmware`.
- **If a test finds a real bug:** don't silently "fix" the test to match. Open a GitHub issue
  (label `bug`). Then either fix it in a separate `fix(<scope>): ...` commit if the fix is small and
  obvious, or keep the test and mark it `GTEST_SKIP() << "see #<issue>"` /
  `@pytest.mark.xfail(reason="#<issue>")`. Mention it in the PR description either way.

## 6. Dependencies

- Add every test-only dependency as `<test_depend>` in the package's `package.xml` and inside
  `if(BUILD_TESTING)` in `CMakeLists.txt`.
- If the docker image lacks a package, add `ros-kilted-<name>` to `docker/Dockerfile.ros2`. CI
  runs in the image built from that file (see `OVERVIEW.md`); if your local image is built from a
  copy of it (e.g. `ros2_ws/Dockerfile.ros2`, see `ros2_ws/AGENTS.md`), update the copy too.
- Don't add new test frameworks. Stick to gtest/gmock, pytest and launch_testing.

## 7. Running the tests

From the workspace root (`ros2_ws/`), inside the docker image (see `ros2_ws/AGENTS.md`; always
limit parallelism, the sandbox has ~2.8 GiB RAM):

```bash
docker run --rm --memory=2g --memory-swap=2g -e MAKEFLAGS=-j2 \
  -v /home/micha/Documents/programming/ros_workspaces/ros2_ws:/ros2_ws -w /ros2_ws \
  mobile_robot_ros2:kilted bash -lc "source /opt/ros/kilted/setup.bash && \
  colcon build --executor sequential --build-base build_docker --install-base install_docker --packages-up-to mobile_robot && \
  colcon test  --executor sequential --build-base build_docker --install-base install_docker \
    --packages-select mobile_robot_hardware mobile_robot_controller mobile_robot && \
  colcon test-result --test-result-base build_docker --verbose"
```

Natively (maintainer machine): `colcon test --packages-select <pkg> && colcon test-result --verbose`.

Run a single gtest quickly: `build_docker/<pkg>/<test_name> --gtest_filter='Suite.*'`.

## 8. Definition of done for a testing issue

- [ ] Every case listed in the issue is covered, or the PR says why a case was dropped.
- [ ] The tests are registered in CMake and pass with `colcon test` in the docker image. Paste the
      `colcon test-result` summary into the PR.
- [ ] No hardware or network access, no sleeps for synchronization.
- [ ] Each test file finishes in under a second (integration tests: under ~30 s).
- [ ] Linters (`ament_lint_auto`) introduce no new failures.
- [ ] The firmware still compiles with arduino-cli if any `esp32_firmware/` file changed.
- [ ] Any production change follows §5 and sits in its own commit.
- [ ] `CLAUDE.md`'s "Tests:" line / `OVERVIEW.md` are updated if new test targets or commands were added.

## 9. Workflow

- Branch from `dev`: `test/<issue-number>-<short-name>` (e.g. `test/12-motor-control`).
- Conventional commits: `test(<scope>): ...`, `refactor(<scope>): ...`, `fix(<scope>): ...`.
  Scopes: `hardware`, `controller`, `nodes`, `firmware`, `ci`.
- One issue = one PR against `dev`, with `Closes #<issue>` in the description.
- Respect the dependencies listed in the issue ("Depends on #N"). Don't start an issue whose
  dependency isn't merged unless you build on that branch.
