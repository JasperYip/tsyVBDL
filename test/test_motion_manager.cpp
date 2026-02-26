#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "motion_manager.h"
#include "config.h"
#include "types.h"
#include "pid_controller.h"

// ---------------- tiny test macros ----------------
static int g_failures = 0;

#define EXPECT_TRUE(cond) do { \
  if (!(cond)) { \
    std::printf("FAIL %s:%d: EXPECT_TRUE(%s)\n", __FILE__, __LINE__, #cond); \
    g_failures++; \
  } \
} while(0)

#define EXPECT_EQ(a,b) do { \
  auto _a = (a); auto _b = (b); \
  if (!((_a) == (_b))) { \
    std::printf("FAIL %s:%d: EXPECT_EQ(%s,%s) got %d vs %d\n", __FILE__, __LINE__, #a, #b, (int)_a, (int)_b); \
    g_failures++; \
  } \
} while(0)

#define EXPECT_NE(a,b) do { \
  auto _a = (a); auto _b = (b); \
  if (((_a) == (_b))) { \
    std::printf("FAIL %s:%d: EXPECT_NE(%s,%s)\n", __FILE__, __LINE__, #a, #b); \
    g_failures++; \
  } \
} while(0)

#define EXPECT_NEAR(a,b,eps) do { \
  float _a = (float)(a); float _b = (float)(b); float _e = (float)(eps); \
  if (std::fabs(_a - _b) > _e) { \
    std::printf("FAIL %s:%d: EXPECT_NEAR(%s,%s,%.6f) got %.6f vs %.6f\n", __FILE__, __LINE__, #a, #b, _e, _a, _b); \
    g_failures++; \
  } \
} while(0)

// Helper: make PID deterministic-ish (mostly P-only).
static PIDController::Config makeTestPid()
{
  PIDController::Config c;
  c.kp = 0.5f;          // moderate proportional
  c.ki = 0.0f;
  c.kd = 0.0f;
  c.dt_s = CONTROL_DT_S;
  c.deadband_mm = 0.0f;
  c.out_min = -1.0f;
  c.out_max =  1.0f;
  c.i_min = -0.0f;
  c.i_max =  0.0f;
  c.d_max_abs = 0.0f;
  return c;
}

static float pctToMm_test(uint8_t pct)
{
  if (pct > 100) pct = 100;
  const float span = STROKE_HARD_MAX_MM - STROKE_HARD_MIN_MM;
  return STROKE_HARD_MIN_MM + (span * ((float)pct / 100.0f));
}

// Helper: step time by calling tick N times.
static void runTicks(MotionManager& mm, const CmdSetpoint& cmd, MotionInputs in, int nTicks)
{
  mm.setCommand(cmd);
  for (int i = 0; i < nTicks; i++) mm.tick(in);
}

static void test_motor_disabled_is_safe_idle()
{
  MotionManager mm;
  mm.setPidConfig(makeTestPid());

  CmdSetpoint cmd{};
  cmd.motor_enable = false;

  MotionInputs in{};
  in.pos_est_mm = 50.0f;
  in.vel_est_mm_s = 0.0f;

  mm.setCommand(cmd);
  mm.tick(in);

  auto mc = mm.motorCommand();
  EXPECT_TRUE(mc.slp_enable == false);
  EXPECT_EQ(mc.pwm, 0);
  EXPECT_TRUE(mm.state() == VbdState::BOOT); // not homed, disabled => BOOT per your code
}

static void test_leak_triggers_fault_and_latches_first()
{
  MotionManager mm;
  mm.setPidConfig(makeTestPid());

  CmdSetpoint cmd{};
  cmd.motor_enable = true;   // even if enabled, leak should fault

  MotionInputs in{};
  in.pos_est_mm = 50.0f;
  in.leak = true;

  mm.setCommand(cmd);
  mm.tick(in);

  EXPECT_TRUE(mm.state() == VbdState::FAULT);
  EXPECT_TRUE((mm.statusFault().hard_faults & HF_LEAK) != 0);
  EXPECT_TRUE(mm.statusFault().first_hard == FirstHardFaultIndex::LEAK);

  auto mc = mm.motorCommand();
  EXPECT_EQ(mc.pwm, 0);
  EXPECT_TRUE(mc.slp_enable == false); // driver disabled in enterFault()
}

static void test_pos_limit_triggers_fault()
{
  MotionManager mm;
  mm.setPidConfig(makeTestPid());

  CmdSetpoint cmd{};
  cmd.motor_enable = true;

  MotionInputs in{};
  in.pos_est_mm = STROKE_HARD_MAX_MM + 1.0f; // beyond hard max

  mm.setCommand(cmd);
  mm.tick(in);

  EXPECT_TRUE(mm.state() == VbdState::FAULT);
  EXPECT_TRUE((mm.statusFault().hard_faults & HF_POS_LIM) != 0);
  EXPECT_TRUE(mm.statusFault().first_hard == FirstHardFaultIndex::POS_LIM);
}

static void test_run_without_homing_is_hard_fault()
{
  MotionManager mm;
  mm.setPidConfig(makeTestPid());

  CmdSetpoint cmd{};
  cmd.motor_enable = true;
  cmd.request_homing = false; // not homing, not homed

  MotionInputs in{};
  in.pos_est_mm = 50.0f;

  mm.setCommand(cmd);
  mm.tick(in);

  EXPECT_TRUE(mm.state() == VbdState::FAULT);
  EXPECT_TRUE((mm.statusFault().hard_faults & HF_NOT_HOMED) != 0);
  EXPECT_TRUE(mm.statusFault().first_hard == FirstHardFaultIndex::NOT_HOMED);
}

static void test_homing_success_transitions_to_run_and_sets_homed()
{
  MotionManager mm;
  mm.setPidConfig(makeTestPid());

  CmdSetpoint cmd{};
  cmd.motor_enable = true;
  cmd.request_homing = true;

  MotionInputs in{};
  in.pos_est_mm = 50.0f;
  in.prox_active = false;

  // First tick => homing, retract effort => dir_extend false
  mm.setCommand(cmd);
  mm.tick(in);
  EXPECT_TRUE(mm.state() == VbdState::HOMING);
  EXPECT_TRUE(mm.motorCommand().slp_enable == true);
  EXPECT_TRUE(mm.motorCommand().dir_extend == false);
  EXPECT_TRUE(mm.motorCommand().pwm > 0);

  // Now prox triggers => becomes homed and RUN
  in.prox_active = true;
  mm.tick(in);

  EXPECT_TRUE(mm.homed() == true);
  EXPECT_TRUE(mm.state() == VbdState::RUN);
}

static void test_homing_timeout_faults()
{
  MotionManager mm;
  mm.setPidConfig(makeTestPid());

  CmdSetpoint cmd{};
  cmd.motor_enable = true;
  cmd.request_homing = true;

  MotionInputs in{};
  in.pos_est_mm = 50.0f;
  in.prox_active = false;

  // Run slightly beyond timeout (CONTROL_DT_S = 0.001)
  const int ticks = (int)std::ceil((HOMING_TIMEOUT_S / CONTROL_DT_S) + 5.0f);

  runTicks(mm, cmd, in, ticks);

  EXPECT_TRUE(mm.state() == VbdState::FAULT);
  EXPECT_TRUE((mm.statusFault().hard_faults & HF_NOT_HOMED) != 0);
  EXPECT_TRUE(mm.statusFault().first_hard == FirstHardFaultIndex::NOT_HOMED);
}

static void test_enable_unhomed_with_homing_request_is_allowed()
{
  MotionManager mm;
  mm.setPidConfig(makeTestPid());

  CmdSetpoint cmd{};
  cmd.motor_enable = true;
  cmd.request_homing = true;

  MotionInputs in{};
  in.pos_est_mm = 50.0f;
  in.prox_active = false;

  mm.setCommand(cmd);
  mm.tick(in);

  EXPECT_TRUE(mm.state() == VbdState::HOMING);
  EXPECT_TRUE(mm.statusFault().hard_faults == 0);
  EXPECT_TRUE(mm.motorCommand().slp_enable == true);
  EXPECT_TRUE(mm.motorCommand().pwm > 0);
}

static void test_relaxed_hold_enters_and_exits()
{
  MotionManager mm;
  mm.setPidConfig(makeTestPid());

  // First: home the system quickly
  CmdSetpoint cmd{};
  cmd.motor_enable = true;
  cmd.request_homing = true;

  MotionInputs in{};
  in.pos_est_mm = 50.0f;
  in.prox_active = true;

  mm.setCommand(cmd);
  mm.tick(in);
  EXPECT_TRUE(mm.homed());
  EXPECT_TRUE(mm.state() == VbdState::RUN);

  // Now command a target that equals current position (error ~0) so hold band counts up.
  CmdSetpoint runCmd{};
  runCmd.motor_enable = true;
  runCmd.request_homing = false;

  // Choose a target_pct that maps near 50mm so error is small.
  // We don’t need exact mapping; we can instead set pos to match target by computing pct from linear mapping.
  // Mapping: 10..140mm over 0..100%
  uint8_t pct = 38;
  runCmd.target_pct = pct;

  const float pos = pctToMm_test(runCmd.target_pct); // <-- define pos
  in.pos_est_mm = pos;
  in.vel_est_mm_s = 0.0f;

  // Run for > HOLD_ENTER_TIME_S in-band
  const int holdTicks = (int)std::ceil((HOLD_ENTER_TIME_S / CONTROL_DT_S) + 5.0f);
  runTicks(mm, runCmd, in, holdTicks);

  // In relaxed hold => pwm should be 0
  EXPECT_EQ(mm.motorCommand().pwm, 0);

  // Now simulate drift beyond HOLD_EXIT_DRIFT_MM to re-engage PID
  in.pos_est_mm = pos - (HOLD_EXIT_DRIFT_MM + 1.0f);
  mm.setCommand(runCmd);
  mm.tick(in);

  EXPECT_TRUE(mm.motorCommand().pwm > 0); // PID should re-engage
}

static void test_velocity_limit_blocks_extend()
{
  MotionManager mm;
  mm.setPidConfig(makeTestPid());

  // Home quickly
  CmdSetpoint homeCmd{};
  homeCmd.motor_enable = true;
  homeCmd.request_homing = true;

  MotionInputs in{};
  in.pos_est_mm = 50.0f;
  in.prox_active = true;

  mm.setCommand(homeCmd);
  mm.tick(in);

  EXPECT_TRUE(mm.homed() == true);
  EXPECT_TRUE(mm.state() == VbdState::RUN);

  CmdSetpoint runCmd{};
  runCmd.motor_enable = true;
  runCmd.target_pct = 100;

  in.prox_active = false;
  in.pos_est_mm = 20.0f;

  // Step 1: under velocity limit -> should command extend with pwm > 0
  in.vel_est_mm_s = 0.0f;
  mm.setCommand(runCmd);
  mm.tick(in);

  EXPECT_TRUE(mm.motorCommand().dir_extend == true);
  EXPECT_TRUE(mm.motorCommand().pwm > 0);

  // Step 2: over velocity limit -> should block -> pwm == 0 (direction irrelevant)
  in.vel_est_mm_s = V_MAX_MM_S + 5.0f;
  mm.tick(in);

  EXPECT_EQ(mm.motorCommand().pwm, 0);
}

static void test_velocity_limit_blocks_retract()
{
  MotionManager mm;
  mm.setPidConfig(makeTestPid());

  // --- Home quickly ---
  CmdSetpoint homeCmd{};
  homeCmd.motor_enable = true;
  homeCmd.request_homing = true;

  MotionInputs in{};
  in.pos_est_mm = 50.0f;
  in.prox_active = true;

  mm.setCommand(homeCmd);
  mm.tick(in);

  EXPECT_TRUE(mm.homed() == true);
  EXPECT_TRUE(mm.state() == VbdState::RUN);

  // --- Command retract (negative effort) ---
  CmdSetpoint runCmd{};
  runCmd.motor_enable = true;
  runCmd.request_homing = false;
  runCmd.target_pct = 0;        // near min -> wants retract

  in.prox_active = false;
  in.pos_est_mm = 130.0f;       // well above target -> error negative -> retract

  // Step 1: under velocity limit -> should command retract with pwm > 0
  in.vel_est_mm_s = 0.0f;
  mm.setCommand(runCmd);
  mm.tick(in);

  EXPECT_TRUE(mm.motorCommand().dir_extend == false);
  EXPECT_TRUE(mm.motorCommand().pwm > 0);

  // Step 2: over velocity limit in retract direction -> should block -> pwm == 0
  in.vel_est_mm_s = -(V_MAX_MM_S + 5.0f);
  mm.tick(in);

  EXPECT_EQ(mm.motorCommand().pwm, 0); // direction irrelevant when pwm=0
}

int main()
{
  test_motor_disabled_is_safe_idle();
  test_leak_triggers_fault_and_latches_first();
  test_pos_limit_triggers_fault();
  test_run_without_homing_is_hard_fault();
  test_homing_success_transitions_to_run_and_sets_homed();
  test_homing_timeout_faults();
  test_relaxed_hold_enters_and_exits();
  test_velocity_limit_blocks_extend();
  test_velocity_limit_blocks_retract();
  test_enable_unhomed_with_homing_request_is_allowed();

  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  } else {
    std::printf("%d TEST(S) FAILED\n", g_failures);
    return 1;
  }
}