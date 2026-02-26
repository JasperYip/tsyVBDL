#include <cstdio>
#include <cstdlib>

#include "Arduino.h"          // mock (from test/)
#include "motor_driver.h"     // from include/
#include "config.h"           // your pins/constants from include/

static void CHECK(bool cond, const char* msg) {
    if (!cond) {
        std::fprintf(stderr, "FAIL: %s\n", msg);
        std::exit(1);
    }
}

int main() {
    using namespace ArduinoMock;

    reset();

    MotorDriver md;
    MotorDriver::Pins p{
        .pin_pwm = PIN_MOTOR_PWM,
        .pin_dir = PIN_MOTOR_DIR,
        .pin_slp = PIN_MOTOR_SLP,
        .pin_flt = PIN_MOTOR_FLT,
        .pin_cs  = PIN_MOTOR_CS
    };

    // begin() safe state
    md.begin(p);
    CHECK(getAnalogWrite(p.pin_pwm) == 0, "begin(): PWM must start at 0");
    CHECK(getDigitalWrite(p.pin_slp) == LOW, "begin(): SLP must start LOW");
    CHECK(md.isEnabled() == false, "begin(): driver starts disabled");

    // setCommand while disabled: direction updates, PWM gated to 0
    md.setCommand(true, 200);
    CHECK(getDigitalWrite(p.pin_dir) == HIGH, "DIR should be HIGH when extend=true");
    CHECK(getAnalogWrite(p.pin_pwm) == 0, "PWM must remain 0 if disabled");
    CHECK(md.lastPwm() == 0, "lastPwm remains 0 if disabled");

    // enable
    md.setSleep(true);
    CHECK(getDigitalWrite(p.pin_slp) == HIGH, "setSleep(true): SLP HIGH");
    CHECK(md.isEnabled() == true, "setSleep(true): enabled true");

    // command when enabled: PWM applies
    md.setCommand(false, 123);
    CHECK(getDigitalWrite(p.pin_dir) == LOW, "DIR LOW when extend=false");
    CHECK(getAnalogWrite(p.pin_pwm) == 123, "PWM applied when enabled");
    CHECK(md.lastPwm() == 123, "lastPwm stored");

    // sleep(false): forces PWM=0
    md.setSleep(false);
    CHECK(getAnalogWrite(p.pin_pwm) == 0, "setSleep(false): PWM forced 0");
    CHECK(getDigitalWrite(p.pin_slp) == LOW, "setSleep(false): SLP LOW");
    CHECK(md.isEnabled() == false, "setSleep(false): enabled false");

    // emergencyStop(): PWM=0 and SLP LOW
    md.setSleep(true);
    md.setCommand(true, 250);
    CHECK(getAnalogWrite(p.pin_pwm) == 250, "precondition PWM running");
    md.emergencyStop();
    CHECK(getAnalogWrite(p.pin_pwm) == 0, "emergencyStop PWM=0");
    CHECK(getDigitalWrite(p.pin_slp) == LOW, "emergencyStop SLP=LOW");
    CHECK(md.isEnabled() == false, "emergencyStop enabled=false");

    // faultAsserted() default active-low
    setDigitalRead(p.pin_flt, HIGH);
    CHECK(md.faultAsserted() == false, "faultAsserted(): HIGH => no fault (active-low)");
    setDigitalRead(p.pin_flt, LOW);
    CHECK(md.faultAsserted() == true, "faultAsserted(): LOW => fault (active-low)");

    // currentSenseRaw
    setAnalogRead(p.pin_cs, 777);
    CHECK(md.currentSenseRaw() == 777, "currentSenseRaw(): returns analogRead");

    std::puts("ALL MOTOR DRIVER TESTS PASSED");
    return 0;
}