#define UNIT_TEST
#include "encoder_interface.h"
#include "fake_quadencoder.h"

#include <iostream>
#include <cmath>

static int fails = 0;

static void expectTrue(bool v, const char* name) {
    if (!v) { std::cout << "FAIL: " << name << "\n"; fails++; }
    else    { std::cout << "PASS: " << name << "\n"; }
}

static void expectEqI32(int32_t a, int32_t b, const char* name) {
    if (a != b) {
        std::cout << "FAIL: " << name << " (expected " << b << ", got " << a << ")\n";
        fails++;
    } else {
        std::cout << "PASS: " << name << "\n";
    }
}

static void expectNear(float a, float b, float tol, const char* name) {
    if (std::fabs(a - b) > tol) {
        std::cout << "FAIL: " << name << " (expected " << b << ", got " << a << ", tol " << tol << ")\n";
        fails++;
    } else {
        std::cout << "PASS: " << name << "\n";
    }
}

int main() {
    EncoderInterface ei;
    QuadEncoder fake;

    ei._setEncoderForTest(&fake);

    // Test 1: counts + position
    ei.reset();
    ei._setMmPerCountForTest(0.5f);

    fake.setCounts(100);
    ei.update(0.001f);

    expectEqI32(ei.rawCounts(), 100, "rawCounts reports encoder counts");
    expectNear(ei.posMm(), 50.0f, 1e-6f, "posMm converts counts to mm");

    // Test 2: velocity with filtering (alpha=0.2)
    ei.reset();
    ei._setMmPerCountForTest(1.0f);

    fake.setCounts(0);
    ei.update(0.001f);

    fake.setCounts(10);
    ei.update(0.001f);

    expectNear(ei.velMmPerSec(), 2000.0f, 1e-3f, "velMmPerSec filtered velocity");

    // Test 3: homing
    ei.reset();
    ei._setMmPerCountForTest(0.25f);

    fake.setCounts(200);
    ei.update(0.001f);

    ei.setHomeAtMm(10.0f);
    expectTrue(ei.isHomed(), "isHomed becomes true after setHomeAtMm");
    expectNear(ei.posMm(), 10.0f, 1e-3f, "posMm equals home after setHomeAtMm");

    fake.setCounts(204);
    ei.update(0.001f);

    expectNear(ei.posMm(), 11.0f, 1e-3f, "posMm advances correctly after homing");

    std::cout << "\n" << (fails == 0 ? "ALL TESTS PASSED" : "TESTS FAILED")
              << " (failures=" << fails << ")\n";

    return fails == 0 ? 0 : 1;
}