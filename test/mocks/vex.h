#pragma once
// ============================================================================
// Mock vex.h — Provides stubs of VEX V5 SDK types for host-side testing.
// ============================================================================
// This header is found BEFORE the real include/vex.h because the test
// build uses  -I test/mocks  before  -I include .
// ============================================================================

#include <string>

namespace vex {

/// Minimal mutex stub (no-op on host — tests are single-threaded)
class mutex {
public:
    void lock()   {}
    void unlock() {}
};

/// Minimal task stub
class task {
public:
    task(int (*)()) {}                   // launch callback (no-op)
    void stop() {}
    static void sleep(int /*ms*/) {}
};

}  // namespace vex
