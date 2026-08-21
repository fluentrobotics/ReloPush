// Shared test harness for vesc_unit_tests / vesc_integration_tests --
// split out of the former monolithic vesc_driver_tests.cpp so both
// binaries report in the identical format ("N checks run, M failed." +
// "[ PASSED ] K tests.") without duplicating the harness. Header-only
// (each including .cpp gets its own g_checks/g_failures via the unnamed
// namespace -- fine since unit/integration are separate binaries, not
// linked together). No gtest dependency, mirrors MPC/tests' style.

#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

int g_checks = 0;
int g_failures = 0;

bool check_true(bool cond, const std::string& what) {
    ++g_checks;
    if (!cond) {
        ++g_failures;
        std::cout << "    CHECK FAILED: " << what << "\n";
    }
    return cond;
}

bool near_eq(double a, double b, double eps = 1e-9) { return std::fabs(a - b) <= eps; }

std::string hex_byte(uint8_t b) {
    static const char* digits = "0123456789ABCDEF";
    std::string s = "0x";
    s += digits[(b >> 4) & 0xF];
    s += digits[b & 0xF];
    return s;
}

std::string hex_bytes(const std::vector<uint8_t>& v) {
    std::string s = "{";
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) s += ",";
        s += hex_byte(v[i]);
    }
    s += "}";
    return s;
}

bool bytes_eq(const std::vector<uint8_t>& actual, const std::vector<uint8_t>& expected,
              const std::string& what) {
    ++g_checks;
    if (actual != expected) {
        ++g_failures;
        std::cout << "    CHECK FAILED: " << what << "\n"
                  << "        expected " << hex_bytes(expected) << "\n"
                  << "        actual   " << hex_bytes(actual) << "\n";
        return false;
    }
    return true;
}

// Runs `tests` in order with a top-level try/catch around each invocation
// (this is what guarantees pty_test::ProcessGuard's destructor runs if a
// check throws between spawning a child and an explicit terminate() call
// -- an exception escaping main() uncaught calls std::terminate() WITHOUT
// running local destructors on this toolchain), then prints the same
// summary main() always has. Returns 0 if all tests passed, 1 otherwise
// -- callers just `return run_registered_tests(tests);` from main().
int run_registered_tests(const std::vector<std::pair<std::string, bool (*)()>>& tests) {
    int failed = 0;
    for (const auto& t : tests) {
        std::cout << "[ RUN      ] " << t.first << "\n";
        bool ok = false;
        try {
            ok = t.second();
        } catch (const std::exception& e) {
            std::cout << "    EXCEPTION: " << e.what() << "\n";
            ok = false;
        } catch (...) {
            std::cout << "    EXCEPTION: (unknown)\n";
            ok = false;
        }
        if (ok) {
            std::cout << "[       OK ] " << t.first << "\n";
        } else {
            std::cout << "[  FAILED  ] " << t.first << "\n";
            ++failed;
        }
    }

    std::cout << "\n" << g_checks << " checks run, " << g_failures << " failed.\n";
    if (failed == 0) {
        std::cout << "[  PASSED  ] " << tests.size() << " tests.\n";
        return 0;
    }
    std::cout << "[  FAILED  ] " << failed << " tests.\n";
    return 1;
}

}  // namespace
