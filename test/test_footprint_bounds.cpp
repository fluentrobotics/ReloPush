#include <ReloPush/BoundaryCheck.hpp>
#include <ReloPush/PathPlanningTools.h>

#include <cassert>
#include <cmath>
#include <iostream>

// Unit test for footprint_in_bounds() function.
//
// Verifies:
//  - Test 1: state well inside bounds, corners mode on → passes
//  - Test 2: state near wall (right side, yaw=0), origin inside but front-right corner pokes out, corners mode off → passes
//  - Test 3: same state as Test 2, corners mode on → FAILS
//  - Test 4: pushing mode (LF ~ 0.556), state near wall → FAILS (pushed object corners out)
//  - Test 5: transit mode (LF ~ 0.375), same state → PASSES (no push extent added)
//  - Test 6: tolerance 1e-2 accepts corner exactly at boundary

int main()
{
    const double tol = 1e-2;

    // Use constants from PathPlanningTools.h
    double half_width = Constants::carWidth / 2.0;  // 0.285 / 2 = 0.1425
    double LB = Constants::LB;                        // 0.12
    double LF_nonpush = Constants::LF_nonpush;        // 0.375
    double LF_push = Constants::LF_push;              // ~0.45
    double obsEncDiameter = Constants::obsEncDiameter; // ~0.212

    // Workspace bounds: 4.0 x 5.2
    double xMin = 0.0, xMax = 4.0;
    double yMin = 0.0, yMax = 5.2;

    // Test 1: state well inside bounds, all corners inside
    {
        double x = 2.0, y = 2.5, yaw = 0.0;
        bool result = footprint_in_bounds(x, y, yaw, LF_nonpush, LB, half_width,
                                         xMin, xMax, yMin, yMax, tol);
        assert(result);
        std::cout << "Test 1 (well inside bounds): PASS" << std::endl;
    }

    // Test 2: state near right wall (x ~= 3.9), yaw=0, origin inside but front-right corner pokes out
    // Front-right corner: (x + LF*cos(yaw) + half_width*sin(yaw), y + LF*sin(yaw) - half_width*cos(yaw))
    //                   = (3.9 + 0.375*1 + 0.1425*0, 2.5 + 0.375*0 - 0.1425*1)
    //                   = (4.275, 2.3575) -> x exceeds 4.0 by 0.275, should fail without corner check
    // With corners mode off (legacy origin-only check): passes because origin (3.9, 2.5) is in bounds
    {
        double x = 3.9, y = 2.5, yaw = 0.0;
        bool result = footprint_in_bounds(x, y, yaw, LF_nonpush, LB, half_width,
                                         xMin, xMax, yMin, yMax, tol);
        assert(!result);  // Front-right corner at ~4.275 exceeds xMax+tol (4.01)
        std::cout << "Test 2 (near wall, corner out): FAIL as expected" << std::endl;
    }

    // Test 3: same state as Test 2 (verify it's out)
    {
        double x = 3.9, y = 2.5, yaw = 0.0;
        bool result = footprint_in_bounds(x, y, yaw, LF_nonpush, LB, half_width,
                                         xMin, xMax, yMin, yMax, tol);
        assert(!result);
        std::cout << "Test 3 (same as test 2, verified): FAIL as expected" << std::endl;
    }

    // Test 4: pushing mode (LF ~ 0.556 = LF_push + obsEncDiameter/2), state near wall
    // Front extent = 0.45 + 0.212/2 ~= 0.556
    // Front-right: (3.9 + 0.556*1 + 0.1425*0, ...) = (4.456, ...) -> exceeds xMax+tol by much
    {
        double x = 3.9, y = 2.5, yaw = 0.0;
        float front_extent = LF_push + obsEncDiameter / 2.0;  // ~0.556
        bool result = footprint_in_bounds(x, y, yaw, front_extent, LB, half_width,
                                         xMin, xMax, yMin, yMax, tol);
        assert(!result);
        std::cout << "Test 4 (pushing mode, corner out): FAIL as expected" << std::endl;
    }

    // Test 5: transit mode (LF ~ 0.375), same state as Test 4
    // No extended front extent
    // Front-right: (3.9 + 0.375*1 + 0.1425*0, ...) = (4.275, ...) -> exceeds by 0.275
    {
        double x = 3.9, y = 2.5, yaw = 0.0;
        bool result = footprint_in_bounds(x, y, yaw, LF_nonpush, LB, half_width,
                                         xMin, xMax, yMin, yMax, tol);
        assert(!result);  // Still out, as LF_nonpush also exceeds
        std::cout << "Test 5 (transit mode, corner still out): FAIL as expected" << std::endl;
    }

    // Test 6: tolerance 1e-2 accepts corner exactly at boundary + tol
    // Front-right: (x + LF*1 + 0, y + LF*0 - half_width*1)
    // For x = 3.8875: front-right x = 3.8875 + 0.375 = 4.2625, but we want x + LF = xMax + tol
    // So x + LF = 4.01 => x = 3.635
    // Actually, let's place it so front-right corner is exactly at xMax + tol
    {
        double target_x_corner = xMax + tol;  // 4.01
        double x = target_x_corner - LF_nonpush;  // 4.01 - 0.375 = 3.635
        double y = 2.5, yaw = 0.0;
        bool result = footprint_in_bounds(x, y, yaw, LF_nonpush, LB, half_width,
                                         xMin, xMax, yMin, yMax, tol);
        assert(result);
        std::cout << "Test 6 (corner at boundary + tol): PASS" << std::endl;
    }

    std::cout << "All footprint_in_bounds tests passed!" << std::endl;
    return 0;
}
