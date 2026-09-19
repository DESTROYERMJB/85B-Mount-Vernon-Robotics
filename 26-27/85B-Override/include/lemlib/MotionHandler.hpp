#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include "units/units.hpp"
#include "units/Pose.hpp"

namespace lemlib::motion_handler {
/**
 * @brief run a motion algorithm
 *
 * Motions run on a single long-lived task rather than spawning a new task per call (each new
 * task would otherwise allocate its own ~80KB stack), so calling this repeatedly across a long
 * autonomous routine doesn't pay that allocation cost each time. If a motion is already running,
 * this call blocks until it finishes before starting the new one - the same as if a new task had
 * been spawned per call, just without the allocation overhead. Adapted from
 * https://github.com/LemLib/LemLib/pull/271.
 *
 * @param f the motion function
 * @param priority the priority the motion should run at. Defaults to the calling task's current
 * priority if not specified
 *
 * @b Example:
 * @code {.cpp}
 * // a simple motion algorithm, as an example
 * void simpleMotion() {
 *   std::uint32_t prevTime = pros::millis();
 *   // run until the task has been notified
 *   while (!pros::Task::notify_take(true, 0)) {
 *     // motion algorithm stuff would go here
 *     // ...
 *     // wait 10 ms to free up CPU time for other tasks
 *     pros::Task::delay_until(&prevTime, 10);
 *   }
 * }
 *
 * // runs during the autonomous period
 * void autonomous() {
 *   // pass the motion to the motion handler
 *   lemlib::motion_handler::move([&] { simpleMotion(); });
 *   // "Hello World!" is printed immediately after the motion starts
 *   std::cout << "Hello World!" << std::endl;
 *   // but if we try to run another motion while one is still running,
 *   // it will wait until the last one stops running
 *   lemlib::motion_handler::move([&] { simpleMotion(); });
 *   std::cout << "Last motion ended, new motion started!" << std::endl;
 * }
 * @endcode
 */
void move(std::function<void(void)> f, std::optional<uint32_t> priority = std::nullopt);
/**
 * @brief cancel the currently running motion, if it exists
 *
 * @b Example:
 * @code {.cpp}
 * // a simple motion algorithm, as an example
 * void simpleMotion() {
 *   std::uint32_t prevTime = pros::millis();
 *   // run until the task has been notified
 *   while (!pros::Task::notify_take(true, 0)) {
 *     // motion algorithm stuff would go here
 *     // ...
 *     // wait 10 ms to free up CPU time for other tasks
 *     pros::Task::delay_until(&prevTime, 10);
 *   }
 * }
 *
 * // runs during the autonomous period
 * void autonomous() {
 *   // pass the motion to the motion handler
 *   lemlib::motion_handler::move([&] { simpleMotion(); });
 *   lemlib::motion_handler::isMoving(); // returns true
 *   // cancel the motion
 *   lemlib::motion_handler::cancel();
 *   pros::delay(10); // give the task time to stop
 *   lemlib::motion_handler::isMoving(); // returns false
 * }
 * @endcode
 */
bool isMoving();
/**
 * @brief cancel the currently running motion, if it exists
 *
 * @b Example:
 * @code {.cpp}
 * // a simple motion algorithm, as an example
 * void simpleMotion() {
 *   std::uint32_t prevTime = pros::millis();
 *   // run until the task has been notified
 *   while (!pros::Task::notify_take(true, 0)) {
 *     // motion algorithm stuff would go here
 *     // ...
 *     // wait 10 ms to free up CPU time for other tasks
 *     pros::Task::delay_until(&prevTime, 10);
 *   }
 * }
 *
 * // runs during the autonomous period
 * void autonomous() {
 *   // pass the motion to the motion handler
 *   lemlib::motion_handler::move([]{ simpleMotion(); });
 *   lemlib::motion_handler::isMoving(); // returns true
 *   // cancel the motion
 *   lemlib::motion_handler::cancel();
 *   pros::delay(10); // give the task time to stop
 *   lemlib::motion_handler::isMoving(); // returns false
 * }
 * @endcode
 */
void cancel();
/**
 * @brief wait until the robot is within a radius of a given point
 *
 * @b Example:
 * @code {.cpp}
 * // runs during the autonomous period
 * void autonomous() {
 *   // move the robot to a (60, 40).
 *   lemlib::moveToPoint({60_in, 40_in}, 5_sec, params, settings);
 *   // wait until the robot is in an 8-inch radius from (50, 30).
 *   lemlib::motion_handler::waitUntilPoint({50_in, 30_in}, 8_in, [] { return odom.getPose(); });
 *   // cancel the motion
 *   lemlib::motion_handler::cancel();
 *   pros::delay(10); // give the task time to stop
 * }
 * @endcode
 */
void waitUntilPoint(units::V2Position target, Length radius, std::function<units::Pose()> poseGetter);
/**
 * @brief wait until the robot has traveled a given distance from where this function was called
 *
 * @b Example:
 * @code {.cpp}
 * // runs during the autonomous period
 * void autonomous() {
 *   // move the robot to a (60, 40).
 *   lemlib::moveToPoint({60_in, 40_in}, 5_sec, params, settings);
 *   // wait until the robot has traveled 12 inches since the function has been called.
 *   lemlib::motion_handler::waitUntilDistance(12_in, [] { return odom.getPose(); });
 *   // cancel the motion
 *   lemlib::motion_handler::cancel();
 *   pros::delay(10); // give the task time to stop
 * }
 * @endcode
 */
void waitUntilDistance(Length dist, std::function<units::Pose()> poseGetter);
} // namespace lemlib::motion_handler