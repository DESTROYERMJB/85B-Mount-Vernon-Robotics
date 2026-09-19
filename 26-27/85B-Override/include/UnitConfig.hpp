#pragma once

#include "units/Angle.hpp"
#include "units/units.hpp"

/**
 * @file UnitConfig.hpp
 *
 * The standard units every plain-number parameter in the Chassis API is interpreted in
 * (positions, headings, timeouts, drivetrain sizes, controller ranges).
 *
 * To change a unit, change the one conversion function below (and its matching `toXxx` function, which is
 * used when reading values back, e.g. Chassis::getX()).
 *
 *   length : inches
 *   heading: degrees, compass style (0 = +y / forward, 90 = +x / right, clockwise positive)
 *   angle  : degrees (an angle *size*, e.g. an error range - not a heading)
 *   time   : milliseconds
 *   rpm    : rotations per minute
 */
namespace lemlib::unit_config {

// ---- plain number -> typed unit ----

/** a distance or coordinate */
inline Length length(double value) { return from_in(value); }
/** a field heading (which way the robot faces) */
inline Angle heading(double value) { return from_cDeg(value); }
/** the size of an angle, e.g. an error range */
inline Angle angle(double value) { return from_stDeg(value); }
/** a duration */
inline Time time(double value) { return from_msec(value); }
/** an angular velocity */
inline AngularVelocity rpm(double value) { return value * 1_rpm; }

// ---- typed unit -> plain number ----

inline double toLength(Length value) { return to_in(value); }
inline double toHeading(Angle value) { return to_cDeg(value); }

} // namespace lemlib::unit_config
