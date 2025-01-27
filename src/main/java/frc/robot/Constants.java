// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  // Swerve constants
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(10.0);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(10.0);
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
  public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

  public static final double DRIVE_GEAR_RATIO = 4.71;
  public static final double TURN_GEAR_RATIO = 9424. / 203.;

  public static final double ODOM_MED_FREQ = 100.0;
  public static final double ODOM_HIGH_FREQ = 250.0;
  // Sensor and intake constants
  public static final double SENSOR_THRESHOLD = 20;
  public static final double INTAKE_ROLLBACK_ROTATIONS = 3;
  public static final boolean isTurnMotorInverted = true;
  public static final boolean isDriveMotorInverted = true;

  public static final Rotation2d FLAbsoluteEncoderOffset = Rotation2d.fromDegrees(0);
  public static final Rotation2d FRAbsoluteEncoderOffset = Rotation2d.fromDegrees(0);
  public static final Rotation2d BLAbsoluteEncoderOffset = Rotation2d.fromDegrees(0);
  public static final Rotation2d BRAbsoluteEncoderOffset = Rotation2d.fromDegrees(0);

  public static enum NoteState {
    Init,
    NO_NOTE,
    NOTE
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
