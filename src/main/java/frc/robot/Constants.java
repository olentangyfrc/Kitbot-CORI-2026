// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double maxLinearSpeed = 5; // mps
  public static final double falconMaxSpeed = 6380;
  public static final double DEADBAND = 0.1;

  public static final double spinUpVelocity = 1500;
  public static final double shootMaxVelocity = 3000;
  public static final double intakeMaxVelocity = 2000;
  public static final double indexerMaxVelocity = 6;
  public static final double shooterVelocityTolerance = 120;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY

    /**
     * Kitbot CAN IDs:
     *
     * <p>Front Left Drive: 31 Front Left Steer: 13
     *
     * <p>Front Right Drive: 30 Front Right Steer: 44
     *
     * <p>Back Left Drive: 33 Back Left Steer: 11
     *
     * <p>Back Right Drive: 32 Back Right Steer: 10
     *
     * <p>Intake/Shooter: 40 Feeder/Indexer: 21
     *
     * <p>PDP: 1 IMU: 5
     */
  }
}
