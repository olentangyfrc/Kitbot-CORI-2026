package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
  public static class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public Rotation2d steerAbsolutePosition = Rotation2d.kZero;
    public double steerVelocityRadPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerCurrentAmps = 0.0;

    public SwerveModuleState swerveState = new SwerveModuleState();
    public SwerveModulePosition swervePosition = new SwerveModulePosition();
    // public double turnCurrentAmps = 0.0;

    // public double[] odometryTimestamps = new double[] {};
    // public double[] odometryDrivePositionsRad = new double[] {};
    // public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void setSwerveState(SwerveModuleState state) {}
}
