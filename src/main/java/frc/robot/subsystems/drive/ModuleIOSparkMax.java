package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ModuleIOSparkMax extends SubsystemBase {
  private SparkMax driveMotor;
  private SparkMax steerMotor;
  private AnalogEncoder encoder;
  private double offset;

  private PIDController steerPIDController;

  // odometry stuff, need change
  private double wheelRadius;
  private double gearRatio;
  private double encoderResolution;

  public ModuleIOSparkMax(
      int driveMotorCanId, int steerMotorCanId, int encoderId, double motorOffset) {
    driveMotor = new SparkMax(driveMotorCanId, MotorType.kBrushless);
    steerMotor = new SparkMax(steerMotorCanId, MotorType.kBrushless);
    encoder = new AnalogEncoder(encoderId, 360, 0);
    offset = motorOffset;

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    SparkMaxConfig steerConfig = new SparkMaxConfig();

    // do sparkmax configs

    // change resetmode and persistmode later
    driveMotor.configure(
        driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    steerMotor.configure(
        steerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    steerPIDController = new PIDController(0, 0, 0);
    steerPIDController.enableContinuousInput(-180, 180);
  }

  public double getEncoderRadians() {
    return Math.toRadians(encoder.get()) + offset;
  }

  public void setState(SwerveModuleState state) {
    Rotation2d encoderRotation2d = new Rotation2d(getEncoderRadians());

    var optimized = SwerveModuleState.optimize(state, encoderRotation2d);

    optimized.speedMetersPerSecond =
        optimized.speedMetersPerSecond
            * Math.cos(optimized.angle.getRadians() - encoderRotation2d.getRadians());

    double steerOutput =
        steerPIDController.calculate(getEncoderRadians(), optimized.angle.getRadians());
    double driveOutput = optimized.speedMetersPerSecond;

    driveMotor.setVoltage(driveOutput);
    driveMotor.setVoltage(steerOutput);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        (driveMotor.getEncoder().getPosition() * wheelRadius * Math.PI * 2 * gearRatio)
            / encoderResolution,
        Rotation2d.fromRadians(getEncoderRadians()));
  }
}
