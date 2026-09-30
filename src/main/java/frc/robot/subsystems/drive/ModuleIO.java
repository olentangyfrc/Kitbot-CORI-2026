package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ModuleIO extends SubsystemBase {
  private final TalonFX driveMotor;
  private final SparkMax steerMotor;
  private final AnalogEncoder encoder;
  private final double offset; // degrees
  private boolean swerveTuningMode = false;
  private double steerTuningOutput = 0; // degrees

  private final TalonFXConfiguration driveConfig;
  private final SparkMaxConfig steerConfig;

  private final PIDController steerPIDController;

  // odometry stuff, need change later
  private double steerP = 0.3;
  private final double wheelRadius = 1.8125; // inches? i think
  private final double gearRatio = 1 / 8.33; // taken from agnes
  private final double encoderResolution = 42; // also agnes

  private double driveVelocity = 0; // meters per second
  private Rotation2d steerAngle = new Rotation2d();

  public ModuleIO(int driveMotorCanId, int steerMotorCanId, int encoderId, double motorOffset) {
    driveMotor = new TalonFX(driveMotorCanId, "rio");
    steerMotor = new SparkMax(steerMotorCanId, MotorType.kBrushless);
    encoder = new AnalogEncoder(encoderId, 2 * Math.PI, 0);
    offset = motorOffset;

    driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotor.getConfigurator().apply(driveConfig, 0.25);

    steerConfig = new SparkMaxConfig();
    steerConfig.idleMode(IdleMode.kBrake);
    steerMotor.configure(
        steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    steerPIDController = new PIDController(0.5, 0, 0);
    steerPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setState(SwerveModuleState state) {
    Rotation2d encoderRotation2d = new Rotation2d(getEncoderRadians());

    state.optimize(encoderRotation2d);
    state.cosineScale(encoderRotation2d);

    driveVelocity = state.speedMetersPerSecond;
    steerAngle = state.angle;

    double steerOutput = 0;

    steerOutput = steerPIDController.calculate(getEncoderRadians(), state.angle.getRadians());

    // if (swerveTuningMode) {
    // steerOutput = steerTuningOutput;
    // }

    double driveOutput = state.speedMetersPerSecond;

    driveMotor.setVoltage(driveOutput);
    steerMotor.setVoltage(steerOutput);
  }

  public double getEncoderRadians() {
    return encoder.get() - Math.PI + Math.toRadians(offset);
  }
  /**
   * Gets the current position of the swerve module. Mostly used for odometry
   *
   * @return Distance in meters, module angle.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        (driveMotor.getPosition().getValueAsDouble() * gearRatio * wheelRadius * Math.PI * 2)
            / encoderResolution,
        Rotation2d.fromRadians(getEncoderRadians()));
  }
  /**
   * Gets the target state of the swerve module.
   *
   * @return Drive velocity, angle.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveVelocity, steerAngle);
  }

  public void periodic() {
    // SmartDashboard.putBoolean("swerveTuningMode", swerveTuningMode);
    // swerveTuningMode = SmartDashboard.getBoolean("swerveTuningMode", swerveTuningMode);

    // if (swerveTuningMode) {
    SmartDashboard.putNumber("steerP", steerP);
    steerP = SmartDashboard.getNumber("steerP", steerP);
    steerPIDController.setP(steerP);

    SmartDashboard.putNumber(
        "steerMotorId" + steerMotor.getDeviceId(), getPosition().angle.getDegrees());

    // SmartDashboard.putNumber("steerTuningOutput" + steerMotor.getDeviceId(), steerTuningOutput);
    // steerTuningOutput =
    //     SmartDashboard.getNumber("steerTuningOutput" + steerMotor.getDeviceId(),
    // steerTuningOutput);
    // }
  }
}
