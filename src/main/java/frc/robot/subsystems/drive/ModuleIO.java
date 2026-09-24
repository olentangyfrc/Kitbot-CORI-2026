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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ModuleIO extends SubsystemBase {
  private TalonFX driveMotor;
  private SparkMax steerMotor;
  private AnalogEncoder encoder;
  private double offset;
  private TalonFXConfiguration driveConfig;
  private SparkMaxConfig steerConfig;

  private PIDController steerPIDController;
  private double steerP = 0;
  private double steerI = 0;
  private double steerD = 0;

  // odometry stuff, need change later
  private double wheelRadius = 2;
  private double gearRatio = 3 / 1;
  private double encoderResolution = 400;

  private double driveVelocity; // meters per second
  private Rotation2d steerAngle;

  public ModuleIO(int driveMotorCanId, int steerMotorCanId, int encoderId, double motorOffset) {
    driveMotor = new TalonFX(driveMotorCanId, "can0");
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

    steerPIDController = new PIDController(steerP, steerI, steerD);
    steerPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setState(SwerveModuleState state) {
    Rotation2d encoderRotation2d = new Rotation2d(getEncoderRadians());

    state.optimize(encoderRotation2d);
    state.cosineScale(encoderRotation2d);

    driveVelocity = state.speedMetersPerSecond;
    steerAngle = state.angle;

    double steerOutput =
        steerPIDController.calculate(getEncoderRadians(), state.angle.getRadians());
    double driveOutput = state.speedMetersPerSecond;

    driveMotor.setVoltage(driveOutput);
    steerMotor.setVoltage(steerOutput);
  }

  public double getEncoderRadians() {
    return encoder.get() + offset;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        (driveMotor.getPosition().getValueAsDouble() / gearRatio * wheelRadius * Math.PI * 2)
            / encoderResolution,
        Rotation2d.fromRadians(getEncoderRadians()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveVelocity, steerAngle);
  }
}
