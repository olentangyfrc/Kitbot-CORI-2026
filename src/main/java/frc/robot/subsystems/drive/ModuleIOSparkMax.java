package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
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
  private TalonFX driveMotor;
  private SparkMax steerMotor;
  private AnalogEncoder encoder;
  private double offset;

  private PIDController steerPIDController;
  private double steerP = 0;
  private double steerI = 0;
  private double steerD = 0;

  // odometry stuff, need change later
  private double wheelRadius = 4;
  private double gearRatio = 3 / 1;
  private double encoderResolution = 400;

  public ModuleIOSparkMax(
      int driveMotorCanId, int steerMotorCanId, int encoderId, double motorOffset) {
    driveMotor = new TalonFX(driveMotorCanId, "can0");
    steerMotor = new SparkMax(steerMotorCanId, MotorType.kBrushless);
    encoder = new AnalogEncoder(encoderId, 360, 0);
    offset = motorOffset;

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    SparkMaxConfig steerConfig = new SparkMaxConfig();

    // do sparkmax configs

    // change resetmode and persistmode later
    driveMotor.getConfigurator().apply(driveConfig);

    steerMotor.configure(
        steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    steerPIDController = new PIDController(steerP, steerI, steerD);
    steerPIDController.enableContinuousInput(-180, 180);
  }

  public double getEncoderRadians() {
    return Math.toRadians(encoder.get()) + offset;
  }

  public void setState(SwerveModuleState state) {
    Rotation2d encoderRotation2d = new Rotation2d(getEncoderRadians());

    state.optimize(encoderRotation2d);
    state.cosineScale(encoderRotation2d);

    double steerOutput =
        steerPIDController.calculate(getEncoderRadians(), state.angle.getRadians());
    double driveOutput = state.speedMetersPerSecond;

    driveMotor.setVoltage(driveOutput);
    driveMotor.setVoltage(steerOutput);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        (driveMotor.getPosition().getValueAsDouble() * wheelRadius * Math.PI * 2 * gearRatio)
            / encoderResolution,
        Rotation2d.fromRadians(getEncoderRadians()));
  }
}
