package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ModuleIOReal implements ModuleIO {
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
  private double steerP = 5;
  private final double wheelRadius = Units.inchesToMeters(1.8125); // inches
  private final double wheelCircumference = 2 * Math.PI * wheelRadius;
  private final double gearRatio = 8.14; // maybe 8.33

  private double driveVelocity = 0; // meters per second
  private Rotation2d steerAngle = new Rotation2d();
  private double steerOutput = 0;

  public ModuleIOReal(int driveMotorCanId, int steerMotorCanId, int encoderId, double motorOffset) {
    driveMotor = new TalonFX(driveMotorCanId, "rio");
    steerMotor = new SparkMax(steerMotorCanId, MotorType.kBrushless);
    // encoder = new AnalogEncoder(encoderId);
    encoder = new AnalogEncoder(encoderId, 2 * Math.PI, 0);

    offset = motorOffset; // radians

    driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0.kS = 0.005755075;
    driveConfig.Slot0.kV = 0.10939;
    driveConfig.Slot0.kA = 0.0027408;
    driveConfig.Slot0.kP = 0.047423;
    driveConfig.Slot0.kI = 0.0;
    driveConfig.Slot0.kD = 0.0;
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveMotor.getConfigurator().apply(driveConfig, 0.25);

    steerConfig = new SparkMaxConfig();
    steerConfig.idleMode(IdleMode.kCoast);
    steerMotor.configure(
        steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    steerPIDController = new PIDController(steerP, 0, 0);
    steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // SmartDashboard.putBoolean("swerveTuningMode", swerveTuningMode);
    // SmartDashboard.putNumber("steerP", steerP);
    // SmartDashboard.putNumber("steerTuningOutput" + steerMotor.getDeviceId(), steerTuningOutput);
  }

  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble());
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveMotor.getVelocity().getValueAsDouble());
    inputs.driveAppliedVolts = driveMotor.getMotorVoltage().getValueAsDouble();

    inputs.steerAbsolutePosition = Rotation2d.fromRotations(getEncoderRadians());
    inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerOutput); // wrong
    inputs.steerAppliedVolts = steerMotor.getBusVoltage();

    inputs.swerveState = new SwerveModuleState(driveVelocity, steerAngle);
    inputs.swervePosition =
        new SwerveModulePosition(
            (driveMotor.getPosition().getValueAsDouble() / gearRatio * wheelCircumference),
            Rotation2d.fromRadians(getEncoderRadians()));
  }

  public void setSwerveState(SwerveModuleState state) {
    Rotation2d encoderRotation2d = new Rotation2d(getEncoderRadians());

    state.optimize(encoderRotation2d);
    state.cosineScale(encoderRotation2d);

    driveVelocity = state.speedMetersPerSecond;
    steerAngle = state.angle;

    double steerOutput =
        steerPIDController.calculate(getEncoderRadians(), state.angle.getRadians());

    this.steerOutput = steerOutput;

    if (swerveTuningMode) {
      steerOutput = steerTuningOutput;
    }
    // SmartDashboard.putNumber("swerveSteetOutput" + steerMotor.getDeviceId(), steerOutput);

    // double driveOutput = (state.speedMetersPerSecond / wheelCircumference) * gearRatio;
    double driveOutput =
        (state.speedMetersPerSecond * Constants.falconMaxSpeed) / Constants.maxLinearSpeed;

    driveMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(driveOutput / 60));
    // driveMotor.setVoltage(driveOutput / 60);
    steerMotor.setVoltage(steerOutput);
  }

  private double getEncoderRadians() {
    return encoder.get() - offset;
  }
}

  // public void periodic() {
    // swerveTuningMode = SmartDashboard.getBoolean("swerveTuningMode", swerveTuningMode);

    // if (swerveTuningMode) {
    //   steerP = SmartDashboard.getNumber("steerP", steerP);
    //   steerPIDController.setP(steerP);

    //   SmartDashboard.putNumber(
    //       "steerMotorId" + steerMotor.getDeviceId(), getPosition().angle.getDegrees());

    //   steerTuningOutput =
    //       SmartDashboard.getNumber(
    //           "steerTuningOutput" + steerMotor.getDeviceId(), steerTuningOutput);
    // }
    // SmartDashboard.putNumber("encoder" + steerMotor.getDeviceId(), getEncoderRadians());
//   }
// }
