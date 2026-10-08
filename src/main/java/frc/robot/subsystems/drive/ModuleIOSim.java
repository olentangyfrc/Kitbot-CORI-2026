package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.simulation.AnalogEncoderSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

public class ModuleIOSim implements ModuleIO {
  private final double driveGearRatio = 8.14;
  private final double steerGearRatio = 12.8;

  private final TalonFX driveMotor;
  private final TalonFXConfiguration driveConfig;
  private final TalonFXSimState driveMotorSim;
  private final DCMotorSim driveMotorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.00005, 1),
          DCMotor.getFalcon500(1));

  private final SparkMax steerMotor;
  private final SparkMaxConfig steerConfig;
  private final SparkMaxSim steerMotorSim;
  private final DCMotorSim steerMotorSimModel =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.0001, 1), DCMotor.getNEO(1));

  private final AnalogEncoder encoder;
  private final AnalogEncoderSim encoderSim;
  private final double offset;

  private double driveVelocity = 0;
  private Rotation2d steerAngle = new Rotation2d();

  private final PIDController steerPIDController;
  private double steerP = 5;

  private final double wheelRadius = Units.inchesToMeters(1.8125); // inches
  private final double wheelCircumference = 2 * Math.PI * wheelRadius;

  public ModuleIOSim(int driveMotorCanId, int steerMotorCanId, int encoderId, double motorOffset) {
    driveMotor = new TalonFX(driveMotorCanId);
    driveMotorSim = driveMotor.getSimState();
    driveMotorSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);

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

    steerMotor = new SparkMax(steerMotorCanId, SparkLowLevel.MotorType.kBrushless);
    steerMotorSim = new SparkMaxSim(steerMotor, DCMotor.getNEO(1));

    steerConfig = new SparkMaxConfig();
    steerConfig.idleMode(IdleMode.kCoast);
    steerMotor.configure(
        steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = new AnalogEncoder(encoderId, 2 * Math.PI, 0);
    encoderSim = new AnalogEncoderSim(encoder);

    offset = motorOffset;

    steerPIDController = new PIDController(steerP, 0, 0.1);
    steerPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void updateInputs(ModuleIOInputs inputs) {
    simMotors();
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble() / driveGearRatio);
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveMotor.getVelocity().getValueAsDouble());
    inputs.driveVelocityMetersPerSec =
        driveMotor.getVelocity().getValueAsDouble() * wheelCircumference / driveGearRatio;
    inputs.driveAppliedVolts = driveMotor.getMotorVoltage().getValueAsDouble();
    inputs.driveCurrentAmps = driveMotorSimModel.getCurrentDrawAmps();

    inputs.steerAbsolutePosition = Rotation2d.fromRadians(getEncoderRadians());
    inputs.steerVelocityRadPerSec =
        Units.rotationsToRadians(steerMotorSimModel.getAngularVelocityRadPerSec()); // wrong
    inputs.steerAppliedVolts = steerMotor.getAppliedOutput();
    inputs.steerCurrentAmps = steerMotorSimModel.getCurrentDrawAmps();

    inputs.swerveState = new SwerveModuleState(driveVelocity, steerAngle);
    inputs.swervePosition =
        new SwerveModulePosition(
            (driveMotor.getPosition().getValueAsDouble() * wheelCircumference / driveGearRatio),
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

    // if (swerveTuningMode) {
    // steerOutput = steerTuningOutput;
    // }
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

  public void simMotors() {
    driveMotorSim.setSupplyVoltage(RoboRioSim.getVInVoltage());

    Voltage driveVoltage = driveMotorSim.getMotorVoltageMeasure();
    Double steerVoltage = steerMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage();

    driveMotorSimModel.setInputVoltage(driveVoltage.in(Volts));
    driveMotorSimModel.update(0.02);

    driveMotorSim.setRawRotorPosition(driveMotorSimModel.getAngularPosition());
    driveMotorSim.setRotorVelocity(driveMotorSimModel.getAngularVelocity());

    steerMotorSimModel.setInputVoltage(steerVoltage);
    steerMotorSimModel.update(0.02);

    encoderSim.set(steerMotorSimModel.getAngularPositionRad() / steerGearRatio);

    steerMotorSim.iterate(
        steerMotorSimModel.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

    // SmartDashboard.putNumber(
    //     "drive pos" + steerMotor.getDeviceId(),
    //     Units.rotationsToDegrees(driveMotor.getPosition().getValueAsDouble()));
    // SmartDashboard.putNumber(
    //     "steer pos" + steerMotor.getDeviceId(),
    //     Units.rotationsToDegrees(Units.rotationsToDegrees(getEncoderRadians())));
    // SmartDashboard.putNumber("drive target velocity mps" + driveMotor.getDeviceID(),
    // driveVelocity);
    // SmartDashboard.putNumber(
    //     "drive current velocity mps" + driveMotor.getDeviceID(),
    //     driveMotor.getVelocity().getValueAsDouble() * wheelCircumference / driveGearRatio);

    // SmartDashboard.putNumber(
    //     "drive volts" + driveMotor.getDeviceID(),
    // driveMotor.getMotorVoltage().getValueAsDouble());
    // SmartDashboard.putNumber(
    //     "steer volts" + steerMotor.getDeviceId(), steerMotor.getAppliedOutput());
  }
}
