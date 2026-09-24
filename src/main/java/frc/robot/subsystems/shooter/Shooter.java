package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// shooter and intake are controlled by the same motor. indexer in same subsystem.
public class Shooter extends SubsystemBase {
  // tune
  private double spinUpVelocity = 1500;
  private double shootMaxVelocity = 3000;
  private double intakeMaxVelocity = -2000;
  private double indexerMaxVelocity = 500;

  private double shooterVelocityTolerance = 25;

  private TalonFXConfiguration shooterConfig;
  private TalonFXConfiguration indexerConfig;

  private final int shooterCanId = 23; // change later
  private TalonFX shooterMotor;

  private final int indexerCanId = 45; // change later
  private TalonFX indexerMotor;

  public void init() {

    shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.Slot0 = new com.ctre.phoenix6.configs.Slot0Configs();
    shooterConfig.Slot0.kP = 0;
    shooterConfig.Slot0.kI = 0;
    shooterConfig.Slot0.kD = 0;
    shooterConfig.Slot0.kS = 0;
    shooterConfig.Slot0.kV = 0;
    shooterConfig.Slot0.kA = 0;
    shooterMotor.getConfigurator().apply(shooterConfig, 0.25);

    indexerConfig = new TalonFXConfiguration();
    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    indexerConfig.Slot0 = new com.ctre.phoenix6.configs.Slot0Configs();
    indexerConfig.Slot0.kP = 0;
    indexerConfig.Slot0.kI = 0;
    indexerConfig.Slot0.kD = 0;
    indexerConfig.Slot0.kS = 0;
    indexerConfig.Slot0.kV = 0;
    indexerConfig.Slot0.kA = 0;
    indexerMotor.getConfigurator().apply(indexerConfig, 0.25);
  }

  public Shooter() {
    shooterMotor = new TalonFX(shooterCanId, "can0");
    indexerMotor = new TalonFX(indexerCanId, "can0");
  }

  public void setShooterSpeed(double speed) { // input rps
    shooterMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(speed));
  }

  public void setIndexerSpeed(double speed) { // input rps
    indexerMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(speed));
  }

  public void holdIndexer() {
    indexerMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(0.0));
  }

  public void stopIndexer() {
    indexerMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
  }

  public void stopShooter() {
    shooterMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
  }

  public void stop() {
    shooterMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
    indexerMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
  }

  public void eject() {
    setShooterSpeed(-intakeMaxVelocity);
    setIndexerSpeed(-indexerMaxVelocity);
  }

  // public void shoot() {
  //   setShooterSpeed(shootMaxVelocity);
  // }

  public void shootForHub(double distanceMeters) {
    ShooterUtil.ShooterParameters params = ShooterUtil.getInterpolatedValues(distanceMeters);
    setShooterSpeed(params.shooterRpm() * 60); // setShooterSpeed takes RPS
  }

  public void spinUp() {
    setShooterSpeed(spinUpVelocity);
  }

  public void intake() {
    setShooterSpeed(intakeMaxVelocity);
    setIndexerSpeed(indexerMaxVelocity);
  }

  public void indexerShoot() {
    setIndexerSpeed(-indexerMaxVelocity);
  }

  public boolean isShooterAtSpeed() {
    return shooterMotor.getClosedLoopError().getValueAsDouble() < shooterVelocityTolerance;
  }

  public double getShooterSpeed() {
    return shooterMotor.getVelocity().getValueAsDouble();
  }

  public void periodic() {
    SmartDashboard.putNumber("shooter rps", getShooterSpeed());
  }
}
