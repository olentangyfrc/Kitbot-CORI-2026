package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private TalonFXConfiguration shooterConfig;

  private final int shooterCanId = 0; // Fill in later
  private TalonFX shooterMotor;

  public void init() {

    shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterConfig.Slot0 = new com.ctre.phoenix6.configs.Slot0Configs();
    shooterConfig.Slot0.kP = 0;
    shooterConfig.Slot0.kI = 0;
    shooterConfig.Slot0.kD = 0;
    shooterConfig.Slot0.kS = 0;
    shooterConfig.Slot0.kV = 0;
    shooterConfig.Slot0.kA = 0;
    shooterMotor.getConfigurator().apply(shooterConfig, 0.25);
  }

  public Shooter() {
    shooterMotor = new TalonFX(shooterCanId, "can0");
  }

  public void start() {
    shooterMotor.setControl(
        new com.ctre.phoenix6.controls.VelocityVoltage(0.0)); // change this value later
  }

  public void stop() {
    shooterMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
  }

  public void warmUp() {
    // fill this in later
  }
}
