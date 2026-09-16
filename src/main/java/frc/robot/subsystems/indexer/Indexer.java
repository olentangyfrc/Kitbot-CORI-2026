package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  private TalonFXConfiguration indexerConfig;

  private final int indexerCanId = 0; // fill this in later
  private TalonFX indexerMotor;

  public void init() {

    indexerConfig = new TalonFXConfiguration();
    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    indexerConfig.Slot0 = new com.ctre.phoenix6.configs.Slot0Configs();
    indexerConfig.Slot0.kP = 0;
    indexerConfig.Slot0.kI = 0;
    indexerConfig.Slot0.kD = 0;
    indexerConfig.Slot0.kS = 0;
    indexerConfig.Slot0.kV = 0;
    indexerConfig.Slot0.kA = 0;
    indexerMotor.getConfigurator().apply(indexerConfig, 0.25);
  }

  public Indexer() {
    indexerMotor = new TalonFX(indexerCanId, "can0");
  }

  public void start(double speed) {
    indexerMotor.setControl(
        new com.ctre.phoenix6.controls.VelocityVoltage(speed)); // change this value later
  }

  public void stop() {
    indexerMotor.setControl(new com.ctre.phoenix6.controls.VoltageOut(0.0));
  }
}
