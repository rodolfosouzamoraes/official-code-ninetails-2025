package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeCoralSubsystem extends SubsystemBase {

  private SparkMax coralMotor;
  private SparkMaxConfig coralConfig;

  public IntakeCoralSubsystem() {
    
    coralMotor = new SparkMax(IntakeConstants.ID_CORAL_MOTOR, MotorType.kBrushless);
    coralConfig = new SparkMaxConfig();

    coralConfig
      .smartCurrentLimit(40, 60)
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setCoralSpeed(double cSpeed) {
    coralMotor.set(cSpeed);
  }
}
