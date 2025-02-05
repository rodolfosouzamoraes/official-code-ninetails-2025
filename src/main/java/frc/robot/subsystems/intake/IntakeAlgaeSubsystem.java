package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeAlgaeSubsystem extends SubsystemBase {
  private SparkMax leftAlgaeMotor;
  private SparkMax rightAlgaeMotor;
  
  private SparkMaxConfig leftAlgaeConfig;
  private SparkMaxConfig rightAlgaeConfig;
  
  public IntakeAlgaeSubsystem() {

    leftAlgaeMotor = new SparkMax(IntakeConstants.ID_LEFT_ALGAE_MOTOR, MotorType.kBrushless);
    leftAlgaeConfig = new SparkMaxConfig();

    leftAlgaeConfig
    .smartCurrentLimit(40, 60)
    .idleMode(IdleMode.kBrake)
    .inverted(false);
    leftAlgaeMotor.configure(leftAlgaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    rightAlgaeMotor = new SparkMax(IntakeConstants.ID_RIGHT_ALGAE_MOTOR, MotorType.kBrushless);
    rightAlgaeConfig = new SparkMaxConfig();

    rightAlgaeConfig
    .smartCurrentLimit(40,60)
    .idleMode(IdleMode.kBrake)
    .inverted(true)
    .follow(IntakeConstants.ID_LEFT_ALGAE_MOTOR);
    rightAlgaeMotor.configure(rightAlgaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAlgaeSpeed(double cSpeed) {
    leftAlgaeMotor.set(cSpeed);
  }

}
