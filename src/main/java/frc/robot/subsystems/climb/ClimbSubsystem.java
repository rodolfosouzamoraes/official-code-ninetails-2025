package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

  private SparkMax climbMotor;
  private SparkMaxConfig climbConfig;

  public ClimbSubsystem() {
    
    climbMotor = new SparkMax(ClimbConstants.ID_CLIMB, MotorType.kBrushless);
    climbConfig = new SparkMaxConfig();

    climbConfig
      .smartCurrentLimit(20, 40)
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    
    climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb/Position Climb", climbMotor.getAbsoluteEncoder().getPosition());
  }

  public void setClimbVoltage(double voltage) {
    climbMotor.set(voltage);
  }
}
