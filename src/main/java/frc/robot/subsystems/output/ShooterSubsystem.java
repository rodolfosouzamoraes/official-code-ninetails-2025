package frc.robot.subsystems.output;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  public SparkMax topMotor;
  public SparkMaxConfig topMotorConfig;

  public SparkMax bottomMotor;
  public SparkMaxConfig bottomMotorConfig;

  public DigitalInput IR_input;

  public ShooterSubsystem() {

    topMotor = new SparkMax(ShooterConstants.ID_TOP_MOTOR, MotorType.kBrushless);
    topMotorConfig = new SparkMaxConfig();

    bottomMotor = new SparkMax(ShooterConstants.ID_BOTTOM_MOTOR, MotorType.kBrushless);
    bottomMotorConfig = new SparkMaxConfig();

    topMotorConfig
      .smartCurrentLimit(40, 60)
      .idleMode(IdleMode.kBrake)
      .inverted(true);

    bottomMotorConfig
      .smartCurrentLimit(40, 60)
      .idleMode(IdleMode.kBrake)
      .follow(ShooterConstants.ID_TOP_MOTOR, true)
      .inverted(false);
    
    IR_input = new DigitalInput(3);

    topMotor.configure(topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomMotor.configure(bottomMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Coral/Velocity Shooter", topMotor.getAbsoluteEncoder().getVelocity());
  }

  public Command setShooterSpeed(double shooterSpeed) {
    return run(()-> topMotor.set(shooterSpeed));
  }
  
  public Command stopShooter() {
    return run(() -> topMotor.stopMotor());
  }

  public BooleanSupplier isCoralTrue() {
    return () -> IR_input.get();
  }

  public BooleanSupplier isCoraFalse() {
    return () -> !IR_input.get();
  }

}
