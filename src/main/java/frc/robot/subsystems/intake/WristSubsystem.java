package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class WristSubsystem extends SubsystemBase {
  private SparkMax wristMotor;
  private SparkMaxConfig wristConfig;

  private SparkClosedLoopController closed_controller;
  private double setpoint;
  
  public WristSubsystem() {
    
    wristMotor = new SparkMax(IntakeConstants.ID_WRIST_MOTOR, MotorType.kBrushless);
    wristConfig = new SparkMaxConfig();
    
    wristConfig
    .smartCurrentLimit(40, 60)
    .idleMode(IdleMode.kBrake)
    .inverted(false);

    wristConfig.closedLoop.maxMotion      
      .maxVelocity(5000)
      .maxAcceleration(1250)
      .allowedClosedLoopError(0.05);


    wristConfig.closedLoop
      .pidf(0.3, 0.002, 0.12, 0.000000001)
      .iZone(0.15)
      .outputRange(-1, 1);
    closed_controller = wristMotor.getClosedLoopController();

    wristConfig.softLimit
      .reverseSoftLimitEnabled(true)
      .forwardSoftLimitEnabled(true)
      .forwardSoftLimit(4)
      .reverseSoftLimit(0);
    resetEncoder();

    wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    


    // wristConfig.encoder.positionConversionFactor(1/16);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Wrist", getEncoderDistance());
    SmartDashboard.putNumber("SetPoint Wrist", setpoint);
    
    
  }

  public void controlWrist(double setpoint) {
    this.setpoint = setpoint;
    closed_controller.setReference(setpoint, ControlType.kMAXMotionPositionControl);
  }

  public void controleWrist(double speed) {
    wristMotor.set(speed);
  }

  public double getEncoderDistance() {
    return wristMotor.getEncoder().getPosition();
  }

  public void resetEncoder() {
    wristMotor.getEncoder().setPosition(0);
  }

  public boolean atSetpoint() {
    if (getEncoderDistance() < setpoint+0.1 && getEncoderDistance() > setpoint-0.1) {
      return true;
    } 
    return false;
  }


}
