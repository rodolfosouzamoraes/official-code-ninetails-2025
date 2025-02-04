// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  
  private SparkMax wristMotor;
  private SparkMaxConfig wristConfig;
  private SparkClosedLoopController closed;
  private double setPoint;
  
  /** Creates a new WristSubsystem. */
  public WristSubsystem() {
    
    wristMotor = new SparkMax(0, MotorType.kBrushless);
    wristConfig = new SparkMaxConfig();
    
    wristConfig
    .smartCurrentLimit(40, 60)
    .idleMode(IdleMode.kBrake)
    .inverted(false);
    wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    wristConfig.closedLoop
      .pid(0, 0, 0)
      .outputRange(0, 0);

    closed = wristMotor.getClosedLoopController();
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setSetpoint(double goal) {
    closed.setReference(goal, ControlType.kMAXMotionPositionControl);
  }

  public void controlWrist() {}

  public double getEncoderDistance() {
    return wristMotor.getEncoder().getPosition();
  }

  public void resetEncoder() {
    wristMotor.getEncoder().setPosition(0);
  }

  public boolean atSetpoint() {
    if (getEncoderDistance() < setPoint+1 && getEncoderDistance() > setPoint-1) {
      return true;
    } 
    return false;
  }


}
