// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private SparkMax onlyMotor;
  private SparkMaxConfig intakeConfig;

  public IntakeSubsystem() {

    onlyMotor = new SparkMax(16, MotorType.kBrushless);
    
    intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(40, 60);
    intakeConfig.idleMode(IdleMode.kBrake)
    .inverted(false);
    onlyMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  @Override
  public void periodic() {
  }

  public void setCurrentPoint(double current) {
    onlyMotor.set(current);
  }

  public void stopIntake() {
    onlyMotor.stopMotor();
  }
   
  public RelativeEncoder getVelocityEncoder() {
    return onlyMotor.getEncoder();
  }

}
