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

  private SparkMax leftAlgaeMotor;
  private SparkMax rightAlgaeMotor;
  private SparkMax coralMotor;
  private SparkMax clawCoralMotor;
  

  private SparkMaxConfig leftAlgaeConfig;
  private SparkMaxConfig rightAlgaeConfig;
  private SparkMaxConfig coralConfig;

  public IntakeSubsystem() {

    leftAlgaeMotor = new SparkMax(16, MotorType.kBrushless);
    leftAlgaeConfig = new SparkMaxConfig();

    leftAlgaeConfig
    .smartCurrentLimit(40, 60)
    .idleMode(IdleMode.kBrake)
    .inverted(false);
    leftAlgaeMotor.configure(leftAlgaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    rightAlgaeMotor = new SparkMax(17, MotorType.kBrushless);
    rightAlgaeConfig = new SparkMaxConfig();

    rightAlgaeConfig
    .smartCurrentLimit(40,60)
    .idleMode(IdleMode.kBrake)
    .inverted(true);
    rightAlgaeMotor.configure(rightAlgaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    coralMotor = new SparkMax(18, MotorType.kBrushless);
    clawCoralMotor = new SparkMax(19, MotorType.kBrushless);
    coralConfig = new SparkMaxConfig();

    coralConfig
    .smartCurrentLimit(40, 60)
    .idleMode(IdleMode.kBrake)
    .inverted(false);
    coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    clawCoralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }

  @Override
  public void periodic() {
  }

  // public void setCurrentPoint(double current) {
  //   onlyMotor.set(current);
  // }

  // public void stopIntake() {
  //   onlyMotor.stopMotor();
  // }
   
  // public RelativeEncoder getVelocityEncoder() {
  //   return onlyMotor.getEncoder();
  // }

}
