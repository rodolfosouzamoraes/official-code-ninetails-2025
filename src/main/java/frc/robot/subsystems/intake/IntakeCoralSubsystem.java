// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeCoralSubsystem extends SubsystemBase {
  private SparkMax coralMotor;
  private SparkMaxConfig coralConfig;

  

  
  /** Creates a new IntakeCoralSubsystem. */
  public IntakeCoralSubsystem() {
    coralMotor = new SparkMax(18, MotorType.kBrushless);
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
