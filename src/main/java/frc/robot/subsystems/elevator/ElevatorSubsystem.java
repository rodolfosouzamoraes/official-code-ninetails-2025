// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax leftElevator;
  private SparkMax rightElevator;
  private Encoder elevator_Encoder;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    leftElevator = new SparkMax(14, MotorType.kBrushless);
    SparkMaxConfig configLeftElevator = new SparkMaxConfig();

    configLeftElevator = new SparkMaxConfig();
    configLeftElevator
    .smartCurrentLimit(40, 60)
    .idleMode(IdleMode.kBrake)
    .inverted(false);

    leftElevator.configure(configLeftElevator, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    
    rightElevator = new SparkMax(15, MotorType.kBrushless);
    SparkMaxConfig configRightElevator = new SparkMaxConfig();

    configRightElevator
    .smartCurrentLimit(40, 60)
    .idleMode(IdleMode.kBrake)
    .inverted(true);
    
    rightElevator.configure(configLeftElevator, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevator_Encoder = new Encoder(0, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getEncoder() {
   return elevator_Encoder.get();
  }

}
