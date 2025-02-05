// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax leftMotor;
  private SparkMax rightMotor;

  private SparkMaxConfig rightMotorConfig;
  private SparkMaxConfig leftMotorConfig;

  private final Encoder encoder;


  private final TrapezoidProfile.Constraints constraints;
  private final ProfiledPIDController pidController;
  private final ElevatorFeedforward feedforward;


  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    leftMotor = new SparkMax(ElevatorConstants.ID_LEFT_MOTOR, MotorType.kBrushless);
    rightMotor = new SparkMax(ElevatorConstants.ID_RIGHT_MOTOR, MotorType.kBrushless);

    leftMotorConfig = new SparkMaxConfig();
    
    leftMotorConfig
    .smartCurrentLimit(40, 60)
    .idleMode(IdleMode.kBrake)
    .inverted(false);
    
    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightMotorConfig = new SparkMaxConfig();

    rightMotorConfig
    .smartCurrentLimit(40, 60)
    .idleMode(IdleMode.kBrake)
    .inverted(true)
    .follow(ElevatorConstants.ID_LEFT_MOTOR);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    encoder = new Encoder(ElevatorConstants.CHANNEL_A, ElevatorConstants.CHANNEL_B, true, EncodingType.k4X);
    encoder.setDistancePerPulse(0.008);

    constraints = new TrapezoidProfile.Constraints(0.37, 0.188468);
    pidController = new ProfiledPIDController(0.005, 0, 0, constraints, 0);
    feedforward = new ElevatorFeedforward(0, 0.22, 32.04, 0.02);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Elevador", getEncoderDistance());
  }


  public void setSetpoint(double goal) {
    pidController.setGoal(goal);
  }

  public void controlElevator() {
    leftMotor.setVoltage(
      pidController.calculate(getEncoderDistance())
      + feedforward.calculate(pidController.getSetpoint().velocity)
    );
  }

  public double getEncoderDistance() {
    return encoder.getDistance();
  }

  public void resetEncoder() {
    encoder.reset();
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

}
