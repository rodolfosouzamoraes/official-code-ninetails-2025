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
  // private final PIDController pidController;
  private final ElevatorFeedforward feedforward;


  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    leftMotor = new SparkMax(ElevatorConstants.ID_LEFT_MOTOR, MotorType.kBrushless);
    rightMotor = new SparkMax(ElevatorConstants.ID_RIGHT_MOTOR, MotorType.kBrushless);

    leftMotorConfig = new SparkMaxConfig();
    
    leftMotorConfig
    .smartCurrentLimit(40, 60)
    .idleMode(IdleMode.kBrake)
    .inverted(true);
    

    rightMotorConfig = new SparkMaxConfig();

    rightMotorConfig
    .smartCurrentLimit(40, 60)
    .idleMode(IdleMode.kBrake)
    .inverted(false)
    .follow(ElevatorConstants.ID_LEFT_MOTOR, true);


    encoder = new Encoder(ElevatorConstants.CHANNEL_A, ElevatorConstants.CHANNEL_B, false, EncodingType.k4X);
    encoder.reset();
    encoder.setDistancePerPulse(108.5/5280.0);
    constraints = new TrapezoidProfile.Constraints(5, 8.0);
    pidController = new ProfiledPIDController(0.6, 60.0, 0.004, constraints);
    pidController.setTolerance(3);
    pidController.setIZone(6);


    feedforward = new ElevatorFeedforward(0.25, 1.2, 1.5, 0.05);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator/Encoder Elevador Distance", getEncoderDistance());
    SmartDashboard.putNumber("Elevator/Encoder Elevador Rate", getEncoderRate());
    // SmartDashboard.putNumber("Elevator/Elevator SetPoint", pidController.getSetpoint().position);
    SmartDashboard.putNumber("Elevator/Elevator SetPoint", pidController.getSetpoint().position);
    SmartDashboard.putBoolean("Elevator/Elevator atSetpooint",  atSetpoint());
    SmartDashboard.putNumber("Elevator/Elevator error", pidController.getPositionError());

  }


  public void setSetpoint(double goal) {
    pidController.reset(goal);  
    // pidController.setGoal(goal);
    // pidController.setSetpoint(goal);
  }

  public void controlElevator() {
    double outputPID = pidController.calculate(getEncoderDistance());
    double outputFeedForward = feedforward.calculate(pidController.getSetpoint().velocity);
    double output = outputPID + outputFeedForward;
    leftMotor.setVoltage(-output*0.3); //Redução da movimentação do elevador para 30%
  }

  public void controlElevatorJoystick(double output) {
    leftMotor.setVoltage(output);
  }

  public double getEncoderDistance() {
    return encoder.getDistance();
  }

  public double getEncoderRate() {
    return encoder.getRate();
  }

  public void resetEncoder() {
    encoder.reset();
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void stopMotors() {
    leftMotor.setVoltage(-feedforward.calculate(pidController.getSetpoint().velocity));
  }

}
