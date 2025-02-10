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
    .inverted(true);
    

    rightMotorConfig = new SparkMaxConfig();

    rightMotorConfig
    .smartCurrentLimit(40, 60)
    .idleMode(IdleMode.kBrake)
    .inverted(false)
    .follow(ElevatorConstants.ID_LEFT_MOTOR, true);


    encoder = new Encoder(ElevatorConstants.CHANNEL_A, ElevatorConstants.CHANNEL_B, false, EncodingType.k4X);
    encoder.reset();
    // Ele tá invertido, e o máximo que eu vi foi 100 - Arthur sábado
    constraints = new TrapezoidProfile.Constraints(0.37, 0.188468);
    pidController = new ProfiledPIDController(0.005, 0, 0, constraints);
    feedforward = new ElevatorFeedforward(0, 0.0, 0, 0);

    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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

  public void controlElevatorJoystick(double output) {
    leftMotor.set(output*0.5);
  }

  public double getEncoderDistance() {
    return encoder.getDistance()/45;
  }

  public void resetEncoder() {
    encoder.reset();
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

}
