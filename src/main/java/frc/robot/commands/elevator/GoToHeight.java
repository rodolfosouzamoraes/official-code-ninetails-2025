// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class GoToHeight extends Command {
  private final ElevatorSubsystem elevator;
  private double setPoint;


  public GoToHeight(ElevatorSubsystem elevatorSubsystem, double setPoint) {
    this.elevator = elevatorSubsystem;
    this.setPoint = setPoint;

    addRequirements(elevator);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    elevator.setSetpoint(setPoint);
    elevator.controlElevator();
  }

  @Override
  public void end(boolean interrupted) { }

  @Override
  public boolean isFinished() {
    return false;
  }
}
