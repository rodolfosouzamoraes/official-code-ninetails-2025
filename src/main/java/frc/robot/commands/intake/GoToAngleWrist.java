// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.WristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToAngleWrist extends Command {
  private final WristSubsystem wrist;
  private double setPoint;


  public GoToAngleWrist(WristSubsystem wrist, double setPoint) {
    this.wrist = wrist;
    this.setPoint = setPoint;

    addRequirements(wrist);
  }

  @Override
  public void initialize() {
    wrist.controlWrist(setPoint);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) { }

  @Override
  public boolean isFinished() {
    if (wrist.atSetpoint()) {
      return true;
    }
    return false;
  }
}
