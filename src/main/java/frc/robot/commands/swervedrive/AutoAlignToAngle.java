// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignToAngle extends Command {
    /** Creates a new AutoAlignAprilTag. */

    SwerveSubsystem swerveSubsystem;
    DoubleSupplier xSpeed;
    DoubleSupplier ySpeed;
    DoubleSupplier zSpeed;    
    double angleSetpoint;
    

    PIDController pidHeading = new PIDController(0, 0, 0);

  public AutoAlignToAngle(
    SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeed, 
    DoubleSupplier ySpeed, double angleSetpoint
  ) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.angleSetpoint = angleSetpoint;
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidHeading.reset();
    zSpeed = () -> 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    zSpeed = () -> pidHeading.calculate(swerveSubsystem.getHeading().getDegrees(), angleSetpoint);
    if (Math.abs(pidHeading.getError()) > Math.abs(angleSetpoint)/2) {
      xSpeed = () -> xSpeed.getAsDouble() / 2;
      ySpeed = () -> xSpeed.getAsDouble() / 2;
    }

    swerveSubsystem.driveCommand(xSpeed, ySpeed, zSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
