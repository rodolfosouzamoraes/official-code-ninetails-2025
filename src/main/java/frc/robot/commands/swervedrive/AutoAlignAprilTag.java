// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignAprilTag extends Command {
    /** Creates a new AutoAlignAprilTag. */

    SwerveSubsystem swerveSubsystem;
    DoubleSupplier xSpeed;
    DoubleSupplier ySpeed;
    DoubleSupplier zSpeed;    
    double tagSetPoint;
    

    PIDController pidHeadingTag = new PIDController(0, 0, 0);

  public AutoAlignAprilTag(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidHeadingTag.reset();
    tagSetPoint = -1.0;
    zSpeed = () -> 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (LimelightHelpers.getTV("limelight")) {
      tagSetPoint = LimelightHelpers.getFiducialID("limelight");
      
    }

    if (tagSetPoint != -1.0) {
      zSpeed = () -> pidHeadingTag.calculate(swerveSubsystem.getHeading().getDegrees(),tagSetPoint);
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
