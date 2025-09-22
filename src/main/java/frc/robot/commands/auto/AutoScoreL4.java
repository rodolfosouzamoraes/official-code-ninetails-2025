// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.elevator.GoToHeight;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.output.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreL4 extends SequentialCommandGroup {
  /** Creates a new AutoScoreL4. */
  public AutoScoreL4(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new GoToHeight(elevatorSubsystem, ElevatorConstants.L3_HEIGHT)
        .until(() -> elevatorSubsystem.atSetpoint()),

      new ParallelCommandGroup(
        new GoToHeight(elevatorSubsystem, ElevatorConstants.L3_HEIGHT),
        new SequentialCommandGroup(
          new WaitCommand(0.25),  // Tempo para chegar até o reef 
          shooterSubsystem.setShooterSpeed(1)
        )
      ).withTimeout(1),

      new ParallelCommandGroup(
        new GoToHeight(elevatorSubsystem, 0.0),
        shooterSubsystem.stopShooter()
      ).until(() -> elevatorSubsystem.atSetpoint())
    );
  }
}
