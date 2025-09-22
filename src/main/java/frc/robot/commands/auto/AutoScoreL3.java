package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.elevator.GoToHeight;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.output.ShooterSubsystem;


public class AutoScoreL3 extends SequentialCommandGroup {
  public AutoScoreL3(ElevatorSubsystem elevatorSubsystem, ShooterSubsystem shooterSubsystem) {
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
