// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.elevator.GoToHeight;
import frc.robot.commands.intake.GoToAngleWrist;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeCoralSubsystem;
import frc.robot.subsystems.intake.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreL3 extends SequentialCommandGroup {

  public ScoreL3(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, IntakeCoralSubsystem intakeCoralSubsystem) {
    addCommands(
      new ParallelCommandGroup(
        new GoToHeight(elevatorSubsystem, ElevatorConstants.L3_HEIGHT),
        new GoToAngleWrist(wristSubsystem, IntakeConstants.POSITION_ANGLE_WRIST_L2_L3)
      ).until(() -> elevatorSubsystem.atSetpoint()),

      new ParallelCommandGroup(
        new GoToHeight(elevatorSubsystem, ElevatorConstants.L3_HEIGHT),
        new GoToAngleWrist(wristSubsystem, IntakeConstants.POSITION_ANGLE_WRIST_L2_L3),

        new SequentialCommandGroup(
          new WaitCommand(0.25),
          new InstantCommand(() -> intakeCoralSubsystem.setCoralSpeed(-0.8), intakeCoralSubsystem)
        )
      ).withTimeout(1),


      new InstantCommand(() -> intakeCoralSubsystem.setCoralSpeed(0), intakeCoralSubsystem),
      new ParallelCommandGroup(
        new GoToHeight(elevatorSubsystem, 0.0),
        new GoToAngleWrist(wristSubsystem, 0.0)
      ).until(() -> elevatorSubsystem.atSetpoint())

    );
  }
}
