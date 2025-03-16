// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.intake.GoToAngleWrist;
import frc.robot.subsystems.intake.IntakeCoralSubsystem;
import frc.robot.subsystems.intake.WristSubsystem;

public class CollectCoral extends SequentialCommandGroup {
  public CollectCoral(IntakeCoralSubsystem intakeCoral, WristSubsystem wristSubsystem) {

    addCommands(
      new GoToAngleWrist(wristSubsystem, IntakeConstants.POSITION_ANGLE_WRIST_COLLECTION).until(() -> wristSubsystem.atSetpoint()),
      new InstantCommand(() -> intakeCoral.setCoralSpeed(0.7), intakeCoral),
      new WaitCommand(4),
      new InstantCommand(() -> intakeCoral.setCoralSpeed(0.0), intakeCoral),
      new GoToAngleWrist(wristSubsystem, 0.3).withTimeout(0.25)
    );
  }
}
