package frc.robot.commands.output;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.output.ShooterSubsystem;

public class ShooterCollect extends SequentialCommandGroup {

  public ShooterCollect(ShooterSubsystem outPutSubsystem) {
    addCommands(

      outPutSubsystem.setShooterSpeed(1)
        .until(outPutSubsystem.isCoralTrue()).andThen(outPutSubsystem.stopShooter()),

      outPutSubsystem.setShooterSpeed(0.5)
        .until(outPutSubsystem.isCoraFalse()).andThen(outPutSubsystem.stopShooter()),

      outPutSubsystem.setShooterSpeed(-0.6)
        .until(outPutSubsystem.isCoralTrue()).andThen(outPutSubsystem.stopShooter())

    );
  }
}
