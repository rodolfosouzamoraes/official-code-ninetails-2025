package frc.robot.commands.output;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.output.ShooterSubsystem;

public class ShooterCollect extends SequentialCommandGroup {

  public ShooterCollect(ShooterSubsystem outPutSubsystem) {
    addCommands(

      outPutSubsystem.setShooterSpeed(1)
        .until(outPutSubsystem.isCoralTrue()),

      outPutSubsystem.setShooterSpeed(0.5)
        .until(outPutSubsystem.isCoraFalse()),

      outPutSubsystem.setShooterSpeed(-0.4)
        .until(outPutSubsystem.isCoralTrue()),

      outPutSubsystem.setShooterSpeed(0.2).withTimeout(0.2)
    );
  }
}
