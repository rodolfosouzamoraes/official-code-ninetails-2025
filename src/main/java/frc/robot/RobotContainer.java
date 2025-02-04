// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.ButtonMonitor;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularMomentum;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.elevator.GoToHeight;
import frc.robot.commands.intake.GoToAngleWrist;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeAlgaeSubsystem;
import frc.robot.subsystems.intake.IntakeCoralSubsystem;
import frc.robot.subsystems.intake.WristSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

import swervelib.SwerveInputStream;

public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final static CommandXboxController driverXbox = new CommandXboxController(0);
  final static XboxController driverController = new XboxController(0);

  final static CommandGenericHID operatorHID = new CommandGenericHID(1);
  final static GenericHID operatorController = new GenericHID(1);
  
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));

  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final IntakeAlgaeSubsystem intakeAlgae = new IntakeAlgaeSubsystem();
  private final IntakeCoralSubsystem intakeCoral = new IntakeCoralSubsystem();
  private final WristSubsystem wrist = new WristSubsystem();


  private final Field2d field;

  public RobotContainer()
  {
    
    field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });




    // Configure the trigger bindings
    configureBindings();
    configureSwerve();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }


  private void configureBindings()
  {
    driverControllerBindings();
    operatorControllerBindings(); 
  }


  private void driverControllerBindings() {

    driverXbox.rightBumper().whileTrue(drivebase.autoAlign(
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX(),
      -125
    ));

    driverXbox.leftBumper().whileTrue(drivebase.autoAlign(
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX(),
      125
    ));
  }

  private void operatorControllerBindings() {
    elevator.setDefaultCommand(new GoToHeight(elevator, 0.0));
    intakeAlgae.setDefaultCommand(new RunCommand(() -> intakeAlgae.setAlgaeSpeed(-0.01), intakeAlgae));
    intakeCoral.setDefaultCommand(new RunCommand(() -> intakeCoral.setCoralSpeed(0), intakeCoral));

    
    operatorHID.button(ButtonConstants.COLLECT_ALGAE).whileTrue(
      new RunCommand(() -> intakeAlgae.setAlgaeSpeed(-0.5), intakeAlgae)
    );    

    operatorHID.button(ButtonConstants.COLLECT_CORAL).whileTrue(
      new RunCommand(() -> intakeCoral.setCoralSpeed(-0.5), intakeCoral)
    );

    operatorHID.button(ButtonConstants.SHOOTER_ALGAE).whileTrue(
      new RunCommand(() -> intakeAlgae.setAlgaeSpeed(0.5), intakeAlgae)
    );

    operatorHID.button(ButtonConstants.SHOOTER_CORAL).whileTrue(
      new RunCommand(() -> intakeCoral.setCoralSpeed(0.5), intakeCoral)
    );

    operatorHID.button(ButtonConstants.GO_TO_L1).whileTrue(
      new ParallelCommandGroup(
        new GoToHeight(elevator, ElevatorConstants.L1_HEIGHT),
        new GoToAngleWrist(wrist, 20)
      ));
    
    operatorHID.button(ButtonConstants.GO_TO_L2).whileTrue(getAutonomousCommand());
    operatorHID.button(ButtonConstants.GO_TO_L3).whileTrue(getAutonomousCommand());
    operatorHID.button(ButtonConstants.GO_TO_L4).whileTrue(getAutonomousCommand());

    operatorHID.button(ButtonConstants.END_GAME).whileTrue(getAutonomousCommand());    

  }


  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  private void configureSwerve() {
    AbsoluteDrive absoluteDrive = new AbsoluteDrive(drivebase,
                                  () -> MathUtil.applyDeadband(driverXbox.getLeftY()*0.75,
                                                              OperatorConstants.LEFT_Y_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getLeftX()*0.75,
                                                              OperatorConstants.LEFT_X_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                              OperatorConstants.RIGHT_X_DEADBAND),
                                  () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                              OperatorConstants.RIGHT_X_DEADBAND));

                                                              
    Command baseDriveCommand = drivebase.driveCommand(        
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightX()*-1,.1));

        Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX(),
      () -> driverXbox.getRightY());

        // drivebase.setDefaultCommand(baseDriveCommand);


    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(
      SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> driverXbox.getLeftY() * -1,
    () -> driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true));

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    
  }


  public static CommandXboxController getDriverXbox() {
    return driverXbox;
  }
}
