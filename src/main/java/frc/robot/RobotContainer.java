// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.auto.CollectCoral;
import frc.robot.commands.auto.ScoreL2;
import frc.robot.commands.auto.ScoreL3;
import frc.robot.commands.auto.ScoreL4;
import frc.robot.commands.elevator.GoToHeight;
import frc.robot.commands.intake.GoToAngleWrist;
import frc.robot.subsystems.climb.ClimbSubsystem;
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
  final static CommandXboxController operatorControllerXbox = new CommandXboxController(1);

  
  // The robot's subsystems and commands are defined here...
  private final static SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));

  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final IntakeAlgaeSubsystem intakeAlgae = new IntakeAlgaeSubsystem();
  private final IntakeCoralSubsystem intakeCoral = new IntakeCoralSubsystem();
  private final ClimbSubsystem climb = new ClimbSubsystem();
  private final WristSubsystem wrist = new WristSubsystem();

  private final Field2d field = new Field2d();

  private final SendableChooser<String> pathChooser = new SendableChooser<>();

  public RobotContainer()
  {
    // Configure the trigger bindings
    
    configureLog();
    configureBindings();
    configureSwerve();
    configureNamedCommand();
    configurePathChooser();
    DriverStation.silenceJoystickConnectionWarning(true);

  }

  private void configureLog() {
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

  }

  private void configureBindings()
  {
    // driverControllerBindings();
    operatorControllerBindings(); 
  }



  // private void driverControllerBindings() {

  //   driverXbox.a().whileTrue(new AutoAlignAprilTag(
  //     drivebase, 
  //     () -> driverXbox.getLeftY() * -1,
  //     () -> driverXbox.getLeftX() * -1
  //   ));

  //   // driverXbox.rightBumper().whileTrue(new AutoAlignToAngle(
  //   //   drivebase,
  //   //   () -> driverXbox.getLeftY() * -1,
  //   //   () -> driverXbox.getLeftX() * -1,
  //   //   -125
  //   // ));

    
  //   driverXbox.leftBumper().whileTrue(new RunCommand( 
  //     () -> drivebase.autoAlignToAngle(
  //       driverXbox.getLeftY() * -1,
  //       driverXbox.getLeftX() * -1,
  //       125), drivebase
  //   ));


  //   // driverXbox.leftBumper().whileTrue(new AutoAlignToAngle(
  //   //   drivebase,
  //   //   () -> driverXbox.getLeftY() * -1,
  //   //   () -> driverXbox.getLeftX() * -1,
  //   //   125
  //   // ));
  // }



  private void operatorControllerBindings() {

    elevator.setDefaultCommand(new GoToHeight(elevator, 0));
    intakeAlgae.setDefaultCommand(new RunCommand(() -> intakeAlgae.setAlgaeSpeed(0), intakeAlgae));
    intakeCoral.setDefaultCommand(new RunCommand(() -> intakeCoral.setCoralSpeed(0.0), intakeCoral));
    wrist.setDefaultCommand(new GoToAngleWrist(wrist, 0.2));
    climb.setDefaultCommand(new RunCommand(() -> climb.setClimbVoltage(0), climb));
    // commandsXboxController();
    commandsHIDController();
  }

  private void commandsHIDController() {
    // Coletar Alga 
    operatorHID.button(ButtonConstants.COLLECT_ALGAE).whileTrue(
      new RunCommand(() -> intakeAlgae.setAlgaeSpeed(-0.8), intakeAlgae));

    // Atirar alga
    operatorHID.button(ButtonConstants.SHOOTER_ALGAE).whileTrue(
      new RunCommand(() -> intakeAlgae.setAlgaeSpeed(0.5), intakeAlgae));

    // Coletar Coral
    operatorHID.button(ButtonConstants.COLLECT_CORAL).whileTrue(new ParallelCommandGroup(
        new RunCommand(() -> intakeCoral.setCoralSpeed(0.5), intakeCoral),
        new GoToAngleWrist(wrist, IntakeConstants.POSITION_ANGLE_WRIST_COLLECTION)
        ));

    // Atirar Coral
    operatorHID.button(ButtonConstants.SHOOTER_CORAL).whileTrue(
      new RunCommand(() -> intakeCoral.setCoralSpeed(-0.5), intakeCoral));

    // Go To L2
    operatorHID.button(ButtonConstants.GO_TO_L2).whileTrue(new ParallelCommandGroup(
      new GoToHeight(elevator, ElevatorConstants.L2_HEIGHT),
      new GoToAngleWrist(wrist, IntakeConstants.POSITION_ANGLE_WRIST_L2)
    ));

      // Go To Ball L3
    operatorHID.button(ButtonConstants.GO_TO_BALL_L3).whileTrue(
      new GoToHeight(elevator, ElevatorConstants.L3_BALL_HEIGHT));

    // Go To L3
    operatorHID.button(ButtonConstants.GO_TO_L3).whileTrue(new ParallelCommandGroup(
      new GoToHeight(elevator, ElevatorConstants.L3_HEIGHT),
      new GoToAngleWrist(wrist, IntakeConstants.POSITION_ANGLE_WRIST_L3)
    ));

    // Go To L4
    operatorHID.button(ButtonConstants.GO_TO_L4).whileTrue(  new GoToHeight(elevator, ElevatorConstants.L4_HEIGHT)      );

    // CLimb
    operatorHID.button(ButtonConstants.CLIMB).whileTrue(
      new RunCommand(() -> climb.setClimbVoltage(14), climb));

    // Unclimb
    operatorHID.button(ButtonConstants.UNCLIMB).whileTrue(
      new RunCommand(() -> climb.setClimbVoltage(-14), climb));
  }

  private void commandsXboxController() {
    // Coletar Alga
    operatorControllerXbox.rightBumper().whileTrue(
      new RunCommand(() -> intakeAlgae.setAlgaeSpeed(0.5), intakeAlgae));

    // Atirar alga
    operatorControllerXbox.rightTrigger().whileTrue(
      new RunCommand(() -> intakeAlgae.setAlgaeSpeed(-0.5), intakeAlgae));

    // Coletar Coral
    operatorControllerXbox.leftBumper().whileTrue(new ParallelCommandGroup(
        new RunCommand(() -> intakeCoral.setCoralSpeed(0.5), intakeCoral),
        new GoToAngleWrist(wrist, IntakeConstants.POSITION_ANGLE_WRIST_COLLECTION)
    ));

    // Atirar Coral
    operatorControllerXbox.leftTrigger().whileTrue(
      new RunCommand(() -> intakeCoral.setCoralSpeed(-0.5), intakeCoral));

    // Go To L2
    operatorControllerXbox.x().whileTrue(new ParallelCommandGroup(
      new GoToHeight(elevator, ElevatorConstants.L2_HEIGHT),
      new GoToAngleWrist(wrist, IntakeConstants.POSITION_ANGLE_WRIST_L2)
    ));

    // NÃO SEI
    operatorControllerXbox.y().whileTrue(
      new GoToAngleWrist(wrist, 3.2));

    // Go To Ball L3
    operatorControllerXbox.povRight().whileTrue(
      new GoToHeight(elevator, ElevatorConstants.L3_BALL_HEIGHT));

    // Go To L3
    operatorControllerXbox.a().whileTrue(new ParallelCommandGroup(
      new GoToHeight(elevator, ElevatorConstants.L3_HEIGHT),
      new GoToAngleWrist(wrist, IntakeConstants.POSITION_ANGLE_WRIST_L3)
    ));

    // Go To L4
    operatorControllerXbox.b().whileTrue(new ParallelCommandGroup(
      new GoToHeight(elevator, ElevatorConstants.L4_HEIGHT),
      new GoToAngleWrist(wrist, IntakeConstants.POSITION_ANGLE_WRIST_L4)
    ));
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
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(
      SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> driverXbox.getLeftY() * -3,
    () -> driverXbox.getLeftX() * -3)
          .withControllerRotationAxis(() -> driverXbox.getRightX() * -2)
      .deadband(0.2)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true));

    Command driveFieldOrientedAnglularSlowVelocity = drivebase.driveFieldOriented(
        SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -0.5,
      () -> driverXbox.getLeftX() * -0.5)
            .withControllerRotationAxis(() -> driverXbox.getRightX() * -0.5)
        .deadband(0.2)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true));

    Command driveFieldOrientedAnglularCollect = drivebase.driveFieldOriented(
      SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> driverXbox.getLeftY() * -1,
    () -> driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(() -> driverXbox.getRightX() * -0.75)
      .deadband(0.2)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true));

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    
    driverXbox.leftTrigger(0.1).whileTrue(driveFieldOrientedAnglularSlowVelocity);
    
    driverXbox.rightTrigger(0.1).whileTrue(driveFieldOrientedAnglularCollect);

  }

  public static CommandXboxController getDriverXbox() {
    return driverXbox;
  }

  public static CommandXboxController getOperatorXbox() {
    return operatorControllerXbox;
  }

  public static SwerveSubsystem getSwerveSubsystem() {
    return drivebase;
  }

  private void configureNamedCommand() 
  {

    NamedCommands.registerCommand(
     "Collect Coral",
     new CollectCoral(intakeCoral, wrist)
    );

    NamedCommands.registerCommand(
     "Score L2",
     new ScoreL2(elevator, wrist, intakeCoral)
    );

    NamedCommands.registerCommand(
     "Score L3",
     new ScoreL3(elevator, wrist, intakeCoral)
    );

    NamedCommands.registerCommand(
     "Score L4",
     new ScoreL4(elevator, wrist, intakeCoral)
    );

  }

  public void configurePathChooser() {
    SmartDashboard.putData("Autonomous Chooses", pathChooser);
    pathChooser.setDefaultOption("Stay", "Stay");
    pathChooser.addOption("Go Out Zone", "Go Out Zone");
    pathChooser.addOption("Middle Right Coral", "Middle Right Coral");
    pathChooser.addOption("Middle Left Coral", "Middle Left Coral");
    pathChooser.addOption("Left Coral", "Left Coral");
    pathChooser.addOption("Right Coral", "Right Coral");
  }

  public Command getAutonomousCommand()
  {
    return drivebase.getAutonomousCommand(pathChooser.getSelected());
  }
}
