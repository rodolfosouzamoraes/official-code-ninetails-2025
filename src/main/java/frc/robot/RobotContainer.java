// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.auto.AutoScoreL3;
import frc.robot.commands.auto.AutoScoreL4;
import frc.robot.commands.elevator.GoToHeight;
import frc.robot.commands.output.ShooterCollect;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.output.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

import org.dyn4j.geometry.Rotation;

import swervelib.SwerveController;
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
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final Field2d field = new Field2d();

  private final SendableChooser<String> pathChooser = new SendableChooser<>();

  public RobotContainer()
  {
    // Configure the trigger bindings
    
    configureLog();
    //configureBindings();
    operatorControllerBindings();
    configureSwerve();
    configureCommandReef();
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
    // driverControllerBindings();       //-> Alinhar com a AprilTag
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
    operatorDefaultCommand();
    // commandsXboxController();
    commandsHIDController();
  }

  private void operatorDefaultCommand(){
    elevator.setDefaultCommand(new GoToHeight(elevator, 0));
    // elevator.setDefaultCommand(new RunCommand(() -> elevator.controlElevatorJoystick(-10), elevator));
    shooter.setDefaultCommand(shooter.stopShooter());
  }

  private void commandsHIDController() {
    // Coletar Coral
    operatorHID.button(ButtonConstants.COLLECT_CORAL)
      .onTrue(new ShooterCollect(shooter));

    // Atirar Coral
    operatorHID.button(ButtonConstants.SHOOTER_CORAL).whileTrue(shooter.setShooterSpeed(1));

    // Go To L2
    operatorHID.button(ButtonConstants.GO_TO_L2)
      .whileTrue(new GoToHeight(elevator, ElevatorConstants.L2_HEIGHT));

    // Go To L3
    operatorHID.button(ButtonConstants.GO_TO_L3)
      .whileTrue(new GoToHeight(elevator, ElevatorConstants.L3_HEIGHT));

    // Go To L4
    operatorHID.button(ButtonConstants.GO_TO_L4)
      .whileTrue(new GoToHeight(elevator, ElevatorConstants.L4_HEIGHT));

    operatorHID.button(ButtonConstants.ALIGN_LEFT_REEF)
      .whileTrue(lockHeadingWhileDriving(FieldConstants.CORAL_STATION_LEFT))
      .onTrue(new ShooterCollect(shooter));
      
    operatorHID.button(ButtonConstants.ALIGN_RIGHT_REEF)
      .whileTrue(lockHeadingWhileDriving(FieldConstants.CORAL_STATION_RIGHT))
      .onTrue(new ShooterCollect(shooter));

  }

  private void commandsXboxController() { //Controle Xbox para o intake

    // Coletar Coral
    operatorControllerXbox.leftBumper();

    // Atirar Coral
    operatorControllerXbox.leftTrigger();

    // Go To L2
    operatorControllerXbox.x().whileTrue(new ParallelCommandGroup(
      new GoToHeight(elevator, ElevatorConstants.L2_HEIGHT)
    ));

    // Go To Ball L3
    operatorControllerXbox.povRight().whileTrue(
      new GoToHeight(elevator, ElevatorConstants.L3_BALL_HEIGHT));

    // Go To L3
    operatorControllerXbox.a().whileTrue(new ParallelCommandGroup(
      new GoToHeight(elevator, ElevatorConstants.L3_HEIGHT)
    ));

    // Go To L4
    operatorControllerXbox.b().whileTrue(new ParallelCommandGroup(
      new GoToHeight(elevator, ElevatorConstants.L4_HEIGHT)
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
          .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
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

    Command driveFieldOrientedAnglularFastVelocity = drivebase.driveFieldOriented(
        SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -5,
      () -> driverXbox.getLeftX() * -5)
            .withControllerRotationAxis(() -> driverXbox.getRightX() * -3)
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

    driverXbox.rightBumper().whileTrue(driveFieldOrientedAnglularFastVelocity);

    driverXbox.a().whileTrue(lockHeadingWhileDriving(FieldConstants.REEF_BACKWARD));
    driverXbox.y().whileTrue(lockHeadingWhileDriving(FieldConstants.REEF_FOWARD));
    driverXbox.b().whileTrue(lockHeadingWhileDriving(FieldConstants.REEF_LEFT_SIDE));
    driverXbox.x().whileTrue(lockHeadingWhileDriving(FieldConstants.REEF_RIGHT_SIDE));

    driverXbox.leftBumper().and(driverXbox.a()).whileTrue(lockHeadingWhileDriving(FieldConstants.REEF_RIGHT_DOWN_SIDE));
    driverXbox.leftBumper().and(driverXbox.b()).whileTrue(lockHeadingWhileDriving(FieldConstants.REEF_RIGHT_UP_SIDE));
    driverXbox.leftBumper().and(driverXbox.x()).whileTrue(lockHeadingWhileDriving(FieldConstants.REEF_LEFT_DOWN_SIDE));
    driverXbox.leftBumper().and(driverXbox.y()).whileTrue(lockHeadingWhileDriving(FieldConstants.REEF_LEFT_UP_SIDE));

  }

  public Command lockHeadingWhileDriving(Rotation2d targetHeading) {
    SwerveController controller = drivebase.getSwerveDrive().getSwerveController();

    return new RunCommand(() -> {

      ChassisSpeeds	speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        -driverXbox.getLeftY() * 2,
        -driverXbox.getLeftX() * 2,
        controller.headingCalculate(
          drivebase.getHeading().getRadians(),
          targetHeading.getRadians()
        ),
        drivebase.getHeading()
      );

      drivebase.drive(speeds);
    },
    drivebase);
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

  private void configureCommandReef() 
  {

    NamedCommands.registerCommand(
     "AutoCollectCoral",
     new ShooterCollect(shooter)
    );
    
    NamedCommands.registerCommand(
     "AutoScoreL3",
     new AutoScoreL3(elevator, shooter)
    );
    
    NamedCommands.registerCommand(
     "AutoScoreL4",
     new AutoScoreL4(elevator, shooter)
    );

  }

  public void configurePathChooser() {
    SmartDashboard.putData("Autonomous Chooses", pathChooser);
    pathChooser.setDefaultOption("Stay", "Stay Auto");
    pathChooser.addOption("Go Out Auto", "Go Out Auto");
    pathChooser.addOption("Center Auto L3", "Center Auto L3");
    pathChooser.addOption("Center Auto L4", "Center Auto L4");
    pathChooser.addOption("Right Auto L3", "Right Auto L3");
  }

  public Command getAutonomousCommand()
  {
    return drivebase.getAutonomousCommand(pathChooser.getSelected());
  }
}
