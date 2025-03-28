// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoDriveToSrc;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algae.pivot.Pivot;
import frc.robot.subsystems.algae.pivot.PivotIO.PivotIOInputs.PivotPosition;
import frc.robot.subsystems.algae.pivot.PivotIOTalonFX;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.endeffector.EndEffectorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.photon.Vision;
import frc.robot.subsystems.photon.VisionIOPhotonVision;
import frc.robot.subsystems.photon.VisionIOPhotonVisionSim;
import java.awt.Color;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final EndEffector endEffector;
  private final Elevator elevator;
  private final Vision photon;
  // private final VisionProcessor visionProcessor;
  private final Climber climb;
  private final Pivot pivot;
  private final Led led;
  private PIDController intakeController = new PIDController(0.1, 0.0, 0.1);

  double pos = 0.0; // REMOVE

  // Driver Controller
  private final UsbCamera elavatorCamera;

  private final CommandXboxController driverController = new CommandXboxController(0);
  // Operator Controller
  private final CommandXboxController operatorController = new CommandXboxController(1);
  // Operator Button Box
  private final CommandGenericHID operatorButtonBox = new CommandGenericHID(2);

  // Triggers
  private final Trigger coralFound;
  private final Trigger coralIn;
  private final Trigger atSetpoint;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedDashboardChooser<DriverStation.Alliance> alliance;
  private final SendableChooser<DriverStation.Alliance> allianceChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        photon =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision("Starboard", CameraConstants.starboardPose),
                new VisionIOPhotonVision("Bow", CameraConstants.bowPose),
                new VisionIOPhotonVision("Stern", CameraConstants.sternPose));
        endEffector =
            new EndEffector(new EndEffectorIOTalonFX(MotorIDConstants.END_EFFECTOR_MOTOR_ID));
        intake = new Intake(new IntakeIOTalonFX(0, 1));
        led = new Led();
        elevator =
            new Elevator(
                new ElevatorIOTalonFX(
                    MotorIDConstants.ELEVATOR_MOTOR_ID1,
                    MotorIDConstants.ELEVATOR_MOTOR_ID2,
                    MotorIDConstants.ELEVATOR_ENCODER_ID,
                    4));
        coralFound = new Trigger(() -> intake.isCoralIn());
        coralIn = new Trigger(() -> intake.isCoralSet());
        atSetpoint = new Trigger(() -> AutoDrive.atSetpoint == true);
        climb =
            new Climber(
                new ClimberIOTalonFX(
                    MotorIDConstants.CLIMBER_MOTOR_ID1, MotorIDConstants.CLIMBER_MOTOR_ID2));
        pivot = new Pivot(new PivotIOTalonFX(MotorIDConstants.PIVOT_MOTOR_ID));
        elavatorCamera = CameraServer.startAutomaticCapture();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        photon =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim("Bow", CameraConstants.starboardPose));
        // new VisionIOPhotonVisionSim("Bow", CameraConstants.bowPose));
        // climber = new Climber(1);
        endEffector = new EndEffector(new EndEffectorIOSim());
        intake = new Intake(new IntakeIOSim());
        led = new Led();
        elevator = new Elevator(new ElevatorIOSim());
        climb =
            new Climber(
                new ClimberIOTalonFX(
                    MotorIDConstants.CLIMBER_MOTOR_ID1, MotorIDConstants.CLIMBER_MOTOR_ID2));
        coralFound = new Trigger(() -> intake.isCoralIn());
        coralIn = new Trigger(() -> intake.isCoralSet());
        atSetpoint = new Trigger(() -> AutoDrive.atSetpoint == true);
        pivot = new Pivot(new PivotIOTalonFX(MotorIDConstants.PIVOT_MOTOR_ID));

        elavatorCamera = CameraServer.startAutomaticCapture();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        photon =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision("Bow", CameraConstants.starboardPose));
        //    new VisionIOPhotonVision("Bow", CameraConstants.bowPose));
        // climber = new Climber(1);
        endEffector =
            new EndEffector(new EndEffectorIOTalonFX(MotorIDConstants.END_EFFECTOR_MOTOR_ID));
        intake = new Intake(new IntakeIOTalonFX(1, 0));
        led = new Led();
        elevator =
            new Elevator(
                new ElevatorIOTalonFX(
                    MotorIDConstants.ELEVATOR_MOTOR_ID1,
                    MotorIDConstants.ELEVATOR_MOTOR_ID2,
                    MotorIDConstants.ELEVATOR_ENCODER_ID,
                    4));
        coralFound = new Trigger(() -> intake.isCoralIn());
        coralIn = new Trigger(() -> intake.isCoralSet());
        atSetpoint = new Trigger(() -> AutoDrive.atSetpoint == true);
        climb = new Climber(new ClimberIO() {});
        pivot = new Pivot(new PivotIOTalonFX(MotorIDConstants.PIVOT_MOTOR_ID));

        elavatorCamera = CameraServer.startAutomaticCapture();
        break;
    }
    allianceChooser = new SendableChooser<DriverStation.Alliance>();
    allianceChooser.addOption("Red", DriverStation.Alliance.Red);
    allianceChooser.addOption("Blue", DriverStation.Alliance.Blue);

    // allianceChooser.setDefaultOption("Blue", DriverStation.Alliance.Blue);

    alliance = new LoggedDashboardChooser<>("Alliance", allianceChooser);

    // Named commands for pathplanner autos
    NamedCommands.registerCommand(
        "IntakeCoral",
        endEffector
            .runEffectorAuto(2)
            .alongWith(elevator.executePreset(ElevatorState.Default))
            .until(intake::isCoralSet)
            .andThen(endEffector::stop));
    NamedCommands.registerCommand("shootCoral", endEffector.runEffector(5).withTimeout(0.4));
    NamedCommands.registerCommand(
        "TestCommand", Commands.run(() -> System.out.println("TestCommand Works")));
    NamedCommands.registerCommand("DefaultPosition", elevator.executePreset(ElevatorState.Default));
    NamedCommands.registerCommand(
        "L2Position",
        elevator.executePreset(ElevatorState.CoralL2).withTimeout(0.5).unless(coralFound));
    NamedCommands.registerCommand(
        "L3Position",
        elevator.executePreset(ElevatorState.CoralL3).withTimeout(0.7).unless(coralFound));
    NamedCommands.registerCommand(
        "L4Position",
        elevator.executePreset(ElevatorState.CoralL4).withTimeout(1.1).unless(coralFound));
    NamedCommands.registerCommand(
        "L4PositionL2",
        elevator.executePreset(ElevatorState.CoralL4).withTimeout(0.7).unless(coralFound));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    elavatorCamera.setResolution(640, 480);
    elavatorCamera.setFPS(24);

    // Configure the button bindings
    configureButtonBindings();

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Rotation2d()));
    // Reset gyro to 0° when B button is pressed
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                new Rotation2d())), // HI THis is Raunak :)
                    drive) // Hola Raunak :0
                .ignoringDisable(true));

    driverController
        .y()
        .onTrue(drive.pathfindToPose(Constants.FieldConstants.SourceL, 0.0, alliance.get()));
    driverController
        .a()
        .onTrue(drive.pathfindToPose(Constants.FieldConstants.bargeMid, 0.0, alliance.get()));

    // driverController.y().onTrue(drive.pathfindToPoseFlipped(Constants.FieldConstants.REEF_D,
    // 0.0));
    driverController.leftBumper().onTrue(pivot.executePreset(PivotPosition.Dealgify));
    driverController.rightBumper().onTrue(pivot.executePreset(PivotPosition.Default));
    driverController.rightTrigger().onTrue(pivot.executePreset(PivotPosition.Dealgify2));

    driverController.povUp().whileTrue(climb.setSpeed(10)).whileFalse(climb.setSpeed(0));
    driverController.povDown().whileTrue(climb.setSpeed(-10)).whileFalse(climb.setSpeed(0));
    // Slow mode
    driverController
        .leftTrigger()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY() * Constants.SLOW_MODE_CONSTANT,
                () -> -driverController.getLeftX() * Constants.SLOW_MODE_CONSTANT,
                () -> -driverController.getRightX() * Constants.SLOW_MODE_CONSTANT));

    operatorController.povDown().onTrue(elevator.executePreset(ElevatorState.Default));
    operatorController.povLeft().onTrue(elevator.executePreset(ElevatorState.CoralL2));
    operatorController.povRight().onTrue(elevator.executePreset(ElevatorState.CoralL3));
    operatorController.povUp().onTrue(elevator.executePreset(ElevatorState.CoralL4));

    // operatorController
    // .leftBumper()
    // .whileTrue(endEffector.runEffector(0.15).until(coralFound));
    operatorController.rightBumper().whileTrue(endEffector.runEffector(4));
    operatorController.leftTrigger().whileTrue(endEffector.runEffectorReverse(6));

    // Add the new functionality for the operator controller A button
    operatorController.a().onTrue(new AutoDriveToSrc(drive));
    // Button Box
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_AL.getButtonID())
        .onTrue(new AutoDrive(Constants.FieldConstants.REEF_AL, drive));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_BL.getButtonID())
        .onTrue(new AutoDrive(Constants.FieldConstants.REEF_BL, drive));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_CL.getButtonID())
        .onTrue(new AutoDrive(Constants.FieldConstants.REEF_CL, drive));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_DL.getButtonID())
        .onTrue(new AutoDrive(Constants.FieldConstants.REEF_DL, drive));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_EL.getButtonID())
        .onTrue(new AutoDrive(Constants.FieldConstants.REEF_EL, drive));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_FL.getButtonID())
        .onTrue(new AutoDrive(Constants.FieldConstants.REEF_FL, drive));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_AR.getButtonID())
        .onTrue(new AutoDrive(Constants.FieldConstants.REEF_AR, drive));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_BR.getButtonID())
        .onTrue(new AutoDrive(Constants.FieldConstants.REEF_BR, drive));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_CR.getButtonID())
        .onTrue(new AutoDrive(Constants.FieldConstants.REEF_CR, drive));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_DR.getButtonID())
        .onTrue(new AutoDrive(Constants.FieldConstants.REEF_DR, drive));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_ER.getButtonID())
        .onTrue(new AutoDrive(Constants.FieldConstants.REEF_ER, drive));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_FR.getButtonID())
        .onTrue(new AutoDrive(Constants.FieldConstants.REEF_FR, drive));

    // operatorButtonBox
    //     .button(Constants.ButtonBoxIds.REEF_AL.getButtonID())
    //     .onTrue(drive.pathfindToPose(Constants.FieldConstants.REEF_AL, 0, alliance.get()));
    // operatorButtonBox
    //     .button(Constants.ButtonBoxIds.REEF_BL.getButtonID())
    //     .onTrue(
    //         drive.pathfindToPose(Constants.FieldConstants.REEF_BL, 0,
    // DriverStation.Alliance.Red));
    // operatorButtonBox
    //     .button(Constants.ButtonBoxIds.REEF_CL.getButtonID())
    //     .onTrue(drive.pathfindToPose(Constants.FieldConstants.REEF_CL, 0, alliance.get()));
    // operatorButtonBox
    //     .button(Constants.ButtonBoxIds.REEF_DL.getButtonID())
    //     .onTrue(drive.pathfindToPose(Constants.FieldConstants.REEF_DL, 0, alliance.get()));
    // operatorButtonBox
    //     .button(Constants.ButtonBoxIds.REEF_EL.getButtonID())
    //     .onTrue(drive.pathfindToPose(Constants.FieldConstants.REEF_EL, 0, alliance.get()));
    // operatorButtonBox
    //     .button(Constants.ButtonBoxIds.REEF_FL.getButtonID())
    //     .onTrue(drive.pathfindToPose(Constants.FieldConstants.REEF_FL, 0, alliance.get()));
    // operatorButtonBox
    //     .button(Constants.ButtonBoxIds.REEF_AR.getButtonID())
    //     .onTrue(drive.pathfindToPose(Constants.FieldConstants.REEF_AR, 0, alliance.get()));
    // operatorButtonBox
    //     .button(Constants.ButtonBoxIds.REEF_BR.getButtonID())
    //     .onTrue(drive.pathfindToPose(Constants.FieldConstants.REEF_BR, 0, alliance.get()));
    // operatorButtonBox
    //     .button(Constants.ButtonBoxIds.REEF_CR.getButtonID())
    //     .onTrue(drive.pathfindToPose(Constants.FieldConstants.REEF_CR, 0, alliance.get()));
    // operatorButtonBox
    //     .button(Constants.ButtonBoxIds.REEF_DR.getButtonID())
    //     .onTrue(drive.pathfindToPose(Constants.FieldConstants.REEF_DR, 0, alliance.get()));
    // operatorButtonBox
    //     .button(Constants.ButtonBoxIds.REEF_ER.getButtonID())
    //     .onTrue(drive.pathfindToPose(Constants.FieldConstants.REEF_ER, 0, alliance.get()));
    // operatorButtonBox
    //     .button(Constants.ButtonBoxIds.REEF_FR.getButtonID())
    //     .onTrue(drive.pathfindToPose(Constants.FieldConstants.REEF_FR, 0, alliance.get()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.ELEVATOR_L1.getButtonID())
        .onTrue(
            elevator
                .executePreset(ElevatorState.CoralL1)
                .withTimeout(0.2)
                .andThen(endEffector.runEffectorAutoCommand())
                .andThen(elevator.executePreset(ElevatorState.Default)));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.ELEVATOR_L2.getButtonID())
        .onTrue(
            elevator
                .executePreset(ElevatorState.CoralL2)
                .withTimeout(0.6)
                .andThen(endEffector.runEffectorAutoCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.5))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.ELEVATOR_L3.getButtonID())
        .onTrue(
            elevator
                .executePreset(ElevatorState.CoralL3)
                .withTimeout(0.8)
                .andThen(
                    endEffector
                        .runEffectorAutoCommand()
                        .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                        .unless(() -> intake.isCoralIn())));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.ELEVATOR_L4.getButtonID())
        .onTrue(
            elevator
                .executePreset(ElevatorState.CoralL4)
                .withTimeout(0.75)
                .andThen(endEffector.runEffectorAutoCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.ABORT.getButtonID())
        .onTrue(
            Commands.run(() -> endEffector.stop(), endEffector)
                .andThen(() -> intake.stop(), intake)
                .andThen(() -> elevator.getCurrentCommand().cancel(), elevator)
                // .andThen(() -> pivot.getCurrentCommand().cancel(), pivot)
                // .andThen(() -> algae.getCurrentCommand().cancel(), algae)
                .andThen(() -> drive.getCurrentCommand().cancel(), drive));

    // Trigger Commands
    coralFound
        .and(() -> DriverStation.isTeleopEnabled())
        .whileTrue(
            Commands.run(
                    () -> {
                      // Calculate the intake speed using the PID controller to prevent overshooting
                      double intakeSpeed = intakeController.calculate(intake.getRPM(), 2800);
                      endEffector.runEffector(intakeSpeed).schedule();
                    })
                .alongWith(led.setColor(Color.RED)));
    coralIn.and(() -> DriverStation.isTeleopEnabled()).whileTrue(led.setColor(Color.GREEN));
    atSetpoint.and(() -> DriverStation.isTeleopEnabled()).whileTrue(led.setColor(Color.PINK));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void onDisabled() {
    drive.setModulesCoast();
  }

  public void onEnabled() {
    drive.setModulesBrake();
  }
}
