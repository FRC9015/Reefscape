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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.AlgaeIOTalonFX;
import frc.robot.subsystems.algae.pivot.Pivot;
import frc.robot.subsystems.algae.pivot.PivotIO.PivotIOInputs.PivotPosition;
import frc.robot.subsystems.algae.pivot.PivotIOTalonFX;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs.ClimberPositions;
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
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
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
  private final Climber climb;
  private final Pivot pivot;
  private final Algae algae;
  private final Led led;

  // Driver Controller
  //   private final UsbCamera elavatorCamera;

  private final CommandXboxController driverController = new CommandXboxController(0);
  // Operator Controller
  private final CommandXboxController operatorController = new CommandXboxController(1);
  // Operator Button Box
  private final CommandGenericHID operatorButtonBox = new CommandGenericHID(2);

  // Triggers
  private final Trigger coralFound;
  private final Trigger coralIn;
  private final Trigger canRangeLeft;
  private final Trigger canRangeMiddle;
  private final Trigger canRangeRight;
  private final Trigger inPosition;
  private final Trigger elevatorToggle;
  private final Trigger groundStall;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Command> bargePos;
  private final LoggedDashboardChooser<DriverStation.Alliance> alliance;
  private final SendableChooser<DriverStation.Alliance> allianceChooser;
  private final SendableChooser<Command> bargePosChooser;
  private final SendableChooser<Integer> bargeModeChooser;
  private final LoggedDashboardChooser<Integer> bargeMode;

  // Elevator state
  private ElevatorState selectedElevatorState = ElevatorState.Default;

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
        intake = new Intake(new IntakeIOTalonFX(9, 8, 46, 45, 44));
        elevator =
            new Elevator(
                new ElevatorIOTalonFX(
                    MotorIDConstants.ELEVATOR_MOTOR_ID1,
                    MotorIDConstants.ELEVATOR_MOTOR_ID2,
                    MotorIDConstants.ELEVATOR_ENCODER_ID,
                    4));
        pivot = new Pivot(new PivotIOTalonFX(MotorIDConstants.PIVOT_MOTOR_ID));
        algae = new Algae(new AlgaeIOTalonFX(MotorIDConstants.GROUND_CORAL_MOTOR_ID));

        coralFound = new Trigger(() -> intake.isCoralIn());
        coralIn = new Trigger(() -> intake.isCoralSet());
        canRangeLeft = new Trigger(() -> intake.canRangeLeftDetected());
        canRangeMiddle = new Trigger(() -> intake.canRangeMiddleDetected());
        canRangeRight = new Trigger(() -> intake.canRangeRightDetected());
        inPosition = new Trigger(() -> intake.inPosition());
        elevatorToggle = new Trigger(() -> elevator.getToggle());
        groundStall = new Trigger(() -> algae.isStalled());
        climb =
            new Climber(
                7,
                new ClimberIOTalonFX(
                    MotorIDConstants.CLIMBER_TOP_MOTOR_ID,
                    MotorIDConstants.CLIMBER_MOTOR_ID1,
                    MotorIDConstants.CLIMBER_MOTOR_ID2,
                    MotorIDConstants.SERVO_CHANNE1,
                    MotorIDConstants.SERVO_CHANNE2));
        // climb =
        //     new Climber(
        //         new ClimberIOTalonFX(
        //             MotorIDConstants.CLIMBER_MOTOR_ID1, MotorIDConstants.CLIMBER_MOTOR_ID2));
        // pivot = new Pivot(new PivotIOTalonFX(MotorIDConstants.PIVOT_MOTOR_ID));
        //  elavatorCamera = CameraServer.startAutomaticCapture();
        led = new Led(Constants.LEDConstants.CANDLE_ID);
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
        elevator = new Elevator(new ElevatorIOSim());
        pivot = new Pivot(new PivotIOTalonFX(MotorIDConstants.PIVOT_MOTOR_ID));
        algae = new Algae(new AlgaeIOTalonFX(MotorIDConstants.GROUND_CORAL_MOTOR_ID));
        // climb =
        //     new Climber(
        //         new ClimberIOTalonFX(
        //             MotorIDConstants.CLIMBER_MOTOR_ID1, MotorIDConstants.CLIMBER_MOTOR_ID2));
        coralFound = new Trigger(() -> intake.isCoralIn());
        coralIn = new Trigger(() -> intake.isCoralSet());
        canRangeLeft = new Trigger(() -> intake.canRangeLeftDetected());
        canRangeMiddle = new Trigger(() -> intake.canRangeMiddleDetected());
        canRangeRight = new Trigger(() -> intake.canRangeRightDetected());
        inPosition = new Trigger(() -> intake.inPosition());
        elevatorToggle = new Trigger(() -> elevator.getToggle());
        groundStall = new Trigger(() -> algae.isStalled());
        // pivot = new Pivot(new PivotIOTalonFX(MotorIDConstants.PIVOT_MOTOR_ID));
        climb =
            new Climber(
                7,
                new ClimberIOTalonFX(
                    MotorIDConstants.CLIMBER_TOP_MOTOR_ID,
                    MotorIDConstants.CLIMBER_MOTOR_ID1,
                    MotorIDConstants.CLIMBER_MOTOR_ID2,
                    MotorIDConstants.SERVO_CHANNE1,
                    MotorIDConstants.SERVO_CHANNE2));

        //  elavatorCamera = CameraServer.startAutomaticCapture();
        led = new Led(Constants.LEDConstants.CANDLE_ID);
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
        intake = new Intake(new IntakeIOTalonFX(1, 0, 44, 45, 46));
        elevator =
            new Elevator(
                new ElevatorIOTalonFX(
                    MotorIDConstants.ELEVATOR_MOTOR_ID1,
                    MotorIDConstants.ELEVATOR_MOTOR_ID2,
                    MotorIDConstants.ELEVATOR_ENCODER_ID,
                    4));
        pivot = new Pivot(new PivotIOTalonFX(MotorIDConstants.PIVOT_MOTOR_ID));
        algae = new Algae(new AlgaeIOTalonFX(MotorIDConstants.GROUND_CORAL_MOTOR_ID));
        coralFound = new Trigger(() -> intake.isCoralIn());
        coralIn = new Trigger(() -> intake.isCoralSet());
        canRangeLeft = new Trigger(() -> intake.canRangeLeftDetected());
        canRangeMiddle = new Trigger(() -> intake.canRangeMiddleDetected());
        canRangeRight = new Trigger(() -> intake.canRangeRightDetected());
        inPosition = new Trigger(() -> intake.inPosition());
        elevatorToggle = new Trigger(() -> elevator.getToggle());
        groundStall = new Trigger(() -> algae.isStalled());

        climb =
            new Climber(
                7,
                new ClimberIOTalonFX(
                    MotorIDConstants.CLIMBER_TOP_MOTOR_ID,
                    MotorIDConstants.CLIMBER_MOTOR_ID1,
                    MotorIDConstants.CLIMBER_MOTOR_ID2,
                    MotorIDConstants.SERVO_CHANNE1,
                    MotorIDConstants.SERVO_CHANNE2));

        //  climb = new Climber(new ClimberIO() {});
        // pivot = new Pivot(new PivotIOTalonFX(MotorIDConstants.PIVOT_MOTOR_ID));

        //     elavatorCamera = CameraServer.startAutomaticCapture();
        led = new Led(Constants.LEDConstants.CANDLE_ID);
        break;
    }
    allianceChooser = new SendableChooser<DriverStation.Alliance>();
    bargePosChooser = new SendableChooser<Command>();
    bargeModeChooser = new SendableChooser<Integer>();
    // allianceChooser.setDefaultOption("Blue", DriverStation.Alliance.Blue);

    // allianceChooser.addOption("Red", DriverStation.Alliance.Red);

    alliance = new LoggedDashboardChooser<>("Alliance", allianceChooser);
    alliance.addOption("Red", DriverStation.Alliance.Red);
    alliance.addOption("Blue", DriverStation.Alliance.Blue);

    // Dropdown code for AutoAlign to Barge Positions
    bargePos = new LoggedDashboardChooser<>("Barge Position", bargePosChooser);
    bargePos.addOption(
        "Barge Left",
        new AutoDrive(() -> Constants.FieldConstants.RedBargeLeft, drive, () -> alliance.get()));
    bargePos.addOption(
        "Barge Middle",
        new AutoDrive(() -> Constants.FieldConstants.RedBargeMiddle, drive, () -> alliance.get()));
    bargePos.addOption(
        "Barge Right",
        new AutoDrive(() -> Constants.FieldConstants.RedBargeMiddle, drive, () -> alliance.get()));

    bargeMode = new LoggedDashboardChooser<>("Barge Auto Drive?", bargeModeChooser);
    bargeMode.addOption("Yes", 1);
    bargeMode.addOption("No", 0);
    // Named commands for pathplanner autos
    NamedCommands.registerCommand(
        "IntakeCoral",
        endEffector
            .runEffectorAuto(2)
            .alongWith(elevator.executePreset(ElevatorState.Default))
            .until(intake::isCoralSet)
            .andThen(endEffector::stop));
    NamedCommands.registerCommand(
        "IntakeCoralDrive",
        endEffector
            .runEffectorAuto(4)
            .alongWith(elevator.executePreset(ElevatorState.Default))
            .until(intake::inRamp));
    NamedCommands.registerCommand("shootCoral", endEffector.runEffector(5.5).withTimeout(0.3));
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
    NamedCommands.registerCommand(
        "AL", new AutoDrive(() -> Constants.FieldConstants.REEF_AL, drive, () -> alliance.get()));
    NamedCommands.registerCommand(
        "CR", new AutoDrive(() -> Constants.FieldConstants.REEF_CR, drive, () -> alliance.get()));
    NamedCommands.registerCommand(
        "BR", new AutoDrive(() -> Constants.FieldConstants.REEF_BR, drive, () -> alliance.get()));
    NamedCommands.registerCommand(
        "BL", new AutoDrive(() -> Constants.FieldConstants.REEF_BL, drive, () -> alliance.get()));
    NamedCommands.registerCommand(
        "EL", new AutoDrive(() -> Constants.FieldConstants.REEF_EL, drive, () -> alliance.get()));
    NamedCommands.registerCommand(
        "FR", new AutoDrive(() -> Constants.FieldConstants.REEF_FR, drive, () -> alliance.get()));
    NamedCommands.registerCommand(
        "FL", new AutoDrive(() -> Constants.FieldConstants.REEF_FL, drive, () -> alliance.get()));

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

    // elavatorCamera.setResolution(640, 480);
    // elavatorCamera.setFPS(24);

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
        .or(coralIn)
        .whileTrue(
            DriveCommands.joystickDriveFacingPose(
                drive,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> Constants.FieldConstants.REEF_CENTER,
                () -> alliance.get()));
    // Reset gyro to 0° when B button is pressed

    driverController.y().whileTrue(algae.setSpeed(10)).whileFalse(algae.setSpeed(0));
    driverController.b().whileTrue(algae.setSpeed(-10)).whileFalse(algae.setSpeed(0));
    driverController.povRight().onTrue(pivot.executePreset(PivotPosition.Score));
    driverController
        .povLeft()
        .onTrue(
            pivot
                .executePreset(PivotPosition.Down)
                .andThen(
                    algae
                        .setSpeed(-5)
                        .until(groundStall)
                        .andThen(pivot.executePreset(PivotPosition.Default))));
    // driverController
    //     .a()
    //     .onTrue(drive.pathfindToPose(Constants.FieldConstants.bargeMid, 0.0, alliance.get()));

    // driverController.y().onTrue(drive.pathfindToPoseFlipped(Constants.FieldConstants.REEF_D,
    // 0.0));
    driverController.rightBumper().onTrue(climb.retractCommand2());
    driverController.povDown().whileTrue(climb.down());
    driverController.povUp().whileTrue(climb.up());
    driverController.rightTrigger().whileTrue(climb.topMotor());

    // driverController.povUp().whileTrue(climb.setSpeed(10)).whileFalse(climb.setSpeed(0));
    // driverController.povDown().whileTrue(climb.setSpeed(-10)).whileFalse(climb.setSpeed(0));
    // Slow modec
    driverController
        .leftTrigger()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY() * Constants.SLOW_MODE_CONSTANT,
                () -> -driverController.getLeftX() * Constants.SLOW_MODE_CONSTANT,
                () -> -driverController.getRightX() * Constants.SLOW_MODE_CONSTANT));
    // driverController
    //     .povLeft()
    //     .onTrue(new AutoDrive(() -> Constants.FieldConstants.SourceL, drive, () ->
    // alliance.get()));
    // driverController
    //     .povRight()
    //     .onTrue(new AutoDrive(() -> Constants.FieldConstants.SourceR, drive, () ->
    // alliance.get()));

    driverController.povLeft().whileTrue(pivot.pivotUp(3));
    driverController.povRight().whileTrue(pivot.pivotDown(3));

    operatorController.povDown().onTrue(elevator.executePreset(ElevatorState.Default));
    operatorController.povLeft().onTrue(elevator.executePreset(ElevatorState.CoralL2));
    operatorController.povRight().onTrue(elevator.executePreset(ElevatorState.CoralL3));
    operatorController.povUp().onTrue(elevator.executePreset(ElevatorState.CoralL4));
    operatorController.b().onTrue(climb.retractCommand2());
    operatorController
        .y()
        .onTrue(
            climb
                .executePreset(ClimberPositions.Up)
                .withTimeout(2.5)
                .andThen(elevator.executePreset(ElevatorState.CoralL3).withTimeout(0.75))
                .andThen(climb.retractCommand2().withTimeout(2))
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.7)));
    // operatorController.x().onTrue(climb.executePreset(ClimberPositions.Default));
    // operatorController
    // .leftBumper()
    // .whileTrue(endEffector.runEffector(0.15).until(coralFound));
    // operatorController.x().onTrue(new AutoDrive(() -> Constants.FieldConstants.SourceL, drive, ()
    // -> alliance.get()));
    driverController
        .x()
        .onTrue(new AutoDrive(() -> Constants.FieldConstants.SourceL, drive, () -> alliance.get()));

    operatorController.rightBumper().whileTrue(endEffector.runEffector(4));
    operatorController.leftTrigger().whileTrue(endEffector.runEffectorReverse(6));
    operatorController.a().onTrue(climb.extendCommand2());

    // Button Box
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_AL.getButtonID())
        .onTrue(
            new AutoDrive(() -> Constants.FieldConstants.REEF_AL, drive, () -> alliance.get())
                .raceWith(autoElevatorCommand(() -> selectedElevatorState))
                .andThen(autoShootCoralCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_AL.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(new AutoDrive(() -> Constants.FieldConstants.REEF_AL, drive, () -> alliance.get()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_BL.getButtonID())
        .onTrue(
            new AutoDrive(() -> Constants.FieldConstants.REEF_BL, drive, () -> alliance.get())
                .raceWith(autoElevatorCommand(() -> selectedElevatorState))
                .andThen(autoShootCoralCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_BL.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(new AutoDrive(() -> Constants.FieldConstants.REEF_BL, drive, () -> alliance.get()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_CL.getButtonID())
        .onTrue(
            new AutoDrive(() -> Constants.FieldConstants.REEF_CL, drive, () -> alliance.get())
                .raceWith(autoElevatorCommand(() -> selectedElevatorState))
                .andThen(autoShootCoralCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_CL.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(new AutoDrive(() -> Constants.FieldConstants.REEF_CL, drive, () -> alliance.get()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_DL.getButtonID())
        .onTrue(
            new AutoDrive(() -> Constants.FieldConstants.REEF_DL, drive, () -> alliance.get())
                .raceWith(autoElevatorCommand(() -> selectedElevatorState))
                .andThen(autoShootCoralCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_DL.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(new AutoDrive(() -> Constants.FieldConstants.REEF_DL, drive, () -> alliance.get()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_EL.getButtonID())
        .onTrue(
            new AutoDrive(() -> Constants.FieldConstants.REEF_EL, drive, () -> alliance.get())
                .raceWith(autoElevatorCommand(() -> selectedElevatorState))
                .andThen(autoShootCoralCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_EL.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(new AutoDrive(() -> Constants.FieldConstants.REEF_EL, drive, () -> alliance.get()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_FL.getButtonID())
        .onTrue(
            new AutoDrive(() -> Constants.FieldConstants.REEF_FL, drive, () -> alliance.get())
                .raceWith(autoElevatorCommand(() -> selectedElevatorState))
                .andThen(autoShootCoralCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_FL.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(new AutoDrive(() -> Constants.FieldConstants.REEF_FL, drive, () -> alliance.get()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_AR.getButtonID())
        .onTrue(
            new AutoDrive(() -> Constants.FieldConstants.REEF_AR, drive, () -> alliance.get())
                .raceWith(autoElevatorCommand(() -> selectedElevatorState))
                .andThen(autoShootCoralCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_AR.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(new AutoDrive(() -> Constants.FieldConstants.REEF_AR, drive, () -> alliance.get()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_BR.getButtonID())
        .onTrue(
            new AutoDrive(() -> Constants.FieldConstants.REEF_BR, drive, () -> alliance.get())
                .raceWith(autoElevatorCommand(() -> selectedElevatorState))
                .andThen(autoShootCoralCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_BR.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(new AutoDrive(() -> Constants.FieldConstants.REEF_BR, drive, () -> alliance.get()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_CR.getButtonID())
        .onTrue(
            new AutoDrive(() -> Constants.FieldConstants.REEF_CR, drive, () -> alliance.get())
                .raceWith(autoElevatorCommand(() -> selectedElevatorState))
                .andThen(autoShootCoralCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_CR.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(new AutoDrive(() -> Constants.FieldConstants.REEF_CR, drive, () -> alliance.get()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_DR.getButtonID())
        .onTrue(
            new AutoDrive(() -> Constants.FieldConstants.REEF_DR, drive, () -> alliance.get())
                .raceWith(autoElevatorCommand(() -> selectedElevatorState))
                .andThen(autoShootCoralCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_DR.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(new AutoDrive(() -> Constants.FieldConstants.REEF_DR, drive, () -> alliance.get()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_ER.getButtonID())
        .onTrue(
            new AutoDrive(() -> Constants.FieldConstants.REEF_ER, drive, () -> alliance.get())
                .raceWith(autoElevatorCommand(() -> selectedElevatorState))
                .andThen(autoShootCoralCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_ER.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(new AutoDrive(() -> Constants.FieldConstants.REEF_ER, drive, () -> alliance.get()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_FR.getButtonID())
        .onTrue(
            new AutoDrive(() -> Constants.FieldConstants.REEF_FR, drive, () -> alliance.get())
                .raceWith(autoElevatorCommand(() -> selectedElevatorState))
                .andThen(autoShootCoralCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.REEF_FR.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(new AutoDrive(() -> Constants.FieldConstants.REEF_FR, drive, () -> alliance.get()));
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
        .and(() -> elevator.getToggle())
        .onTrue(
            Commands.runOnce(
                () -> {
                  selectedElevatorState = ElevatorState.CoralL1;
                  Logger.recordOutput("selectedState", selectedElevatorState);
                }));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.ELEVATOR_L1.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(
            elevator
                .executePreset(ElevatorState.CoralL1)
                .withTimeout(0.2)
                .andThen(endEffector.runEffectorAutoCommandL1())
                .andThen(elevator.executePreset(ElevatorState.Default))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.ELEVATOR_L2.getButtonID())
        .and(() -> elevator.getToggle())
        .onTrue(
            Commands.runOnce(
                () -> {
                  selectedElevatorState = ElevatorState.CoralL2;
                  Logger.recordOutput("selectedState", selectedElevatorState);
                }));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.ELEVATOR_L2.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(
            elevator
                .executePreset(ElevatorState.CoralL2)
                .withTimeout(0.6)
                .andThen(endEffector.runEffectorAutoCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.5))
                .unless(() -> intake.isCoralIn()));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.ELEVATOR_L3.getButtonID())
        .and(() -> elevator.getToggle())
        .onTrue(
            Commands.runOnce(
                () -> {
                  selectedElevatorState = ElevatorState.CoralL3;
                  Logger.recordOutput("selectedState", selectedElevatorState);
                }));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.ELEVATOR_L3.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(
            elevator
                .executePreset(ElevatorState.CoralL3)
                .withTimeout(0.75)
                .andThen(endEffector.runEffectorAutoCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                .unless(() -> intake.isCoralIn()));

    operatorButtonBox
        .button(Constants.ButtonBoxIds.ELEVATOR_L4.getButtonID())
        .and(() -> elevator.getToggle())
        .onTrue(
            Commands.runOnce(
                () -> {
                  selectedElevatorState = ElevatorState.CoralL4;
                  Logger.recordOutput("selectedState", selectedElevatorState);
                }));
    operatorButtonBox
        .button(Constants.ButtonBoxIds.ELEVATOR_L4.getButtonID())
        .and(() -> !elevator.getToggle())
        .onTrue(
            elevator
                .executePreset(ElevatorState.CoralL4)
                .withTimeout(0.85)
                .andThen(endEffector.runEffectorAutoCommand())
                .andThen(elevator.executePreset(ElevatorState.Default).withTimeout(0.75))
                .unless(() -> intake.isCoralIn()));

    operatorButtonBox
        .button(Constants.ButtonBoxIds.ABORT.getButtonID())
        .onTrue(
            elevator
                .toggle()
                .alongWith(
                    Commands.runOnce(
                        () -> {
                          selectedElevatorState = ElevatorState.Default;
                          Logger.recordOutput("selectedState", selectedElevatorState);
                        })));
    coralFound.and(() -> DriverStation.isTeleopEnabled()).whileTrue(endEffector.runEffector(2));

    canRangeLeft
        .whileTrue(
            led.setColor(Color.YELLOW)
                .alongWith(new InstantCommand(() -> SmartDashboard.putBoolean("Left", true))))
        .whileFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Left", false)));
    canRangeRight
        .whileTrue(
            led.setColor(Color.RED)
                .alongWith(new InstantCommand(() -> SmartDashboard.putBoolean("Right", true))))
        .whileFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Right", false)));
    canRangeMiddle
        .whileTrue((new InstantCommand(() -> SmartDashboard.putBoolean("Middle", true))))
        .whileFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Middle", false)));
    inPosition
        .whileTrue(
            led.setColor(Color.MAGENTA)
                .alongWith(new InstantCommand(() -> SmartDashboard.putBoolean("Locked", true))))
        .whileFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Locked", false)));
    canRangeLeft.and(canRangeMiddle).whileTrue(led.setColor(Color.YELLOW));
    canRangeRight.and(canRangeMiddle).whileTrue(led.setColor(Color.RED));

    //     (operatorController.a())
    //         .onTrue(
    //             Commands.runOnce(
    //                 () -> {
    //                   final Command bargePosCommand = this.getBargePositionCommand();
    //                   if (bargePosCommand != null) {
    //                     bargePosCommand.schedule();
    //                   }
    //                   bargePosCommand.cancel();
    //               }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  //   public Command getBargePositionCommand() {
  //     return bargePos.get();
  //   }

  public void onDisabled() {
    drive.setModulesCoast();
  }

  public void onEnabled() {
    drive.setModulesBrake();
    // climb.extend2();
  }

  private Command autoElevatorCommand(Supplier<ElevatorState> stateSupplier) {
    return elevator.executePreset(stateSupplier).onlyIf(elevatorToggle);
  }

  private Command autoShootCoralCommand() {
    return endEffector.runEffectorAutoCommand().onlyIf(elevatorToggle);
  }
}
