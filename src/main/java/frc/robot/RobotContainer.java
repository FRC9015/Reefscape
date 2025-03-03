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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.AlgaeIOSim;
import frc.robot.subsystems.algae.AlgaeIOTalonFX;
import frc.robot.subsystems.algae.pivot.Pivot;
import frc.robot.subsystems.algae.pivot.PivotIOSim;
import frc.robot.subsystems.algae.pivot.PivotIOTalonFX;
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
import frc.robot.subsystems.photon.Vision;
import frc.robot.subsystems.photon.VisionIOPhotonVision;
import frc.robot.subsystems.photon.VisionIOPhotonVisionSim;
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
  private final Pivot pivot;
  private final Algae algae;

  private final Vision photon;
  // Driver Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  // Operator Controller
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Triggers
  private final Trigger coralFound;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
                new VisionIOPhotonVision("Bow", CameraConstants.bowPose));
        endEffector =
            new EndEffector(new EndEffectorIOTalonFX(MotorIDConstants.END_EFFECTOR_MOTOR_ID));
        intake = new Intake(new IntakeIOTalonFX(0));
        elevator =
            new Elevator(
                new ElevatorIOTalonFX(
                    MotorIDConstants.ELEVATOR_MOTOR_ID1,
                    MotorIDConstants.ELEVATOR_MOTOR_ID2,
                    MotorIDConstants.ELEVATOR_ENCODER_ID,
                    0));
        pivot = new Pivot(new PivotIOTalonFX(MotorIDConstants.PIVOT_MOTOR_ID));
        algae = new Algae(new AlgaeIOTalonFX(MotorIDConstants.ALGAE_MOTOR_ID));
        coralFound = new Trigger(() -> intake.isCoralDetected());
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
        photon = new Vision(drive::addVisionMeasurement, 
            new VisionIOPhotonVisionSim("Starboard",CameraConstants.starboardPose), 
            new VisionIOPhotonVisionSim("Bow",CameraConstants.bowPose));
        // climber = new Climber(1);
        endEffector = new EndEffector(new EndEffectorIOSim());
        intake = new Intake(new IntakeIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        algae = new Algae(new AlgaeIOSim());
        pivot = new Pivot(new PivotIOSim());
        coralFound = new Trigger(() -> intake.isCoralDetected());
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
                new VisionIOPhotonVision("Starboard", CameraConstants.starboardPose),
                new VisionIOPhotonVision("Bow", CameraConstants.bowPose));
        // climber = new Climber(1);
        endEffector =
            new EndEffector(new EndEffectorIOTalonFX(MotorIDConstants.END_EFFECTOR_MOTOR_ID));
        intake = new Intake(new IntakeIOTalonFX(0));
        elevator =
            new Elevator(
                new ElevatorIOTalonFX(
                    MotorIDConstants.ELEVATOR_MOTOR_ID1,
                    MotorIDConstants.ELEVATOR_MOTOR_ID2,
                    MotorIDConstants.ELEVATOR_ENCODER_ID,
                    0));
        pivot = new Pivot(new PivotIOTalonFX(MotorIDConstants.PIVOT_MOTOR_ID));
        algae = new Algae(new AlgaeIOTalonFX(MotorIDConstants.ALGAE_MOTOR_ID));
        coralFound = new Trigger(() -> intake.isCoralDetected());
        break;
    }

    // Named commands for pathplanner autos
    NamedCommands.registerCommand("IntakeCoral", endEffector.runEffector(0.25));
    NamedCommands.registerCommand("shootCoral", endEffector.runEffectorReverse(0.5).withTimeout(1));
    NamedCommands.registerCommand(
        "TestCommand", Commands.run(() -> System.out.println("TestCommand Works")));

    NamedCommands.registerCommand(
        "DefaultPosition", elevator.executePreset(ElevatorState.Default).withTimeout(1));
    NamedCommands.registerCommand("L2Position", elevator.executePreset(ElevatorState.CoralL2));
    NamedCommands.registerCommand("L3Position", elevator.executePreset(ElevatorState.CoralL3));
    NamedCommands.registerCommand(
        "L4Position", elevator.executePreset(ElevatorState.CoralL4).withTimeout(1));

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

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
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
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driverController.x().onTrue(drive.pathfindToPose(Constants.FieldConstants.REEF_BR, 0.0));
    driverController.y().onTrue(drive.pathfindToPose(Constants.FieldConstants.SourceL, 0.0));
    driverController.a().onTrue(drive.pathfindToPose(Constants.FieldConstants.bargeMid, 0.0));
    // driverController.y().onTrue(drive.pathfindToPoseFlipped(Constants.FieldConstants.REEF_D,
    // 0.0));
    // driverController.leftBumper().whileTrue(pivot.pivotDown(0.25));
    // driverController.rightBumper().whileTrue(pivot.pivotUp(0.25));
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
    //     .leftBumper()
    //     .whileTrue(endEffector.runEffectorReverse(0.15).until(coralFound));
    operatorController.rightBumper().whileTrue(endEffector.runEffectorReverse(0.5));
    operatorController.a().whileTrue(algae.setSpeed(5)).whileFalse(algae.setSpeed(0));
    operatorController.b().whileTrue(algae.setSpeed(-5)).whileFalse(algae.setSpeed(0));
    operatorController.leftTrigger().whileTrue(endEffector.runEffector(0.15));
    operatorController.x().whileTrue(pivot.pivotUp(1)).whileFalse(pivot.pivotUp(0));
    operatorController.y().whileTrue(pivot.pivotUp(-1)).whileFalse(pivot.pivotUp(0));

    coralFound.whileTrue(endEffector.runEffectorReverse(0.10));
    // Pathfind to source
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
