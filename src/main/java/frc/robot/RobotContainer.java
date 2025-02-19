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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
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
import frc.robot.subsystems.photon.PhotonInterface;
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
  private final Climber climber;
  private final Intake intake;
  private final EndEffector endEffector;
  private final Elevator elevator;


  private final PhotonInterface photonInterface = new PhotonInterface();
  // Driver Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  // Operator Controller
  private final CommandXboxController operatorController = new CommandXboxController(1);


  // Triggers
  // private final Trigger robotInPosition;

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
                new ModuleIOTalonFX(TunerConstants.BackRight),
                photonInterface);

        climber = new Climber(1);
        endEffector = new EndEffector(new EndEffectorIOTalonFX(2));
        intake = new Intake(new IntakeIOTalonFX(5));
        elevator = new Elevator(new ElevatorIOTalonFX(7, 8, 9, 10));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight),
                photonInterface);

        climber = new Climber(1);
        endEffector = new EndEffector(new EndEffectorIOSim());
        intake = new Intake(new IntakeIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                photonInterface);
        climber = new Climber(1);
        endEffector = new EndEffector(new EndEffectorIOTalonFX(2));
        intake = new Intake(new IntakeIOTalonFX(5));
        elevator = new Elevator(new ElevatorIOTalonFX(7, 8, 9, 10));
        break;
    }
    // subsystem initialization
    elevator = new Elevator(elevatorIO);
    intake = new Intake(intakeIO);
    endEffector = new EndEffector(endEffectorIO);

    // Named commands for pathplanner autos
    NamedCommands.registerCommand(
        "shootCoral", Commands.runOnce(() -> endEffector.setRPM(3000), endEffector));
    NamedCommands.registerCommand(
        "IntakeCoral", Commands.runOnce(() -> endEffector.setRPM(-3000), endEffector));
    NamedCommands.registerCommand(
        "L2Position",
        Commands.runOnce(() -> elevatorIO.setElevatorPosition(ElevatorState.CoralL2), elevator));
    NamedCommands.registerCommand(
        "L3Position",
        Commands.runOnce(() -> elevatorIO.setElevatorPosition(ElevatorState.CoralL3), elevator));
    NamedCommands.registerCommand(
        "L4Position",
        Commands.runOnce(() -> elevatorIO.setElevatorPosition(ElevatorState.CoralL4), elevator));
    NamedCommands.registerCommand(
        "IntakeCoral", Commands.runOnce(() -> intake.setRPM(-3000), intake));
    NamedCommands.registerCommand(
        "EjectCoral", Commands.runOnce(() -> intake.setRPM(3000), intake));
    NamedCommands.registerCommand(
        "WarmUpBeforeAuto",
        AutoCommands.pathfindToAutoStartPoseWhileWarmup("4 L1 Coral Auto", endEffector, intake));

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

    driverController.x().onTrue(drive.pfToPose(Constants.FieldConstants.REEF_D, 0.0));
    driverController.y().onTrue(drive.pathfindToPoseFlipped(Constants.FieldConstants.REEF_D, 0.0));
    driverController.povDown().onTrue(climber.unwindCommand());
    driverController.povUp().onTrue(climber.retractCommand());
    driverController
        .rightBumper()
        .whileTrue(
            intake
                .runIntake(0.3)
                .alongWith(endEffector.runEffector(0.25))
                .until(() -> intake.isCoralDetected()));
    driverController
        .leftBumper()
        .whileTrue(intake.runIntake(-0.3).alongWith(endEffector.runEffectorReverse(0.25)));
    // Slow mode
    driverController
        .leftTrigger()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY() * Constants.SLOW_MODE_CONSTANT,
                () -> -driverController.getLeftX() * Constants.SLOW_MODE_CONSTANT,
                () -> -driverController.getRightX() * Constants.SLOW_MODE_CONSTANT));

    operatorController.povLeft().onTrue(elevator.executePreset(ElevatorState.Default));
    operatorController.povRight().onTrue(elevator.executePreset(ElevatorState.CoralL2));
    operatorController.povUp().onTrue(elevator.executePreset(ElevatorState.CoralL3));
    operatorController.povDown().onTrue(elevator.executePreset(ElevatorState.CoralL4));

    controller.x().onTrue(drive.pathfindToPose(Constants.FieldConstants.bargeFar, 0.0));

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