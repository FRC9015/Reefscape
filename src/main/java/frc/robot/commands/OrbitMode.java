// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.FieldConstants;
// import frc.robot.subsystems.drive.Drive;

// import java.util.function.DoubleSupplier;

// import javax.xml.crypto.dsig.spec.XSLTTransformParameterSpec;

// import org.littletonrobotics.junction.Logger;

// public class OrbitMode extends Command {
//     PIDController rotationalPID = new PIDController(0.175, 0.0025, 0.016);
//     Drive drive;
//     DoubleSupplier xSupplier;
//     DoubleSupplier ySupplier;
//     public OrbitMode(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
//         rotationalPID.enableContinuousInput(-180, 180);
//         this.drive = drive;
//         this.xSupplier = xSupplier;
//         this.ySupplier = ySupplier;
//     }

//     @Override
//     public void initialize() {
//         rotationalPID.reset();
//     }

//     @Override
//     public void execute() {
//         DoubleSupplier inputX = xSupplier;
//         DoubleSupplier inputY = ySupplier;
//         double inputMagnitude = Math.hypot(inputX, inputY);
//         inputMagnitude = MathUtil.applyDeadband(inputMagnitude, 0.1);
//         double inputDir = Math.atan2(inputY, inputX);
//         double forwardDirectionSign = (RobotContainer.IsRed() ? -1.0 : 1.0);

//         drive.runVelocity(new ChassisSpeeds(-yVelocity, -xVelocity, rotationalVelocity));
//     }

//     @Override
//     public void end(boolean interrupted) {
//     }
// }
