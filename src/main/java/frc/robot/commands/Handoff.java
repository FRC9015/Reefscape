package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Handoff extends ParallelCommandGroup {

   
    public Handoff(IntakeSubsystem intake, AmpSubsystem amp){
        addCommands(
                intake.intakeNote(),
                amp.ampIntake()
        );
        
   
    }
}