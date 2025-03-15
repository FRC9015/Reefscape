package frc.robot.subsystems.algae;

@SuppressWarnings("all")
public class AlgaeIOSim implements AlgaeIO {
  private static final double MAX_RPM = 3000.0;
  private static final double GEAR_REDUCTION = 1.0; // Adjust if gearing is used
  private static final double MOMENT_OF_INERTIA = 0.001; // kg * m^2, adjust as needed
}
