package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.math.filter.Debouncer;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Current;
// import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {

  // public final TalonFX motor;
  // private final NeutralOut neutralOut = new NeutralOut();
  // private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  // private final StatusSignal<AngularVelocity> rpmSignal;
  // private final StatusSignal<Voltage> appliedVoltsSignal;
  // private final StatusSignal<Current> currentSignal;

  // private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);

  private final DigitalInput coralInSensor;
  private final DigitalInput coralSetSensor;
  private final CANrange middleRange;
  private final CANrange sideRange1;
  private final CANrange sideRange2;

  private final StatusSignal<Distance> rangeDistanceMiddle;
  private final StatusSignal<Distance> rangeSTDdevsMiddle;
  private final StatusSignal<Boolean> rangeIsDetectedMiddle;
  private final StatusSignal<Distance> rangeDistanceSide1;
  private final StatusSignal<Distance> rangeSTDdevsSide1;
  private final StatusSignal<Boolean> rangeIsDetectedSide1;
  private final StatusSignal<Distance> rangeDistanceSide2;
  private final StatusSignal<Distance> rangeSTDdevsSide2;
  private final StatusSignal<Boolean> rangeIsDetectedSide2;

  public IntakeIOTalonFX(
      int coralInChannel, int coralSetChannel, int canRangeID1, int canRangeID2, int canRangeID3) {

    coralInSensor = new DigitalInput(coralInChannel);
    coralSetSensor = new DigitalInput(coralSetChannel);
    middleRange = new CANrange(canRangeID1);
    sideRange1 = new CANrange(canRangeID2);
    sideRange2 = new CANrange(canRangeID3);

    CANrangeConfiguration sideConfig = new CANrangeConfiguration();
    CANrangeConfiguration middleConfig = new CANrangeConfiguration();

    middleConfig.FovParams.FOVRangeX = 6.75;
    middleConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2000;
    middleConfig.ProximityParams.ProximityThreshold = 0.7;
    middleConfig.ToFParams.UpdateMode = UpdateModeValue.LongRangeUserFreq;
    middleConfig.ToFParams.UpdateFrequency = 50.0;

    sideConfig.FovParams.FOVRangeX = 15;
    sideConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2000;
    sideConfig.ProximityParams.ProximityThreshold = 0.7;
    sideConfig.ToFParams.UpdateMode = UpdateModeValue.LongRangeUserFreq;
    sideConfig.ToFParams.UpdateFrequency = 50.0;
    middleRange.getConfigurator().apply(middleConfig);
    sideRange1.getConfigurator().apply(sideConfig);
    sideRange2.getConfigurator().apply(sideConfig);

    rangeDistanceMiddle = middleRange.getDistance();
    rangeSTDdevsMiddle = middleRange.getDistanceStdDev();
    rangeIsDetectedMiddle = middleRange.getIsDetected();
    rangeDistanceSide1 = sideRange1.getDistance();
    rangeSTDdevsSide1 = sideRange1.getDistanceStdDev();
    rangeIsDetectedSide1 = sideRange1.getIsDetected();
    rangeDistanceSide2 = sideRange2.getDistance();
    rangeSTDdevsSide2 = sideRange2.getDistanceStdDev();
    rangeIsDetectedSide2 = sideRange2.getIsDetected();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Refresh signals
    BaseStatusSignal.refreshAll(
        rangeDistanceMiddle,
        rangeSTDdevsMiddle,
        rangeIsDetectedMiddle,
        rangeDistanceSide1,
        rangeSTDdevsSide1,
        rangeIsDetectedSide1,
        rangeDistanceSide2,
        rangeSTDdevsSide2,
        rangeIsDetectedSide2);
    // Update inputs
    // inputs.intakeEncoderConnected = encoderConnectedDebounce.calculate(motorStatus.isOK());
    // inputs.intakeRPM = rpmSignal.getValueAsDouble();
    // inputs.intakeAppliedVolts = appliedVoltsSignal.getValueAsDouble(
    // inputs.intakeCurrentAmps = currentSignal.getValueAsDouble();

    // Commented out for now
    inputs.coralIn = coralInSensor.get(); // Coral detected if the sensor is triggered
    inputs.coralSet = coralSetSensor.get();
    inputs.middleSTDdevs = rangeSTDdevsMiddle.getValueAsDouble();
    inputs.middleDistance = rangeDistanceMiddle.getValueAsDouble();
    inputs.middleIsDetected = rangeIsDetectedMiddle.getValue();
    inputs.side1STDdevs = rangeSTDdevsSide1.getValueAsDouble();
    inputs.side1Distance = rangeDistanceSide1.getValueAsDouble();
    inputs.side1IsDetected = rangeIsDetectedSide1.getValue();
    inputs.side2STDdevs = rangeSTDdevsSide2.getValueAsDouble();
    inputs.side2Distance = rangeDistanceSide2.getValueAsDouble();
    inputs.side2IsDetected = rangeIsDetectedSide2.getValue();
    // (active low)
    Logger.recordOutput("coralIn?", !coralInSensor.get());
    Logger.recordOutput("coralSet?", !coralSetSensor.get());
    Logger.recordOutput("getDistance", middleRange.getDistance(true).getValueAsDouble());
    Logger.recordOutput("getSTDdevs", middleRange.getDistanceStdDev(true).getValueAsDouble());
  }

  @Override
  public void stop() {
    // motor.setControl(neutralOut);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // motor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setRPM(double rpm) {
    // motor.set(-rpm);
  }
}
