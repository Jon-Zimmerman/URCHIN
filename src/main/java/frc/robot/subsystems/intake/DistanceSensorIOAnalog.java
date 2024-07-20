package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotMap;

public class DistanceSensorIOAnalog implements DistanceSensorIO {
  private final AnalogInput proxSensor;
  private int sustain;

  public DistanceSensorIOAnalog() {
    this.proxSensor = new AnalogInput(RobotMap.Intake.BEAM_BREAK_CHANNEL_ID);
    sustain = 0;
  }

  @Override
  public void updateInputs(DistanceSensorIOInputs inputs) {
    inputs.distance = proxSensor.getValue();
    inputs.sustain = this.sustain;
  }

  @Override
  public void increaseSustain() {
    sustain++;
  }

  @Override
  public void resetSustain() {
    sustain = 0;
  }
}
