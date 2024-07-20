// Copyright 2021-2024 FRC 6328
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

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  private FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), 1.5, 0.004);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private boolean positionControl = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;
  private double positionRotations = 0.0;
  private double positionRotationsSetpoint = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (closedLoop) {
      if (positionControl) {
        appliedVolts =
            MathUtil.clamp(
                pid.calculate(positionRotations) + ffVolts,
                -12.0,
                12.0);
        sim.setInputVoltage(appliedVolts);
      } else {
        appliedVolts =
            MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
        sim.setInputVoltage(appliedVolts);
      }
    }

    sim.update(0.02);

    inputs.positionRad = sim.getAngularVelocityRadPerSec() * 0.02;
    positionRotations =
        positionRotations + sim.getAngularVelocityRadPerSec() * 0.02 / (2 * Math.PI);
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setVelocitySetPoint(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    positionControl = false;
    pid.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  @Override
  public void setPositionSetpoint(double setpoint) {
    closedLoop = true;
    positionControl = true;
    pid.setSetpoint(setpoint);
  }

  @Override
  public void rollBack() {
    closedLoop = true;
    positionControl = true;
    setPositionSetpoint(positionRotations - Constants.INTAKE_ROLLBACK_ROTATIONS);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
