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

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO = 1.5;

  private final CANSparkMax rollerMotor = new CANSparkMax(0, MotorType.kBrushless);
  private final RelativeEncoder encoder = rollerMotor.getEncoder();
  private final SparkPIDController pid = rollerMotor.getPIDController();

  public IntakeIOSparkMax() {
    rollerMotor.restoreFactoryDefaults();

    rollerMotor.setCANTimeout(250);

    rollerMotor.setInverted(false);

    rollerMotor.enableVoltageCompensation(12.0);
    rollerMotor.setSmartCurrentLimit(30);

    rollerMotor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage();
    inputs.currentAmps = rollerMotor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double volts) {
    rollerMotor.setVoltage(volts);
  }

  @Override
  public void setVelocitySetPoint(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    rollerMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
    pid.setFF(0, 0);
  }
}
