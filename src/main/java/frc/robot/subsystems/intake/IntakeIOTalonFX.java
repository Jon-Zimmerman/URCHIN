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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class IntakeIOTalonFX implements IntakeIO {
  private static final double GEAR_RATIO = 1.5;

  private final TalonFX rollerMotor = new TalonFX(RobotMap.Intake.MOTOR_ID);

  private final StatusSignal<Double> motorPosition = rollerMotor.getPosition();
  private final StatusSignal<Double> motorVelocity = rollerMotor.getVelocity();
  private final StatusSignal<Double> motorAppliedVolts = rollerMotor.getMotorVoltage();
  private final StatusSignal<Double> motorCurrent = rollerMotor.getSupplyCurrent();

  public IntakeIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerMotor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);
    rollerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorPosition, motorVelocity, motorAppliedVolts, motorCurrent);
    inputs.positionRad = Units.rotationsToRadians(motorPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(motorVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    rollerMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocitySetPoint(double velocityRadPerSec, double ffVolts) {
    rollerMotor.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec),
            0.0,
            true,
            ffVolts,
            0,
            false,
            false,
            false));
  }

  @Override
  public void rollBack() {
    rollerMotor.setControl(
        new PositionVoltage(
            motorPosition.getValueAsDouble() - Constants.INTAKE_ROLLBACK_ROTATIONS));
  }

  @Override
  public void stop() {
    rollerMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    rollerMotor.getConfigurator().apply(config);
  }
}
