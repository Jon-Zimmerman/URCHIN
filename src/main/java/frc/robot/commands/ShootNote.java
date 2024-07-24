// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class ShootNote extends SequentialCommandGroup {
  /** Creates a new ShootNote Command */
  public ShootNote(
      double setPointRPMShooter, double setPointRPMIntake, Shooter shooter, Intake intake) {

    addCommands(
        new InstantCommand(() -> shooter.runVelocity(setPointRPMShooter), shooter),
        new WaitCommand(2),
        new InstantCommand(() -> intake.runVelocity(setPointRPMIntake)),
        new WaitCommand(1),
        new InstantCommand(shooter::stop, shooter),
        new InstantCommand(intake::stop, intake));
  }
}
