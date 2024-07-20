// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NoteState;
import frc.robot.subsystems.intake.Intake;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */
  private final Intake intake;
  private double setpoint;

  public IntakeNote(double setpoint, Intake intake) {
    this.intake = intake;
    addRequirements(intake);
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.runVelocity(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    intake.rollBack();
    // if (shooter.seesNote()) ;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.seesNote();
  }
}
