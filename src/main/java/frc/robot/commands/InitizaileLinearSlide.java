// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LinearSlide;

public class InitizaileLinearSlide extends Command {

  LinearSlide linearSlide = null;
  boolean finished = false;
  Timer timer = new Timer();

  /** Creates a new InitizaileLinearSlide. */
  public InitizaileLinearSlide(LinearSlide linearSlide) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(linearSlide);
    this.linearSlide = linearSlide;
    finished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    linearSlide.setInitializing(true);
    linearSlide.motor.set(0.25);
    timer.reset();
    timer.start();
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.hasElapsed(1.0)) {
      if (linearSlide.pdh.getCurrent(5) > 1.0) {
        finished = true;
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    linearSlide.motor.set(0);
    linearSlide.relEncoder.setPosition(0);
    linearSlide.setInitializing(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
