// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.BASE;
import frc.robot.subsystems.Wrist;

public class ToggleWristControlMode extends Command {
  /** Creates a new ToggleWristControlMode. */
  private final Wrist m_wrist;

  public ToggleWristControlMode(Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wrist = wrist;

    addRequirements(m_wrist);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (m_wrist.getClosedLoopControlMode() != BASE.CONTROL_MODE.CLOSED_LOOP)
      m_wrist.setClosedLoopControlMode(BASE.CONTROL_MODE.OPEN_LOOP);
    else if (m_wrist.getClosedLoopControlMode() != BASE.CONTROL_MODE.CLOSED_LOOP)
      m_wrist.setClosedLoopControlMode(BASE.CONTROL_MODE.OPEN_LOOP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
