// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.BASE.CONTROL_MODE;
import frc.robot.constants.BASE.SETPOINT;
import frc.robot.subsystems.Wrist;

public class AutoWristSetpoint extends Command {
  private Wrist m_Wrist;
  private SETPOINT m_desiredState;

  public AutoWristSetpoint(Wrist wrist, SETPOINT desiredState) {
    m_Wrist = wrist;
    m_desiredState = desiredState;

    addRequirements(m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Wrist.setClosedLoopControlMode(CONTROL_MODE.CLOSED_LOOP);
    m_Wrist.setUserSetpoint(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Wrist.SetWristDesiredSetpoint(m_desiredState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
