// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveClimberCommand extends Command {
  /** Creates a new Climber. */
  private final ClimberSubsystem m_controlArm;
  private final CommandJoystick m_controller;

  public MoveClimberCommand(ClimberSubsystem controlArm, CommandJoystick controller) {
    m_controlArm = controlArm;
    m_controller = controller;

    addRequirements(controlArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double trigger = 0.0;
    if (m_controller.button(3).getAsBoolean()) {
      trigger = -1;
    } else if (m_controller.button(4).getAsBoolean()) {
      trigger = 1;
    } else {
      trigger = 0;
    }
    
    m_controlArm.driveArm(trigger);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
