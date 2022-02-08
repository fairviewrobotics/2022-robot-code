// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import frc.robot.subsystems.ExamplePneumaticSubsystem
import edu.wpi.first.wpilibj2.command.CommandBase

/** An example command that uses an example subsystem.  */
class ExamplePneumaticCommandForward(subsystem: ExamplePneumaticSubsystem) : CommandBase() {

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        subsystem.setForward()

    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        subsystem.setOff()
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}