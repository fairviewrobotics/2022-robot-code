// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ShooterSubsystem

/** An example command that uses an example subsystem.  */
class ShooterLQR(val shooterSubsystem: ShooterSubsystem, val setPt: () -> Double ) : CommandBase() {

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    init {
        addRequirements(shooterSubsystem)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        ShooterSubsystem.setLatencyCompensate(0.025)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        shooterSubsystem.LQROn(setPt())
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        shooterSubsystem.LQROff()
        shooterSubsystem.setSpeed(0.0)
    }

    // Returns true when the command should end.
    override fun isFinished() = false
}