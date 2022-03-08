package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ShooterSubsystem

/**
 * Drive the shooter motors at a fixed speed
 */
class FixedShooterSpeed(val shooterSubsystem: ShooterSubsystem, val speed: () -> Double) : CommandBase() {
    init {
        addRequirements(shooterSubsystem)
    }

    override fun execute() {
        shooterSubsystem.setSpeed(speed())
    }

    override fun end(interrupted: Boolean) {
        shooterSubsystem.setSpeed(0.0)
    }

    override fun isFinished() = false
}
