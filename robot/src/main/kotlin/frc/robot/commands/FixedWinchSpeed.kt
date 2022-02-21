package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.WinchSubsystem

class FixedWinchSpeed(val climber: WinchSubsystem, val speed: () -> Double) : CommandBase() {
    /**
     * @param m_subsystem The subsystem used by this command.
     */
    init {
        addRequirements(climber)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        climber.setSpeed(speed())
    }

    override fun end(interrupted: Boolean) {
        climber.setSpeed(0.0)
    }

    // Returns true when the command should end.
    override fun isFinished() = false
}