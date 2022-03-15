package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.WinchSubsystem

class LimitedWinchCommand(val climber: WinchSubsystem, val voltage: () -> Double) : CommandBase() {
    /**
     * @param m_subsystem The subsystem used by this command.
     */
    init {
        addRequirements(climber)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        climber.setVoltage(voltage())
    }

    override fun end(interrupted: Boolean) {
        climber.setVoltage(0.0)
    }

    // Returns true when the command should end.
    override fun isFinished() : Boolean {
        if (voltage() < 0){
            return climber.hitLower
        } else{
            return climber.hitUpper
        }
        
    }
}