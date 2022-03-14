package frc.robot.commands

import edu.wpi.first.wpilibj.DoubleSolenoid
import frc.robot.subsystems.SolenoidSubsystem
import edu.wpi.first.wpilibj2.command.CommandBase

// A command that sets a solenoid to a specific state
class PneumaticCommand(val solenoid: SolenoidSubsystem, val state: DoubleSolenoid.Value) : CommandBase() {
    init {
        addRequirements(solenoid)
    }

    override fun execute() {
            solenoid.set(state)
    }

    override fun isFinished() = false
}