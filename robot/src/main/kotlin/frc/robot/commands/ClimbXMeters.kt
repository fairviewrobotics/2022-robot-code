package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.WinchSubsystem

class ClimbXMeters(val climber: WinchSubsystem, val meters: Double) : CommandBase() {
    val distancePid = PIDController(1.0,0.0,0.0)

    init {
        addRequirements(climber)
        climber.setSpeed(0.0)
        climber.resetEncoder()
    }

    override fun execute() {
        val output = distancePid.calculate(climber.position(), meters)
        climber.setSpeed(output)
    }

    override fun end(interrupted: Boolean) {
        climber.setSpeed(0.0)
        climber.resetEncoder()
    }

    override fun isFinished(): Boolean {
        return Math.abs(meters - climber.position()) < 0.01
    }
}