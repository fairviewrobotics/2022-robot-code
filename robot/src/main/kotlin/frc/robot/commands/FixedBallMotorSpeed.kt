package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.BallMotorSubsystem
import frc.robot.subsystems.WinchSubsystem

/*
Set a ball motor to the speed supplied by the speed lambda.
This command does not halt its execution.
 */
class FixedBallMotorSpeed(val system: BallMotorSubsystem, val speed: () -> Double) : CommandBase() {
    init {
        addRequirements(system)
    }

    override fun execute() {
        system.setSpeed(speed())
    }

    override fun end(interrupted: Boolean) {
        system.setSpeed(0.0)
    }

    override fun isFinished() = false
}