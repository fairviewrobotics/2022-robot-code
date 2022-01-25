package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.subsystems.ShooterSubsystem
import frc.robot.Constants

/**
 * Drive the shooter at some angular velocity setpoint (radians / s), using a PID controller
 */
class ShooterPID(val shooterSubsystem: ShooterSubsystem, val setPt: () -> Double) : CommandBase() {
    val pid = PIDController(Constants.shooterP, Constants.shooterI, Constants.shooterD)
    val ff = SimpleMotorFeedforward(Constants.shooterFFS, Constants.shooterFFV, Constants.shooterFFA)

    init {
        addRequirements(shooterSubsystem)
    }

    override fun execute() {
        val speed = pid.calculate(shooterSubsystem.getVelocity(), setPt()) + ff.calculate(setPt())
        shooterSubsystem.setVoltage(speed)
    }

    override fun end(interrupted: Boolean) {
        shooterSubsystem.setVoltage(0.0)
    }

    override fun isFinished() = false
}
