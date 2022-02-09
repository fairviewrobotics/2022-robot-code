package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.WhichMotor
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.subsystems.ShooterSubsystem
import frc.robot.Constants

/**
 * Drive the shooter at some angular velocity setpoint (radians / s), using a PID controller
 */
class ShooterPID(val shooterSubsystem: ShooterSubsystem, val setPt: () -> Double) : CommandBase() {
    val pidLower = PIDController(Constants.shooterP, Constants.shooterI, Constants.shooterD)
    val pidHigher = PIDController(Constants.shooterP, Constants.shooterI, Constants.shooterD)
    val ff = SimpleMotorFeedforward(Constants.shooterFFS, Constants.shooterFFV, Constants.shooterFFA)

    init {
        addRequirements(shooterSubsystem)
    }

    override fun execute() {
        val speedLower = pidLower.calculate(shooterSubsystem.getVelocity(WhichMotor.LOWER), setPt()) + ff.calculate(setPt())
        shooterSubsystem.setVoltage(speedLower, WhichMotor.LOWER)
        val speedUpper = pidLower.calculate(shooterSubsystem.getVelocity(WhichMotor.UPPER), setPt()) + ff.calculate(setPt())
        shooterSubsystem.setVoltage(speedUpper, WhichMotor.UPPER)
    }

    override fun end(interrupted: Boolean) {
        shooterSubsystem.setVoltage(0.0, WhichMotor.LOWER)
        shooterSubsystem.setVoltage(0.0, WhichMotor.UPPER)
    }

    override fun isFinished() = false
}
