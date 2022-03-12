package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ShooterSubsystem
import frc.robot.Constants

/**
 * Drive the shooter at some angular velocity setpoint (radians / s), using a PID controller
 */
class ShooterPID(val shooterSubsystem: ShooterSubsystem, val setPt: () -> Double, val debug: Boolean = false) : CommandBase() {
    val pid = PIDController(Constants.shooterP, Constants.shooterI, Constants.shooterD)
    val ff = SimpleMotorFeedforward(Constants.shooterFFS, Constants.shooterFFV, Constants.shooterFFA)

    init {
        addRequirements(shooterSubsystem)
    }

    override fun execute() {
        val pidOut = pid.calculate(shooterSubsystem.getVelocity(), setPt())
        val ffOut = ff.calculate(setPt())

        val voltage = pidOut + ffOut
        shooterSubsystem.setVoltage(voltage)

        if(debug) {
            SmartDashboard.putNumber("Shooter Setpoint (rad/s)", setPt())
            SmartDashboard.putNumber("Shooter_Velocity (rad/s)", shooterSubsystem.getVelocity())
            SmartDashboard.putNumber("Feed Forward out (V)", ffOut)
            SmartDashboard.putNumber("PID out (V)", pidOut)
            SmartDashboard.putNumber("Total Out (V)", voltage)
        }
    }

    override fun end(interrupted: Boolean) {
        shooterSubsystem.setVoltage(0.0)
    }

    override fun isFinished() = false
}
