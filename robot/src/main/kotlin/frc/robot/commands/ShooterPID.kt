package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.robot.subsystems.ShooterSubsystem
import frc.robot.Constants

class ShooterPIDController(val shooter: ShooterSubsystem, val debug: Boolean = false) {
    val pid = PIDController(Constants.shooterP, Constants.shooterI, Constants.shooterD)
    val ff = SimpleMotorFeedforward(Constants.shooterFFS, Constants.shooterFFV, Constants.shooterFFA)

    fun execute(setPt: Double) {
        val pidOut = pid.calculate(shooter.getVelocity(), setPt)
        val ffOut = ff.calculate(setPt)
        val voltage = pidOut + ffOut
        shooter.setVoltage(voltage)
        shooter.setTarget(setPt)

        if(debug) {
            SmartDashboard.putNumber("Shooter Setpoint (rad/s)", setPt)
            SmartDashboard.putNumber("Shooter_Velocity (rad/s)", shooter.getVelocity())
            SmartDashboard.putNumber("Feed Forward out (V)", ffOut)
            SmartDashboard.putNumber("PID out (V)", pidOut)
            SmartDashboard.putNumber("Total Out (V)", voltage)
        }
    }
}

// The base speed of a dual shooter and an adjustement for shooter1
data class DualShootSpeed(val speed: Double, val adjust: Double)

class DualShooterPIDController(val shooter1: ShooterSubsystem, val shooter2: ShooterSubsystem, val debug: Boolean = false) {
    val control1 = ShooterPIDController(shooter1)
    val control2 = ShooterPIDController(shooter2)

    fun execute(setPt: DualShootSpeed) {
        control1.execute(setPt.speed + setPt.adjust)
        control2.execute(setPt.speed)
    }
}

/**
 * Drive the shooter at some angular velocity setpoint (radians / s), using a PID controller
 */
class ShooterPID(val shooterSubsystem: ShooterSubsystem, val setPt: () -> Double, val debug: Boolean = false) : CommandBase() {
    val control = ShooterPIDController(shooterSubsystem, debug)

    init {
        addRequirements(shooterSubsystem)
    }

    override fun execute() {
        control.execute(setPt())
    }

    override fun end(interrupted: Boolean) {
        shooterSubsystem.setVoltage(0.0)
        shooterSubsystem.setTarget(0.0)
    }

    override fun isFinished() = false
}

// drive two shooters based on a (speed, adjust) setpoint
class DualShooterPID(val shooter1: ShooterSubsystem, val shooter2: ShooterSubsystem, val speed: () -> DualShootSpeed): CommandBase() {
    val control = DualShooterPIDController(shooter1, shooter2)

    init {
        addRequirements(shooter1, shooter2)
    }

    override fun execute() {
        control.execute(speed())
    }

    override fun end(interrupted: Boolean) {
        shooter1.setVoltage(0.0)
        shooter1.setTarget(0.0)
        shooter2.setVoltage(0.0)
        shooter2.setTarget(0.0)
    }

    override fun isFinished() = false
}