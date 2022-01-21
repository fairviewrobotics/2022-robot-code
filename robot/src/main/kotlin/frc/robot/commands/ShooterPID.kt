package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.subsystems.ShooterSubsystem
import frc.robot.Constants

/**
 * Drive the shooter at some angular velocity setpoint (radians / s), using a PID controller
 */
class ShooterPID(val shooterSubsystem: ShooterSubsystem, val setPt: () -> Double) :
    PIDCommand(PIDController(Constants.shooterP, Constants.shooterI, Constants.shooterD),
        { shooterSubsystem.getVelocity() },
        setPt, { output -> shooterSubsystem.setSpeed(output) }, shooterSubsystem
    ) {

    init {
        Constants.addListener("shooterP") { p -> controller.p = p }
        Constants.addListener("shooterI") { i -> controller.i = i }
        Constants.addListener("shooterD") { d -> controller.d = d }
    }

    override fun isFinished() = false
}
