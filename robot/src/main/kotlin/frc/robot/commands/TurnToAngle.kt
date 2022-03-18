package frc.robot.commands

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.DrivetrainSubsystem

class TurnToAngleController(val drivetrain: DrivetrainSubsystem) {
    val pid = PIDController(Constants.kTTAPidP, Constants.kTTAPidI, Constants.kTTAPidD)

    init {
        pid.enableContinuousInput(-Math.PI, Math.PI)
        pid.setTolerance(Constants.kDrivetrainAngleTolerance, Constants.kDrivetrainVelTolerance)
    }

    fun execute(setPt: () -> Double) {
        val output = pid.calculate(drivetrain.heading, setPt())
        val effort = clamp(output, -Constants.kTTAClamp, Constants.kTTAClamp)

        drivetrain.tankDriveVolts(effort * 12.0, -effort * 12.0)
    }

    fun finished(): Boolean {
        return pid.atSetpoint()
    }
}

// Turn the robot to the given angle (absolute).
class TurnToAngle(val driveSubsystem: DrivetrainSubsystem, val targetAngle: () -> Double) :
    CommandBase() {
    val control = TurnToAngleController(driveSubsystem)

    override fun execute() {
        control.execute { targetAngle() }
    }

    override fun isFinished(): Boolean {
        return control.finished()
    }

    override fun end(interrupted: Boolean) {
        driveSubsystem.tankDriveVolts(0.0, 0.0)
    }
}