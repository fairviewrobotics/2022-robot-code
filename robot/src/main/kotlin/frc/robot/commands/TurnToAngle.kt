package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.Constants
import frc.robot.subsystems.DrivetrainSubsystem

// Turn the robot to the given angle (absolute).
class TurnToAngle(val driveSubsystem: DrivetrainSubsystem, targetAngle: () -> Double, forwardSpeed: Double) : PIDCommand(
    PIDController(
        Constants.kTTAPidP,
        Constants.kTTAPidI,
        Constants.kTTAPidD,
    ),
    driveSubsystem::heading,
    targetAngle,
    { output: Double -> driveSubsystem.arcadeDrive(forwardSpeed, MathUtil.clamp(output, -0.3, 0.3)) },
    driveSubsystem) {

    init {
        controller.enableContinuousInput(-Math.PI, Math.PI)
        /** reload pid parameters from network tables */
        setPIDParams()
    }

    fun setPIDParams() {
        controller.setTolerance(
            Constants.kDrivetrainAngleTolerance,
            Constants.kDrivetrainVelTolerance
        )
    }

    override fun isFinished(): Boolean {
        /* check if we hit setpoint yet */
        return controller.atSetpoint()
    }

}