package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DrivetrainSubsystem
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

class ArcadePIDDrive(val drivetrain: DrivetrainSubsystem, val controller: XboxController) : CommandBase() {
   init {
       addRequirements(drivetrain)
   }

    val leftPID = PIDController(
        6.2296,
        0.1,
        0.0)

    val rightPID = PIDController(
        6.2296,
        0.1,
        0.0)

    val kinematics = DifferentialDriveKinematics(21.5)

    override fun execute() {
        val speeds = kinematics.toWheelSpeeds(ChassisSpeeds(-controller.leftY * abs(controller.leftY), 0.0, controller.leftX * abs(controller.leftX)))

        val desiredLeft = speeds.leftMetersPerSecond
        val desiredRight = speeds.rightMetersPerSecond

        val currentLeft = drivetrain.wheelSpeeds.leftMetersPerSecond
        val currentRight = drivetrain.wheelSpeeds.rightMetersPerSecond

        val newLeft = leftPID.calculate(currentLeft, desiredLeft)
        val newRight = rightPID.calculate(currentRight, desiredRight)

        drivetrain.tankDriveVolts(newLeft, newRight)
    }

    override fun end(interrupted: Boolean) {
        drivetrain.tankDriveVolts(0.0,0.0)
    }

    override fun isFinished() = false
}