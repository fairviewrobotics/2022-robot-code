package frc.robot.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DrivetrainSubsystem

class DebugDrive(val drivetrain: DrivetrainSubsystem, val controller: XboxController) : CommandBase() {
    init {
        addRequirements(drivetrain)
    }

    val kinematics = DifferentialDriveKinematics(21.5)

    override fun execute() {
        val speeds = kinematics.toWheelSpeeds(ChassisSpeeds(-controller.leftY, 0.0, controller.leftX))

        val desiredLeft = speeds.leftMetersPerSecond
        val desiredRight = speeds.rightMetersPerSecond

        val newSpeeds = kinematics.toChassisSpeeds(DifferentialDriveWheelSpeeds(desiredLeft, desiredRight))

        drivetrain.drive.arcadeDrive(newSpeeds.vxMetersPerSecond, newSpeeds.omegaRadiansPerSecond)
    }

    override fun isFinished() = false
}