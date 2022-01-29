package frc.robot.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.DrivetrainSubsystem

class DrivetrainPIDController(val drivetrain: DrivetrainSubsystem, val speeds: () -> DifferentialDriveWheelSpeeds) {
    var kDrivetrainPidP = Constants.kDrivetrainPidP
    var kDrivetrainPidI = Constants.kDrivetrainPidI
    var kDrivetrainPidD = Constants.kDrivetrainPidD
    var kDrivetrainMaxVelocity = Constants.kDrivetrainMaxVelocity
    var kDrivetrainMaxAcceleration = Constants.kDrivetrainMaxAcceleration


    val leftPIDController = ProfiledPIDController(
        kDrivetrainPidP,
        kDrivetrainPidI,
        kDrivetrainPidD,
        TrapezoidProfile.Constraints(kDrivetrainMaxVelocity, kDrivetrainMaxAcceleration)
    )

    val rightPIDController = ProfiledPIDController(
        kDrivetrainPidP,
        kDrivetrainPidI,
        kDrivetrainPidD,
        TrapezoidProfile.Constraints(kDrivetrainMaxVelocity, kDrivetrainMaxAcceleration)
    )

    fun execute() {
        println(kDrivetrainPidP)
        // get vals
        val currentRightSpeed = drivetrain.wheelSpeeds.rightMetersPerSecond
        val currentLeftSpeed = drivetrain.wheelSpeeds.leftMetersPerSecond

        // calculate

        val speedsFrame = speeds()
        val leftSpeedToSet = leftPIDController.calculate(currentLeftSpeed, speedsFrame.leftMetersPerSecond)
        val rightSpeedToSet = rightPIDController.calculate(currentRightSpeed, speedsFrame.rightMetersPerSecond)

        // drive
        drivetrain.tankDriveVolts(leftSpeedToSet, rightSpeedToSet)
    }
}

class DrivetrainPIDCommand(val drivetrain: DrivetrainSubsystem, val periodic: () -> DifferentialDriveWheelSpeeds) : CommandBase() {
    val controller = DrivetrainPIDController(drivetrain, periodic)

    init {
        addRequirements(drivetrain)
    }

    override fun execute() {
        controller.execute()
    }

    override fun end(interrupted: Boolean) {
        drivetrain.tankDriveVolts(0.0, 0.0)
    }

    override fun isFinished() = false
}


fun DebugDriveCommand(drivetrain: DrivetrainSubsystem) : DrivetrainPIDCommand {
    return DrivetrainPIDCommand(drivetrain) {
        DifferentialDriveWheelSpeeds(Constants.kDrivetrainMaxVelocity, Constants.kDrivetrainMaxVelocity)
    }
}

fun ArcadeDriveCommand(drivetrain: DrivetrainSubsystem, controller: XboxController) : DrivetrainPIDCommand {
    val kDrivetrainPidInematics = DifferentialDriveKinematics(21.5)
    return DrivetrainPIDCommand(drivetrain) {
        val speeds = ChassisSpeeds(-controller.leftY * Constants.kDrivetrainMaxVelocity, 0.0, -controller.leftX * Constants.kDrivetrainMaxAngularVelocity)

        kDrivetrainPidInematics.toWheelSpeeds(speeds)
    }
}

