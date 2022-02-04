package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.DrivetrainSubsystem

class DrivetrainPIDController(val drivetrain: DrivetrainSubsystem, val speeds: () -> DifferentialDriveWheelSpeeds) {
    var kDrivetrainPidP = Constants.kDrivetrainPidP
    var kDrivetrainPidI = Constants.kDrivetrainPidI
    var kDrivetrainPidD = Constants.kDrivetrainPidD
    var kDrivetrainMaxVelocity = Constants.kDrivetrainMaxVelocity
    var kDrivetrainMaxAcceleration = Constants.kDrivetrainMaxAcceleration


    val leftPIDController = PIDController(
        -kDrivetrainPidP,
        kDrivetrainPidI,
        kDrivetrainPidD)

    val rightPIDController = PIDController(
        -kDrivetrainPidP,
        kDrivetrainPidI,
        kDrivetrainPidD)

    fun execute() {
        // get vals
        val currentRightSpeed = drivetrain.wheelSpeeds.rightMetersPerSecond
        val currentLeftSpeed = drivetrain.wheelSpeeds.leftMetersPerSecond

        // calculate

        val speedsFrame = speeds()
        val leftSpeedToSet = leftPIDController.calculate(currentLeftSpeed, speedsFrame.leftMetersPerSecond)
        val rightSpeedToSet = rightPIDController.calculate(currentRightSpeed, speedsFrame.rightMetersPerSecond)
        SmartDashboard.putNumber("PV", currentLeftSpeed)
        SmartDashboard.putNumber("SP", speedsFrame.leftMetersPerSecond)
        SmartDashboard.putNumber("Left Volts", leftSpeedToSet)
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

fun ArcadeDriveCommand(drivetrain: DrivetrainSubsystem, controller: XboxController) : DrivetrainPIDCommand {
    val kDrivetrainPidInematics = DifferentialDriveKinematics(21.5)
    return DrivetrainPIDCommand(drivetrain) {
        val speeds = ChassisSpeeds(-controller.leftY * Constants.kDrivetrainMaxVelocity, 0.0, -controller.leftX * Constants.kDrivetrainMaxAngularVelocity)

        kDrivetrainPidInematics.toWheelSpeeds(speeds)
    }
}

