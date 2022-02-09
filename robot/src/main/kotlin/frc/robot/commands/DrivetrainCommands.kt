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
import kotlin.math.abs

class DrivetrainPIDController(val drivetrain: DrivetrainSubsystem) {
    val leftPID = PIDController(
        6.2296,
        0.1,
        0.0)

    val rightPID = PIDController(
        6.2296,
        0.1,
        0.0)


    fun execute(speeds: DifferentialDriveWheelSpeeds) {
        // get vals
        val currentRightSpeed = drivetrain.getWheelSpeeds().rightMetersPerSecond
        val currentLeftSpeed = drivetrain.getWheelSpeeds().leftMetersPerSecond

        if (abs(speeds.leftMetersPerSecond) < 0.1) { leftPID.p = 120.0; leftPID.i = 0.0; } else { leftPID.p = 6.2296; leftPID.i = 0.1; }
        if (abs(speeds.rightMetersPerSecond) < 0.1) { rightPID.p = 120.0; rightPID.i = 0.0; } else { rightPID.p = 6.2296; rightPID.i = 0.1; }

        // calculate
        val leftSpeedToSet = leftPID.calculate(currentLeftSpeed, speeds.leftMetersPerSecond)
        val rightSpeedToSet = rightPID.calculate(currentRightSpeed, speeds.rightMetersPerSecond)

        if (currentLeftSpeed != 0.0) {
            println("ping")
        }

        SmartDashboard.putNumber("Right speed", currentLeftSpeed)
        SmartDashboard.putNumber("Set point", speeds.rightMetersPerSecond)
        SmartDashboard.putNumber("Right Voltage", rightSpeedToSet)
        // drive
        drivetrain.tankDriveVolts(leftSpeedToSet, rightSpeedToSet)
    }
}

class DrivetrainPIDCommand(val drivetrain: DrivetrainSubsystem, val periodic: () -> DifferentialDriveWheelSpeeds) : CommandBase() {
    val controller = DrivetrainPIDController(drivetrain)

    init {
        addRequirements(drivetrain)
    }

    override fun execute() {
        controller.execute(periodic())
    }

    override fun end(interrupted: Boolean) {
        drivetrain.tankDriveVolts(0.0, 0.0)
    }

    override fun isFinished() = false
}

fun ArcadeDrive(drivetrain: DrivetrainSubsystem, controller: XboxController) : DrivetrainPIDCommand {
    val kinematics = DifferentialDriveKinematics(21.5)
    return DrivetrainPIDCommand(drivetrain) {
        var forward = -controller.leftY * abs(controller.leftY) / 0.25
        var rotation = controller.leftX * abs(controller.leftX) / 0.25

        SmartDashboard.putNumber("Controller", controller.getLeftX())
        SmartDashboard.putNumber("Rotation", rotation)

        kinematics.toWheelSpeeds(ChassisSpeeds(forward, 0.0, rotation))
    }
}

