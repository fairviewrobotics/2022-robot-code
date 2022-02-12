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
import frc.robot.subsystems.DrivetrainOldSubsystem
import kotlin.math.abs
import kotlin.math.atan2


class DrivetrainPIDControllerOld(val drivetrain: DrivetrainOldSubsystem) {
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

class DrivetrainPIDCommandOld(val drivetrain: DrivetrainOldSubsystem, val periodic: () -> DifferentialDriveWheelSpeeds) : CommandBase() {
    val controller = DrivetrainPIDControllerOld(drivetrain)

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

fun ArcadeDrive(drivetrain: DrivetrainOldSubsystem, controller: XboxController) : DrivetrainPIDCommandOld {
    val kinematics = DifferentialDriveKinematics(21.5)
    return DrivetrainPIDCommandOld(drivetrain) {
        var forward = -controller.leftY * abs(controller.leftY) / 0.25
        var rotation = controller.leftX * abs(controller.leftX) / 0.25

        SmartDashboard.putNumber("Controller", controller.getLeftX())
        SmartDashboard.putNumber("Rotation", rotation)
        SmartDashboard.putNumber("Actual Gyro Heading", drivetrain.heading)

        kinematics.toWheelSpeeds(ChassisSpeeds(forward, 0.0, rotation))
    }
}

class DrivetrainPIDAngularControllerOld(val drivetrainSubsystem: DrivetrainOldSubsystem) {
    val driveController = DrivetrainPIDControllerOld(drivetrainSubsystem)
    val angleController = PIDController(
        0.05,
        0.0,
        0.0
    )

    init {
        angleController.enableContinuousInput( -1.0 * Math.PI, 1.0* Math.PI)
    }

    val kinematics = DifferentialDriveKinematics(21.5)

    fun execute(forward: Double, angle: Double) {
        val currentAngle = drivetrainSubsystem.heading

        val offset = angleController.calculate(currentAngle, angle)

        val newSpeeds = kinematics.toWheelSpeeds(ChassisSpeeds(forward, 0.0, offset))

        driveController.execute(newSpeeds)
    }
}

class DrivetrainPIDAngularCommandOld(val drivetrain: DrivetrainOldSubsystem, val periodic: () -> Pair<Double, Double>): CommandBase() {
    val controller = DrivetrainPIDAngularControllerOld(drivetrain)

    init {
        addRequirements(drivetrain)
    }

    override fun execute() {
        val step = periodic()
        controller.execute(step.first, step.second)
    }

    override fun end(interrupted: Boolean) {
        drivetrain.tankDriveVolts(0.0, 0.0)
    }

    override fun isFinished() = false
}

fun JoystickDrive(drivetrain: DrivetrainOldSubsystem, controller: XboxController) : DrivetrainPIDAngularCommandOld {
    return DrivetrainPIDAngularCommandOld(drivetrain) {
        var angle = atan2(controller.leftY, controller.leftX)

        SmartDashboard.putNumber("Setpoint (Degrees)", angle)
        SmartDashboard.putNumber("Actual Gyro Heading", drivetrain.heading)


        Pair<Double, Double>(0.0, angle)
    }
}

