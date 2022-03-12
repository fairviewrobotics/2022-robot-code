package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DrivetrainSubsystem
import kotlin.math.abs
import kotlin.math.atan2

class DrivetrainPIDController(val drivetrain: DrivetrainSubsystem) {
    val leftPID = PIDController(
        3.0,
        0.0,
        0.0)

    val rightPID = PIDController(
        3.0,
        0.0,
        0.0)


    fun execute(speeds: DifferentialDriveWheelSpeeds) {
        // get vals
        val currentRightSpeed = drivetrain.wheelSpeeds.rightMetersPerSecond
        val currentLeftSpeed = drivetrain.wheelSpeeds.leftMetersPerSecond

        /*if (abs(speeds.leftMetersPerSecond) < 0.1) { leftPID.p = 120.0; leftPID.i = 0.0; } else { leftPID.p = 6.2296; leftPID.i = 0.1; }
        if (abs(speeds.rightMetersPerSecond) < 0.1) { rightPID.p = 120.0; rightPID.i = 0.0; } else { rightPID.p = 6.2296; rightPID.i = 0.1; }*/

        // calculate
        val leftSpeedToSet = leftPID.calculate(currentLeftSpeed, speeds.leftMetersPerSecond)
        val rightSpeedToSet = rightPID.calculate(currentRightSpeed, speeds.rightMetersPerSecond)

        SmartDashboard.putNumber("Right speed", currentRightSpeed)
        SmartDashboard.putNumber("Set point", speeds.rightMetersPerSecond)
        SmartDashboard.putNumber("Right Voltage", rightSpeedToSet)
        // drive
        drivetrain.tankDriveVolts(leftSpeedToSet, rightSpeedToSet)
    }
}

class DrivetrainPIDAngularController(val drivetrainSubsystem: DrivetrainSubsystem) {
    val driveController = DrivetrainPIDController(drivetrainSubsystem)
    val angleController = PIDController(
        0.05,
        0.0,
        0.0
    )

    init {
        angleController.enableContinuousInput(0.0, 2.0 * Math.PI)
    }

    val kinematics = DifferentialDriveKinematics(21.5)

    fun execute(forward: Double, angle: Double) {
        val currentAngle = drivetrainSubsystem.heading

        val offset = angleController.calculate(currentAngle, angle)

        val newSpeeds = kinematics.toWheelSpeeds(ChassisSpeeds(forward, 0.0, offset))

        driveController.execute(newSpeeds)
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

class DrivetrainPIDAngularCommand(val drivetrain: DrivetrainSubsystem, val periodic: () -> Pair<Double, Double>): CommandBase() {
    val controller = DrivetrainPIDAngularController(drivetrain)

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

fun JoystickDrive(drivetrain: DrivetrainSubsystem, controller: XboxController) : DrivetrainPIDAngularCommand {
    return DrivetrainPIDAngularCommand(drivetrain) {
        var angle = atan2(controller.leftY, controller.leftX)
        if(angle < 0) {
           angle = 2.0 * Math.PI + angle
        }

        SmartDashboard.putNumber("Setpoint (Degrees)", angle)
        SmartDashboard.putNumber("Measure (Degrees)", drivetrain.gyro.angle)

        Pair<Double, Double>(0.0, angle)
    }
}

fun ArcadeDrive(drivetrain: DrivetrainSubsystem, controller: XboxController) : DrivetrainPIDCommand {
    val kinematics = DifferentialDriveKinematics(21.5)
    return DrivetrainPIDCommand(drivetrain) {
        var forward = -controller.leftY * abs(controller.leftY) * 1.25
        var rotation = controller.leftX * abs(controller.leftX) * 0.25

        kinematics.toWheelSpeeds(ChassisSpeeds(forward, 0.0, rotation))
    }
}

class DirectJoystickDrive(val drivetrain: DrivetrainSubsystem,
                          val controller: XboxController): CommandBase() {
    init {
        addRequirements(drivetrain)
    }

    override fun execute() {
        /* 2 joystick arcade drive:
        * Left joystick for faster motion, right joystick for aiming.
        * Right joystick y is inverted for shooting.
        * */
        drivetrain.arcadeDrive(controller.leftY - controller.rightY, controller.leftX + controller.rightX * 0.5)
    }
}

