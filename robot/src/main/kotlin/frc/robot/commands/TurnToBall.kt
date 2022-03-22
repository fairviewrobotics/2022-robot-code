package frc.robot.commands

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.Constants
import frc.robot.subsystems.DrivetrainSubsystem

object BallVisionNT {
    val ntInst = NetworkTableInstance.getDefault()
    val table = ntInst.getTable("ML")
    val angle = table.getEntry("targetAngle")
    val foundTarget = table.getEntry("foundTarget")
}


class TurnToBall(val controller: XboxController, val drivetrain: DrivetrainSubsystem) : CommandBase() {
    val control = TurnToAngleController(drivetrain)

    override fun execute() {
        val targetAngle = {
            if (BallVisionNT.foundTarget.getBoolean(false))
                drivetrain.heading + BallVisionNT.angle.getDouble(0.0) / 1.6
            else
                drivetrain.heading
        }
        control.execute(
            targetAngle,
            4.0 * (controller.leftY + controller.rightY * Constants.kDrivetrainFineForwardSpeed)
        )
    }

    override fun isFinished(): Boolean {
        //return !BallVisionNT.foundTarget.getBoolean(false)
        return false
    }
}

// Turn the drivetrain towards the location of the ball.
// The ball location is taken when the command begins and is not updated as the command executes.
class TurnToBallFixed(val controller: XboxController, val drivetrain: DrivetrainSubsystem): CommandBase() {
    val control = TurnToAngleController(drivetrain)
    var targetAngle = 0.0

    override fun initialize() {
        targetAngle = BallVisionNT.angle.getDouble(0.0)
    }

    override fun execute() {
        control.execute({ targetAngle },
            4.0 * (controller.leftY + controller.rightY * Constants.kDrivetrainFineForwardSpeed)
        )
    }

    override fun isFinished(): Boolean {
        return !BallVisionNT.foundTarget.getBoolean(false) || control.finished()
    }
}

// Turn the drivetrain towards the location of the ball without using the gyro.
class TurnToBallDirect(val controller: XboxController, val drivetrain: DrivetrainSubsystem): CommandBase() {
    val pid = PIDController(Constants.kTTAPidP, Constants.kTTAPidI, Constants.kTTAPidD)

    override fun initialize() {
        pid.enableContinuousInput(-Math.PI, Math.PI)
        pid.setTolerance(Constants.kDrivetrainAngleTolerance, Constants.kDrivetrainVelTolerance)
    }

    override fun execute() {
        val forward = 4.0 * (controller.leftY + controller.rightY * Constants.kDrivetrainFineForwardSpeed)

        // get error directly from ball angle in networktables
        val error = BallVisionNT.angle.getDouble(0.0)

        val output = pid.calculate(error, 0.0)
        val effort = MathUtil.clamp(output, -Constants.kTTAClamp, Constants.kTTAClamp)

        drivetrain.tankDriveVolts(effort * 12.0 + forward, -effort * 12.0 + forward)
    }

    override fun isFinished(): Boolean {
        return BallVisionNT.foundTarget.getBoolean(false) || pid.atSetpoint()
    }
}
