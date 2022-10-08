package frc.robot.commands

import CheckVisionOrRumble
import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.subsystems.BallMotorSubsystem
import frc.robot.subsystems.DrivetrainSubsystem
import frc.robot.subsystems.ShooterSubsystem
import java.lang.Math.abs
import kotlin.math.pow
//import org.photonvision.PhotonCamera

/*
Control the indexer and gate while shooting is happening.
This command does not control the ShooterSubsystem, but
monitors its speed.

If endAfterOneBall is true, the command stops after it thinks a ball has been shot
 */
class ShootBallMotor(
    val shooter1: ShooterSubsystem,
    val shooter2: ShooterSubsystem,
    val gate: BallMotorSubsystem,
    val indexer: BallMotorSubsystem,
    val endAfterOneBall: Boolean = false
) : CommandBase() {

    companion object {
        // tolerance (in rad/s) on shooter being at speed
        val speedTolerance = 7.0
    }

    // number of cycles that we have been at speed for
    var atSpeedCycles = 0

    // if we have gotten to speed
    var atSpeedOnce = false

    // number of cycles that we have not been at speed for
    var belowSpeedCycles = 0

    // if we have gotten to speed and then slowed down,
    // ie, had a ball go through the shooter
    var belowSpeedOnce = false

    // if we got up to speed and then dropped (ie

    init {
        addRequirements(gate)
        addRequirements(indexer)
        // we don't add requirements on shooter1 and shooter2 because we are monitoring their speeds, not controlling them
    }

    // check if motors are at their target speed
    fun atSpeed(): Boolean {

        return abs(shooter1.getTargetDiff()) <= speedTolerance &&
                abs(shooter2.getTargetDiff()) <= speedTolerance
    }

    override fun execute() {
        val ready = atSpeed()
        // check if we are at speed
        if (ready) {
            atSpeedCycles++
            if (atSpeedCycles > 10) {
                atSpeedOnce = true
            }
            belowSpeedCycles = 0
        } else {
            atSpeedCycles = 0
            if (atSpeedOnce) {
                belowSpeedCycles++
                if (belowSpeedCycles > 10) {
                    belowSpeedOnce = true
                }
            }
        }

        if (ready) {
            // if we can shoot, run the gate
            gate.setSpeed(Constants.gateSpeed)
            // only run the indexer if we've already shot one ball
            // otherwise, running the indexer + gate will advance the second ball to far
            if (belowSpeedOnce) {
                indexer.setSpeed(Constants.intakeIndexerSpeed)
            } else {
                indexer.setSpeed(0.0)
            }
        } else {
            indexer.setSpeed(Constants.intakeIndexerSpeed)
            gate.setSpeed(0.0)
        }
    }

    override fun end(interrupted: Boolean) {
        gate.setSpeed(0.0)
        indexer.setSpeed(0.0)

        atSpeedOnce = false
        atSpeedCycles = 0
        belowSpeedOnce = false
        belowSpeedCycles = 0
    }

    override fun isFinished(): Boolean {
        return if(!endAfterOneBall) {
            false
        } else {
            // end after we were once at speed, then dropped, then are now at speed again.
            // this means we speed up, shot, and have had time for the ball to fully clear
            atSpeedOnce && belowSpeedOnce && atSpeed()
        }
    }
}

object HighGoalVision {
    /**
    val photonVision = PhotonCamera("photonvision")

    val result = photonVision.getLatestResult()

    val found_target get() = result.getBestTarget()
    val yaw get() = found_target.getYaw()
    val pitch get() = found_target.getPitch()
    val center_distance get() = PhotonUtils.calculateDistanceToTargetMeters(
        0, // camera height meters
        0, // camera target meters
        0, // camera pitch radians
        Units.degreesToRadians(pitch)
    )
    **/

    val ntInst = NetworkTableInstance.getDefault()
    val table = ntInst.getTable("high_goal")
    val found_target = table.getEntry("target_found")
    val yaw = table.getEntry("yaw")
    val pitch = table.getEntry("pitch")
    val center_distance = table.getEntry("center_distance")

}

/**
 * Turn the drivetrain towards the target location reported in network tables.
 * This uses the continuously updated target position to reorient the robot, and may not stabilize with noisy vision data.
 */
class TurnToHighGoal(val drivetrain: DrivetrainSubsystem) : CommandBase() {
    val control = TurnToAngleController(drivetrain)

    override fun execute() {
        control.execute({
            // target location is current position + vision offset.
            // the 1.2 factor is used to dampen the amount that we turn at each step to remove oscillation caused by time delay on vision data.
            // this is non ideal, but the robot's position should asymptotically approach the target
            drivetrain.heading + HighGoalVision.yaw.getDouble(0.0) / 1.2
        })
    }

    override fun isFinished(): Boolean {
        return (!HighGoalVision.found_target.getBoolean(false)) || control.finished()
    }

    override fun end(interrupted: Boolean) {
        drivetrain.tankDriveVolts(0.0, 0.0)
    }
}

/**
 * Turn the drivetrain towards the target location reported in networktables.
 * This fixes the location at the start of the command, and does not adjust if the reported position changes.
 */
class TurnToFixedHighGoal(val drivetrain: DrivetrainSubsystem): CommandBase() {
    val control = TurnToAngleController(drivetrain)
    var angle = 0.0

    override fun initialize() {
        angle = drivetrain.heading + HighGoalVision.yaw.getDouble(0.0)
    }

    override fun execute() {
        control.execute({ angle })
    }

    override fun end(interrupted: Boolean) {
        drivetrain.tankDriveVolts(0.0, 0.0)
    }

    override fun isFinished(): Boolean {
        return control.finished()
    }
}

/**
 * Given a distance from the camera to the center of the target (in m),
 * calculate optimal shooter speed and lower shooter adjustment (in rad/s).
 * Returns Pair(speed, lower adjust).
 */
fun get_shoot_speed_for_distance(distance_to_target_center: Double): DualShootSpeed {
    // Data calibrated at 1619:
    /*// Clamp distance to range that we collected data for
    val dist = clamp(distance_to_target_center, 2.5, 4.0)
    // These are just curves fit to the empirical data from 3/12
    val speed = 557.0 + -202.0 * dist + 50.6 * dist.pow(2.0)
    val adjust = -480.0 + 317.0 * dist + -76.5 * dist.pow(2.0)

    return DualShootSpeed(max(speed, 570.0), min(adjust, -speed))*/


    // Data calibrated at ssd on 3/18:
    /*val dist = clamp(distance_to_target_center, 2.4, 4.48)
    val speed = 812 + -343*dist + 60.7 * dist.pow(2.0)
    val adjust = -290 + 163*dist + -26.7 * dist.pow(2.0)

    return DualShootSpeed(speed, adjust)*/

    // Data calibrated on comp practice field
    val dist = clamp(distance_to_target_center, 2.7, 4.15)
    val speed = 624.0 + -219.0 * dist + 43.7 * dist.pow(2.0)
    val adjust = -586.0 + 321.0 * dist - 52.4 * dist.pow(2.0)

    return DualShootSpeed(speed, adjust)
}

/**
 * Run the shooter at the speed reported by the vision system.
 * This uses continuously updated vision results. This is good for initial speed up while robot is still in motion,
 * but not good for actually stabilizing on a speed (b/c distance may not stabilize).
 */
fun ShooterSpinUpVision(shooter1: ShooterSubsystem, shooter2: ShooterSubsystem): Command {
    return DualShooterPID(shooter1, shooter2) {
        get_shoot_speed_for_distance(HighGoalVision.center_distance.getDouble(Constants.shooterDefaultDist))
    }
}

/**
 * Run the shooter at the speed reported by the vision system at the beginning of the command.
 * This means the distance is fixed, so shooter speed should quickly stabilize and allow for putting a ball through.
 */
class ShooterFixedVision(val shooter1: ShooterSubsystem, val shooter2: ShooterSubsystem) : CommandBase() {
    val control = DualShooterPIDController(shooter1, shooter2)

    var speeds = DualShootSpeed(0.0, 0.0)

    init {
        addRequirements(shooter1, shooter2)
    }

    override fun initialize() {
        val distance = HighGoalVision.center_distance.getDouble(Constants.shooterDefaultDist)
        speeds = get_shoot_speed_for_distance(distance)
    }

    override fun execute() {
        control.execute(speeds)
    }

    override fun end(interrupted: Boolean) {
        shooter1.setVoltage(0.0)
        shooter1.setTarget(0.0)
        shooter2.setVoltage(0.0)
        shooter2.setTarget(0.0)
    }
}


// --- These are complete shooting modes that are meant to be triggered by the user ---

// Turn to the high goal vision target and shoot based on vision distance.
// alertController is the controller to rumble if no vision is available.
fun ShootVision(
    drivetrain: DrivetrainSubsystem,
    shooter1: ShooterSubsystem,
    shooter2: ShooterSubsystem,
    gate: BallMotorSubsystem,
    indexer: BallMotorSubsystem,
    alertController: XboxController,
    endAfterOneBall: Boolean = false
): Command {
    return SequentialCommandGroup(
        // rumble controller if no vision
        CheckVisionOrRumble(alertController),
        // spin up shooter while turning to target
        TurnToHighGoal(drivetrain).raceWith(
            ShooterSpinUpVision(shooter1, shooter2)
        ).withTimeout(2.0),
        // maintain shooter speed and shoot
        ShootBallMotor(shooter1, shooter2, gate, indexer, endAfterOneBall).raceWith(
            ParallelCommandGroup(
                MaintainAngle(drivetrain),
                ShooterFixedVision(shooter1, shooter2),
            )
        )
    )
}

// Run the shooter and indexer/gate for the default distance.
fun ShootDefaultDistance(
    shooter1: ShooterSubsystem,
    shooter2: ShooterSubsystem,
    gate: BallMotorSubsystem,
    indexer: BallMotorSubsystem,
    endAfterOneBall: Boolean = false
): Command {
    val speeds = get_shoot_speed_for_distance(Constants.shooterDefaultDist)
    return ShootBallMotor(shooter1, shooter2, gate, indexer, endAfterOneBall).raceWith(
        DualShooterPID(shooter1, shooter2, { speeds }),
    )
}

