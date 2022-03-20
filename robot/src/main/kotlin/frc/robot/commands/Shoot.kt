package frc.robot.commands

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.PIDCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.subsystems.BallMotorSubsystem
import frc.robot.subsystems.DrivetrainSubsystem
import frc.robot.subsystems.ShooterSubsystem
import java.lang.Math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

/*
Control the indexer and gate while shooting is happening.
This command does not control the ShooterSubsystem, but
monitors its speed.
 */
class ShootBallMotor(
    val shooter1: ShooterSubsystem,
    val shooter2: ShooterSubsystem,
    val gate: BallMotorSubsystem,
    val indexer: BallMotorSubsystem
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
                indexer.setSpeed(Constants.indexerSpeed)
            } else {
                indexer.setSpeed(0.0)
            }
        } else {
            indexer.setSpeed(Constants.indexerSpeed)
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

    override fun isFinished() = false
}

object HighGoalVisionNT {
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
            drivetrain.heading + HighGoalVisionNT.yaw.getDouble(0.0) / 1.2
        })
    }

    override fun isFinished(): Boolean {
        return (!HighGoalVisionNT.found_target.getBoolean(false)) || control.finished()
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
        angle = drivetrain.heading + HighGoalVisionNT.yaw.getDouble(0.0)
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
    // Clamp distance to range that we collected data for
    val dist = clamp(distance_to_target_center, 2.5, 4.0)
    // These are just curves fit to the empirical data from 3/12
    val speed = 557.0 + -202.0 * dist + 50.6 * dist.pow(2.0)
    val adjust = -480.0 + 317.0 * dist + -76.5 * dist.pow(2.0)

    return DualShootSpeed(max(speed, 570.0), min(adjust, -speed))
}

/**
 * Run the shooter at the speed reported by the vision system.
 * This uses continuously updated vision results. This is good for initial speed up while robot is still in motion,
 * but not good for actually stabilizing on a speed (b/c distance may not stabilize).
 */
fun ShooterSpinUpVision(shooter1: ShooterSubsystem, shooter2: ShooterSubsystem): Command {
    return DualShooterPID(shooter1, shooter2) {
        get_shoot_speed_for_distance(HighGoalVisionNT.center_distance.getDouble(Constants.shooterDefaultDist))
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
        val distance = HighGoalVisionNT.center_distance.getDouble(Constants.shooterDefaultDist)
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

/**
 * Do nothing if a vision target is found.
 * Otherwise, rumble the given controller. This is mean to make it clear to the driver they need to shoot manually.
 */
class CheckVisionOrRumble(val controller: XboxController) : CommandBase() {
    override fun execute() {
        if (!HighGoalVisionNT.found_target.getBoolean(false)) {
            controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0)
            controller.setRumble(GenericHID.RumbleType.kRightRumble, 1.0)
        }
    }

    override fun end(interrupted: Boolean) {
        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0)
        controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.0)
    }

    override fun isFinished(): Boolean {
        return HighGoalVisionNT.found_target.getBoolean(false)
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
    alertController: XboxController
): Command {
    return SequentialCommandGroup(
        // rumble controller if no vision
        CheckVisionOrRumble(alertController),
        // spin up shooter while turning to target
        TurnToHighGoal(drivetrain).raceWith(
            ShooterSpinUpVision(shooter1, shooter2)
        ),
        // maintain shooter speed and shoot
        ParallelCommandGroup(
            MaintainAngle(drivetrain),
            ShooterFixedVision(shooter1, shooter2),
            ShootBallMotor(shooter1, shooter2, gate, indexer)
        )
    )
}

// Run the shooter and indexer/gate for the default distance.
fun ShootDefaultDistance(
    shooter1: ShooterSubsystem,
    shooter2: ShooterSubsystem,
    gate: BallMotorSubsystem,
    indexer: BallMotorSubsystem,
): Command {
    val speeds = get_shoot_speed_for_distance(Constants.shooterDefaultDist)
    return ParallelCommandGroup(
        DualShooterPID(shooter1, shooter2, { speeds }),
        // run gate + magazine if shooters are running fast enough
        ShootBallMotor(shooter1, shooter2, gate, indexer)
    )
}

