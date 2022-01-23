package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.ADXRS450_Gyro
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj.simulation.EncoderSim
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.IEEErem

/**
 * Subsystem for interacting with the drivetrain. Controls drivetrain motors + encoders, and the gyroscope. Also handles simulation for those things.
 */
class DrivetrainSubsystem(val leftMotors: MotorControllerGroup, val rightMotors: MotorControllerGroup, val leftEncoder: Encoder, val rightEncoder: Encoder, val gyro: ADXRS450_Gyro) : SubsystemBase() {
    val encoderDistancePerPulse = 1.0
    val gyroReversed = false

    val drive = DifferentialDrive(leftMotors, rightMotors)
    val odometry: DifferentialDriveOdometry

    var driveSim: DifferentialDrivetrainSim? = null
    val leftEncoderSim: EncoderSim
    val rightEncoderSim: EncoderSim
    val fieldSim: Field2d
    val gyroSim: ADXRS450_GyroSim

    init {
        rightMotors.inverted = true

        leftEncoder.distancePerPulse = encoderDistancePerPulse
        rightEncoder.distancePerPulse = encoderDistancePerPulse

        resetEncoders()
        odometry = DifferentialDriveOdometry(Rotation2d.fromDegrees(heading))
        
        if (RobotBase.isSimulation()) {
            driveSim = DifferentialDrivetrainSim.createKitbotSim(
                DifferentialDrivetrainSim.KitbotMotor.kDoubleNEOPerSide,
                DifferentialDrivetrainSim.KitbotGearing.k10p71,
                DifferentialDrivetrainSim.KitbotWheelSize.kSixInch,
                null
            )
        }

        leftEncoderSim = EncoderSim(leftEncoder)
        rightEncoderSim = EncoderSim(rightEncoder)
        gyroSim = ADXRS450_GyroSim(gyro)

        fieldSim = Field2d()
        SmartDashboard.putData("Field", fieldSim)
    }

    override fun periodic() {
        odometry.update(
            Rotation2d.fromDegrees(heading),
            leftEncoder.distance,
            rightEncoder.distance)

        fieldSim.robotPose = pose
    }

    override fun simulationPeriodic() {
        driveSim?.let {
            it.setInputs(
                leftMotors.get() * RobotController.getBatteryVoltage(),
                rightMotors.get() * RobotController.getBatteryVoltage())
            it.update(.020)

            leftEncoderSim.distance = it.leftPositionMeters
            leftEncoderSim.rate = it.leftVelocityMetersPerSecond
            rightEncoderSim.distance = it.rightPositionMeters
            rightEncoderSim.rate = it.rightVelocityMetersPerSecond

            gyroSim.setAngle(it.heading.degrees)
        }
    }

    fun tankDriveVolts(leftVolts: Double, rightVolts: Double) {
        leftMotors.setVoltage(leftVolts)
        rightMotors.setVoltage(rightVolts)
    }

    fun setMaxOutput(maxOutput: Double) {
        drive.setMaxOutput(maxOutput)
    }

    /** Angular velocity, radians per second. */
    val angularVelocity: Double get() = Units.degreesToRadians(gyro.rate) * (if (gyroReversed) { -1.0 } else { 1.0 })
    /** Heading, in radians. */
    val heading: Double get() = Units.degreesToRadians(gyro.angle.IEEErem(360.0) * (if (gyroReversed) { -1.0 } else { 1.0 }))

    // MARK: Diagnostic-type variables
    val averageEncoderDistance: Double get() = (leftEncoder.distance + rightEncoder.distance) / 2.0
    val drawnCurrentAmps: Double? get() = driveSim?.let { return it.currentDrawAmps }
    val pose: Pose2d get() = odometry.poseMeters

    val wheelSpeeds: DifferentialDriveWheelSpeeds get() = DifferentialDriveWheelSpeeds(leftEncoder.rate, rightEncoder.rate)

    // MARK: Diagnostic-type functions
    fun resetEncoders() {
        leftEncoder.reset()
        rightEncoder.reset()
    }

    fun zeroHeading() {
        gyro.reset()
    }

    fun resetOdometry(pose: Pose2d) {
        resetEncoders()
        driveSim?.let { it.pose = pose }
        odometry.resetPosition(pose, Rotation2d.fromDegrees(heading))
    }
}