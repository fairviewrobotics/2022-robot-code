package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.kauailabs.navx.frc.AHRS
import com.revrobotics.CANSparkMax
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
import edu.wpi.first.wpilibj.motorcontrol.MotorController
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj.simulation.EncoderSim
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.commands.MotorTest
import kotlin.math.IEEErem

abstract class DrivetrainSubsystem : SubsystemBase() {
    abstract val leftMotors: MotorControllerGroup
    abstract val rightMotors: MotorControllerGroup
    abstract val gyro: AHRS

    abstract fun tankDriveVolts(leftVolts: Double, rightVolts: Double)
    abstract fun setMaxOutput(maxOutput: Double)

    abstract val angularVelocity: Double
    abstract val heading: Double
    abstract val wheelSpeeds: DifferentialDriveWheelSpeeds

    abstract fun resetEncoders()
    abstract fun zeroHeading()
    abstract fun resetOdometry(pose: Pose2d)
}

class CANSparkMaxDrivetrainSubsystem(
    motorLF: CANSparkMax,
    motorLB: CANSparkMax,
    motorRF: CANSparkMax,
    motorRB: CANSparkMax,
    override val gyro: AHRS) : DrivetrainSubsystem() {
    override val leftMotors = MotorControllerGroup(motorLF, motorLB)
    override val rightMotors = MotorControllerGroup(motorRF, motorRB)

    val leftEncoder = motorLF.encoder
    val rightEncoder = motorRF.encoder

    val drive = DifferentialDrive(leftMotors, rightMotors)
    val odometry: DifferentialDriveOdometry

    init {
        rightMotors.inverted = true
        resetEncoders()
        odometry = DifferentialDriveOdometry(Rotation2d.fromDegrees(heading))
    }

    override fun periodic() {
        odometry.update(
            Rotation2d.fromDegrees(heading),
            leftEncoder.position,
            rightEncoder.position)
    }

    override fun tankDriveVolts(leftVolts: Double, rightVolts: Double) {
        leftMotors.setVoltage(leftVolts)
        rightMotors.setVoltage(rightVolts)
        drive.feed()
    }

    override fun setMaxOutput(maxOutput: Double) {
        drive.setMaxOutput(maxOutput)
    }

    /** Angular velocity, radians per second. */
    override val angularVelocity: Double get() = Units.degreesToRadians(gyro.rate)
    /** Heading, in radians. */
    override val heading: Double get() = Units.degreesToRadians(gyro.angle.IEEErem(360.0))

    // MARK: Diagnostic-type variables
    val averageEncoderDistance: Double get() = (leftEncoder.position + rightEncoder.position) / 2.0
    val pose: Pose2d get() = odometry.poseMeters

    fun rotationsPerMinuteToMetersPerSecond(velocityRPM: Double, wheelDiameterMeters: Double): Double {
        return velocityRPM * (1.0 / 60.0) * (wheelDiameterMeters * Math.PI / 1.0)
    }

    override val wheelSpeeds get() = DifferentialDriveWheelSpeeds(
        rotationsPerMinuteToMetersPerSecond(leftEncoder.velocity / 10.75, Units.inchesToMeters(6.0)),
        -rotationsPerMinuteToMetersPerSecond(rightEncoder.velocity /* TODO: Fix rate, this is in pulses */ / 10.75, Units.inchesToMeters(6.0))
    )

    // MARK: Diagnostic-type functions
    override fun resetEncoders() {
        // todo: Reset encoder distance
    }

    override fun zeroHeading() {
        gyro.reset()
    }

    override fun resetOdometry(pose: Pose2d) {
        resetEncoders()
        odometry.resetPosition(pose, Rotation2d.fromDegrees(heading))
    }
}

class TalonSRXDrivetrainSubsystem(
    motorLF: WPI_TalonSRX,
    motorLB: WPI_TalonSRX,
    motorRF: WPI_TalonSRX,
    motorRB: WPI_TalonSRX,
    override val gyro: AHRS,
    leftEncoderPortA: Int,
    leftEncoderPortB: Int,
    rightEncoderPortA: Int,
    rightEncoderPortB: Int,
    pulsesPerRevolution: Double) : DrivetrainSubsystem() {
    override val leftMotors = MotorControllerGroup(motorLF, motorLB)
    override val rightMotors = MotorControllerGroup(motorRF, motorRB)

    val leftEncoder = Encoder(leftEncoderPortA, leftEncoderPortB)
    val rightEncoder = Encoder(rightEncoderPortA, rightEncoderPortB)

    val drive = DifferentialDrive(leftMotors, rightMotors)
    val odometry: DifferentialDriveOdometry

    init {
        rightMotors.inverted = true
        resetEncoders()
        odometry = DifferentialDriveOdometry(Rotation2d.fromDegrees(heading))

        leftEncoder.distancePerPulse = pulsesPerRevolution
        rightEncoder.distancePerPulse = pulsesPerRevolution
    }

    override fun periodic() {
        odometry.update(
            Rotation2d.fromDegrees(heading),
            leftEncoder.distance,
            rightEncoder.distance)
    }

   override fun tankDriveVolts(leftVolts: Double, rightVolts: Double) {
        leftMotors.setVoltage(leftVolts)
        rightMotors.setVoltage(rightVolts)
        drive.feed()
    }

    override fun setMaxOutput(maxOutput: Double) {
        drive.setMaxOutput(maxOutput)
    }

    /** Angular velocity, radians per second. */
    override val angularVelocity: Double get() = Units.degreesToRadians(gyro.rate)
    /** Heading, in radians. */
    override val heading: Double get() = Units.degreesToRadians(gyro.angle.IEEErem(360.0))

    // MARK: Diagnostic-type variables
    val averageEncoderDistance: Double get() = (leftEncoder.distance + rightEncoder.distance) / 2.0
    val pose: Pose2d get() = odometry.poseMeters

    fun rotationsPerMinuteToMetersPerSecond(velocityRPM: Double, wheelDiameterMeters: Double): Double {
        return velocityRPM * (1.0 / 60.0) * (wheelDiameterMeters * Math.PI / 1.0)
    }

    override val wheelSpeeds get() = DifferentialDriveWheelSpeeds(
        rotationsPerMinuteToMetersPerSecond(leftEncoder.rate / 10.75, Units.inchesToMeters(6.0)),
        -rotationsPerMinuteToMetersPerSecond(rightEncoder.rate /* TODO: Fix rate, this is in pulses */ / 10.75, Units.inchesToMeters(6.0))
    )

    // MARK: Diagnostic-type functions
    override fun resetEncoders() {
        // todo: Reset encoder distance
    }

    override fun zeroHeading() {
        gyro.reset()
    }

    override fun resetOdometry(pose: Pose2d) {
        resetEncoders()
        odometry.resetPosition(pose, Rotation2d.fromDegrees(heading))
    }
}