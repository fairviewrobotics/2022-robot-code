package frc.robot.subsystems

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
class DrivetrainSubsystemOld(
    val motorLF: CANSparkMax,
    val motorLB: CANSparkMax,
    val motorRF: CANSparkMax,
    val motorRB: CANSparkMax,
    val gyro: AHRS,
    val leftEncoderPortA: Int,
    val leftEncoderPortB: Int,
    val rightEncoderPortA: Int,
    val rightEncoderPortB: Int,
    val pulsesPerRevolution: Double) : SubsystemBase() {
    val leftMotors = MotorControllerGroup(motorLF, motorLB)
    val rightMotors = MotorControllerGroup(motorRF, motorRB)

    val leftEncoder = Encoder(leftEncoderPortA, leftEncoderPortB)
    val rightEncoder = Encoder(rightEncoderPortA, rightEncoderPortB)

    val drive = DifferentialDrive(leftMotors, rightMotors)
    val odometry: DifferentialDriveOdometry

    var driveSim: DifferentialDrivetrainSim? = null
    var leftEncoderSim: EncoderSim? = null
    var rightEncoderSim: EncoderSim? = null
    var fieldSim: Field2d? = null
    var gyroSim: ADXRS450_GyroSim? = null

    init {
        rightMotors.inverted = true
        resetEncoders()
        odometry = DifferentialDriveOdometry(Rotation2d.fromDegrees(heading))

        leftEncoder.distancePerPulse = pulsesPerRevolution
        rightEncoder.distancePerPulse = pulsesPerRevolution

        if (RobotBase.isSimulation()) {
            driveSim = DifferentialDrivetrainSim.createKitbotSim(
                DifferentialDrivetrainSim.KitbotMotor.kDoubleNEOPerSide,
                DifferentialDrivetrainSim.KitbotGearing.k10p71,
                DifferentialDrivetrainSim.KitbotWheelSize.kSixInch,
                null
            )
            leftEncoderSim = EncoderSim(Encoder(0, 1))
            rightEncoderSim = EncoderSim(Encoder(0,1))
            gyroSim = ADXRS450_GyroSim(ADXRS450_Gyro())

            fieldSim = Field2d()
            SmartDashboard.putData("Field", fieldSim)
        }
    }

    override fun periodic() {
        odometry.update(
            Rotation2d.fromDegrees(heading),
            leftEncoder.distance,
            rightEncoder.distance)
    }

    override fun simulationPeriodic() {
        driveSim?.let {
            fieldSim?.robotPose = pose
            it.setInputs(
                leftMotors.get() * RobotController.getBatteryVoltage(),
                rightMotors.get() * RobotController.getBatteryVoltage())
            it.update(.020)

            leftEncoderSim?.distance = it.leftPositionMeters
            leftEncoderSim?.rate = it.leftVelocityMetersPerSecond
            rightEncoderSim?.distance = it.rightPositionMeters
            rightEncoderSim?.rate = it.rightVelocityMetersPerSecond

            gyroSim?.setAngle(it.heading.degrees)
        }
    }

    fun tankDriveVolts(leftVolts: Double, rightVolts: Double) {
        leftMotors.setVoltage(leftVolts)
        rightMotors.setVoltage(rightVolts)
        drive.feed()
    }

    fun setMaxOutput(maxOutput: Double) {
        drive.setMaxOutput(maxOutput)
    }

    /** Angular velocity, radians per second. */
    val angularVelocity: Double get() = Units.degreesToRadians(gyro.rate)
    /** Heading, in radians. */
    val heading: Double get() = Units.degreesToRadians(gyro.angle.IEEErem(360.0))

    // MARK: Diagnostic-type variables
    val averageEncoderDistance: Double get() = (leftEncoder.distance + rightEncoder.distance) / 2.0
    val drawnCurrentAmps: Double? get() = driveSim?.let { return it.currentDrawAmps }
    val pose: Pose2d get() = odometry.poseMeters

    fun rotationsPerMinuteToMetersPerSecond(velocityRPM: Double, wheelDiameterMeters: Double): Double {
        return velocityRPM * (1.0 / 60.0) * (wheelDiameterMeters * Math.PI / 1.0)
    }

    fun getWheelSpeeds(): DifferentialDriveWheelSpeeds {

        return DifferentialDriveWheelSpeeds(
            rotationsPerMinuteToMetersPerSecond(leftEncoder.rate / 10.75, Units.inchesToMeters(6.0)),
            -rotationsPerMinuteToMetersPerSecond(rightEncoder.rate /* TODO: Fix rate, this is in pulses */ / 10.75, Units.inchesToMeters(6.0))
        )
    }


    // MARK: Diagnostic-type functions
    fun resetEncoders() {
        // todo: Reset encoder distance
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