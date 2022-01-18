package frc.robot.subsystems

// aaaaaaaaaaaa
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.ADXRS450_Gyro
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.interfaces.Gyro
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj.simulation.EncoderSim
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import kotlin.math.IEEErem

object DrivetrainSubsystemConstants {
    val drivetrainPlant = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3)
    val driveGearbox: DCMotor = DCMotor.getNEO(1) // ???????? ASK EDWARD
    const val driveGearing = 8.0
    const val trackWidthMeters = 0.69 // nice
    const val wheelDiameterMeters = 0.15
    const val gyroReversed = false
}

/**
 * DrivetrainSubsystem is the subsystem that controls the robot's tank drive.
 */
class DrivetrainSubsystem(val leftMotors: MotorControllerGroup, val rightMotors: MotorControllerGroup, val leftEncoder: Encoder, val rightEncoder: Encoder, val gyro: ADXRS450_Gyro) : SubsystemBase() {
    val drive = DifferentialDrive(leftMotors, rightMotors)
    val odometry: DifferentialDriveOdometry

    var driveSim: DifferentialDrivetrainSim? = null
    val leftEncoderSim: EncoderSim
    val rightEncoderSim: EncoderSim

    val fieldSim: Field2d
    val gyroSim: ADXRS450_GyroSim

    init {
        rightMotors.inverted = true

        leftEncoder.distancePerPulse = Constants.encoderDistancePerPulse
        rightEncoder.distancePerPulse = Constants.encoderDistancePerPulse

        resetEncoders()
        odometry = DifferentialDriveOdometry(Rotation2d.fromDegrees(heading))
        
        if (RobotBase.isSimulation()) {
            driveSim = DifferentialDrivetrainSim(
                DrivetrainSubsystemConstants.drivetrainPlant,
                DrivetrainSubsystemConstants.driveGearbox,
                DrivetrainSubsystemConstants.driveGearing,
                DrivetrainSubsystemConstants.trackWidthMeters,
                DrivetrainSubsystemConstants.wheelDiameterMeters / 2.0,
                VecBuilder.fill(0.0, 0.0, 0.0001, 0.1, 0.1, 0.005, 0.005))
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
            it.update(0.020)

            leftEncoderSim.distance = it.leftPositionMeters
            leftEncoderSim.rate = it.leftVelocityMetersPerSecond
            rightEncoderSim.distance = it.rightPositionMeters
            rightEncoderSim.rate = it.rightVelocityMetersPerSecond

            gyroSim.setAngle(it.heading.degrees)
        }
    }

    val drawnCurrentAmps: Double? get() = driveSim?.let { return it.currentDrawAmps }
    val pose: Pose2d get() = odometry.poseMeters
    val wheelSpeeds: DifferentialDriveWheelSpeeds get() = DifferentialDriveWheelSpeeds(leftEncoder.rate, rightEncoder.rate)

    fun resetOdometry(pose: Pose2d) {
        resetEncoders()
        driveSim?.let { it.pose = pose }
        odometry.resetPosition(pose, Rotation2d.fromDegrees(heading))
    }

    fun arcadeDrvie(fwd: Double, rot: Double) {
        drive.arcadeDrive(fwd, rot)
    }

    fun tankDriveVolts(leftVolts: Double, rightVolts: Double) {
        leftMotors.setVoltage(leftVolts)
        rightMotors.setVoltage(rightVolts)
    }

    fun resetEncoders() {
        leftEncoder.reset()
        rightEncoder.reset()
    }

    val averageEncoderDistance: Double get() = (leftEncoder.distance + rightEncoder.distance) / 2.0

    fun setMaxOutput(maxOutput: Double) {
        drive.setMaxOutput(maxOutput)
    }

    fun zeroHeading() {
        gyro.reset()
    }

    val heading: Double get() = gyro.angle.IEEErem(360.0) * (if (DrivetrainSubsystemConstants.gyroReversed) { -1.0 } else { 1.0 })
}