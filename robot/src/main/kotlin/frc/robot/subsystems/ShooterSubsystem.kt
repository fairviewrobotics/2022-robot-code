package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.revrobotics.*
import com.revrobotics.REVPhysicsSim
import edu.wpi.first.math.system.plant.*
import edu.wpi.first.math.*
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.controller.*
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Encoder
import frc.robot.Constants
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.EncoderSim
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

// subsystem for shooter (hypothetical for now)

class ShooterSubsystem(val flywheelMotor: CANSparkMax) : SubsystemBase() {
    val encoder = flywheelMotor.getEncoder()

    // the value passed into getNEO represents the number of motors in the gearbox.
    val flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), Constants.shooterInertia, Constants.shooterGearing)
    // kalman filter
    val observer = KalmanFilter(
            Nat.N1(),
            Nat.N1(),
            flywheelPlant,
            VecBuilder.fill(Constants.shooterStateStdev), // state stdev
            VecBuilder.fill(Constants.shooterEncStdev), // encoder stdev
            Constants.refreshInterval // refresh rate
    )

    var lqrEnabled = false
    val controller = LinearQuadraticRegulator(
            flywheelPlant,
            VecBuilder.fill(Constants.shooterQ),
            VecBuilder.fill(Constants.shooterR),
            Constants.refreshInterval
    )

    val loop = LinearSystemLoop(
            flywheelPlant,
            controller,
            observer,
            Constants.shooterVolts,
            Constants.refreshInterval
    )

    var flywheelSim: FlywheelSim? = null
    var encoderSim: EncoderSim? = null

    init {
        if(RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(flywheelMotor, DCMotor.getNEO(1))

            flywheelSim = FlywheelSim(DCMotor.getNEO(1), 1.0, Constants.shooterInertia)
            // Rev doesn't allow its encoders to be simulated, so we create a fake digital encoder and manipulate it
            encoderSim = EncoderSim(Encoder(0, 1))
        }
    }

    override fun periodic() {
        loop.correct(VecBuilder.fill(Units.degreesToRadians(encoder.getVelocity()))) // may not have right units
        loop.predict(Constants.refreshInterval)
        val nextVoltage = loop.getU(0)
        if(lqrEnabled) {
            flywheelMotor.setVoltage(nextVoltage)
        }
    }

    fun LQROn(velocity: Double){
        lqrEnabled = true
        loop.setNextR(VecBuilder.fill(velocity))
    }

    fun LQROff(){
        lqrEnabled = false
        loop.setNextR(VecBuilder.fill(0.0))
    }

    /* get current encoder velocity (in rad / s) */
    fun getVelocity(): Double {
        if(!RobotBase.isSimulation()) {
            return Units.degreesToRadians(encoder.velocity)
        } else {
            return encoderSim?.rate ?: 0.0
        }
    }

    /* set a motor speed */
    fun setSpeed(speed: Double) {
        flywheelMotor.setVoltage(speed * 12)
    }

    fun setVoltage(volts: Double) {
        flywheelMotor.setVoltage(volts)
    }

    /* set spark to coast. Needed for bang bang */
    fun setCoast(){
        flywheelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast)
    }

    fun isCoast(): Boolean{
        return flywheelMotor.getIdleMode() == CANSparkMax.IdleMode.kCoast
    }

    override fun simulationPeriodic() {
        REVPhysicsSim.getInstance().run()

        flywheelSim?.let { flywheelSim ->
            flywheelSim.setInputVoltage(Math.min(flywheelMotor.appliedOutput, 12.0))
            flywheelSim.update(Constants.refreshInterval)
            encoderSim?.rate = flywheelSim.angularVelocityRadPerSec
        }
    }

}