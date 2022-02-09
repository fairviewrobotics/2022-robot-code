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
// todo: multiple returns?
// todo: we can also use cansparkmax.follow() to follow another controller but inverted.


enum class WhichMotor {
    LOWER, UPPER
}

class ShooterSubsystem(val flywheelMotorLower: CANSparkMax, val flywheelMotorUpper: CANSparkMax) : SubsystemBase() {
    /*
    val refreshInterval = 0.02 // usually this on most normal robot loops, can be lowered using notifiers
    // the value passed into getNEO represents the number of motors in the gearbox.
    val flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), Constants.shooterInertia, Constants.shooterGearing)
    // kalman filter
    val observer = KalmanFilter(
            Nat.N1(),
            Nat.N1(),
            flywheelPlant,
            VecBuilder.fill(Constants.shooterStateStdev), // state stdev:  how accurate we think model is
            VecBuilder.fill(Constants.shooterEncStdev), // encoder stdev: how accurate we think encoder is
            refreshInterval // refresh rate
    )

    var lqrEnabled = false
    val controller = LinearQuadraticRegulator(
            flywheelPlant,
            VecBuilder.fill(Constants.shooterQ), // tune up to make more conservative (less weight on large errors)
            VecBuilder.fill(Constants.shooterR), // tune down to penalize control effort (less aggressive)
            refreshInterval
    )

    val loop = LinearSystemLoop(
            flywheelPlant,
            controller,
            observer,
            Constants.shooterVolts,
            Constants.refreshInterval
    )
    */

    var flywheelSim: FlywheelSim? = null
    var encoderSim: EncoderSim? = null

    init {
        if(RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(flywheelMotorUpper, DCMotor.getNEO(1))
            REVPhysicsSim.getInstance().addSparkMax(flywheelMotorLower, DCMotor.getNEO(1))

            flywheelSim = FlywheelSim(DCMotor.getNEO(1), 1.0, Constants.shooterInertia) // todo: might not work for 2 motors
            // Rev doesn't allow its encoders to be simulated, so we create a fake digital encoder and manipulate it
            encoderSim = EncoderSim(Encoder(0, 1))
        }
    }

    override fun periodic() {
        /* 
        loop.correct(VecBuilder.fill(Units.degreesToRadians(encoder.getVelocity()))) // may not have right units
        loop.predict(Constants.refreshInterval)
        val nextVoltage = loop.getU(0)
        if(lqrEnabled) {
            flywheelMotor.setVoltage(nextVoltage)
        }
        */
    }

    /* 
    fun setLatencyCompensate(sensorDelay: Double){
        controller.latencyCompensate(flywheelPlant, refreshInterval, sensorDelay)
    }

    fun LQROn(){
        loop.setNextR(VecBuilder.fill(Constants.shooterSpinupRadS));
    }

    fun LQROff(){
        lqrEnabled = false
        loop.setNextR(VecBuilder.fill(0.0))
    }
    */

    /* set a motor speed, with one reversed */
    fun setSpeed(speed: Double) {
        flywheelMotorLower.set(speed * 12.0)
        flywheelMotorUpper.set(speed * -12.0)
    }


    
    fun getEncoder(which: WhichMotor) : RelativeEncoder{
        if (which == WhichMotor.LOWER){
            // lower
            return flywheelMotorLower.getEncoder()
        }
        return flywheelMotorUpper.getEncoder()
    }
        
    /* get current encoder velocity (in rad / s) */
    fun getVelocity(which: WhichMotor): Double {
        if(!RobotBase.isSimulation()) {
            val enc = getEncoder(which)
            return Units.degreesToRadians(enc.getVelocity())
        } else {
            return encoderSim?.rate ?: 0.0
        }
    }

    fun setVoltage(volts: Double, which: WhichMotor) {
        if (which == WhichMotor.LOWER){
            flywheelMotorLower.setVoltage(volts)
        } else{
            flywheelMotorUpper.setVoltage(volts)
        }
        
    }

    /* set spark to coast. Needed for bang bang */
    fun setCoast(){
        // ALWAYS RUN THIS ONCE BEFORE DOING ANY SORT OF BANG-BANG CONTROL!
        flywheelMotorLower.setIdleMode(CANSparkMax.IdleMode.kCoast)
        flywheelMotorLower.set(0.0)
        flywheelMotorUpper.setIdleMode(CANSparkMax.IdleMode.kCoast)
        flywheelMotorUpper.set(0.0)
    }

    fun isCoast(): Boolean{
        /*
            until we can guarantee that the setCoast method works, this is meant as a further layer of validation.
            it's blocking so it's probably not the best to keep this
         */
        return (flywheelMotorLower.getIdleMode() == CANSparkMax.IdleMode.kCoast) && (flywheelMotorUpper.getIdleMode() == CANSparkMax.IdleMode.kCoast)
    }

    override fun simulationPeriodic() {
        REVPhysicsSim.getInstance().run()

        flywheelSim?.let { flywheelSim ->
            flywheelSim.setInputVoltage(Math.min(flywheelMotorLower.appliedOutput, 12.0))
            flywheelSim.setInputVoltage(Math.min(flywheelMotorUpper.appliedOutput, 12.0))
            flywheelSim.update(Constants.refreshInterval)
            encoderSim?.rate = flywheelSim.angularVelocityRadPerSec
        }
    }

}
