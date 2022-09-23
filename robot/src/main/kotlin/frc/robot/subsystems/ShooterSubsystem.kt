package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.revrobotics.*
import com.revrobotics.REVPhysicsSim
import edu.wpi.first.math.system.plant.*
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Encoder
import frc.robot.Constants
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.simulation.EncoderSim
import edu.wpi.first.wpilibj.simulation.FlywheelSim

abstract class ShooterSubsystem: SubsystemBase() {
    private var targetSpeed: Double = 0.0

    // set speed in [0,1]
    abstract fun setSpeed(speed: Double)
    // set motor voltage in [0,12]
    abstract fun setVoltage(voltage: Double)
    // get velocity in rad / s
    abstract fun getVelocity() : Double


    // set target speed in rad / s.
    // note: this target is only used for getTargetDiff,
    // and should not actually affect the system output.
    // A command may use the target to drive the shooter.
//    fun setTarget(target: Double) {
//        targetSpeed = target
//    }
    // get the difference between the target and current velocity (in rad/s)
    fun getTargetDiff() : Double {
        return targetSpeed - getVelocity()
    }

    abstract fun setCoast(coast: Boolean)
}

class SparkShooterSubsystem(val motor: CANSparkMax, val speedGain: Double) : ShooterSubsystem() {

    var flywheelSim: FlywheelSim? = null
    var encoderSim: EncoderSim? = null

    init {
        if(RobotBase.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1))

            flywheelSim = FlywheelSim(DCMotor.getNEO(1), 1.0, Constants.shooterInertia)
            // Rev doesn't allow its encoders to be simulated, so we create a fake digital encoder and manipulate it
            encoderSim = EncoderSim(Encoder(0, 1))
        }
    }


    /* set a motor speed [0,1] */
    override fun setSpeed(speed: Double) {
        motor.set(speed * speedGain)
    }
        
    /* get current encoder velocity (in rad / s) */
    override fun getVelocity(): Double {
        if(!RobotBase.isSimulation()) {
            return Units.degreesToRadians(motor.getEncoder().getVelocity() / speedGain)
        } else {
            return encoderSim?.rate ?: 0.0
        }
    }

    override fun setVoltage(voltage: Double) {
        motor.setVoltage(voltage * speedGain)
        
    }

    /* set spark to coast. Needed for bang bang */
    override fun setCoast(coast: Boolean){
        motor.set(0.0)
        if(coast) {
            // ALWAYS RUN THIS ONCE BEFORE DOING ANY SORT OF BANG-BANG CONTROL!
            motor.setIdleMode(CANSparkMax.IdleMode.kCoast)
        } else {
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        }
    }

    fun isCoast(): Boolean{
        return motor.getIdleMode() == CANSparkMax.IdleMode.kCoast
    }

    override fun simulationPeriodic() {
        REVPhysicsSim.getInstance().run()

        flywheelSim?.let { flywheelSimUpper ->
            flywheelSimUpper.setInputVoltage(Math.min(motor.appliedOutput, 12.0))

            flywheelSimUpper.update(Constants.refreshInterval)
            
            encoderSim?.rate = flywheelSimUpper.angularVelocityRadPerSec
        }

    }

}

class TalonFXShooterSubsystem(val motor: WPI_TalonFX, val speedGain: Double): ShooterSubsystem() {
    init {
        // set motor to use integrated encoder for pid + feedback
        val config = TalonFXConfiguration()
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor
        motor.configAllSettings(config)
    }

    override fun setSpeed(speed: Double) {
        motor.set(speed * speedGain)
    }

    override fun setVoltage(voltage: Double) {
        motor.setVoltage(voltage * speedGain)
    }

    override fun getVelocity(): Double {
        // getSelectedSensorVelocity gives us a velocity in encoder ticks per 100 ms
        val ticksPer100ms = motor.getSelectedSensorVelocity(0)
        // convert to ticks per second
        val ticksVel = ticksPer100ms * 10.0
        // Talon FX has 2048 encoder ticks per revolution
        val rotPerSecond = ticksVel / 2048.0
        // Convert to rad / s
        val radPerS = rotPerSecond * (2 * Math.PI)

        return radPerS / speedGain
    }

    override fun setCoast(coast: Boolean) {
        motor.set(0.0)
        if(coast) {
            motor.setNeutralMode(NeutralMode.Coast)
        } else {
            motor.setNeutralMode(NeutralMode.Brake)
        }
    }

}
