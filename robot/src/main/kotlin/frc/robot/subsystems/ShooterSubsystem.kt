package frc.robot.subsystems

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

class ShooterSubsystem(val motor: CANSparkMax) : SubsystemBase() {

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

    override fun periodic() {
    }


    /* set a motor voltage */
    fun setSpeed(speed: Double) {
        motor.set(speed)
    }
    
    fun getEncoder() : RelativeEncoder{
        return motor.getEncoder()
    }
        
    /* get current encoder velocity (in rad / s) */
    fun getVelocity(): Double {
        if(!RobotBase.isSimulation()) {
            return Units.degreesToRadians(getEncoder().getVelocity())
        } else {
            return encoderSim?.rate ?: 0.0
        }
    }

    fun setVoltage(volts: Double) {
        motor.setVoltage(volts)
        
    }

    /* set spark to coast. Needed for bang bang */
    fun setCoast(){
        // ALWAYS RUN THIS ONCE BEFORE DOING ANY SORT OF BANG-BANG CONTROL!
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast)
        motor.set(0.0)
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