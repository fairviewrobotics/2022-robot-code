package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.revrobotics.*
import edu.wpi.first.math.system.plant.*
import edu.wpi.first.math.*
import edu.wpi.first.math.numbers.*
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.controller.*
import edu.wpi.first.math.system.LinearSystemLoop
import frc.robot.Constants

// subsystem for shooter (hypothetical for now)

class ShooterSubsystem(val flywheelMotor: CANSparkMax) : SubsystemBase() {
    val refreshInterval = 0.02 // usually this on most normal robot loops, can be lowered using notifiers
    // the value passed into getNEO represents the number of motors in the gearbox.
    val flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), Constants.shooterInertia, Constants.shooterGearing)
    // kalman filter
    val observer = KalmanFilter(
            Nat.N1(),
            Nat.N1(),
            flywheelPlant,
            VecBuilder.fill(Constants.shooterStateStdev), // state stdev
            VecBuilder.fill(Constants.shooterEncStdev), // encoder stdev
            refreshInterval // refresh rate
    )

    val controller = LinearQuadraticRegulator(
            flywheelPlant,
            VecBuilder.fill(Constants.shooterQ),
            VecBuilder.fill(Constants.shooterR),
            refreshInterval
    )

    val loop = LinearSystemLoop(
            flywheelPlant,
            controller,
            observer,
            Constants.shooterVolts,
            refreshInterval
    )



    val PID = PIDController(Constants.shooterP, Constants.shooterI, Constants.shooterD)
    override fun periodic() {
        val encoder = flywheelMotor.getEncoder()
        loop.correct(VecBuilder.fill(encoder.getVelocity())) // may not have right units
        loop.predict(refreshInterval)
        var nextVoltage = loop.getU(0)
        flywheelMotor.setVoltage(nextVoltage)

    }

    fun LQROn(){
        loop.setNextR(VecBuilder.fill(Constants.shooterSpinupRadS));
    }

    fun LQROff(){
        loop.setNextR(VecBuilder.fill(0.0));
    }

    /* set a motor speed */
    fun setSpeed(speed: Double) {
        flywheelMotor.set(speed)
    }

    fun getPidController(): SparkMaxPIDController {
        return flywheelMotor.getPIDController()
    }

    
    fun getEncoder() : RelativeEncoder{
        return flywheelMotor.getEncoder()
    }

    fun setCoast(){
        // ALWAYS RUN THIS ONCE BEFORE DOING ANY SORT OF BANG-BANG CONTROL!
        flywheelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast)
        flywheelMotor.set(0.0)
    }

    fun isCoast(): Boolean{
        /*
            until we can guarantee that the setCoast method works, this is meant as a further layer of validation.
            it's blocking so it's probably not the best to keep this
         */
        return flywheelMotor.getIdleMode() == CANSparkMax.IdleMode.kCoast
    }

}