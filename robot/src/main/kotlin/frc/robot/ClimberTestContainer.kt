package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.XboxController.Button.*
import edu.wpi.first.wpilibj2.command.button.JoystickButton

import com.revrobotics.*


class ClimberTestContainer {
    // Hardware and subsystem initialization
    val controller0 = XboxController(0)

    // climb
    val winchMotor = CANSparkMax(Constants.climbWinchID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val winch = WinchSubsystem(winchMotor, DigitalInput(0), DigitalInput(1))
    val climbSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.climbSolenoidLeftID.first,Constants.climbSolenoidLeftID.second)
    val climbPneumatics = SolenoidSubsystem(climbSolenoid)


}