package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.XboxController.Button.*
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.PneumaticsModuleType

import com.revrobotics.*
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.subsystems.SolenoidSubsystem
import frc.robot.subsystems.WinchSubsystem


class ClimberTestContainer {
    // Hardware and subsystem initialization
    val controller0 = XboxController(0)

    // climb
    val winchMotor = CANSparkMax(Constants.climbWinchID, CANSparkMaxLowLevel.MotorType.kBrushless)
    val winch = WinchSubsystem(winchMotor, DigitalInput(0), DigitalInput(1))
    val climbSolenoid = DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.climbSolenoidLeftID.first,Constants.climbSolenoidLeftID.second)
    val climbPneumatics = SolenoidSubsystem(climbSolenoid)


}