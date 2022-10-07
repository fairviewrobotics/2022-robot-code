import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.commands.HighGoalVision


/**
 * Do nothing if a vision target is found.
 * Otherwise, rumble the given controller. This is mean to make it clear to the driver they need to shoot manually.
 */
class CheckVisionOrRumble(val controller: XboxController) : CommandBase() {
    override fun execute() {
        if (!HighGoalVision.found_target.getBoolean(false)) {
            controller.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0)
            controller.setRumble(GenericHID.RumbleType.kRightRumble, 1.0)
        }
    }

    override fun end(interrupted: Boolean) {
        controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0)
        controller.setRumble(GenericHID.RumbleType.kRightRumble, 0.0)
    }

    override fun isFinished(): Boolean {
        return HighGoalVision.found_target.getBoolean(false)
    }
}