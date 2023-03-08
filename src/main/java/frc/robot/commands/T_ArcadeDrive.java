package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.DriveTrain.*;

public class T_ArcadeDrive extends CommandBase {
    private Drivetrain m_driveInst;
    // private SlewRateLimiter m_turnRate;

    public T_ArcadeDrive(){

        m_driveInst = Drivetrain.getInstance();
        addRequirements(m_driveInst);
    }

    @Override
    public void initialize(){

    }

    /**
     * controls the drivetrain to move around according to the driver's controls
     */
    @Override
    public void execute() {
        double y = MathUtil.applyDeadband(RobotContainer.s_driver.getLeftY(), VAL_DEADBAND);
        double x = MathUtil.applyDeadband(RobotContainer.s_driver.getRightX(), VAL_DEADBAND);

        y = y * y * y;
        x = x * x * x;

        y *= 0.6;
        x *= 0.7;
        double leftSpeed = -y + x;
        double rightSpeed = -y - x;
        m_driveInst.setLeft(leftSpeed);
        m_driveInst.setRight(rightSpeed);
    }
}