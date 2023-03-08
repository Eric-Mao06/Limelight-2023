package frc.robot.subsystems;


import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Subsystem610;
import static frc.robot.Constants.Vision.*;

public class Vision extends Subsystem610 {
    static Vision s_visionInst;
    static Drivetrain m_drivetrain;
    private int m_tv;
    static boolean s_isAimed;
    private int m_ledMode, m_camMode;
    NetworkTable m_networkTable;
    PIDController m_pid;

    /**
     * Singleton class to get Vision instance
     * @return Vision instance
     */
    public static Vision getInstance() {
        if (s_visionInst == null)
            s_visionInst = new Vision();
        return s_visionInst;
    }

    private Vision() {
        super("Limelight");
        
        m_drivetrain = Drivetrain.getInstance();
        m_ledMode = m_camMode = 1;
        m_pid = new PIDController(VAL_KP, VAL_KI, VAL_KD);

        // driveTab.add(new HttpCamera("limelight", "http://10.6.10.11:5800/stream.mjpg"))
        //     .withWidget(BuiltInWidgets.kCameraStream)
        //     .withPosition(0, 0)
        //     .withSize(4, 4);
    }

    /**
     * @return The current Limelight mode
     */
    public int getCamMode() {
        return m_camMode;
    }

    /**
     * @return
     */
    public PIDController getPID() {
        return m_pid;
    }

    /**
     * @return The current Limelight LED mode
     */
    public int getLedMode() {
        return m_ledMode;
    }

    /**
     * Sets the Limelight camera mode
     * @param m_camMode Desired camera mode
     */
    public void setCamMode(int m_camMode) {
        this.m_camMode = m_camMode;
        m_networkTable.getEntry("camMode").setNumber(getCamMode());
    }

    /**
     * Sets the Limelight LED mode
     * @param m_ledMode Desired LED mode
     */
    public void setLedMode(int m_ledMode) {
        this.m_ledMode = m_ledMode;
        m_networkTable.getEntry("ledMode").setNumber(getLedMode());
    }

    /**
     * Fetch the horizontal offset from crosshair to target (tx)
     * @return tx
     */
    public double calcTx(){ 
        // ternary operator to return the horizontal angle if a valid target is detected
        return m_tv == 0 ? 0 : m_networkTable.getEntry("tx").getDouble(0.0);
    }

    /**
     * Fetch the vertical offset from crosshair to target (ty)
     * @return ty
     */
    public double calcTy(){
        return m_tv == 0 ? 0 : m_networkTable.getEntry("ty").getDouble(0.0);
    }


    /**
     * hiiiiiiiiii! This is Eric
     * @return meep moop :D
     */
    public double[] meep_moop(){
        return m_networkTable.getEntry("botpose").getDoubleArray(new double[6]);
    }

    /**
     * @return The distance from the Limelight to the target
     */
    public double calcDistance(){
        return m_tv == 0 ? 0 : 209.0 / Math.tan(Math.toRadians(21 + calcTy()));
    }

    public void writeDashboard(){
        SmartDashboard.putNumber("tx", Math.round(calcTx() * 1e5) / 1e5);
        SmartDashboard.putNumber("distance", Math.round(calcDistance() * 1e5) / 1e5);
    }

    public boolean checkAim(){
        return Vision.s_isAimed = Math.abs(calcTx()) < 2 && m_tv > 0;
    }

    /**
     * Use WPILib PID to move the robot to the desired direction
     */
    public void aim() {
        m_pid.setTolerance(1.4, 0);
        m_drivetrain.setLeft(-m_pid.calculate(calcTx(), 0)*0.3);
        m_drivetrain.setRight(m_pid.calculate(calcTx(), 0)*0.3);
    }

    @Override
    public void periodic() {
        m_networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_tv = (int) m_networkTable.getEntry("tv").getDouble(0.0);
        writeDashboard();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
    }

    /**
     * Adds necessary info to our custom DriveTeam tab on Shuffleboard
     */
    @Override
    public void addToDriveTab(ShuffleboardTab tab) {
        tab.add(new HttpCamera("limelight", "http://10.6.10.11:5800/stream.mjpg"))
            .withWidget(BuiltInWidgets.kCameraStream)
            .withPosition(0, 0)
            .withSize(6, 4);
    }

}