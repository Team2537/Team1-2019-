package frc.robot.drive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;



public class DriveSubsystem extends Subsystem {

    private WPI_TalonSRX m_Left;
    private int ticks;
    private int ticksPer20MS;
    private int prevTicks = 0;

    @SuppressWarnings("unused") private Subsystem accelUpdater;

    //Constructor to initialize objects
    public DriveSubsystem() {
        m_Left = new WPI_TalonSRX(3);
        // Clear any non default configuration/settings
        m_Left.configFactoryDefault();
        // A quadrature encoder is connected to the TalonSRX
        m_Left.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        // Reset encoder count to 0
        m_Left.setSelectedSensorPosition(0);

        /* set closed loop gains in slot0 */

        // m_Left.config_kF(.1);
        // m_Left.config_kP(Constants.kPIDLoopIdx, 0.2, Constants.kTimeoutMs);
        // m_Left.config_kI(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);
        // m_Left.config_kD(Constants.kPIDLoopIdx, 0, Constants.kTimeoutMs);
    }

    @Override
    public void initDefaultCommand() {
        //setDefaultCommand(new DriveStraightCommand());
    }

    //gets a value of encoder
    public int getWheelEncoder(){
        return m_Left.getSelectedSensorPosition();
        //return m_Left.getSelectedSensorVelocity();
        
    }

    public int getEncoderSpeed(){
        return m_Left.getSelectedSensorVelocity();
        
    }

    public double getEncoderCurrent(){
        return m_Left.getOutputCurrent();
        
    }

    public double getOutput(){
        return m_Left.get();
    }

    public double EncoderPerMS(){
        ticks = getWheelEncoder();

        ticksPer20MS = ticks - prevTicks;

        prevTicks = ticks;
        return(ticksPer20MS); 
        
        
    }

    public double revPerMin(){
        return ticksPer20MS*(6000.0/1440.0);
    }


    public void setSpeed(double percentOutput){
        m_Left.set(percentOutput);
    }
    public void resetEncoder(){
        m_Left.setSelectedSensorPosition(0);
    }


 //   rev in cm ticks in cm
//284000
}