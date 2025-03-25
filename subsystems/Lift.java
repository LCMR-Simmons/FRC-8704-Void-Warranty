package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Lift extends SubsystemBase {
    private final RelativeEncoder m_liftEncoder;
    private final SparkClosedLoopController m_closedLoopController;
    private final DigitalInput m_limitSwitch;
    private double m_targetPosition = LiftConstants.kGroundPosition;
    private boolean m_isHoming = false;
    private static final double HOMING_SPEED = -.15; // Negative value to move down
    private final SparkMax leader, follower;

    public Lift() {
        m_limitSwitch = new DigitalInput(LiftConstants.kLimitSwitchPort);
        
        // Initialize motors
        var leaderConfig = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)
            .inverted(false);
     
            leaderConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(LiftConstants.kP, LiftConstants.kI, LiftConstants.kD)
                .outputRange(-1.0, 1.0);
            

        leader = new SparkMax(LiftConstants.kleftLiftMotorCanId, MotorType.kBrushless);
        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        var followerConfig = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30)
            .follow(leader, true);
       
        follower = new SparkMax(LiftConstants.krightLiftMotorCanId, MotorType.kBrushless);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       
        m_liftEncoder = leader.getEncoder();
        m_closedLoopController = leader.getClosedLoopController();
        
        resetEncoder();
        startHoming();
    }

    @Override
    public void periodic() {
        if (m_isHoming) {
            if (isLimitSwitchPressed()) {
                leader.set(0.0);
                resetEncoder();
                m_isHoming = false;
                m_targetPosition = LiftConstants.kGroundPosition;
            } else {
                leader.set(HOMING_SPEED);
            }
        }
    }
    
    // Starts the homing sequence to find the lower limit
    public void startHoming() {
        m_isHoming = true;
    }
    
    public boolean isLimitSwitchPressed() {
        return !m_limitSwitch.get();
    }

    public void setGroundPosition() {
        // If we're not at the limit switch, initiate homing sequence
        if (!isLimitSwitchPressed()) {
            startHoming();
        } else {
            setPosition(LiftConstants.kGroundPosition);
        }
    }

    public void setLowPosition() {
        setPosition(LiftConstants.kLowPosition);
    }

    public void setMediumPosition() {
        setPosition(LiftConstants.kMediumPosition);
    }

    private void setPosition(double position) {
        // Don't change position if we're homing
        if (!m_isHoming) {
            m_targetPosition = position;
            m_closedLoopController.setReference(position, ControlType.kPosition);
        }
    }

    public double getCurrentPosition() {
        return m_liftEncoder.getPosition();
    }

    public boolean isAtPosition() {
        return Math.abs(getCurrentPosition() - m_targetPosition) < 0.5; // Tolerance of 0.5 encoder units
    }

    public void resetEncoder() {
        m_liftEncoder.setPosition(0);
    }
}