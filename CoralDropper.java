package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Configs;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class CoralDropper extends SubsystemBase {
    public static final class CoralDropperConstants {
        public static final int kLeftMotorCanId = 11;
        public static final int kRightMotorCanId = 12;
        public static final int kCurrentLimit = 30;
        public static final double kMaxSpeed = 0.25;
        public static final double kRotationSpeed = 0.3;
        public static final double kRotationsToExecute = 0.50;
    }
    
    private final SparkMax m_leftCoralMotor;
    private final SparkMax m_rightCoralMotor;
    private final RelativeEncoder m_leftCoralEncoder;
    private boolean m_isRotating = false;
    private double m_targetRotations = 0;
    private SparkClosedLoopController m_leftClosedLoopController;

    /** Creates a new CoralDropper subsystem. */
    public CoralDropper() {
        // Initialize motors with CAN IDs
        m_leftCoralMotor = new SparkMax(CoralDropperConstants.kLeftMotorCanId, MotorType.kBrushless);
        m_rightCoralMotor = new SparkMax(CoralDropperConstants.kRightMotorCanId, MotorType.kBrushless);

        configureMotors();
    
        m_leftCoralEncoder = m_leftCoralMotor.getEncoder();
   
        m_leftClosedLoopController = m_leftCoralMotor.getClosedLoopController();
        
        resetEncoders();
    }

    private void configureMotors() {
        // Create configuration for the left motor
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .inverted(true);
        
        // Configure the PID settings
        leftConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.1, 0, 0)
                .outputRange(-1.0, 1.0);
        m_leftCoralMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .inverted(false);  

        m_rightCoralMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void resetEncoders() {
        m_leftCoralEncoder.setPosition(0);
    }
    
    public void setMotorsFromTriggers(double leftTrigger, double rightTrigger) {
        // Don't accept trigger inputs if we're in the middle of a rotation
        if (m_isRotating) {
            return;
        }

        m_leftCoralMotor.set(leftTrigger * CoralDropperConstants.kMaxSpeed);
        m_rightCoralMotor.set(rightTrigger * CoralDropperConstants.kMaxSpeed);
    }

    /**
     * Starts a rotation of the motors for the specified number of rotations.
     * 
     * @param rotations Number of rotations to complete
     */
    public void startRotation(double rotations) {
        if (m_isRotating) {
            return; // Already rotating
        }

        m_isRotating = true;
        m_targetRotations = rotations;
        resetEncoders();

        m_leftClosedLoopController.setReference(rotations, ControlType.kPosition);
        // Start motors at rotation speed
        m_leftCoralMotor.set(CoralDropperConstants.kRotationSpeed);
        m_rightCoralMotor.set(CoralDropperConstants.kRotationSpeed);
    }

    public void stopMotors() {
        m_leftCoralMotor.set(0);
        m_rightCoralMotor.set(0);
        m_isRotating = false;
    }
 
    public void intake() {
        m_leftCoralMotor.set(0.50);  
        m_rightCoralMotor.set(0.50);  
    }
    
    public void fullSpeed() {
        m_leftCoralMotor.set(0.5);  
        m_rightCoralMotor.set(0.5);  
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (m_isRotating) {
            // Check if rotation is complete based on left encoder position only
            if (Math.abs(m_leftCoralEncoder.getPosition()) >= m_targetRotations) {
                stopMotors();
            }
        }
    }
}
