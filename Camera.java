package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
    private final UsbCamera camera;

    public Camera() {
        // Start automatically capturing from USB camera
        camera = CameraServer.startAutomaticCapture();
        
        // Configure camera settings
        camera.setResolution(320, 240);
        camera.setFPS(20);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}