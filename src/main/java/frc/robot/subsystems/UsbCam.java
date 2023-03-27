package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class UsbCam extends SubsystemBase implements Loggable{
    private static UsbCamera usbCameraServer;

    public UsbCam(){
        usbCameraServer = CameraServer.startAutomaticCapture();
        usbCameraServer.setExposureAuto();
        usbCameraServer.setWhiteBalanceAuto();
        Shuffleboard.getTab("GamePieces").add(usbCameraServer).withWidget(BuiltInWidgets.kCameraStream).withPosition(6, 3).withSize(2, 2);
         
    }

    public void setResolution(int width,int height){
        usbCameraServer.setResolution(width, height);
    }

    public void setFPS(int fps){
        usbCameraServer.setFPS(fps);
    }

    @Log.CameraStream(name="USB Camera", tabName="GamePieces", showControls = true, showCrosshairs = true, rotation = "QUARTER_CW", columnIndex = 6,rowIndex = 3 )
    public UsbCamera getCameraFeed(){
        return usbCameraServer;
    }

    @Override
    public void periodic() {
        
    }
    
    
}
