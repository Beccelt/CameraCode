
package org.usfirst.frc.team20.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.ParticleFilterCriteria;
import com.ni.vision.NIVision.ParticleFilterCriteria2;
import com.ni.vision.NIVision.*;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.image.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    String autoSelected;
    SendableChooser chooser;
    int count = 0, session;
    double currentTime;
    //Image frame;
    NIVision.Rect rect;
    RGBImage frame;
    BinaryImage thresholdimage;
    BinaryImage bigObjectsImage;
    BinaryImage convexHullImage;
    BinaryImage filteredImage;
    
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);
        
        //CameraServer cams = CameraServer.getInstance();
        //cams.startAutomaticCapture("cam1");
        
        rect = new NIVision.Rect(10, 10, 100, 100);
        try {
			frame = new RGBImage();
		} catch (NIVisionException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
        //frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
        session = NIVision.IMAQdxOpenCamera("cam1", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
        NIVision.IMAQdxConfigureGrab(session);
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    public void autonomousInit() {
    	autoSelected = (String) chooser.getSelected();
//		autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	switch(autoSelected) {
    	case customAuto:
        //Put custom auto code here   
            break;
    	case defaultAuto:
    	default:
    	//Put default auto code here
            break;
    	}
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
        currentTime = Timer.getMatchTime();
        if(count > 3 ){
        	NIVision.IMAQdxGrab(session, frame.image, 1);
        	try {
				thresholdimage = frame.thresholdHSV(78, 100, 149, 255, 142, 255);
				//bigObjectsImage = thresholdimage.removeSmallObjects(false, 2);
				System.out.println("Threshold Height: " + thresholdimage.getHeight());
				System.out.println("Threshold Width: " + thresholdimage.getWidth());
				convexHullImage = thresholdimage.convexHull(false);
				ParticleFilterCriteria2[] criteria = new ParticleFilterCriteria2[1];
				criteria[0]= new NIVision.ParticleFilterCriteria2(
						NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA,25.0,100.0,0,0);
				filteredImage = convexHullImage.particleFilter(criteria);
				ParticleAnalysisReport[] reports = filteredImage.getOrderedParticleAnalysisReports();
				for(int i = 0; i < reports.length; i++){
					Rect r = new Rect(reports[i].boundingRectLeft, reports[i].boundingRectTop,
							reports[i].boundingRectWidth, reports[i].boundingRectHeight);
					NIVision.imaqDrawShapeOnImage(frame.image, frame.image, r,DrawMode.DRAW_VALUE , ShapeMode.SHAPE_RECT, 0f);
				}
				
				filteredImage.particleFilter(criteria);
			} catch (NIVisionException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
        	
//        	try {
//				bigObjectsImage = thresholdimage.removeSmallObjects(false, 2);
//			} catch (NIVisionException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
        	
//        	try {
//				BinaryImage convexHullImage = bigObjectsImage.convexHull(false);
//			} catch (NIVisionException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
        	
//        	try {
//        		BinaryImage filteredImage = convexHullImage.particleFilter(criteria);
//        	} catch (NIVisionException e){
//        		e.printStackTrace();
//        	}
        	//NIVision.imaqDrawShapeOnImage(frame.image, frame.image, rect,DrawMode.DRAW_VALUE , ShapeMode.SHAPE_RECT, 125f);
        	//CameraServer.getInstance().setImage(thresholdimage.image);
        	CameraServer.getInstance().setImage(frame.image);
        	count = 0;
        }
        count += 1; 
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    public void operatorControl(){
    	NIVision.IMAQdxStartAcquisition(session);
    	
    	CameraServer.getInstance().setImage(thresholdimage.image);
    }
    
}
