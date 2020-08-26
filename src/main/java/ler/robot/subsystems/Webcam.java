/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import ler.robot.vision.GripPipeline;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem to represent webcam used to track balls for intaking.
 */
public class Webcam extends SubsystemBase {

  private static final int WIDTH = 320;
  private static final int HEIGHT = 240;

  private VisionThread visionThread;

  private double posX = 0.0;
  private double posY = 0.0;

  private final Object imgLock = new Object();

  /**
   * Creates a new Webcam.
   */
  public Webcam() {

    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(WIDTH, HEIGHT);
    //camera.setFPS(30);

    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
      if(!pipeline.findContoursOutput().isEmpty()){
        Rect r = Imgproc.boundingRect(pipeline.findContoursOutput().get(0));
        synchronized(imgLock){
          posX = r.x + (r.width / 2);

          if(posX == r.width / 2){
            System.out.println("MIDDLE");
          } else {
            //turn robot
          }
        }
        System.out.println(posX);

      }
      //System.out.println(pipeline.findContoursOutput());
    });

    visionThread.start();
  }

  /**
   * Get the X offset from the ball.
   * @return X offset in pixels.
   */
  public double getX(){
    return (WIDTH/2) - posX;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
