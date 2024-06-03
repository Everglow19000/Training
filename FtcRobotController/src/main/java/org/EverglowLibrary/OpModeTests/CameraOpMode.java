package org.EverglowLibrary.OpModeTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.EverglowLibrary.Systems.CameraSystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Dictionary;
import java.util.Enumeration;
import java.util.List;

@TeleOp(name = "Camera AprilTag", group = "test")
public class CameraOpMode extends LinearOpMode {
    @Override
    public void runOpMode(){
        CameraSystem cs = new CameraSystem(this);
        waitForStart();

        Dictionary<AprilTagDetection, CameraSystem.DetectionLocation> DTL;
        AprilTagDetection Key;
        Enumeration<AprilTagDetection> AE;
        List<AprilTagDetection> detections;
        while (opModeIsActive()){
            try {
                detections = cs.DetectAprilTags();
                DTL = cs.GetDetectionLocation(detections);
                AE = DTL.keys();

                if(AE.hasMoreElements())
                    Key = AE.nextElement();
                else
                    continue;

                telemetry.addData("amount places:", DTL.size());
                telemetry.addData("amount total detection:", detections.size());
                while (AE.hasMoreElements()) {
                    telemetry.addData(DTL.get(Key).name() + ":", Key.id);
                    Key = AE.nextElement();
                }
                telemetry.update();
                sleep(200);
            }
            catch (Exception e){
                telemetry.addData("Exception:", e);
                telemetry.update();
                cs.CloseCamera();
                break;
            }
        }
    }
}