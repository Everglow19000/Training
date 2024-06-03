package org.EverglowLibrary.Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Arrays;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;


public class CameraSystem {
    private final AprilTagProcessor m_AprilTag;
    private final OpMode m_OpMode;
    private final VisionPortal m_Camera;
    private final TfodProcessor m_Prop;
    public boolean m_IsBack;
    private boolean m_IsRight;

    private static final String TFOD_MODEL_FILE_RED = "/sdcard/FIRST/tflitemodels/RedCubeFile.tflite"; //model_20240130_235607
    private static final String TFOD_MODEL_FILE_BLUE = "/sdcard/FIRST/tflitemodels/BlueCubeFile.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABEL_RED = {
            "RedCube"
    };

    private static final String[] LABEL_BLUE = {
            "BlueCube"
    };

    public enum DetectionLocation{
        LEFT, MIDDLE, RIGHT
    }
    public CameraSystem(OpMode opMode){
        m_IsBack = false;
        m_OpMode = opMode;
        m_Prop = new TfodProcessor
                .Builder()
                .setModelFileName(TFOD_MODEL_FILE_RED)
                .setModelLabels(LABEL_RED)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        m_AprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(m_OpMode.hardwareMap.get(WebcamName.class, "webcam"));
        builder.addProcessor(m_AprilTag);
        builder.addProcessor(m_Prop);
        //builder.addProcessor(m_BlueProp);

        m_Camera =  builder.build();
    }

    public CameraSystem(OpMode opMode, boolean isRedProp, boolean isBack)
    {
        this(opMode, isRedProp);
        m_IsBack = isBack;
        m_IsRight = isRedProp;
    }

    public CameraSystem(OpMode opMode, boolean isRedProp)
    {
        m_OpMode = opMode;
        if(isRedProp)
        {
            m_Prop = new TfodProcessor
                    .Builder()
                    .setModelFileName(TFOD_MODEL_FILE_RED)
                    .setModelLabels(LABEL_RED)
                    .setIsModelTensorFlow2(true)
                    .setIsModelQuantized(true)
                    .setModelInputSize(300)
                    .setModelAspectRatio(16.0 / 9.0)
                    .build();
        }
        else {
            m_Prop = new TfodProcessor
                    .Builder()
                    .setModelFileName(TFOD_MODEL_FILE_BLUE)
                    .setModelLabels(LABEL_BLUE)
                    .setIsModelTensorFlow2(true)
                    .setIsModelQuantized(true)
                    .setModelInputSize(300)
                    .setModelAspectRatio(16.0 / 9.0)
                    .build();
        }

        m_AprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(m_OpMode.hardwareMap.get(WebcamName.class, "webcam"));
        builder.addProcessor(m_AprilTag);
        builder.addProcessor(m_Prop);

        m_Camera =  builder.build();
    }
    public List<AprilTagDetection> DetectAprilTags(){
        return m_AprilTag.getDetections();
    }
    public Dictionary<AprilTagDetection,DetectionLocation> GetDetectionLocation(List<AprilTagDetection> detections){
        if(detections.size() == 0)
            return new Hashtable<AprilTagDetection,DetectionLocation>();

        Dictionary<AprilTagDetection, DetectionLocation> detectionToLocation =
                new Hashtable<AprilTagDetection, DetectionLocation>();

        Dictionary<Double, AprilTagDetection> doubleToDetection = new Hashtable<Double, AprilTagDetection>();

        for (int i = 0; i<detections.size(); i++){
            doubleToDetection.put(detections.get(i).ftcPose.x, detections.get(i));
        }

        double[] posDouble = new double[doubleToDetection.size()]; //the right is the smallest, the left is the highest
        Enumeration<Double> allValues = doubleToDetection.keys();
        int count =0;
        while (allValues.hasMoreElements()){
            posDouble[count] = allValues.nextElement();
            count++;
        }

        try {
            posDouble = Arrays.stream(posDouble).sorted().toArray();

        }catch (NullPointerException nullEX){
            return null;
        }

        if(posDouble.length == 1) {
            detectionToLocation.put(doubleToDetection.get(posDouble[0]), DetectionLocation.MIDDLE);
            return detectionToLocation;
        }
        for (int i =0; i<doubleToDetection.size(); i++){
            switch (i){
                case 0: {
                    detectionToLocation.put(doubleToDetection.get(posDouble[0]), DetectionLocation.LEFT);
                    break;
                }
                case 1:{
                    detectionToLocation.put(doubleToDetection.get(posDouble[1]), DetectionLocation.MIDDLE);
                    break;
                }
                case 2:{
                    detectionToLocation.put(doubleToDetection.get(posDouble[2]), DetectionLocation.RIGHT);
                }
            }
        }

        return detectionToLocation;
    }

    public Dictionary<AprilTagDetection,DetectionLocation> GetDetectionLocation() {
        return GetDetectionLocation(DetectAprilTags());
    }

    public List<Recognition> DetectProp(){
        return m_Prop.getRecognitions();
    }

    public Recognition getClosetRecognition(List<Recognition> recognitions){
        Recognition lowestRec = recognitions.get(0);
        double lowY = 10000, y;
        for (Recognition rec:recognitions) {
            y = ConvertInchToCm(ConvertRecognitionToPos(rec, false));
            if(y < lowY)
            {
                lowestRec = rec;
                lowY = y;
            }
        }
        return lowestRec;
    }

    public DetectionLocation DetectAndFindPropLocation(){
        //takes time!
        DetectionLocation detectionLocation = null;
        long start = System.currentTimeMillis();
        final int TIME_WAIT_MILL = 3 * 1000;
        boolean isTimeEnd = false;
        List<Recognition> recognitions;

        m_OpMode.telemetry.addLine("start detecting...");
        m_OpMode.telemetry.update();
        while (detectionLocation == null && !isTimeEnd){
            recognitions = DetectProp();
            if(recognitions.size() != 0){
                detectionLocation = RecognitionToLocation(recognitions.get(0));
            }
            isTimeEnd = (System.currentTimeMillis() - start) >= TIME_WAIT_MILL;
        }

        if(isTimeEnd)
            detectionLocation = DetectionLocation.RIGHT;

        m_OpMode.telemetry.addData("finished detection; point:", detectionLocation.name());

        return detectionLocation;
    }

    public DetectionLocation RecognitionToLocation(Recognition recognition){
        double MIN_RIGHT_LOCATION = 300;
        double MIN_LEFT_LOCATION = 500;
        if(recognition == null){
            return DetectionLocation.LEFT;
        }

        double distanceFromCameraX;
        distanceFromCameraX = ConvertRecognitionToPos(recognition, true);

        if((m_IsBack && !m_IsRight)||(!m_IsBack && m_IsRight)){
            m_OpMode.telemetry.addLine("In the Back");
            m_OpMode.telemetry.addData("distance", distanceFromCameraX);
            MIN_RIGHT_LOCATION = 180;
            if(distanceFromCameraX <= MIN_LEFT_LOCATION && distanceFromCameraX >= MIN_RIGHT_LOCATION)
                return DetectionLocation.MIDDLE;
            else if (distanceFromCameraX <= MIN_RIGHT_LOCATION)
                return DetectionLocation.LEFT;
            else
                return DetectionLocation.RIGHT;
        }
        else {
            if (distanceFromCameraX <= MIN_RIGHT_LOCATION)
                return DetectionLocation.LEFT;
            else
                return DetectionLocation.MIDDLE;
        }

    }

    public double ConvertRecognitionToPos(Recognition rec, boolean isX){
        if(isX)
            return (rec.getLeft() + rec.getRight()) / 2 ;
        else
            return (rec.getTop()  + rec.getBottom()) / 2;
    }

    public static double ConvertInchToCm(double inch){
        return inch*2.54;
    }
    public void CloseCamera() { m_Camera.close();}
}
