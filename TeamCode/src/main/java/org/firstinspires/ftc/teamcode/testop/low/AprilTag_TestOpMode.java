// Linear Test Code

package org.firstinspires.ftc.teamcode.testop.low;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.time.Duration;
import java.time.Instant;
import java.util.List;

import org.firstinspires.ftc.teamcode.hardware.IMUHW;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp(name = "AprilTag_TestOpMode", group = "Low")
public class AprilTag_TestOpMode extends OpMode {
    public AprilTagProcessor myAprilTagProcessor;
    @Override
    public void init() {

        AprilTagLibrary myAprilTagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary();

        myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(myAprilTagLibrary)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        TfodProcessor myTfodProcessor;

        myTfodProcessor = new TfodProcessor.Builder()
                .setMaxNumRecognitions(10)
                .setUseObjectTracker(true)
                .setTrackerMaxOverlap((float) 0.2)
                .setTrackerMinSize(16)
                .build();

        VisionPortal myVisionPortal;

        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        List<AprilTagDetection> myAprilTagDetections;  // list of all detections
        AprilTagDetection myAprilTagDetection;         // current detection in for() loop
        int myAprilTagIdCode;                           // ID code of current detection, in for() loop
        String myAprilTagName;
        double myAprilTagSize;

        myAprilTagDetections = myAprilTagProcessor.getDetections();

        for (int i = 0; i < myAprilTagDetections.size(); i++) {
            myAprilTagDetection = myAprilTagDetections.get(i);
            if (myAprilTagDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                myAprilTagIdCode = myAprilTagDetection.id;
                myAprilTagName = myAprilTagDetection.metadata.name;
                myAprilTagSize = myAprilTagDetection.metadata.tagsize;

                double myTagPoseX = myAprilTagDetection.ftcPose.x;
                double myTagPoseY = myAprilTagDetection.ftcPose.y;
                double myTagPoseZ = myAprilTagDetection.ftcPose.z;
                double myTagPosePitch = myAprilTagDetection.ftcPose.pitch;
                double myTagPoseRoll = myAprilTagDetection.ftcPose.roll;
                double myTagPoseYaw = myAprilTagDetection.ftcPose.yaw;

                double myTagPoseRange = myAprilTagDetection.ftcPose.range;
                double myTagPoseBearing = myAprilTagDetection.ftcPose.bearing;
                double myTagPoseElevation = myAprilTagDetection.ftcPose.elevation;

                telemetry.addData("IdCode", myAprilTagIdCode);
                telemetry.addData("Size\n", myAprilTagSize);
                telemetry.addData("PoseX", myTagPoseX);
                telemetry.addData("PoseY", myTagPoseY);
                telemetry.addData("PoseZ", myTagPoseZ);
                telemetry.addData("Pitch", myTagPosePitch);
                telemetry.addData("Yaw", myTagPoseYaw);
                telemetry.addData("Roll", myTagPoseRoll);
                telemetry.addData("Range", myTagPoseRange);
                telemetry.addData("Bearing", myTagPoseBearing);
                telemetry.addData("Elevation", myTagPoseElevation);
            }
        }

        telemetry.update();
    }

}
