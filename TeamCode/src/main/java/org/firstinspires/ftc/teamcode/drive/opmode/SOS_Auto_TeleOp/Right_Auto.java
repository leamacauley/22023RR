package org.firstinspires.ftc.teamcode.drive.opmode.SOS_Auto_TeleOp;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.teamcode.drive.opmode.MaristBaseRobot2022_Quad;

import android.sax.StartElementListener;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.MaristBaseRobot_Arms2022;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Config

public class Right_Auto extends LinearOpMode {
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initialize the Arms and Servo System
        MaristBaseRobot_Arms2022 robot = new MaristBaseRobot_Arms2022();
        robot.init(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();
        //0.5 (open), 0.85 (close)

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        Trajectory first = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(0, -5))
                .build();
        Trajectory second = drive.trajectoryBuilder(first.end())
                .lineTo(new Vector2d(-58, -5))
                .build();
        /*
        Trajectory third = drive.trajectoryBuilder(second.end())
                .lineTo(new Vector2d(-56, -20))
                .build();

         */
        Trajectory thirdFor = drive.trajectoryBuilder(second.end())
                .lineTo(new Vector2d(-48, -20))
                .build();
        Trajectory thirdBack = drive.trajectoryBuilder(thirdFor.end())
                .lineTo(new Vector2d(-52, -20))
                .build();
        Trajectory fourth = drive.trajectoryBuilder((thirdBack.end()))
                .lineToSplineHeading(new Pose2d(-56, 22.5, Math.toRadians(90))) //95
                .build();
        Trajectory fifth = drive.trajectoryBuilder(fourth.end())
                .lineToSplineHeading((new Pose2d(-46, -20, Math.toRadians(0)))) //was 46, -17
                .build();
        Trajectory fifthBack = drive.trajectoryBuilder(fifth.end())
                .lineTo(new Vector2d(-51, -20))
                .build();
        //color sense drive code
        Trajectory red = drive.trajectoryBuilder(fifthBack.end())
                .lineTo(new Vector2d(-51, -35))
                .build();
        Trajectory blue = drive.trajectoryBuilder(fifthBack.end())
                .lineTo(new Vector2d(-51, -4))
                .build();
        Trajectory yellow = drive.trajectoryBuilder(fifthBack.end())
                .lineTo(new Vector2d(-51, 21))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(first);
        drive.followTrajectory(second);
        //add color sense here during second
        // Run the Loop to read
        float hue = robot.getHue();
        double waitTime = 0; //3

        double count = 1;
        double total = 0;
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < waitTime)) {
            hue = robot.getHue();
            total +=hue;
            count++;
            double average = total/count;
            telemetry.addData ("Average hue", hue);
            telemetry.update();
        }

        //drive.followTrajectory(third);
        setLeftArmPos(3000, 0.8);
        drive.followTrajectory(thirdFor);
        robot.leftHand.setPosition(0.5);
        sleep(500);
        drive.followTrajectory(thirdBack);
        setLeftArmPos(-2200, 0.8); //750 pos
        drive.followTrajectory(fourth);
        robot.leftHand.setPosition(0.85);
        sleep(1000);
        setLeftArmPos(2300, 0.8);
        drive.followTrajectory(fifth);
        robot.leftHand.setPosition(0.5);
        drive.followTrajectory(fifthBack);


        //COLOR SENSOR CODE
        double finalAverage = total / count;

        // Make Decision - Run Functions

        int location = 1;

        if (finalAverage < 65) { // Red
            location = 1;
            telemetry.addData("Location", location);
            telemetry.update();
            drive.followTrajectory(red);
        }

        else if (150 < finalAverage) { // Blue
            location = 2;
            telemetry.addData("Location", location);
            telemetry.update();
            drive.followTrajectory(blue);
        }

        else    { // Yellow
            location = 3;
            telemetry.addData("Location", location);
            telemetry.update();
            drive.followTrajectory(yellow);
        }

        setLeftArmPos(-3100, 0.8);
        // Display output
        telemetry.addData("Final Hue", finalAverage);
        telemetry.update();

        telemetry.addData("Location", location);
        telemetry.update();
    }

//arm control
    public void setLeftArmPos(int pos, double power) {
        DcMotor leftArm = hardwareMap.dcMotor.get("leftarm");
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArm.setTargetPosition(pos);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArm.setPower(power);

        while (leftArm.isBusy() && !isStopRequested()) {
            // Wait for Sequence to complete
        }


    }

}