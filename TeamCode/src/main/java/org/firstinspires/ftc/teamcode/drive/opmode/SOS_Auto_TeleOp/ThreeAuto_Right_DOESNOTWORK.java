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

public class ThreeAuto_Right_DOESNOTWORK extends LinearOpMode {
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
                .lineTo(new Vector2d(-56, -5))
                .build();
        Trajectory third = drive.trajectoryBuilder(second.end())
                .lineTo(new Vector2d(-56, -21))
                .build();
        Trajectory thirdFor = drive.trajectoryBuilder(third.end())
                .lineTo(new Vector2d(-49, -21))
                .build();
        Trajectory thirdBack = drive.trajectoryBuilder(thirdFor.end())
                .lineTo(new Vector2d(-53, -21))
                .build();
        Trajectory fourth = drive.trajectoryBuilder((thirdBack.end()))
                .lineToLinearHeading(new Pose2d(-51, 20, Math.toRadians(82))) //-80  //-56
                .build();
        Trajectory fifth = drive.trajectoryBuilder(fourth.end())
                .lineToLinearHeading((new Pose2d(-46, 4.3, Math.toRadians(0))))
                .build();
        Trajectory sixth = drive.trajectoryBuilder(fifth.end())
                .lineToLinearHeading(new Pose2d(-51, 20, Math.toRadians(82)))//24
                .build();
        Trajectory seventh = drive.trajectoryBuilder(sixth.end())
                .lineToLinearHeading(new Pose2d(-46, 4.3, Math.toRadians(0)))
                .build();
        Trajectory sevBack = drive.trajectoryBuilder((seventh.end()))
                .lineToLinearHeading(new Pose2d(-51, 4.3))
                .build();
        //I don't know if I did something wrong, but it was working for 3 cone but it then stopped.
        //color sense drive code

        Trajectory red = drive.trajectoryBuilder(sevBack.end())//fifthBack
                .lineTo(new Vector2d(-51, -35))
                .build();
        Trajectory blue = drive.trajectoryBuilder(sevBack.end())//fifthBack
                .lineTo(new Vector2d(-51, -4))
                .build();
        Trajectory yellow = drive.trajectoryBuilder(sevBack.end())//fifthBack
                .lineTo(new Vector2d(-51, 21))
                .build();



        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(first);
        drive.followTrajectory(second);
        //add color sense here during second
        // Run the Loop to read
        float hue = robot.getHue();
        double waitTime = 1;

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
//0.5 open, 0.85 close - New Auto
        drive.followTrajectory(third);
        setLeftArmPos(3000, 0.8);
        drive.followTrajectory(thirdFor);
        robot.leftHand.setPosition(0.5);
        sleep(300);
        drive.followTrajectory(thirdBack);
        setLeftArmPos(-2250, 0.8); //800 pos
        drive.followTrajectory(fourth);
        robot.leftHand.setPosition(0.85);
        sleep(1000);
        setLeftArmPos(1200, 0.8); //1600
        drive.followTrajectory(fifth);
        robot.leftHand.setPosition(0.5);
        drive.followTrajectory(sixth);
        setLeftArmPos(-1350, 0.8);
        sleep(1000);
        robot.leftHand.setPosition(0.85);
        sleep(1000 );
        setLeftArmPos(1500, 0.8);
        drive.followTrajectory(seventh);
        robot.leftHand.setPosition(0.5);
        sleep(1000);
        drive.followTrajectory(sevBack);


        //color sense stuffsssss
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

        //setLeftArmPos(-3100, 0.8);
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

