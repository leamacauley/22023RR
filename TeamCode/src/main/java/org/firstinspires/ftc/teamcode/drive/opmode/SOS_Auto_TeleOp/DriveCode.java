package org.firstinspires.ftc.teamcode.drive.opmode.SOS_Auto_TeleOp;
import org.firstinspires.ftc.teamcode.drive.opmode.MaristBaseRobot2022_Quad;

/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Modified by michaudc 2017
Additional modifications by michaudc 2021
 */

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Config

@TeleOp(name="Strafer States 2023", group="Android Studio")
//@Disabled
public class DriveCode extends OpMode {

    /* Declare OpMode members. */

    MaristBaseRobot2022_Quad robot = new MaristBaseRobot2022_Quad();



    // could also use HardwarePushbotMatrix class.
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    int armPos = 0;

    private double SPEED_CONTROL = 0.6;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");    //

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPos = robot.leftArm.getCurrentPosition();


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftX;
        double leftY;
        double rightX;
        // Strafer Mode
        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y ;
        rightX = gamepad1.right_stick_x ;

        double leftRearPower = (leftY + leftX - rightX);
        double leftFrontPower = (leftY - leftX - rightX);
        double rightRearPower = (leftY - leftX + rightX);
        double rightFrontPower = (leftY + leftX + rightX);

        robot.leftFront.setPower(leftFrontPower * SPEED_CONTROL);
        robot.leftRear.setPower(leftRearPower * SPEED_CONTROL);
        robot.rightFront.setPower(rightFrontPower * SPEED_CONTROL);
        robot.rightRear.setPower(rightRearPower *SPEED_CONTROL);

        // Alternative Method:  Single Paddle on right  (Commented out)
        //left = gamepad1.right_stick_y + gamepad1.right_stick_x;
        //right = gamepad1.right_stick_y - gamepad1.right_stick_x;
        //robot.leftMotor.setPower(left);
        //robot.rightMotor.setPower(right);

        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper) {
            //clawOffset += CLAW_SPEED;
            robot.leftHand.setPosition(0.78); //close
        }
        else if (gamepad1.left_bumper) {
            //clawOffset -= CLAW_SPEED;
            robot.leftHand.setPosition(0.5); //open
        }

        /*
        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.leftHand.setPosition(0.1);
        robot.rightHand.setPosition(0.4);
        */

        // Use gamepad buttons to move the arm up (Y) and down (A)

        //if (gamepad1.y)
        //    robot.rightArm.setPower(robot.ARM_UP_POWER);
        //else if (gamepad1.a)
        //    robot.rightArm.setPower(robot.ARM_DOWN_POWER);
        //else
        //    robot.rightArm.setPower(0.0);




        // Control Arm with Right and Left Triggers
        double armMotorPower = gamepad1.right_trigger - gamepad1.left_trigger;
        // Limit Power to -0.4 to 0.4
        if (armMotorPower > 0.6) { //0.4
            armMotorPower = 0.6;
            armPos += gamepad1.right_trigger * 8;
        }

        if (armMotorPower < -0.6) {
            armMotorPower = -0.6;
            armPos -= gamepad1.left_trigger * 3;
        }


        // Set positions for buttons
        if(gamepad1.a) {  //ground
            armPos = 0;
        }
        if(gamepad1.b) { //low junction
            armPos =1700;
        }
        if(gamepad1.y) { //medium junction
            armPos = 3000; // 2900
        }
        if(gamepad1.x) { //high junction
            armPos = 4200;
        }

        //safety code <3333
        if(armPos<0){
            armPos = 0;
        }

        if(armPos>4200){
            armPos = 4200;
        }


        if (armPos > 2500) {
            SPEED_CONTROL = 0.5;
        }
        else {
            SPEED_CONTROL = 0.6;
        }

        //robot.leftArm.setPower(armMotorPower);
        robot.leftArm.setTargetPosition(armPos);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftArm.setPower(0.7);



        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", clawOffset);

        telemetry.addData("ArmPos", armPos);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
