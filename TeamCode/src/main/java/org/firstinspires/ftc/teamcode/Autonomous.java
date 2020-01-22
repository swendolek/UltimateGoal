package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * Created by Sam W on 9/17/2019.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Autonomous")
public class Autonomous extends OpMode{

    SolidRobot robot; //Robot object

    boolean aPressed = false, bPressed = false;

    Thread auto = new Thread(){
        public void run(){
            //robot.redMainAuto();

            if(robot.autoColor == SolidRobot.color.red){
                if(robot.autoProgram == SolidRobot.program.main){
                    //red main
                    robot.steroids();
                }
                else if(robot.autoProgram == SolidRobot.program.minus){
                    robot.redAutoMinusFoundation();
                }
                else{
                    //red alt
                    robot.redAltAuto();
                }
            }
            else{
                if(robot.autoProgram == SolidRobot.program.main){
                    robot.blueMainAuto();
                }
                else if(robot.autoProgram == SolidRobot.program.minus){
                    robot.blueAutoMinusFoundation();
                }
                else{
                    //blue alt
                    robot.blueAltAuto();
                }
            }

        }
    };

    /**
     * init - runs once after the init button is pressed
     */
    public void init(){

        robot = new SolidRobot(hardwareMap, true);
        robot.FLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     * init_loop - runs continuously after the init button is pressed
     */
    public void init_loop(){

        //Autonomous program selector with the controller

        if(robot.autoColor  == SolidRobot.color.red) telemetry.addData("Color", "Red");
        else telemetry.addData("Color", "Blue");

        if(robot.autoProgram  == SolidRobot.program.main) telemetry.addData("Program", "Default");
        else if(robot.autoProgram == SolidRobot.program.minus) telemetry.addData("Program", "Minus");
        else telemetry.addData("Program", "Safe");

        if(gamepad1.a){
            if(!aPressed){
                aPressed = true;
                if(robot.autoColor  == SolidRobot.color.red){
                    robot.autoColor = SolidRobot.color.blue;
                }
                else{
                    robot.autoColor = SolidRobot.color.red;
                }
            }
        }
        else{
            aPressed = false;
        }

        if(gamepad1.b){
            if(!bPressed){
                bPressed = true;
                if(robot.autoProgram  == SolidRobot.program.main){
                    robot.autoProgram = SolidRobot.program.minus;

                }
                else if(robot.autoProgram == SolidRobot.program.minus){
                    robot.autoProgram  = SolidRobot.program.alt;
                }
                else{
                    robot.autoProgram  = SolidRobot.program.main;
                }
            }
        }
        else{
            bPressed = false;
        }

    }

    /**
     * start - runs once after the start button is pressed
     */
    public void start(){
        robot.resetAngle();
        auto.start();
    }

    /**
     * loop - runs continuously after the play button is pressed
     */
    public void loop(){
        telemetry.addData("BRW", robot.BRW.getCurrentPosition());
        telemetry.addData("gyro", robot.getAngle());
        telemetry.addData("target", robot.targetVisible);
        telemetry.addData("Skystone Position", robot.blockPos);
        robot.setAllPositions();

        getVuforiaPositions();
    }

    public void getVuforiaInfo() {
        // check all the trackable targets to see which one (if any) is visible.
        robot.targetVisible = false;
        for (VuforiaTrackable trackable : robot.allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {

                robot.targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    robot.lastLocation = robotLocationTransform;
                }
                break;
            }
        }
    }

    public void getVuforiaPositions(){
        // check all the trackable targets to see which one (if any) is visible.
        robot.targetVisible = false;
        for (VuforiaTrackable trackable : robot.allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                robot.targetVisible = true;
                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    robot.lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        if (robot.targetVisible) {

            // express position (translation) of robot in inches.
            VectorF translation = robot.lastLocation.getTranslation();
            //robot.blockYPos = translation.get(1) / robot.mmPerInch;

            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / robot.mmPerInch, translation.get(1) / robot.mmPerInch, translation.get(2) / robot.mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(robot.lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            //robot.blockYPos = -1.0;
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
    }

}
