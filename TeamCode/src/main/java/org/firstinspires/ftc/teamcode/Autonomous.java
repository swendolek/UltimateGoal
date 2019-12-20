package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * Created by Sam W on 9/17/2019.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Autonomous")
public class Autonomous extends OpMode{

    SolidRobot robot; //Robot object

    boolean aPressed = false, bPressed = false;

    Thread auto = new Thread(){
        public void run(){
            telemetry.addData("Status", "Thread Started");
            /*if(robot.autoColor == SolidRobot.color.red){
                if(robot.autoProgram == SolidRobot.program.pro){
                    //red main
                }
                else{
                    //red alt
                }
            }
            else{
                if(robot.autoProgram == SolidRobot.program.pro){
                    //blue main
                }
                else{
                    //blue alt
                }
            }*/

            //robot.doDrive();
            robot.drive(0.3, 1000, robot.BRW, 0, 3);
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

        robot.setMinimumDrivePower(0.1);

    }

    /**
     * init_loop - runs continuously after the init button is pressed
     */
    public void init_loop(){

        //Autonomous program selector with the controller

        if(robot.autoColor  == SolidRobot.color.red) telemetry.addData("Color", "Red");
        else telemetry.addData("Color", "Blue");

        if(robot.autoProgram  == SolidRobot.program.pro) telemetry.addData("Program", "Default");
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
                if(robot.autoProgram  == SolidRobot.program.pro){
                    robot.autoProgram  = SolidRobot.program.safe;
                }
                else{
                    robot.autoProgram  = SolidRobot.program.pro;
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
        auto.start();
    }

    /**
     * loop - runs continuously after the play button is pressed
     */
    public void loop(){
        telemetry.addData("BRW", robot.BRW.getCurrentPosition());
        telemetry.addData("gyro", robot.gyroPosition());
        telemetry.addData("target", robot.targetVisible);
        telemetry.addData("Skystone Position", robot.skystonePos);
        telemetry.addData("Pos", robot.pos);
        robot.setAllPositions();

        getVuforiaInfo();
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
}
