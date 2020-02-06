package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.vuforia.CameraDevice;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Sam W on 9/17/2019.
 */

public class SolidRobot {

    HardwareMap hardwareMap;


    //Creating motor/sensor variables
    public DcMotor FLW, FRW, BRW, BLW; //Drive motors
    public double frontLeftWheel = 0.0, frontRightWheel = 0.0, backRightWheel = 0.0, backLeftWheel = 0.0;

    public CRServo TLL, BLL, BRL, TRL; //Double-reverse four-bar lift vex motors
    public double leftLift = 0.0, rightLift = 0.0;

    public Servo RFC, LFC; //Foundation grabber servos
    public double rightFoundationClaw = 0.0, leftFoundationClaw = 0.0;

    public Servo LC, RC; //Claw
    public double leftClaw = 0.0, rightClaw = 0.0;

    public BNO055IMU gyro; //Gyro
    Orientation lastAngles = new Orientation();
    public double globalAngle = 0.0;

    //Driving constants
    private final double ENCODER_TICKS_PER_REVOLUTION = 383.6;
    private final double INCHES_TO_TICKS = ENCODER_TICKS_PER_REVOLUTION / (2 * Math.PI);

    //Autonomous variables
    public int blockPos = -1;

    //Autonomous enumerations
    enum color{ //Alliance color
        red, blue
    }

    enum program{ //Autonomous program
        main, foundationCenter, foundationWall
    }

    //Enumeration variables
    color autoColor = color.red;
    program autoProgram = program.main;

    //Vuforia variables
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "ATwGhwr/////AAAAGSP3/Jq5X0Zfkfo72G/VFE1ZvdzzZHwcCBd3MYXbw3/sNurlCSnfSksQh10JVp7KsgxEXn2zuImquxtNVs5s5sFifcmkHE98FiGOOHTTLVhwi9svT7cLW+hQfaQQ1QmGBZ9M5nna4+LVmgEtiAD8XAVmLBJb3xwkFfTqfAB8fWXmbiByf1f9ovWuG3a9v2FQQB9BqnDOFSnzQhDQFcgcEVyePXGSdVykMPC6zGPLGMgyNnF8pp1d+Tm0/HZYwEyZVCVma5S+1mozDg1wAWIeonPhJ8o9EAq63O/rGxwWho7lnuqIfSlPMzYlFg0OY97Vl30i4okUh3rqRH1zaKUA2NncE7QPqXkQpI+yXBIyCLRL";

    public static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = (6) * mmPerInch;

    public static final float stoneZ = 2.00f * mmPerInch;

    public static final float bridgeZ = 6.42f * mmPerInch;
    public static final float bridgeY = 23 * mmPerInch;
    public static final float bridgeX = 5.18f * mmPerInch;
    public static final float bridgeRotY = 59;
    public static final float bridgeRotZ = 180;

    public static final float halfField = 72 * mmPerInch;
    public static final float quadField  = 36 * mmPerInch;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    public OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia = null;
    public boolean targetVisible = false;
    public float phoneXRotate    = 0;
    public float phoneYRotate    = 0;
    public float phoneZRotate    = 0;

    public  TFObjectDetector tfod;

    /**
     * SolidRobot - constructor for robot class
     * @param hardwareMap - OpMode hardware map variable
     * @param auto - Parameter to track whether it is an autonomous program
     */
    public SolidRobot(HardwareMap hardwareMap, boolean auto){

        this.hardwareMap = hardwareMap;

        //Initializing hardware components
        FLW = hardwareMap.dcMotor.get("FLW"); //Drive motors
        FRW = hardwareMap.dcMotor.get("FRW");
        BRW = hardwareMap.dcMotor.get("BRW");
        BLW = hardwareMap.dcMotor.get("BLW");

        TLL = hardwareMap.get(CRServo.class, "TLL"); //Lift motors
        BLL = hardwareMap.get(CRServo.class, "BLL");
        BRL = hardwareMap.get(CRServo.class, "BRL");
        TRL = hardwareMap.get(CRServo.class, "TRL");

        RFC = hardwareMap.servo.get("RFC"); //Foundation claw servos
        LFC = hardwareMap.servo.get("LFC");

        LC = hardwareMap.servo.get("LC"); //Claw servos
        RC = hardwareMap.servo.get("RC");


        if(true == true){
            gyro = hardwareMap.get(BNO055IMU.class, "imu"); //Gyro
            gyro.initialize(getParam());
            resetAngle();
        }

        //Setting hardware attributes
        FRW.setDirection(DcMotorSimple.Direction.REVERSE);
        BRW.setDirection(DcMotorSimple.Direction.REVERSE);

        FLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        FLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        TLL.setDirection(DcMotorSimple.Direction.REVERSE);
        BLL.setDirection(DcMotorSimple.Direction.REVERSE);

        RC.setDirection(Servo.Direction.REVERSE);

        RFC.setDirection(Servo.Direction.REVERSE);

        //Initialize vuforia
        if(true) initVuforia();

    }

    /**
     * setAllPositions - Function to set powers and positions of hardware components. It is called
     * in the loop function so that it doesn't keep going after the stop button is pressed
     */
    public void setAllPositions(){
        FLW.setPower(frontLeftWheel);
        BLW.setPower(backLeftWheel);
        BRW.setPower(backRightWheel);
        FRW.setPower(frontRightWheel);

        TLL.setPower(leftLift);
        BLL.setPower(leftLift);
        BRL.setPower(rightLift);
        TRL.setPower(rightLift);

        RFC.setPosition(rightFoundationClaw);
        LFC.setPosition(leftFoundationClaw);

        LC.setPosition(leftClaw);
        RC.setPosition(rightClaw);
    }

    /**
     * doDaSleep - Stop actions for a set amount of time
     * @param milli - Time in milliseconds to stop for
     */
    void doDaSleep( int milli )
    {
        try
        {
            Thread.sleep(milli);
        } catch (InterruptedException ex)
        {

        }

    }

    /**
     * getParam - function to create a parameter profile for the gyro
     * @return parameter profile
     */
    private BNO055IMU.Parameters getParam(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        return parameters;
    }

    /**
     * resetAngle - function to reset the gyro position to zero
     */
    public void resetAngle()
    {
        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * getAngle - function to get the angular position of the robot
     * @return the angular position of the robot
     */
    public double getAngle() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * turn - function to turn to an angle
     * @param degrees - target orientation
     * @param power - power to run the motors at
     * @param correct - whether to realign to the orientation at a slower pace
     * @param correctDegrees - orientation to correct to
     */
    public void turn(double degrees, double power, boolean correct, double correctDegrees){

        FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double  leftPower, rightPower;
        double startAngle = getAngle();
        double correctionVal = 1.25;

        if (degrees < startAngle){
            leftPower = power;
            rightPower = -power;
            correctionVal = -1.25;
        }
        else if (degrees > startAngle){
            leftPower = -power;
            rightPower = power;
        }
        else return;

        frontLeftWheel = leftPower;
        backLeftWheel = leftPower;
        backRightWheel = rightPower;
        frontRightWheel = rightPower;

        if (degrees < startAngle) {
            while (getAngle() == startAngle) {doDaSleep(10);}

            while (getAngle() > degrees) {doDaSleep(10);}
        }
        else while (getAngle() < degrees) {doDaSleep(10);}

        powerWheels(0.0);
        if(correct) turn(correctDegrees + correctionVal, 0.1, false, correctDegrees);
    }

    /**
     * correct - align the robot exactly to an orientation
     * @param initDegrees - target orientation
     * @param exact - whether or not to narrow the range of error
     */
    private void correct(double initDegrees, boolean exact){
        FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double  leftPower, rightPower;
        double startAngle = getAngle();
        double correctionVal = 1.25;

        if (initDegrees < startAngle){
            leftPower = 0.1;
            rightPower = -0.1;
            correctionVal = -1.25;
        }
        else if (initDegrees > startAngle){
            leftPower = -0.1;
            rightPower = 0.1;
        }
        else return;

        if(exact) correctionVal = 0.0;

        double degrees = initDegrees + correctionVal;

        frontLeftWheel = leftPower;
        backLeftWheel = leftPower;
        backRightWheel = rightPower;
        frontRightWheel = rightPower;

        if (degrees < startAngle) {
            while (getAngle() == startAngle) {}

            while (getAngle() > degrees) {}
        }
        else while (getAngle() < degrees) {}

        powerWheels(0.0);
    }

    /**
     * initVuforia - initialize Vuforia
     */
    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

         //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection
        allTrackables.addAll(targetsSkyStone);

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 2.0f * mmPerInch;   // eg: Camera is 2.0 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 2.875f * mmPerInch;   // eg: Camera is 2.875 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = -6.5f * mmPerInch;     // eg: Camera is 6.5 inches from the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.

         */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        targetsSkyStone.activate();
    }

    /**
     * initTfod - Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    /**
     * clipToMotorRange - clip motor power variables to make sure that they don't go outside the domain of motor powers ([-1.0, 1.0])
     * @param val - initial power variable
     * @return the clipped power
     */
    public double clipToMotorRange(double val){
        double temp = val;
        if(val > 1.0) temp = 1.0;
        else if(val < -1.0) temp = -1.0;
        return temp;
    }

    /**
     * powerWheels - set wheel motors to a power
     * @param power - power to set the wheels to
     */
    public void powerWheels(double power){
        frontLeftWheel = power;
        frontRightWheel = power;
        backLeftWheel = power;
        backRightWheel = power;
    }

    /**
     * powerStrafe - set wheel motors to strafe to a power
     * @param power - power to set the wheels to
     */
    public void powerStrafe(double power){
        backLeftWheel = -power;
        frontLeftWheel = power;
        frontRightWheel = -power;
        backRightWheel = power;
    }

    /**
     * drive - drive exactly an amount of inches using the motor's TARGET_POSITION mode
     * @param inches - inches to move
     * @param power - power to set the wheels to
     */
    private void drive(double inches, double power){
        targetDriveWithTicks((int) (inches * INCHES_TO_TICKS * (24 / 22.5)), power);
    }

    /**
     * targetDriveWithTicks - drive exactly an amount of encoder ticks using the motor's TARGET_POSITION mode
     * @param ticks - ticks to move
     * @param power - power to set the wheels to
     */
    private void targetDriveWithTicks(int ticks, double power){
        FLW.setTargetPosition(ticks + FLW.getCurrentPosition());
        BLW.setTargetPosition(ticks + BLW.getCurrentPosition());
        BRW.setTargetPosition(ticks + BRW.getCurrentPosition());
        FRW.setTargetPosition(ticks + FRW.getCurrentPosition());

        FLW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRW.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(ticks < 0) powerWheels(-power);
        else powerWheels(power);

        while(FLW.isBusy() || BLW.isBusy() || BRW.isBusy() || FRW.isBusy()){

        }
        powerWheels(0.0);
    }

    /**
     * powerLift - set the lift motors to a power
     * @param power - power to set the lift to
     */
    public void powerLift(double power){
        leftLift = power * 0.7;
        rightLift = power * 0.7;
    }

    /**
     * timeDrive - drive for an amount of time
     * @param power - power to set the motors to
     * @param millis - milliseconds to drive for
     */
    public void timeDrive(double power, long millis){
        FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        powerWheels(power);
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + millis){

        }
        powerWheels(0.0);
    }

    /**
     * strafe - strafe exactly an amount of inches using the motor's TARGET_POSITION mode
     * @param inches - inches to move
     * @param power - power to set the motors to
     */
    private void strafe(double inches, double power){
        strafeWithTicks((int) (inches * 90), power);
    }

    /**
     * strafeWithTicks - strafe exactly an amount of encoder ticks using the motor's TARGET_POSITION mode
     * @param ticks - ticks to move
     * @param power - power to set the motors to
     */
    private void strafeWithTicks(int ticks, double power){

        FLW.setTargetPosition(ticks + FLW.getCurrentPosition());
        BLW.setTargetPosition(-ticks + BLW.getCurrentPosition());
        BRW.setTargetPosition(ticks + BRW.getCurrentPosition());
        FRW.setTargetPosition(-ticks + FRW.getCurrentPosition());

        FLW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRW.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(ticks < 0){
            backLeftWheel = 1.0;
            frontLeftWheel = -1.0;
            frontRightWheel = 1.0;
            backRightWheel = -1.0;
        }
        else{
            backLeftWheel = -1.0;
            frontLeftWheel = 1.0;
            frontRightWheel = -1.0;
            backRightWheel = 1.0;
        }

        while(FLW.isBusy() || BLW.isBusy() || BRW.isBusy() || FRW.isBusy()){

        }
        powerWheels(0.0);
    }

    /**
     * powerStrafeWithTicks - strafe for an amount of encoder ticks without correctiong
     * @param power - power to set the motors to
     * @param absTicks - ticks to move
     */
    public void powerStrafeWithTicks(double power, int absTicks){

        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int startTicks = BRW.getCurrentPosition();
        backLeftWheel = -power;
        frontLeftWheel = power;
        frontRightWheel = -power;
        backRightWheel = power;
        if(power > 0) while(BRW.getCurrentPosition() < startTicks + absTicks){ }
        else while(BRW.getCurrentPosition() > startTicks - absTicks){ }
        powerWheels(0.0);
    }

    /**
     * powerCorrectionDrive - drive for an amount of encoder ticks while using the gyro to keep aligned
     * @param power - power to set the motors to
     * @param ticks - ticks to move
     * @param correctionDegree - gyro position to use as reference
     */
    public void powerCorrectionDrive(double power, int ticks, double correctionDegree){
        double correctionVal = 1.2;

        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int startTicks = BRW.getCurrentPosition();
        powerWheels(power);
        while(BRW.getCurrentPosition() < startTicks + ticks){
            if(getAngle() > correctionDegree + 0.5){
                frontLeftWheel = power * correctionVal;
                backLeftWheel = power * correctionVal;
                backRightWheel = power;
                frontRightWheel = power;
            }
            else if(getAngle() < correctionDegree - 0.5){
                frontLeftWheel = power;
                backLeftWheel = power;
                backRightWheel = power * correctionVal;
                frontRightWheel = power * correctionVal;
            }
            else{
                frontLeftWheel = power;
                backLeftWheel = power;
                backRightWheel = power;
                frontRightWheel = power;
            }
        }
        powerWheels(0.0);
    }

    /**
     * powerCorrectionDriveBack - drive back for an amount of encoder ticks while using the gyro to keep aligned
     * @param power - power to set the motors to
     * @param ticks - ticks to move
     * @param correctionDegree - gyro position to use as reference
     */
    public void powerCorrectionDriveBack(double power, int ticks, double correctionDegree){
        double correctionVal = 1.2;

        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int startTicks = BRW.getCurrentPosition();
        powerWheels(power);
        while(BRW.getCurrentPosition() > startTicks - ticks){
            if(getAngle() > correctionDegree + 0.5){
                frontLeftWheel = power;
                backLeftWheel = power;
                backRightWheel = power * correctionVal;
                frontRightWheel = power * correctionVal;
            }
            else if(getAngle() < correctionDegree - 0.5){
                frontLeftWheel = power * correctionVal;
                backLeftWheel = power * correctionVal;
                backRightWheel = power;
                frontRightWheel = power;
            }
            else{
                frontLeftWheel = power;
                backLeftWheel = power;
                backRightWheel = power;
                frontRightWheel = power;
            }
        }
        powerWheels(0.0);
    }

    /**
     * setMotorMode - set wheel motors to a mode
     * @param mode - mode to set the motors to
     */
    public void setMotorMode(DcMotor.RunMode mode){
        FLW.setMode(mode);
        BLW.setMode(mode);
        BRW.setMode(mode);
        FRW.setMode(mode);
        doDaSleep(50);
    }

    /**
     * blueAuto - main autonomous program for blue side. Delivers both skystones and parks in the middle.
     */
    public void blueAuto(){
        drive(20, 1.0);
        //correct(0, true);
        doDaSleep(1250);
        if(targetVisible) blockPos = 0;
        else{
            powerStrafeWithTicks(0.15, 525);
            correct(0, true);
            doDaSleep(1250);
            if(targetVisible) blockPos = 1;
            else{
                blockPos = 2;
                powerStrafeWithTicks(0.15, 525);
                correct(0, true);
            }
        }

        pickUpBlock();

        turn(75, 0.5, false, 90);
        doDaSleep(500);
        correct(90, true);

        int move = 35;
        if(blockPos == 1) move += 7;
        else if(blockPos == 2) move += 16;
        powerCorrectionDrive(.9, (int) (move * INCHES_TO_TICKS * (24 / 22.5)), 90);

        leftClaw = 0.6;
        rightClaw = 0.6;

        correct(90, true);

        doDaSleep(500);

        int moveTwo = 60;
        if(blockPos == 1 || blockPos == 2) moveTwo += 8;
        //else if(blockPos == 2) moveTwo += 14;

        powerCorrectionDriveBack(-0.9, (int) ((moveTwo) * INCHES_TO_TICKS * (24 / 22.5)), 90);


        turn(15, 0.5, true, 0);
        correct(0, true);

        pickUpBlock();

        turn(75, 0.5, false, 0);
        correct(90, true);

        int moveThree = moveTwo;

        powerCorrectionDrive(1.0, (int) (moveThree * INCHES_TO_TICKS * (24 / 22.5)), 90);

        leftClaw = 0.6;
        rightClaw = 0.6;

        doDaSleep(500);

        drive(-12, 1.0);

        powerStrafe(0.1);
        doDaSleep(1000);
        powerStrafe(0.0);

    }

    /**
     * redAuto - main autonomous program for blue side. Delivers both skystones and parks in the middle.
     */
    public void redAuto(){
        drive(20, 1.0);
        //correct(0, true);
        doDaSleep(1250);
        if(targetVisible) blockPos = 0;
        else{
            powerStrafeWithTicks(-0.15, 525);
            correct(0, true);
            doDaSleep(1250);
            if(targetVisible) blockPos = 1;
            else{
                blockPos = 2;
                powerStrafeWithTicks(-0.15, 525);
                correct(0, true);
            }
        }

        pickUpBlock();

        turn(-75, 0.5, false, -90);
        doDaSleep(500);
        correct(-90, true);

        int move = 35;
        if(blockPos == 1) move += 8;
        else if(blockPos == 2) move += 16;
        powerCorrectionDrive(.9, (int) (move * INCHES_TO_TICKS * (24 / 22.5)), -90);

        leftClaw = 0.6;
        rightClaw = 0.6;

        correct(-90, true);

        doDaSleep(500);

        int moveTwo = 60;
        if(blockPos == 1 || blockPos == 2) moveTwo += 8;
        //else if(blockPos == 2) moveTwo += 14;

        powerCorrectionDriveBack(-0.9, (int) ((moveTwo) * INCHES_TO_TICKS * (24 / 22.5)), -90);


        turn(-15, 0.5, true, 0);
        correct(0, true);

        //powerCorrectionDrive(0.5, 500, 0);
        pickUpBlock();


        turn(-70, 0.5, true, -90);
        correct(-90, true);

        int moveThree = moveTwo;

        powerCorrectionDrive(1.0, (int) (moveThree * INCHES_TO_TICKS * (24 / 22.5)), -90);

        leftClaw = 0.6;
        rightClaw = 0.6;

        doDaSleep(500);

        drive(-12, 1.0);

        powerStrafe(-0.1);
        doDaSleep(1000);
        powerStrafe(0.0);
    }

    /**
     * pickUpBlock - function to pick up a block from 10 inches away
     */
    public void pickUpBlock(){
        powerLift(-1.0);
        leftClaw = 0.55;
        rightClaw = 0.8;
        doDaSleep(400);
        powerLift(1.0);
        doDaSleep(700);
        powerLift(0.0);
        int startTicks = BRW.getCurrentPosition();
        timeDrive(0.3, 750);
        leftClaw = 0.6;
        rightClaw = 0.6;
        timeDrive(0.3, 250);
        doDaSleep(200);
        timeDrive(0.3, 600);
        leftClaw = 1.0;
        rightClaw = 1.0;
        doDaSleep(500);
        int finalTicks = BRW.getCurrentPosition();
        targetDriveWithTicks(startTicks - finalTicks, 1.0);
    }

    /**
     * redFoundationCenterAuto - autonomous program to get the foundation and park near the center
     */
    public void redFoundationCenterAuto(){
        powerStrafe(.5);
        doDaSleep(500);
        powerStrafe(0.0);

        timeDrive(-.3, 400);

        powerCorrectionDrive(.8, (int) (21 * INCHES_TO_TICKS * (24/22.5)), 0);

        powerWheels(0.15);
        doDaSleep(500);
        leftFoundationClaw = 0.36;
        rightFoundationClaw = 0.36;
        doDaSleep(250);
        powerWheels(0.0);

        powerCorrectionDriveBack(-0.5, (int) (18 * INCHES_TO_TICKS * (24/22.5)), 0);

        turn(-75, .5, true, -90);
        correct(-90, true);

        leftFoundationClaw = 0.6;
        rightFoundationClaw = 0.6;

        powerStrafe(-.2);
        doDaSleep(500);
        powerStrafe(0.0);

        timeDrive(.3, 1500);

        powerStrafe(-.2);
        doDaSleep(1000);
        powerStrafe(0.0);

        powerLift(-1.0);
        doDaSleep(400);
        powerLift(1.0);
        doDaSleep(400);
        powerLift(0.0);

        powerCorrectionDriveBack(-.5, (int) (40 * INCHES_TO_TICKS * (24/22.5)), -90);

        powerStrafe(-0.2);
        doDaSleep(800);
        powerStrafe(0.0);

    }

    /**
     * redFoundationWallAuto - autonomous program to get the foundation and park near the wall
     */
    public void redFoundationWallAuto(){
        powerStrafe(.5);
        doDaSleep(500);
        powerStrafe(0.0);

        timeDrive(-.3, 400);

        powerCorrectionDrive(.8, (int) (21 * INCHES_TO_TICKS * (24/22.5)), 0);

        powerWheels(0.15);
        doDaSleep(400);
        leftFoundationClaw = 0.36;
        rightFoundationClaw = 0.36;
        doDaSleep(250);
        powerWheels(0.0);

        powerCorrectionDriveBack(-0.5, (int) (18 * INCHES_TO_TICKS * (24/22.5)), 0);

        turn(-75, .5, true, -90);
        correct(-90, true);

        leftFoundationClaw = 0.6;
        rightFoundationClaw = 0.6;

        powerCorrectionDriveBack(-.3, (int) ( 4 * INCHES_TO_TICKS * (24/22.5)), 0);

        powerStrafe(-.5);
        doDaSleep(500);
        powerStrafe(0.0);

        timeDrive(.3, 1500);

        powerStrafe(.2);
        doDaSleep(4000);
        powerStrafe(0.0);

        powerStrafe(.15);
        doDaSleep(250);
        powerStrafe(0.0);

        powerLift(-1.0);
        doDaSleep(400);
        powerLift(1.0);
        doDaSleep(400);
        powerLift(0.0);

        powerCorrectionDriveBack(-.5, (int) (40 * INCHES_TO_TICKS * (24/22.5)), -90);

        powerStrafe(0.2);
        doDaSleep(700);
        powerStrafe(0.0);
    }

    /**
     * blueFoundationWallAuto - autonomous program to get the foundation and park near the wall
     */
    public void blueFoundationWallAuto()
    {
        powerStrafe(-.5);
        doDaSleep(500);
        powerStrafe(0.0);

        timeDrive(-.3, 400);

        powerCorrectionDrive(.8, (int) (21 * INCHES_TO_TICKS * (24/22.5)), 0);

        powerWheels(0.15);
        doDaSleep(400);
        leftFoundationClaw = 0.36;
        rightFoundationClaw = 0.36;
        doDaSleep(250);
        powerWheels(0.0);

        powerCorrectionDriveBack(-0.5, (int) (18 * INCHES_TO_TICKS * (24/22.5)), 0);

        turn(75, .5, true, 90);
        correct(90, true);

        leftFoundationClaw = 0.6;
        rightFoundationClaw = 0.6;

        powerCorrectionDriveBack(-.3, (int) ( 4 * INCHES_TO_TICKS * (24/22.5)), 0);

        powerStrafe(.5);
        doDaSleep(500);
        powerStrafe(0.0);

        timeDrive(.3, 1500);

        powerStrafe(-.2);
        doDaSleep(3000);
        powerStrafe(0.0);

        powerStrafe(.3);
        doDaSleep(450);
        powerStrafe(0.0);

        powerLift(-1.0);
        doDaSleep(400);
        powerLift(1.0);
        doDaSleep(400);
        powerLift(0.0);

        powerCorrectionDriveBack(-.5, (int) (40 * INCHES_TO_TICKS * (24/22.5)), 90);

        powerStrafe(-0.2);
        doDaSleep(700);
        powerStrafe(0.0);
    }

    /**
     * blueFoundationCenterAuto - autonomous program to get the foundation and park near the center
     */
    public void blueFoundationCenterAuto()
    {
        powerStrafe(-.5);
        doDaSleep(500);
        powerStrafe(0.0);

        timeDrive(-.3, 400);

        powerCorrectionDrive(.8, (int) (21 * INCHES_TO_TICKS * (24/22.5)), 0);

        powerWheels(0.15);
        doDaSleep(500);
        leftFoundationClaw = 0.36;
        rightFoundationClaw = 0.36;
        doDaSleep(250);
        powerWheels(0.0);

        powerCorrectionDriveBack(-0.5, (int) (18 * INCHES_TO_TICKS * (24/22.5)), 0);

        turn(75, .5, true, 90);
        correct(90, true);

        leftFoundationClaw = 0.6;
        rightFoundationClaw = 0.6;

        powerCorrectionDriveBack(-.3, (int) ( 4 * INCHES_TO_TICKS * (24/22.5)), 0);

        powerStrafe(.5);
        doDaSleep(500);
        powerStrafe(0.0);

        powerStrafe(.2);
        doDaSleep(500);
        powerStrafe(0.0);

        timeDrive(.3, 1500);

        powerStrafe(.2);
        doDaSleep(600);
        powerStrafe(0.0);

        powerLift(-1.0);
        doDaSleep(400);
        powerLift(1.0);
        doDaSleep(400);
        powerLift(0.0);

        powerCorrectionDriveBack(-.5, (int) (40 * INCHES_TO_TICKS * (24/22.5)), 90);

        powerStrafe(0.2);
        doDaSleep(800);
        powerStrafe(0.0);
    }

}
