package org.firstinspires.ftc.teamcode;

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

    public DcMotor FLW, FRW, BRW, BLW; //Drive motors
    public double frontLeftWheel = 0.0, frontRightWheel = 0.0, backRightWheel = 0.0, backLeftWheel = 0.0;

    public CRServo LL, RL; //Double-reverse four-bar lift vex motors
    public double leftLift = 0.0, rightLift = 0.0;

    public Servo RFC, LFC; //Foundation grabber servos
    public double rightFoundationClaw = 0.0, leftFoundationClaw = 0.0;

    public Servo LC, RC; //Claw
    public double leftClaw = 0.0, rightClaw = 0.0;

    public BNO055IMU gyro;
    Orientation lastAngles = new Orientation();
    public double globalAngle = 0.0;

    //int skystonePos = 0;

    //encoder ticks per revolution / 2pi
    private final double ENCODER_TICKS_PER_REVOLUTION = 383.6;
    private final double INCHES_TO_TICKS = ENCODER_TICKS_PER_REVOLUTION / (2 * Math.PI);
    private double MINIMUM_DRIVE_POWER = 0.1;

    public int blockPos = -1;

    enum color{
        red, blue
    }

    enum program{
        main, alt
    }

    color autoColor = color.red;
    program autoProgram = program.main;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "ATwGhwr/////AAAAGSP3/Jq5X0Zfkfo72G/VFE1ZvdzzZHwcCBd3MYXbw3/sNurlCSnfSksQh10JVp7KsgxEXn2zuImquxtNVs5s5sFifcmkHE98FiGOOHTTLVhwi9svT7cLW+hQfaQQ1QmGBZ9M5nna4+LVmgEtiAD8XAVmLBJb3xwkFfTqfAB8fWXmbiByf1f9ovWuG3a9v2FQQB9BqnDOFSnzQhDQFcgcEVyePXGSdVykMPC6zGPLGMgyNnF8pp1d+Tm0/HZYwEyZVCVma5S+1mozDg1wAWIeonPhJ8o9EAq63O/rGxwWho7lnuqIfSlPMzYlFg0OY97Vl30i4okUh3rqRH1zaKUA2NncE7QPqXkQpI+yXBIyCLRL";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    public static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    public static final float bridgeZ = 6.42f * mmPerInch;
    public static final float bridgeY = 23 * mmPerInch;
    public static final float bridgeX = 5.18f * mmPerInch;
    public static final float bridgeRotY = 59;                                 // Units are degrees
    public static final float bridgeRotZ = 180;

    // Constants for perimeter targets
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
     * SolidRobot - constructor for robot object, initializes hardware
     * @param hardwareMap - hardware map lmao
     */
    public SolidRobot(HardwareMap hardwareMap, boolean auto){

        this.hardwareMap = hardwareMap;

        FLW = hardwareMap.dcMotor.get("FLW"); //Drive motors
        FRW = hardwareMap.dcMotor.get("FRW");
        BRW = hardwareMap.dcMotor.get("BRW");
        BLW = hardwareMap.dcMotor.get("BLW");

        LL = hardwareMap.get(CRServo.class, "LL");
        RL = hardwareMap.get(CRServo.class, "RL");

        RFC = hardwareMap.servo.get("RFC");
        LFC = hardwareMap.servo.get("LFC");

        LC = hardwareMap.servo.get("LC");
        RC = hardwareMap.servo.get("RC");

        if(true == true){
            gyro = hardwareMap.get(BNO055IMU.class, "imu");
            gyro.initialize(getParam());
            resetAngle();
        }

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

        LL.setDirection(DcMotorSimple.Direction.REVERSE);

        RC.setDirection(Servo.Direction.REVERSE);

        RFC.setDirection(Servo.Direction.REVERSE);

        if(true) initVuforia();
        //initTfod();



    }

    public void start(){
        if (tfod != null) {
            tfod.activate();
        }
    }

    public void setAllPositions(){
        FLW.setPower(frontLeftWheel);
        BLW.setPower(backLeftWheel);
        BRW.setPower(backRightWheel);
        FRW.setPower(frontRightWheel);

        LL.setPower(leftLift);
        RL.setPower(rightLift);

        RFC.setPosition(rightFoundationClaw);
        LFC.setPosition(leftFoundationClaw);

        LC.setPosition(leftClaw);
        RC.setPosition(rightClaw);
    }

    void doDaSleep( int milli )
    {
        try
        {
            Thread.sleep(milli);
        } catch (InterruptedException ex)
        {

        }

    }

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

    public void resetAngle()
    {
        lastAngles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

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

    private void turn(double degrees, double power, boolean correct){

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
        if(correct) turn(degrees + correctionVal, 0.1, false);
    }

    private void correct(double initDegrees){
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
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void powerWheels(double power){
        frontLeftWheel = power;
        frontRightWheel = power;
        backLeftWheel = power;
        backRightWheel = power;
    }

    private void drive(double inches, double power){
        targetDriveWithTicks((int) (inches * INCHES_TO_TICKS * (24 / 22.5)), power);
    }

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
        //I used to set the motors to run without encoder here
    }

    public void powerLift(double power){
        leftLift = power * 0.7;
        rightLift = power * 0.7;
    }

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

    private void strafe(double inches, double power){
        strafeWithTicks((int) (inches * 154.167), power);
    }

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

    public void redMainAuto(){
        drive(16, 1.0);
        turn(-90 + 5, 0.5, true);
        doDaSleep(250);
        if(targetVisible) blockPos = 0;
        else{
            drive(8, 0.5);
            doDaSleep(250);
            if(targetVisible) blockPos = 1;
            else blockPos = 2;
        }

        if(blockPos == 0){

        }
        else if(blockPos == 1){

        }
        else{
            drive(-6.5, 0.5);
            strafe(-12, 0.25);

            //correct(-90);

            powerLift(-1.0);
            doDaSleep(900);
            leftClaw = 0.6;
            rightClaw = 0.6;
            doDaSleep(500);
            powerLift(0.0);

            timeDrive(0.25, 250);

            doDaSleep(1000);

            leftClaw = 1.0;
            rightClaw = 1.0;

            doDaSleep(500);

            turn(-35 - 5, 0.5, true);
            drive(-18, 1.0);
            turn(-90 + 5, 0.5, true);
            drive(-36, 1.0);

            turn(0 + 5, 0.5, true);
            drive(9, 1.0);
            rightClaw = 0.6;
            leftClaw = 0.6;
            doDaSleep(250);
            drive(-8, 1.0);
            turn(-90, 0.5, true);

            drive(72, 1.0);

        }

    }

    public void blueMainAuto(){

    }

    public void foundationAuto(){
        drive(30.5, 1.0);
        strafe(6, 0.25);
        rightFoundationClaw = 0.36;
        leftFoundationClaw = 0.36;
        timeDrive(0.15, 5000);
        strafe(52, 0.25);
    }
}
