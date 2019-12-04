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
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

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
    int target = 0;

    int skystonePos = 0;
    float pos;
    boolean detect = false;

    public final double motorRatio = 0.71474359;
    private final double MINIMUM_DRIVE_POWER = 0.1;

    enum color{
        red, blue
    }

    enum program{
        pro, safe
    }

    color autoColor = color.red;
    program autoProgram = program.pro;

    VectorF translation;

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

    // Class Members
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

        if(auto){
            gyro = hardwareMap.get(BNO055IMU.class, "imu");
            gyro.initialize(getParam());
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

        if(auto) initVuforia();
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
        try {
            Thread.sleep(milli);
        } catch (InterruptedException ex) {

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

    public int gyroPosition(){
        return 15 + (int) gyro.getAngularOrientation().firstAngle * -1;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

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

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
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
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

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
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

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

    public int getPos(DcMotor motor){
        return motor.getCurrentPosition();
    }

    private void powerWheels(double power){
        frontLeftWheel = power;
        frontRightWheel = power;
        backLeftWheel = power;
        backRightWheel = power;
    }


    private void smartDrive(double maxPower, int ticks, DcMotor encoder, int angle, int angleTolerance){
        double d = MINIMUM_DRIVE_POWER;
        double a = maxPower - d;
        double b = (Math.PI) / ticks;

        final double angleCorrectionValue = 1.7;

        int startTicks = encoder.getCurrentPosition();
        while(BRW.getCurrentPosition() < startTicks + ticks){
            double currentPower = a * Math.sin(b * ticks) + d; //Derivative is a(cos(bx) * (db/dt)) + sin(bx) * (db/dx)

            if(gyroPosition() < angle - angleTolerance){
                //power left side more
                frontLeftWheel = currentPower * angleCorrectionValue;
                backLeftWheel = currentPower * angleCorrectionValue;
                backRightWheel = currentPower;
                frontRightWheel = currentPower;
            }
            else if(gyroPosition() > angle + angleTolerance){
                //power right side more
                frontLeftWheel = currentPower;
                backLeftWheel = currentPower;
                backRightWheel = currentPower * angleCorrectionValue;
                frontRightWheel = currentPower * angleCorrectionValue;
            }
            else{
                //Power both sides the same
                frontLeftWheel = currentPower;
                backLeftWheel = currentPower;
                backRightWheel = currentPower;
                frontRightWheel = currentPower;
            }

        }
    }

    private void drive(double power, int ticks, DcMotor encoder){
        int startTicks = getPos(encoder);

        powerWheels(power);
        while(getPos(encoder) < startTicks + ticks){

        }
        powerWheels(0.0);
    }

    private void correctionDrive(double power, int ticks, DcMotor encoder, int degrees){
        int startTicks = getPos(encoder);
        double multiplier = 1.4;
        powerWheels(power);
        while(getPos(encoder) < startTicks + ticks){
            if(gyroPosition() < degrees - 1){
                frontLeftWheel = power * multiplier;
                backLeftWheel = power * multiplier;
                backRightWheel = power;
                frontRightWheel = power;
            }
            else if(gyroPosition() > degrees + 1){
                frontLeftWheel = power;
                backLeftWheel = power;
                backRightWheel = power * multiplier;
                frontRightWheel = power * multiplier;
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

    private void correctionTimeDrive(double power, int millis, int degrees){
        double multiplier = 1.4;
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + millis){
            if(gyroPosition() < degrees - 1){
                frontLeftWheel = power * multiplier;
                backLeftWheel = power * multiplier;
                backRightWheel = power;
                frontRightWheel = power;
            }
            else if(gyroPosition() > degrees + 1){
                frontLeftWheel = power;
                backLeftWheel = power;
                backRightWheel = power * multiplier;
                frontRightWheel = power * multiplier;
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

    private void strafeLeft(double power, int ticks, DcMotor encoder){
        int startTicks = getPos(encoder);
        frontLeftWheel = -power;
        backLeftWheel = power;
        backRightWheel = -power;
        frontRightWheel = power;
        while(getPos(encoder) > startTicks - ticks){

        }
        powerWheels(0.0);
    }

    private void strafeRight(double power, int ticks, DcMotor encoder){
        int startTicks = getPos(encoder);
        frontLeftWheel = power;
        backLeftWheel = -power;
        backRightWheel = power;
        frontRightWheel = -power;
        while(getPos(encoder) < startTicks + ticks){

        }
        powerWheels(0.0);
    }

    private void driveBackwards(double power, int ticks, DcMotor encoder){
        int startTicks = getPos(encoder);

        powerWheels(-power);
        while(getPos(encoder) > startTicks - ticks){

        }
        powerWheels(0.0);
    }

    public void redProAuto(){
        strafeRight(0.16, 600, BRW);
        doDaSleep(50);

        leftLift = -0.7;
        rightLift = -0.7;
        doDaSleep(500);
        leftLift = 0.0;
        rightLift = 0.0;

        correctionDrive(0.12, 165, BRW, 0);
        doDaSleep(750);
        if(!targetVisible){
            //correctionDrive(0.12, 150, BRW, 0);
            drive(0.12, 150, BRW);
            doDaSleep(750);
            if(!targetVisible) {
                //correctionDrive(0.12, 150, BRW, 0);
                drive(0.12, 150, BRW);
                skystonePos = 3;
            }
            else{
                skystonePos = 2;
            }
        }
        else{
            skystonePos = 1;
        }

        if(skystonePos == 1){
            drive(0.16, 150, BRW);
            doDaSleep(250);
            leftLift = -0.5;
            rightLift = -0.5;
            strafeRight(0.2, 530, BRW);
            leftLift = 0.0;
            rightLift = 0.0;
        }
        else if(skystonePos == 2){
            /*translation = lastLocation.getTranslation();
            pos = translation.get(1) / mmPerInch;
            powerWheels(-0.1);
            while(pos > 0.0){
                doDaSleep(250);
                powerWheels(0.0);
                doDaSleep(250);
                powerWheels(-0.1);
                translation = lastLocation.getTranslation();
                pos = translation.get(1) / mmPerInch;
            }*/
            powerWheels(0.0);
            driveBackwards(0.12, 355, BRW);
            doDaSleep(250);
            leftLift = -0.5;
            rightLift = -0.5;
            strafeRight(0.2, 530, BRW);
            leftLift = 0.0;
            rightLift = 0.0;
        }
        else{
            powerWheels(0.0);
            driveBackwards(0.12, 340, BRW);
            doDaSleep(250);
            leftLift = -0.5;
            rightLift = -0.5;
            strafeRight(0.2, 530, BRW);
            leftLift = 0.0;
            rightLift = 0.0;
        }

        doDaSleep(500);
        powerWheels(0.1);
        doDaSleep(500);
        powerWheels(0.0);

        leftClaw = 1.0;
        rightClaw = 1.0;

        doDaSleep(1000);

        frontLeftWheel = -0.12;
        backLeftWheel = -0.12;
        backRightWheel = 0.12;
        frontRightWheel = 0.12;
        while(gyroPosition() > -90){

        }
        powerWheels(0.0);
        doDaSleep(500);
        correctionDrive(0.16, 200, BRW, -90);

        frontLeftWheel = -0.12;
        backLeftWheel = -0.12;
        backRightWheel = 0.12;
        frontRightWheel = 0.12;
        while(gyroPosition() > -170 && gyroPosition() < 0){

        }
        int ticks;
        if(skystonePos == 1) ticks = 1800;
        else if(skystonePos == 2) ticks = 1500;
        else ticks = 1650;
        int startingTicks = BRW.getCurrentPosition();
        while(BRW.getCurrentPosition() < startingTicks + ticks){
            if(gyroPosition() < 179 && gyroPosition() > 0){
                frontLeftWheel = 0.25 * 1.4;
                backLeftWheel = 0.25 * 1.4;
                backRightWheel = 0.25;
                frontRightWheel = 0.25;
            }
            else if(gyroPosition() > -179 && gyroPosition() < -45){
                frontLeftWheel = 0.25;
                backLeftWheel = 0.25;
                backRightWheel = 0.25 * 1.4;
                frontRightWheel = 0.25 * 1.4;
            }
            else{
                frontLeftWheel = 0.25;
                backLeftWheel = 0.25;
                backRightWheel = 0.25;
                frontRightWheel = 0.25;
            }
        }
        powerWheels(0.0);

        frontLeftWheel = -0.12;
        backLeftWheel = -0.12;
        backRightWheel = 0.12;
        frontRightWheel = 0.12;

        while(!(gyroPosition() > 85 && gyroPosition() < 95)){

        }
        powerWheels(0.0);

        leftLift = -0.5;
        rightLift = -0.5;
        doDaSleep(500);
        powerWheels(0.16);
        doDaSleep(1000);
        leftLift = 0.0;
        rightLift = 0.0;
        powerWheels(0.0);
        doDaSleep(1000);
        leftClaw = 0.6;
        rightClaw = 0.6;
        powerWheels(0.16);
        doDaSleep(500);
        powerWheels(0.0);
        leftFoundationClaw = 0.36;
        rightFoundationClaw = 0.36;
        doDaSleep(500);
        long startTime = System.currentTimeMillis();
        powerWheels(-0.16);
        while(System.currentTimeMillis() < startTime + 3000){
            if(gyroPosition() > 91){
                frontLeftWheel = -0.16 * 1.4;
                backLeftWheel = -0.16 * 1.4;
                backRightWheel = -0.16;
                frontRightWheel = -0.16;
            }
            else if(gyroPosition() < 89){
                frontLeftWheel = -0.16;
                backLeftWheel = -0.16;
                backRightWheel = -0.16 * 1.4;
                frontRightWheel = -0.16 * 1.4;
            }
            else{
                frontLeftWheel = -0.16;
                backLeftWheel = -0.16;
                backRightWheel = -0.16;
                frontRightWheel = -0.16;
            }
        }
        frontLeftWheel = -0.16;
        backLeftWheel = 0.16;
        backRightWheel = -0.16;
        frontRightWheel = 0.16;
        leftFoundationClaw = 0.0;
        rightFoundationClaw = 0.0;
        doDaSleep(4000);
        powerWheels(0.0);
    }

    public void blueProAuto(){
        strafeLeft(0.16, 595, BRW);
        doDaSleep(50);

        leftLift = -0.7;
        rightLift = -0.7;
        doDaSleep(500);
        leftLift = 0.0;
        rightLift = 0.0;

        correctionDrive(0.12, 165, BRW, 0);
        doDaSleep(750);
        if(!targetVisible){
            //correctionDrive(0.12, 150, BRW, 0);
            drive(0.12, 150, BRW);
            doDaSleep(750);
            if(!targetVisible) {
                //correctionDrive(0.12, 150, BRW, 0);
                drive(0.12, 150, BRW);
                skystonePos = 3;
            }
            else{
                skystonePos = 2;
            }
        }
        else{
            skystonePos = 1;
        }

        if(skystonePos == 1){
            drive(0.16, 130, BRW);
            doDaSleep(500);
            leftLift = -0.5;
            rightLift = -0.5;
            strafeLeft(0.2, 435, BRW);
            leftLift = 0.0;
            rightLift = 0.0;
        }
        else if(skystonePos == 2){
            /*translation = lastLocation.getTranslation();
            pos = translation.get(1) / mmPerInch;
            powerWheels(-0.1);
            while(pos > 0.0){
                doDaSleep(250);
                powerWheels(0.0);
                doDaSleep(250);
                powerWheels(-0.1);
                translation = lastLocation.getTranslation();
                pos = translation.get(1) / mmPerInch;
            }*/
            powerWheels(0.0);
            driveBackwards(0.12, 355, BRW);
            doDaSleep(250);
            leftLift = -0.5;
            rightLift = -0.5;
            strafeLeft(0.2, 435, BRW);
            leftLift = 0.0;
            rightLift = 0.0;
        }
        else{
            powerWheels(0.0);
            driveBackwards(0.12, 340, BRW);
            doDaSleep(250);
            leftLift = -0.5;
            rightLift = -0.5;
            strafeLeft(0.2, 435, BRW);
            leftLift = 0.0;
            rightLift = 0.0;
        }

        doDaSleep(500);
        powerWheels(0.1);
        doDaSleep(500);
        powerWheels(0.0);

        leftClaw = 1.0;
        rightClaw = 1.0;

        doDaSleep(1000);

        frontLeftWheel = 0.12;
        backLeftWheel = 0.12;
        backRightWheel = -0.12;
        frontRightWheel = -0.12;
        while(gyroPosition() < 90){

        }
        powerWheels(0.0);
        doDaSleep(500);
        correctionDrive(0.16, 200, BRW, 90);

        frontLeftWheel = 0.12;
        backLeftWheel = 0.12;
        backRightWheel = -0.12;
        frontRightWheel = -0.12;
        while(gyroPosition() < 170 && gyroPosition() > 0){

        }
        powerWheels(0.0);
        int ticks;
        if(skystonePos == 1) ticks = 2000;
        else if(skystonePos == 2) ticks = 1600;
        else ticks = 1850;
        int startingTicks = BRW.getCurrentPosition();
        while(BRW.getCurrentPosition() < startingTicks + ticks){
            if(gyroPosition() < 179 && gyroPosition() > 0){
                frontLeftWheel = 0.25 * 1.4;
                backLeftWheel = 0.25 * 1.4;
                backRightWheel = 0.25;
                frontRightWheel = 0.25;
            }
            else if(gyroPosition() > -179 && gyroPosition() < -45){
                frontLeftWheel = 0.25;
                backLeftWheel = 0.25;
                backRightWheel = 0.25 * 1.4;
                frontRightWheel = 0.25 * 1.4;
            }
            else{
                frontLeftWheel = 0.25;
                backLeftWheel = 0.25;
                backRightWheel = 0.25;
                frontRightWheel = 0.25;
            }
        }
        powerWheels(0.0);

        frontLeftWheel = 0.12;
        backLeftWheel = 0.12;
        backRightWheel = -0.12;
        frontRightWheel = -0.12;

        while(!(gyroPosition() < -85 && gyroPosition() > -95)){

        }
        powerWheels(0.0);

        leftLift = -0.5;
        rightLift = -0.5;
        doDaSleep(500);
        powerWheels(0.16);
        doDaSleep(1000);
        leftLift = 0.0;
        rightLift = 0.0;
        powerWheels(0.0);
        doDaSleep(1000);
        leftClaw = 0.6;
        rightClaw = 0.6;
        powerWheels(0.16);
        doDaSleep(500);
        powerWheels(0.0);
        leftFoundationClaw = 0.36;
        rightFoundationClaw = 0.36;
        doDaSleep(500);
        long startTime = System.currentTimeMillis();
        powerWheels(-0.16);
        while(System.currentTimeMillis() < startTime + 3000){
            if(gyroPosition() < -91){
                frontLeftWheel = -0.16 * 1.4;
                backLeftWheel = -0.16 * 1.4;
                backRightWheel = -0.16;
                frontRightWheel = -0.16;
            }
            else if(gyroPosition() > -89){
                frontLeftWheel = -0.16;
                backLeftWheel = -0.16;
                backRightWheel = -0.16 * 1.4;
                frontRightWheel = -0.16 * 1.4;
            }
            else{
                frontLeftWheel = -0.16;
                backLeftWheel = -0.16;
                backRightWheel = -0.16;
                frontRightWheel = -0.16;
            }
        }
        frontLeftWheel = 0.16;
        backLeftWheel = -0.16;
        backRightWheel = 0.16;
        frontRightWheel = -0.16;
        leftFoundationClaw = 0.0;
        rightFoundationClaw = 0.0;
        doDaSleep(4000);
        powerWheels(0.0);
    }

    public void redSafeAuto(){
        strafeRight(0.16, 600, BRW);
        doDaSleep(50);

        leftLift = -0.7;
        rightLift = -0.7;
        doDaSleep(500);
        leftLift = 0.0;
        rightLift = 0.0;

        correctionDrive(0.12, 165, BRW, 0);
        doDaSleep(750);
        if(!targetVisible){
            //correctionDrive(0.12, 150, BRW, 0);
            drive(0.12, 150, BRW);
            doDaSleep(750);
            if(!targetVisible) {
                //correctionDrive(0.12, 150, BRW, 0);
                drive(0.12, 150, BRW);
                skystonePos = 3;
            }
            else{
                skystonePos = 2;
            }
        }
        else{
            skystonePos = 1;
        }

        if(skystonePos == 1){
            drive(0.16, 150, BRW);
            doDaSleep(250);
            leftLift = -0.5;
            rightLift = -0.5;
            strafeRight(0.2, 430, BRW);
            leftLift = 0.0;
            rightLift = 0.0;
        }
        else if(skystonePos == 2){
            /*translation = lastLocation.getTranslation();
            pos = translation.get(1) / mmPerInch;
            powerWheels(-0.1);
            while(pos > 0.0){
                doDaSleep(250);
                powerWheels(0.0);
                doDaSleep(250);
                powerWheels(-0.1);
                translation = lastLocation.getTranslation();
                pos = translation.get(1) / mmPerInch;
            }*/
            powerWheels(0.0);
            driveBackwards(0.12, 355, BRW);
            doDaSleep(250);
            leftLift = -0.5;
            rightLift = -0.5;
            strafeRight(0.2, 430, BRW);
            leftLift = 0.0;
            rightLift = 0.0;
        }
        else{
            powerWheels(0.0);
            driveBackwards(0.12, 340, BRW);
            doDaSleep(250);
            leftLift = -0.5;
            rightLift = -0.5;
            strafeRight(0.2, 430, BRW);
            leftLift = 0.0;
            rightLift = 0.0;
        }

        doDaSleep(500);
        powerWheels(0.1);
        doDaSleep(500);
        powerWheels(0.0);

        leftClaw = 1.0;
        rightClaw = 1.0;

        doDaSleep(1000);

        frontLeftWheel = -0.12;
        backLeftWheel = -0.12;
        backRightWheel = 0.12;
        frontRightWheel = 0.12;
        while(gyroPosition() > -90){

        }
        powerWheels(0.0);
        doDaSleep(500);
        correctionDrive(0.16, 200, BRW, -90);

        frontLeftWheel = -0.12;
        backLeftWheel = -0.12;
        backRightWheel = 0.12;
        frontRightWheel = 0.12;
        while(gyroPosition() > -180 && gyroPosition() < 0){

        }
        powerWheels(0.0);
        doDaSleep(500);

        powerWheels(0.25);
        int ticks;
        if(skystonePos == 1) ticks = 800;
        else if(skystonePos == 2) ticks = 500;
        else ticks = 650;
        int startingTicks = BRW.getCurrentPosition();
        while(BRW.getCurrentPosition() < startingTicks + ticks){
            if(gyroPosition() < 179 && gyroPosition() > 0){
                frontLeftWheel = 0.25 * 1.4;
                backLeftWheel = 0.25 * 1.4;
                backRightWheel = 0.25;
                frontRightWheel = 0.25;
            }
            else if(gyroPosition() > -179 && gyroPosition() < -45){
                frontLeftWheel = 0.25;
                backLeftWheel = 0.25;
                backRightWheel = 0.25 * 1.4;
                frontRightWheel = 0.25 * 1.4;
            }
            else{
                frontLeftWheel = 0.25;
                backLeftWheel = 0.25;
                backRightWheel = 0.25;
                frontRightWheel = 0.25;
            }
        }
        powerWheels(0.0);
        rightClaw = 0.6;
        leftClaw = 0.6;
        doDaSleep(500);
        driveBackwards(0.16, 1000, BRW);
        powerWheels(0.0);
    }

    public void blueSafeAuto(){
        strafeLeft(0.16, 595, BRW);
        doDaSleep(50);

        leftLift = -0.7;
        rightLift = -0.7;
        doDaSleep(500);
        leftLift = 0.0;
        rightLift = 0.0;

        correctionDrive(0.12, 165, BRW, 0);
        doDaSleep(750);
        if(!targetVisible){
            //correctionDrive(0.12, 150, BRW, 0);
            drive(0.12, 150, BRW);
            doDaSleep(750);
            if(!targetVisible) {
                //correctionDrive(0.12, 150, BRW, 0);
                drive(0.12, 150, BRW);
                skystonePos = 3;
            }
            else{
                skystonePos = 2;
            }
        }
        else{
            skystonePos = 1;
        }

        if(skystonePos == 1){
            drive(0.16, 130, BRW);
            doDaSleep(500);
            leftLift = -0.5;
            rightLift = -0.5;
            strafeLeft(0.2, 435, BRW);
            leftLift = 0.0;
            rightLift = 0.0;
        }
        else if(skystonePos == 2){
            /*translation = lastLocation.getTranslation();
            pos = translation.get(1) / mmPerInch;
            powerWheels(-0.1);
            while(pos > 0.0){
                doDaSleep(250);
                powerWheels(0.0);
                doDaSleep(250);
                powerWheels(-0.1);
                translation = lastLocation.getTranslation();
                pos = translation.get(1) / mmPerInch;
            }*/
            powerWheels(0.0);
            driveBackwards(0.12, 355, BRW);
            doDaSleep(250);
            leftLift = -0.5;
            rightLift = -0.5;
            strafeLeft(0.2, 435, BRW);
            leftLift = 0.0;
            rightLift = 0.0;
        }
        else{
            powerWheels(0.0);
            driveBackwards(0.12, 340, BRW);
            doDaSleep(250);
            leftLift = -0.5;
            rightLift = -0.5;
            strafeLeft(0.2, 435, BRW);
            leftLift = 0.0;
            rightLift = 0.0;
        }

        doDaSleep(500);
        powerWheels(0.1);
        doDaSleep(500);
        powerWheels(0.0);

        leftClaw = 1.0;
        rightClaw = 1.0;

        doDaSleep(1000);

        frontLeftWheel = 0.12;
        backLeftWheel = 0.12;
        backRightWheel = -0.12;
        frontRightWheel = -0.12;
        while(gyroPosition() < 90){

        }
        powerWheels(0.0);
        doDaSleep(500);
        correctionDrive(0.16, 200, BRW, 90);

        frontLeftWheel = 0.12;
        backLeftWheel = 0.12;
        backRightWheel = -0.12;
        frontRightWheel = -0.12;
        while(gyroPosition() < 180 && gyroPosition() > 0){

        }
        powerWheels(0.0);
        doDaSleep(500);

        powerWheels(0.25);
        int ticks;
        if(skystonePos == 1) ticks = 2000;
        else if(skystonePos == 2) ticks = 1500;
        else ticks = 1650;
        int startingTicks = BRW.getCurrentPosition();
        while(BRW.getCurrentPosition() < startingTicks + ticks){
            if(gyroPosition() < 179 && gyroPosition() > 0){
                frontLeftWheel = 0.25 * 1.4;
                backLeftWheel = 0.25 * 1.4;
                backRightWheel = 0.25;
                frontRightWheel = 0.25;
            }
            else if(gyroPosition() > -179 && gyroPosition() < -45){
                frontLeftWheel = 0.25;
                backLeftWheel = 0.25;
                backRightWheel = 0.25 * 1.4;
                frontRightWheel = 0.25 * 1.4;
            }
            else{
                frontLeftWheel = 0.25;
                backLeftWheel = 0.25;
                backRightWheel = 0.25;
                frontRightWheel = 0.25;
            }
        }
        powerWheels(0.0);
        rightClaw = 0.6;
        leftClaw = 0.6;
        doDaSleep(500);
        driveBackwards(0.16, 1000, BRW);
        powerWheels(0.0);
    }

    private void oldAuto(){
        drive(0.1, 450, BRW);
        doDaSleep(50);

        int startPos = gyroPosition();
        int amount = 90;
        /*if(startPos + amount > 360) target = startPos + amount - 360;
        else if(startPos + amount < 0) target = startPos + amount + 360;
        else target = startPos + amount;*/

        frontLeftWheel = -0.16;
        backLeftWheel = -0.16;
        backRightWheel = 0.16;
        frontRightWheel = 0.16;
        while(gyroPosition() > -80){

        }
        powerWheels(0.0);
        doDaSleep(100);

        leftLift = -0.7;
        rightLift = -0.7;
        doDaSleep(500);
        leftLift = 0.0;
        rightLift = 0.0;

        correctionDrive(0.12, 165, BRW, -90);
        doDaSleep(750);
        if(!targetVisible){
            correctionDrive(0.12, 150, BRW, -90);
            doDaSleep(750);
            if(!targetVisible) {
                correctionDrive(0.12, 150, BRW, -90);
                skystonePos = 3;
            }
            else{
                skystonePos = 2;
            }
        }
        else{
            skystonePos = 1;
        }

        if(skystonePos == 1){

        }
        else if(skystonePos == 2){
            /*translation = lastLocation.getTranslation();
            pos = translation.get(1) / mmPerInch;
            powerWheels(-0.1);
            while(pos > 0.0){
                doDaSleep(250);
                powerWheels(0.0);
                doDaSleep(250);
                powerWheels(-0.1);
                translation = lastLocation.getTranslation();
                pos = translation.get(1) / mmPerInch;
            }*/
            powerWheels(0.0);
            driveBackwards(0.12, 335, BRW);
            doDaSleep(250);
            leftLift = -0.35;
            rightLift = -0.35;
            strafeRight(0.2, 490, BRW);
            leftLift = 0.0;
            rightLift = 0.0;
        }
        else{

        }

        doDaSleep(500);
        powerWheels(0.1);
        doDaSleep(500);
        powerWheels(0.0);

        leftClaw = 1.0;
        rightClaw = 1.0;

        doDaSleep(1000);

        frontLeftWheel = 0.12;
        backLeftWheel = 0.12;
        backRightWheel = -0.12;
        frontRightWheel = -0.12;
        while(gyroPosition() < -5){

        }
        powerWheels(0.0);
        doDaSleep(500);
        driveBackwards(0.16, 500, BRW);

        frontLeftWheel = 0.12;
        backLeftWheel = 0.12;
        backRightWheel = -0.12;
        frontRightWheel = -0.12;
        while(gyroPosition() < 90){

        }
        powerWheels(0.0);
        doDaSleep(1500);

        correctionDrive(0.33, 1900, BRW, 90);
    }

    public void autoForSomeRandomTeam(){
        leftLift = -0.7;
        rightLift = -0.7;
        doDaSleep(500);
        leftLift = 0.0;
        rightLift = 0.0;
        doDaSleep(1000);

        driveBackwards(0.12, 350, BRW);

    }

}
