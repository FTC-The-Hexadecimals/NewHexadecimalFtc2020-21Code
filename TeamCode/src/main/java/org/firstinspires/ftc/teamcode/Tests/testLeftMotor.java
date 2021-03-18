/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//Hexadecimal team TelaOp Code.
//Ultimate Goal 2020-21
//Started Development 11/21/2020

//Package Code
package org.firstinspires.ftc.teamcode.Tests;

//Import Library's

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


//TeleOp
@TeleOp(name ="Hexadecimal Tele-Op")
public class testLeftMotor extends LinearOpMode
{
    //Motor
    private DcMotor backLeft;
    private DcMotor backRight;

    //Spin motor
    private DcMotor shooter;

    //Intake system
    private DcMotor intake;

    //Conveyor belt
    private DcMotor conveyor_belt;

    //Arm
    private DcMotor arm;
    private CRServo claw;

    boolean shooter_power;
    boolean conveyor;

    //For image recognition
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    //Licenece Key
    private static final String VUFORIA_KEY =
            "ATFJ5Xn/////AAABmdfWhF4SuUWbrabci8PI2ws9KNTxn/7WE5EzCr8E9A1rfjjUGuw+JPkwT9bfsniH4NWMdwVDstqd2oaDN/5cbhotU1Bz8Q/USYmbE9dAUpWjCXXL4IytOPmjOoTN58TCZLXaZsodheLH1J861UDAQY+9+krP39tERjxfE9wybz5TSToM8ANTkw3NQsNNnpwZgqS048xE61A09PE/EFRkxfUtXCCMTBRhSAAKhzChoxVfoHo2QxnpY7uL6JcCfto1MSJTx0urj5ukmZQDVN+Qk+ieodcNx5fsmDBxVYAFv1HhIZxjoN5DPg4jsUacfVZJHt7Dr/TRo7zu/ZwkrrJh+SXn/Ly6tlRMRDYWIfDC92pF";
    private static final String VUFORIA_KEY_LOCATION =
            "Ac5mom3/////AAABmX1mmoe5OEUQvZDJT3MKKd4bvlQmi/7HZyWlfBeGhk1iVBDEfh70JEDPRdrAxE4bOqaog8GqbDw6eOvjqOhabY+Qq6YqINOVFB0cQxcsRM+BmFZ3I3EcoXdtWWybcyYYP3MI71KVBubLiQa5PI5SpQrPMj2bqPJPquqdG0Q43Mir12YhIcTVSuUYFeG301ZTf/EB3mfejAhfOYHiONq3dsTT/t8klHLUhOFS7p9wrdt2CbOnoz1G8/CUNm5WHLfVIadhedmpNVMj9H6pe+zzh4AvMYA1YRaYnR/piKEEPpHguGBfwM6YNHk0l3rcnDBwThHLcXVd9AEiAinelaHnkZzpOp7qhBdCu49k0l/7H+Ez";

    //Tenserflow Detection
    private TFObjectDetector tfod;
    //Boost mode for the wheels
    private boolean boost;

    //Distance sensor
    private DistanceSensor sensorRange;

    //Gyro
    private BNO055IMU gyroSensor;

    //Orientation field
    Orientation angles;

    //Camera fields
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension..
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private VuforiaLocalizer vuforia_location = null;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    //All trackable items
    private List<VuforiaTrackable> allTrackables;

    //Ultimate goal targets
    private VuforiaTrackables targetsUltimateGoal;

    //The Webcam class
    WebcamName webcamName = null;

    Orientation angleO;

    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)/2;
    static final double     DRIVE_SPEED             = 0.2;
    static final double     FAST_DRIVE_SPEED        = 0.5;
    static final double     TURN_SPEED              = 0.04;
    static final double     FASTER_TURN_SPEED              = 0.08;

    //Veforia parameters
    VuforiaLocalizer.Parameters parameters;

    private static final double     HEADING_THRESHOLD       = 4;      // As tight as we can make it with an integer gyro
    private static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable

    private boolean close;
    private boolean open;

    //Overide Run Op Mode Function
    @Override
    public void runOpMode()
    {

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.


        //Start motors and servos
        motorServerSetup();

        //Boost mode for the wheels
        boost = false;


        //Init Vuforia
        initVuforia();

        //Setup image recognition
        setupRecognition();

        //Setup Location
        setUpLocation();

        //Wait for start of match
        waitForStart();

        //Setup Gyro
        gyroSetup();
        double x = 0;
        double y = 0;
        double [] array = new double[2];

        //Run While Active
        while (opModeIsActive())
        {
            backLeft.setPower(1.0);
        }

        //Shutdown recognition
        if (tfod != null)
            tfod.shutdown();

        // Disable Tracking when we are done;
        targetsUltimateGoal.deactivate();
    }

    //This method sets up motors and servos
    private void motorServerSetup (){
        //Set Motors
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        backRight = hardwareMap.dcMotor.get("backRight");
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //Conveyor Belt
        conveyor_belt = hardwareMap.dcMotor.get("conveyor");
        conveyor_belt.setDirection(DcMotor.Direction.FORWARD);

        //Set Spin Motor Name
        arm = hardwareMap.dcMotor.get("arm");
        arm.setDirection(DcMotor.Direction.REVERSE);

        claw = hardwareMap.crservo.get("claw");

        //Set Arm Servo Names
        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotor.Direction.FORWARD);

        shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        close = false;
        open = false;
        shooter_power = false;
        conveyor = false;
    }

    //This method setups the gyro
    private void gyroSetup (){
        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        parameters.mode = BNO055IMU.SensorMode.IMU;

        // Retrieve and initialize the IMU. I2C port, "AdaFruit IMU"
        gyroSensor = hardwareMap.get(BNO055IMU.class, "imu");
        gyroSensor.initialize(parameters);

        //Wait for gyro to calibrate
        while(!isStopRequested() && !gyroSensor.isGyroCalibrated()) {}

    }

    //This method setups location
    private void setUpLocation (){

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = this.vuforia_location.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option. We are not using the camera on the hone ignore this
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward. Do we need this cuz this is for the Phone Camera
        if (CAMERA_CHOICE == BACK)
            phoneYRotate = -90;
        else
            phoneYRotate = 90;

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT)
            phoneXRotate = 90 ;

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 8.915f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.8f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 8.25f * mmPerInch;     // eg: Camera is ON the robot's center line

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

        targetsUltimateGoal.activate();
    }

    //This method sets up image recognition
    private void setupRecognition (){
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initTfod();


        //Activate TensorFlow Object Detection before we wait for the start command.
        //Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }
    }

    //This method controls inputs for gamepad1
    private void gamepad1Method (){
        /****************************************/
        /***************GamePad1*****************/
        /****************************************/
        //Set Variables*double left_stick_x1 = gamepad1.left_stick_x;
        double right_stick_x1 = gamepad1.right_stick_x;
        double left_stick_y1 = gamepad1.left_stick_y;

        //Range Clip Variables
        left_stick_y1 = Range.clip(left_stick_y1, -1, 1);
        right_stick_x1 = Range.clip(right_stick_x1, -1, 1);

        //Set correct Range Value
        left_stick_y1 = (float) scaleInput(left_stick_y1);
        right_stick_x1 = (float) scaleInput(right_stick_x1);

        //if(!boost) {
            left_stick_y1 = left_stick_y1 / 5;
            right_stick_x1 = right_stick_x1 / 5;
        //}

        if(gamepad1.b)
            boost = !boost;


        //Move Robot
        backLeft.setPower(left_stick_y1 - (right_stick_x1/2));
        backRight.setPower((left_stick_y1 + (right_stick_x1/2)));

            //90
            if(gamepad1.dpad_up)
                gyroTurnFaster( FASTER_TURN_SPEED, 90);


            //270
            if(gamepad1.dpad_down)
                gyroTurnFaster( FASTER_TURN_SPEED, 270);


            //0
            if(gamepad1.dpad_right)
                gyroTurnFaster( FASTER_TURN_SPEED, 0);


            //180
            if(gamepad1.dpad_left)
                gyroTurnFaster( FASTER_TURN_SPEED, 180);
    }

    //This method controls inputs for gamepad2
    private void gamepad2Method (){
        /****************************************/
        /***************GamePad2*****************/
        /****************************************/
        //Set Variables
        double left_stick_x2 = gamepad2.left_stick_x;
        double right_stick_y2 = gamepad2.right_stick_y;

        //Range Clip Variables
        left_stick_x2 = Range.clip(left_stick_x2, -1, 1);
        right_stick_y2= Range.clip(right_stick_y2, -1, 1);

        //Set correct Range Value
        left_stick_x2 = (float) scaleInput(left_stick_x2);
        right_stick_y2 = (float) scaleInput(right_stick_y2);

        //Move Spin Motor
        intake.setPower(left_stick_x2/2);

        //Move Arm
        arm.setPower(right_stick_y2/3);

        if(gamepad2.a) {
            shooter_power = !shooter_power;

            telemetry.addData("Pressed A", shooter_power);
            telemetry.update();
        }
        if(gamepad2.b) {
            conveyor = !conveyor;
            telemetry.addData("Pressed B", conveyor);
            telemetry.update();
        }
        if(gamepad2.x) {
            close = !close;
            telemetry.addData("Pressed X", close);
            telemetry.update();
        }
        if(gamepad2.y) {
            open = !open;
            telemetry.addData("Pressed Y", open);
            telemetry.update();
        }

        if(shooter_power){
            shooter.setPower(0.55);
        }

        if(conveyor) {
            conveyor_belt.setPower(0.5);
        }

        if(close) {
            claw.setPower(-0.2);
        }

        if(open){
            claw.setPower(0.2);
        }
    }

    //Scale the input of the controller
    //Idea taken from Terry Wang Flex group TeleOp
    double scaleInput(double point)
    {
        //Create array of possible values
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        //Set index
        int index = (int) (point * 16.0);

        //Index is out of range
        if (index < 0)
            index = -index;

        //Index is out of range
        if (index > 16)
            index = 16;

        //Set Scale
        double ret_val = 0.0;

        //Scale out of range
        if (point < 0)
            ret_val = -scaleArray[index];

        //Set dScale
        else
            ret_val = scaleArray[index];

        //Return dScale
        return ret_val;
    }


    //Initialize the Vuforia localization engine.
    private void initVuforia() {
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY_LOCATION;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia_location = ClassFactory.getInstance().createVuforia(parameters);
    }


    //Initialize the TensorFlow Object Detection engine.
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia_location);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    //Return List of items
    private List<Recognition> findItems() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
            }

            return updatedRecognitions;
        }
        return null;
    }

    private double [] getLocation(){
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;

        double [] ret = new double[2];

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            ret [0] = translation.get(0) / mmPerInch;
            ret [1] = translation.get(1) / mmPerInch;

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }

        return ret;
    }

    private double getAngle(){
        angleO = gyroSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);
        return unNormalize(angleO.firstAngle);
    }

    private double unNormalize(float degrees){

        if(degrees == 180 || degrees == -180)
            return 270.00;
        else if(degrees >= 0) {
            return degrees + 90;
        }
        else if(degrees >= -90)
            return 90 + degrees;
        else if(degrees > -180)
            return 450 + degrees;
        return 0.00;
    }


    /*private void turn(double degrees){
        while (!(getAngle() >= degrees - 3 && getAngle() <= degrees + 3) && opModeIsActive()){
            backRight.setPower(0.3);
            backLeft.setPower(-0.3);
        }
    }*/

    public void gyroTurn (double speed, double angle){
        gyroTurnWithError(speed, angle+3);
    }

    public void gyroTurnFaster (double speed, double angle){
        gyroTurnWithError(speed, angle+9);
    }

    public void gyroTurnWithError (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) { }
    }


    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        backLeft.setPower(leftSpeed);
        backRight.setPower(rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError = 0.00;

        robotError = targetAngle - getAngle();
        while (robotError > 360)  robotError -= 360;
        while (robotError < 0) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    private void goTo(double x, double y){
        double [] location = getLocation();

        double x_distance = Math.abs(x - location[0]) * -1;
        double y_distance = y - location[1];
        double angle = 180;
        //double hypotenuse =  Math.sqrt(Math.pow(x_distance,2.00) + Math.pow(y_distance,2.00));

        //double angle = Math.atan(y_distance/x_distance);

        if (location[0] > x)
            angle = 0;

        gyroTurn( TURN_SPEED, angle);

        encoderDrive(DRIVE_SPEED,  x_distance,  x_distance, 30.0);
    }

    private void moveTo(double x_distance, double y_distance){
        double hypotenuse =  Math.sqrt(Math.pow(x_distance,2.00) + Math.pow(y_distance,2.00));

        double angle = Math.atan(y_distance/x_distance);

        gyroTurn(TURN_SPEED, angle);

        encoderDrive(DRIVE_SPEED,  hypotenuse,  hypotenuse, 60.0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS){
        if (leftInches > 0) {
            leftInches -= 1;
            rightInches -= 1;
        }
        else {
            leftInches += 1;
            rightInches += 1;
        }
        encoderDriveWithError(speed,(leftInches),(rightInches), timeoutS);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDriveWithError(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = backLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = backRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            backLeft.setTargetPosition(newLeftTarget);
            backRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (backLeft.isBusy() && backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        backLeft.getCurrentPosition(),
                        backRight.getCurrentPosition());
            }

            // Stop all motion;
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}