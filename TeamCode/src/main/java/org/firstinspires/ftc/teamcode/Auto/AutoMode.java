/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name="Hexadecimal Auto")
public class AutoMode extends LinearOpMode {
    //Motor
    private DcMotor backLeft;
    private DcMotor backRight;

    //Spin motor
    private DcMotor shooter1;
    private DcMotor shooter2;

    //Servo Arm
    private DcMotor arm1;
    private DcMotor arm2;

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

    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)/2;
    static final double     DRIVE_SPEED             = 0.2;
    static final double     FASTER_DRIVE_SPEED      = 0.4;
    static final double     TURN_SPEED              = 0.04;
    static final double     FASTER_TURN_SPEED              = 0.08;

    //Veforia parameters
    VuforiaLocalizer.Parameters parameters;

    static final double     HEADING_THRESHOLD       = 4;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable

    //Current cords of the robot
    double x_cord;
    double y_cord;

    boolean soundFound;

    @Override
    public void runOpMode() {

        int soundId = hardwareMap.appContext.getResources().getIdentifier("sound", "raw", hardwareMap.appContext.getPackageName());

        // Determine if sound resources are found.
        // Note: Preloading is NOT required, but it's a good way to verify all your sounds are available before you run.
        if (soundId != 0)
            soundFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, soundId);

        int num_rings = 0;

        List<Recognition> data = null;

        //Start motors and servos
        motorServerSetup();

        gyroSetup();

        //Init Vuforia
        initVuforia();

        //Setup image recognition
        setupRecognition();

        //Wait for start of the match
        waitForStart();

        //Setup Gyro
        encoderDrive(DRIVE_SPEED,-12,-12,60.00);

        for (int i=0;i<1500000;++i) {
            data = findItems();

            if(data != null)
                break;
        }

        if(data != null) {
            if (data.get(0).getLabel() == "Quad")
                num_rings = 4;
            else if (data.get(0).getLabel() == "Single")
                num_rings = 1;
        }
        else
            num_rings = 0;

            telemetry.addData("Data: ", data);
            telemetry.addData("Number rings: ", num_rings);
            telemetry.update();

        if (num_rings == 4) {
            for (int i=0;i<4;++i)
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundId);
            encoderDriveFaster(FASTER_DRIVE_SPEED, -96, -96, 60.0);
            encoderDriveFaster(FASTER_DRIVE_SPEED, 41, 41, 60.0);
            gyroTurnFaster(FASTER_TURN_SPEED,270,true);
        }
        if (num_rings == 0) {
            encoderDrive(DRIVE_SPEED, -53, -53, 60.00);

            gyroTurnFaster(FASTER_TURN_SPEED,270,true);
        }
        if (num_rings == 1){
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundId);
            encoderDriveFaster(FASTER_DRIVE_SPEED,-68,-68,60.00);
            gyroTurn(TURN_SPEED,140,true);
            encoderDriveFaster(FASTER_DRIVE_SPEED,-10,-10,60.00);

            gyroTurn(TURN_SPEED,90,false);
            encoderDriveFaster(FASTER_DRIVE_SPEED,15,15,60.00);
            gyroTurnFaster(FASTER_TURN_SPEED,270,true);
        }


        //moveTo(130,11.325);
        /*encoderDrive(DRIVE_SPEED,125,125,60.0);
        gyroTurn(TURN_SPEED,85, false);
        encoderDrive(DRIVE_SPEED,5,5, 60.0);*/

        //Shutdown recognition
        if (tfod != null)
            tfod.shutdown();

    }

    //This method sets up motors and servos
    private void motorServerSetup (){
        //Set Motors
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        backRight = hardwareMap.dcMotor.get("backRight");
        backRight.setDirection(DcMotor.Direction.REVERSE);

        /*//Set Spin Motor Name
        shooter1 = hardwareMap.dcMotor.get("shooter1");
        shooter1.setDirection(DcMotor.Direction.FORWARD);

        //Set Spin Motor Name
        shooter2 = hardwareMap.dcMotor.get("shooter2");
        shooter2.setDirection(DcMotor.Direction.REVERSE);*/

        /*//Set Arm Servo Names
        arm1 = hardwareMap.dcMotor.get("arm1");
        arm1.setDirection(DcMotor.Direction.REVERSE);

        arm2 = hardwareMap.dcMotor.get("arm2");
        arm2.setDirection(DcMotor.Direction.REVERSE);*/

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    private void moveTo(double x_distance, double y_distance){
        double hypotenuse =  Math.sqrt(Math.pow(x_distance,2.00) + Math.pow(y_distance,2.00));

        double angle = Math.atan(y_distance/x_distance);

        gyroTurn(TURN_SPEED, angle, false);

        encoderDrive(DRIVE_SPEED,  hypotenuse * -1 ,  hypotenuse * -1, 60.0);

        x_cord += x_distance;
        y_cord += y_distance;

        telemetry.addData("x_cord: ", x_cord);
        telemetry.addData("y_cord: ", y_cord);
        telemetry.update();
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

        encoderDriveWithError(speed,leftInches,rightInches, timeoutS);
    }

    public void encoderDriveFaster(double speed,
                             double leftInches, double rightInches,
                             double timeoutS){
        if (leftInches > 0) {
            leftInches -= 3;
            rightInches -= 3;
        }
        else {
            leftInches += 3;
            rightInches += 3;
        }

        encoderDriveWithError(speed,leftInches,rightInches, timeoutS);
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
                    (backLeft.isBusy() && backRight.isBusy())) { }

            // Stop all motion;
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void gyroTurn (double speed, double angle, boolean left){
        gyroTurnWithError(speed, angle+3, left);
    }

    public void gyroTurnFaster (double speed, double angle, boolean left){
        gyroTurnWithError(speed, angle+9, left);
    }

    public void gyroTurnWithError (  double speed, double angle, boolean left) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF,left)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
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
    boolean onHeading(double speed, double angle, double PCoeff, boolean left) {
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

            if (!left)
                leftSpeed   = -rightSpeed;
            else{
                leftSpeed = rightSpeed;
                rightSpeed *= -1;
            }
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

        double robotError;

        // calculate error in -179 to +180 range  (
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

    private double getAngle(){
        angleO = gyroSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);
        return unNormalize(angleO.firstAngle);
    }

    private double unNormalize(float degrees){

        if(degrees == 180 || degrees == -180)
            return 270.00;
        else if(degrees >= 0)
            return degrees + 90;
        else if(degrees >= -90)
            return 90 + degrees;
        else if(degrees > -180)
            return 450 + degrees;
        return 0.00;
    }

    //Return List of items
    private List<Recognition> findItems() {
        int i=0;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                for (Recognition recognition : updatedRecognitions) {
                    ++i;
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
            }

            telemetry.update();

            if(i == 0)
                return null;
            return updatedRecognitions;
        }
        return null;
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
}






















