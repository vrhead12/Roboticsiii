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

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import java.util.Locale;



/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


@TeleOp(name="Rhead Test", group="Linear Opmode")
// @Disabled

public class CompV1 extends LinearOpMode {

    // Static Fields
    private static boolean blueRobot = true;

    private static final double CLICKS_PER_REV = 735;    // 723.24 larger is further
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double WHEEL_DIAMETER_CM = WHEEL_DIAMETER_INCHES * 2.54;
    private static final double CLICKS_PER_INCH = CLICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
    private static final double CLICKS_PER_CM = CLICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_CM);
    private static final double RIGHT_LATERAL_ADJUST = 0;         // Motor speed adjustment to go straight laterally .2

    private static final double ENCODER_DRIVE_SPEED     = .5;       // was .2
    private static final double ENCODER_ROTATE_SPEED    = .3;      // was .05
    private static final double DRIVE_SPEED             = 0.4;     // Nominal speed for better accuracy.
    private static final double TURN_SPEED              = 0.4;     // was .25 Nominal half speed for better accuracy.

    private static final double HEADING_THRESHOLD       = 0.5;      // As tight as we can make it with without gyrating back and forth
    private static final double P_TURN_COEFF            = 0.03;     // was .03 Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFF           = 0.03;     // was .03 Larger is more responsive, but also less stable
    private static final double MIN_TURN_SPEED          = .35;      // was .1 Minimum motor speed for active turn
    private static final double SPEED_DELTA             = .2;       // .15 Single motor speed delta to maintain straight line

    // Declare motor variables
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftLateral = null;
    private DcMotor rightLateral = null;

    // Declare Timers
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime optime = new ElapsedTime();

    // Declare servo variables


    // Declare sensor variables
    ModernRoboticsI2cRangeSensor rangeSensor;
    ColorSensor colorSensor;
    BNO055IMU imu;  // IMU sensor included in Rev hub
    Orientation angles;
    Acceleration gravity;

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AcfKRnX/////AAABmX+JA9YwME1AmwEXwgYkhNiEM5nU1pQCbikV+dBZsxTD7+JfzhMy42P5w478betxEoyfo1OuuZgeSTw/+8TGPGWIPUK31uk3amFLSDA6oGGI2uk+g6Fhsrw/cc1NXj8PA/4ZlGTAWgUsXBkycgd+w65C/TaBWImZWshIgbHChYmMIyN3xiY2hu6m5eNuX+2DtzoJOFCYKw9FZ5nVctzX9XpFZfqg1Ye7FTWagQ3ShRvA+dqutW1+rDKlOQnjNQbk7xAzblKXbtrg+NCpGrVutWT0kLOGET4IxHCPhZ7QVDQc2hj78RhEqnMtyU6ZGHdW7OVK2uWQ4MywO0xvhJMBJECsxmdTNxdVN8yvVQDnv3xC";


    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float cmPerInch       = 2.54f;
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private List<VuforiaTrackable> allTrackables;

    // Declare robot location variables
    private int roosterSoundID;
    private int mooSoundID;
    private double robotX = 0;
    private double robotY = 0;
    private double robotZ = 0;
    private double robotAngle = 0;
    private double[] x, y, ang;
    private int numSamples = 10;
    private String currentTarget;
    private boolean slowMode = true;

    private String stoneTargetName = "Stone Target";
    private String blueRearBridgeName = "Blue Rear Bridge";
    private String redRearBridgeName = "Red Rear Bridge";
    private String redFrontBridgeName = "Red Front Bridge";
    private String blueFrontBridgeName = "Blue Front Bridge";
    private String red1Name = "Red Perimeter 1";
    private String red2Name = "Red Perimeter 2";
    private String front1Name = "Front Perimeter 1";
    private String front2Name = "Front Perimeter 2";
    private String blue1Name = "Blue Perimeter 1";
    private String blue2Name = "Blue Perimeter 2";
    private String rear1Name = "Rear Perimeter 1";
    private String rear2Name = "Rear Perimeter 2";

    // Vuforia Target Calibration
    //    Place robot location read point at intersection point of the four tiles that make up a corner
    //    Vuforia (0, 0) coordinate is at the center of the field
    //    Positive x axis points towards the rear wall
    //    Postitive y axis points towards blue side
    //    (rear, blue) calibration point (47.25, 47.25)
    //    (rear, red) (47.25, -47.25)
    //    (front, blue) (-47.25, 47.25)
    //    (front, red) (-47.25, -47.25)
    // The offset values are used to correct the robot reading to the known x, y location value
    private double checkValue = 47.25;
    private double stoneTargetOffsetX = 0.0;
    private double stoneTargetOffsetY = 0.0;
    private double blueRearBridgeOffsetX = 0.0;
    private double blueRearBridgeOffsetY = 0.0;
    private double redRearBridgeOffsetX = 0.0;
    private double redRearBridgeOffsetY = 0.0;
    private double redFrontBridgeOffsetX = 0.0;
    private double redFrontBridgeOffsetY = 0.0;
    private double blueFrontBridgeOffsetX = 0.0;
    private double blueFrontBridgeOffsetY = 0.0;
    private double red1OffsetX = 0.0;
    private double red1OffsetY = 0.0;
    private double red2OffsetX = 0.0;
    private double red2OffsetY = 0.0;
    private double front1OffsetX = 0.0;
    private double front1OffsetY = 0.0;
    private double front2OffsetX = 0.0;
    private double front2OffsetY = 0.0;
    private double blue1OffsetX = 0.0;
    private double blue1OffsetY = 0.0;
    private double blue2OffsetX = 0.0;
    private double blue2OffsetY = 0.0;
    private double rear1OffsetX = 0.0;
    private double rear1OffsetY = 0.0;
    private double rear2OffsetX = 0.0;
    private double rear2OffsetY = 0.0;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "forwardLeft");
        rightDrive = hardwareMap.get(DcMotor.class, "forwardRight");
        leftLateral  = hardwareMap.get(DcMotor.class, "lateralLeft");
        rightLateral = hardwareMap.get(DcMotor.class, "lateralRight");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftLateral.setDirection(DcMotor.Direction.FORWARD);
        rightLateral.setDirection(DcMotor.Direction.REVERSE);

        // Sensor Initializations
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distanceSensorFront");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Set the LED in the beginning
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;
        //colorSensor.enableLed(true);

        // Initialize sound files
        boolean roosterSoundFound = false;
        boolean mooSoundFound = false;
        roosterSoundID = hardwareMap.appContext.getResources().getIdentifier("roost", "raw", hardwareMap.appContext.getPackageName());
        mooSoundID   = hardwareMap.appContext.getResources().getIdentifier("moo",   "raw", hardwareMap.appContext.getPackageName());

        if (roosterSoundID != 0)
            roosterSoundFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, roosterSoundID);

        if (mooSoundID != 0)
            mooSoundFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, mooSoundID);

        // Display sound status
        telemetry.addData("rooster sound",   roosterSoundFound ?   "Found" : "NOT found\n Add rooster.wav to /src/main/res/raw" );
        telemetry.addData("moo sound", mooSoundFound ? "Found" : "Not found\n Add moo.wav to /src/main/res/raw" );
        telemetry.update();

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parametersVu = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parametersVu = new VuforiaLocalizer.Parameters();

        parametersVu.vuforiaLicenseKey = VUFORIA_KEY;
        parametersVu.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parametersVu);

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
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

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

        currentTarget = "";

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 5.5f * mmPerInch;   // eg: Camera is 8.5 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 2.5f * mmPerInch;   // eg: Camera is 1.75 Inches right of robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parametersVu.cameraDirection);
        }
        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
        double checkTime = runtime.milliseconds() + 3000;
        while (runtime.milliseconds() < checkTime)
        {
            getRobotLocation();
            if (targetVisible)
                break;
        }
        if (targetVisible)
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, roosterSoundID); // Play Sound found location
        else
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, mooSoundID); // Did not find location for autonomous

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        boolean initialized = imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

        // Set up our telemetry dashboard
        //composeTelemetry();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("After composeTelemetry", "IMU initialized: %b", false);
        telemetry.update();
        sleep(2000);

        telemetry.addData("Vuforia", "Initialized");
        telemetry.addData("Target Visible", targetVisible);
        telemetry.addData("Visible Target", currentTarget);
        telemetry.addData("Location", "{X, Y, Z} = %.1f, %.1f, %.1f",
                robotX, robotY, robotZ);
        telemetry.addData("Angle", "%.0f Degrees", robotAngle);
        telemetry.addData("Gyro", "IMU initialized: %b", initialized);
        telemetry.addData("Gyro Heading", "Angle: %.2f", angles.firstAngle);
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Get Current Heading
            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

            // Get Robot Location on Field
            getRobotLocation();

            Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                    (int) (colorSensor.green() * SCALE_FACTOR),
                    (int) (colorSensor.blue() * SCALE_FACTOR),
                    hsvValues);

            // Tank Mode uses one stick to control each axis direction.
            double drivePower;
            double lateralPower;

            if (slowMode)
            {
                drivePower = -gamepad1.right_stick_y/2.0;
                lateralPower = gamepad1.left_stick_x/2.0;
            }
            else
            {
                drivePower = Math.pow(-gamepad1.right_stick_y, 3);
                lateralPower = Math.pow(gamepad1.left_stick_x, 3);
            }
            // Send calculated power to wheels
            leftDrive.setPower(drivePower);
            rightDrive.setPower(drivePower);
            leftLateral.setPower(lateralPower);
            rightLateral.setPower(lateralPower);

            double rotationPower;
            if (gamepad1.right_trigger > .05)
            {
                if (slowMode)
                {
                    rotationPower = gamepad1.right_trigger/2.0;
                }
                else
                {
                    rotationPower = Math.pow(gamepad1.right_trigger, 2);
                }
                leftDrive.setPower(rotationPower);
                rightDrive.setPower(-rotationPower);
                leftLateral.setPower(-rotationPower);
                rightLateral.setPower(rotationPower);
            }
            else if (gamepad1.left_trigger > .05)
            {
                if (slowMode)
                {
                    rotationPower = gamepad1.left_trigger/2.0;
                }
                else {
                    rotationPower = Math.pow(gamepad1.left_trigger, 2);
                }
                leftDrive.setPower(-rotationPower);
                rightDrive.setPower(rotationPower);
                leftLateral.setPower(rotationPower);
                rightLateral.setPower(-rotationPower);
            }

            if (gamepad1.dpad_up)
            {
                //moveDriveDistanceInch(ENCODER_DRIVE_SPEED, 23.0+5.0/8.0, true, 5000);
                checkTime = runtime.milliseconds() + 3000;
                while (runtime.milliseconds() < checkTime)
                {
                    setLateralStraightSpeed(.5);
                }
                leftLateral.setPower(0.0);
                rightLateral.setPower(0.0);
            }
            else if (gamepad1.dpad_down)
            {
                //moveDriveDistanceInch(ENCODER_DRIVE_SPEED, -(23.0+5.0/8.0), true, 5000);
                checkTime = runtime.milliseconds() + 3000;
                while (runtime.milliseconds() < checkTime)
                {
                    setLateralStraightSpeed(-.5);
                }
                leftLateral.setPower(0.0);
                rightLateral.setPower(0.0);
            }
            else if (gamepad1.dpad_right)
            {
                findBlackStone();
            }
            else if (gamepad1.dpad_left)
            {
                //driveUntilDistanceInch (.5, 15.0, true, 5000);
                moveLateralDistanceInch(.5, (23.0+5.0/8.0)*4.0, true, 4000);
            }

            if (gamepad1.right_bumper)
            {
                gyroDrive(DRIVE_SPEED, 23.0+5.0/8.0, 0.0, true);
            }
            else if (gamepad1.left_bumper)
            {
                if (slowMode)
                    slowMode = false;
                else
                    slowMode = true;
            }

            if (gamepad1.x)
            {
                //gyroTurn( TURN_SPEED, 90.0, true, 2500);
                moveToLocation(47.25, -36, .3, 5000);
            }
            else if (gamepad1.b)
            {
                //gyroTurn( TURN_SPEED, 270, true, 2500);
                moveToLocation(-47.25, -36, .3, 5000);
            }
            else if (gamepad1.y)
            {
                //gyroTurn( TURN_SPEED, 0.0, true, 2500);
                moveToLocation(0.0, -47.25, .3, 5000);
            }
            else if (gamepad1.a)
            {
                //gyroTurn( TURN_SPEED, 180.0, true, 2500);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Drive (%.2f), Lateral (%.2f)", drivePower, lateralPower);
            telemetry.addData("Target Visible", "%b %s", targetVisible, currentTarget);
            telemetry.addData("Location", "{X, Y} = %.1f, %.1f", robotX, robotY);
            telemetry.addData("Angle", "%.1f Degrees", robotAngle);
            telemetry.addData("Gyro Heading", "Angle: %.2f", angles.firstAngle);
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.update();
        }
    }

    private void getRobotLocation()
    {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                currentTarget = trackable.getName();
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

            robotX = translation.get(0) / mmPerInch;
            robotY = translation.get(1) / mmPerInch;
            robotZ = translation.get(2) / mmPerInch;
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            robotAngle = rotation.thirdAngle;

            if (currentTarget.equals(blueRearBridgeName))
            {
                robotX += blueRearBridgeOffsetX;
                robotY += blueRearBridgeOffsetY;
            }
            else if (currentTarget.equals(redRearBridgeName))
            {
                robotX += redRearBridgeOffsetX;
                robotY += redRearBridgeOffsetY;
            }
            else if (currentTarget.equals(redFrontBridgeName))
            {
                robotX += redFrontBridgeOffsetX;
                robotY += redFrontBridgeOffsetY;
            }
            else if (currentTarget.equals(blueFrontBridgeName))
            {
                robotX += blueFrontBridgeOffsetX;
                robotY += blueFrontBridgeOffsetY;
            }
            else if (currentTarget.equals(red1Name))
            {
                robotX += red1OffsetX;
                robotY += red1OffsetY;
            }
            else if (currentTarget.equals(red2Name))
            {
                robotX += red2OffsetX;
                robotY += red2OffsetY;
            }
            else if (currentTarget.equals(front1Name))
            {
                robotX += front1OffsetX;
                robotY += front1OffsetY;
            }
            else if (currentTarget.equals(front2Name))
            {
                robotX += front2OffsetX;
                robotY += front2OffsetY;
            }
            else if (currentTarget.equals(blue1Name))
            {
                robotX += blue1OffsetX;
                robotY += blue1OffsetY;
            }
            else if (currentTarget.equals(blue2Name))
            {
                robotX += blue2OffsetX;
                robotY += blue2OffsetY;
            }
            else if (currentTarget.equals(rear1Name))
            {
                robotX += rear1OffsetX;
                robotY += rear1OffsetY;
            }
            else if (currentTarget.equals(rear2Name))
            {
                robotX += rear2OffsetX;
                robotY += rear2OffsetY;
            }
        }
        else {
            currentTarget = "None";
        }
    }

    private void moveDriveDistanceInch (double speed, double inches, boolean brake, double maxTimeMS)
    {
        moveDriveDistanceCM (speed, inches*cmPerInch, brake, maxTimeMS);
    }

    private void moveDriveDistanceCM (double speed, double cm, boolean brake, double maxTimeMS) {
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (brake)
        {
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else
        {
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        int leftTargetPosition = leftDrive.getCurrentPosition() + (int) (CLICKS_PER_CM * cm);
        int rightTargetPosition = rightDrive.getCurrentPosition() + (int) (CLICKS_PER_CM * cm);

        leftDrive.setTargetPosition(leftTargetPosition);
        rightDrive.setTargetPosition(rightTargetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        optime.reset();
        optime.startTime();
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() && (optime.milliseconds() < maxTimeMS) &&
                (leftDrive.isBusy() || rightDrive.isBusy()))
        {
            // Display it for the driver.
            telemetry.addData("Left Drive ",  "Running to %7d :%7d",
                    leftDrive.getCurrentPosition(), leftTargetPosition);
            telemetry.addData("Right Drive ",  "Running to %7d :%7d",
                    rightDrive.getCurrentPosition(), rightTargetPosition);
            telemetry.update();
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Reset Mode to Original State
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void moveLateralDistanceInch (double speed, double inches, boolean brake, double maxTimeMS)
    {
        moveLateralDistanceCM (speed, inches*cmPerInch, brake, maxTimeMS);
    }

    private void moveLateralDistanceCM (double speed, double cm, boolean brake, double maxTimeMS)
    {
        leftLateral.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLateral.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (brake)
        {
            leftLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else
        {
            leftLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        int leftTargetPosition = leftLateral.getCurrentPosition() + (int) (CLICKS_PER_CM * cm);
        int rightTargetPosition = rightLateral.getCurrentPosition() + (int) (CLICKS_PER_CM * cm);

        leftLateral.setTargetPosition(leftTargetPosition);
        rightLateral.setTargetPosition(rightTargetPosition);

        leftLateral.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLateral.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        optime.reset();
        optime.startTime();
        leftLateral.setPower(Math.abs(speed));
        rightLateral.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() && (optime.milliseconds() < maxTimeMS) &&
                (leftLateral.isBusy() || rightLateral.isBusy()))
        {
            // Display it for the driver.
            telemetry.addData("Left Lateral ",  "Running to %7d :%7d",
                    leftDrive.getCurrentPosition(), leftTargetPosition);
            telemetry.addData("Right Lateral ",  "Running to %7d :%7d",
                    rightDrive.getCurrentPosition(), rightTargetPosition);
            telemetry.update();
        }

        // Stop all motion;
        leftLateral.setPower(0);
        rightLateral.setPower(0);

        // Reset Mode to Original State
        leftLateral.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLateral.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void moveToLocation (double x, double y, double speed, double maxTimeMS)
    {
        if (targetVisible) {
            getRobotLocation();
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, roosterSoundID); // Play Sound found location
            double minX = Math.abs(robotX - x);
            double minY = Math.abs(robotY - y);
            gyroTurn(.4, 0.0, true, 500);

            moveDriveDistanceInch(speed, robotY - y, true, maxTimeMS);
            gyroTurn(.4, 0.0, true, 500);
            moveLateralDistanceInch(speed, robotX - x, true, maxTimeMS);

        }
        else {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, mooSoundID); // Did not find location for autonomous
        }
    }

    private void driveUntilDistanceInch (double speed, double inches, boolean brake, double maxTimeMS)
    {
        driveUntilDistanceCM (speed, inches*cmPerInch, brake, maxTimeMS);
    }

    public void driveUntilDistanceCM(double speed, double distanceCM, boolean brake, double maxTimeMS)
    {
        if (brake)
        {
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else
        {
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // keep looping while we have time remaining.
        ElapsedTime moveTimer = new ElapsedTime();
        moveTimer.reset();

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        double scaleDistance = 50;
        double calcDistance = 50;
        while (opModeIsActive() && rangeSensor.rawUltrasonic() > distanceCM && moveTimer.milliseconds() < maxTimeMS) {
            double dist = rangeSensor.rawUltrasonic();
            if (dist < scaleDistance)
            {
                calcDistance = dist - distanceCM;
            }
            double speedScaleFactor = calcDistance/scaleDistance;
            double scaleSpeed = speed*speedScaleFactor;
            if (scaleSpeed < .2)
                scaleSpeed = .2;
            leftDrive.setPower(scaleSpeed);
            rightDrive.setPower(scaleSpeed);

            // Update telemetry & Allow time for other processes to run.
            //telemetry.addData("Target Distance", distanceCM);
            //telemetry.addData("Current Distance", rangeSensor.rawUltrasonic());
            //telemetry.addData("Time", "%.2f of %.2f", moveTimer.milliseconds(), maxMoveTime);
            //telemetry.update();
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftLateral.setPower(0);
        rightLateral.setPower(0);
    }

    void findBlackStone()
    {
        // Move close to stones
        driveUntilDistanceInch (.5, 2.0, true, 5000);
        gyroTurn(.4, 0, true, 500);

        // Move sideways until target is visible

        while (colorSensor.red() > 20) {
            setLateralStraightSpeed(.2);
            //leftLateral.setPower(.3);
            //rightLateral.setPower(.3);
        }
        leftLateral.setPower(0);
        rightLateral.setPower(0);


    }

    void setLateralStraightSpeed (double speed)
    {
        //getRobotLocation();
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        robotAngle = angles.firstAngle;

        if (blueRobot)
        {
            // Move lateral towards front wall
            if (speed > 0)
            {
                if (robotAngle > HEADING_THRESHOLD)
                {
                    rightLateral.setPower(speed+RIGHT_LATERAL_ADJUST+SPEED_DELTA);
                    leftLateral.setPower(speed);
                    telemetry.addData("Adjust", " RIGHT");
                }
                else if (robotAngle < -HEADING_THRESHOLD)
                {
                    leftLateral.setPower(speed+SPEED_DELTA);
                    rightLateral.setPower(speed+RIGHT_LATERAL_ADJUST);
                    telemetry.addData("Adjust", " LEFT");
                }
                else
                {
                    leftLateral.setPower(speed);
                    rightLateral.setPower(speed+RIGHT_LATERAL_ADJUST);
                    telemetry.addData("Adjust", " STRAIGHT");
                }
            }
            else
            {
                if (robotAngle < -HEADING_THRESHOLD)
                {
                    rightLateral.setPower(speed-RIGHT_LATERAL_ADJUST-SPEED_DELTA);
                    leftLateral.setPower(speed);
                    telemetry.addData("Adjust", " LEFT");
                }
                else if (robotAngle > HEADING_THRESHOLD)
                {
                    leftLateral.setPower(speed-SPEED_DELTA);
                    rightLateral.setPower(speed-RIGHT_LATERAL_ADJUST);
                    telemetry.addData("Adjust", " RIGHT");
                }
                else
                {
                    leftLateral.setPower(speed);
                    rightLateral.setPower(speed+RIGHT_LATERAL_ADJUST);
                    telemetry.addData("Adjust", " STRAIGHT");
                }
            }
        }

        // Red Robot
        else
        {
            telemetry.addData("Robot", " RED");

        }
        telemetry.addData("Lateral", "Left (%.2f), Right (%.2f)", leftLateral.getPower(),
                rightLateral.getPower());
        telemetry.addData("Angle", "%.1f Degrees", robotAngle);
        telemetry.update();
    }

    void driveToLocationEncoders (double x, double y, double speed, boolean brake, double timeOutS)
    {

    }

    void composeTelemetry()
    {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable()
        {
            @Override public void run()
            {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
            }
        });

        telemetry.addLine().addData("status",
                new Func<String>() {
                    @Override public String value()
                    {
                        return imu.getSystemStatus().toShortString();
                    }
                }).addData("calib", new Func<String>() {
            @Override public String value() {
                return imu.getCalibrationStatus().toString();
            }
        });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed, double distance, double angle, boolean brake)
    {
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            if (brake)
            {
                leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else
            {
                leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                leftLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                rightLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * CLICKS_PER_INCH);
            newLeftTarget = leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = rightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftDrive.setPower(speed);
            rightDrive.setPower(speed);

            // Adjust angle to be positive 0 - 360
            if (angle < 0)
                angle += 360;

            // keep looping and adjusting steer angle while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                Range.clip(steer, -TURN_SPEED, TURN_SPEED);
                leftSpeed = steer;
                rightSpeed = -steer;

                leftLateral.setPower(leftSpeed);
                rightLateral.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.addData("Steer Speed",   "%5.2f, %5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            leftLateral.setPower(0);
            rightLateral.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle, boolean brake, double maxTimeMS)
    {
        if (brake)
        {
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else
        {
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightLateral.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        ElapsedTime turnTimer = new ElapsedTime();
        turnTimer.reset();

        if (angle < 0)
            angle += 360;

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF) && (turnTimer.milliseconds() < maxTimeMS)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftLateral.setPower(0);
        rightLateral.setPower(0);
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
        double   leftSpeed;
        double   rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
        }

        // Send desired speeds to motors.
        leftSpeed = steer;
        rightSpeed = -steer;
        leftLateral.setPower(leftSpeed);
        rightLateral.setPower(rightSpeed);
        leftDrive.setPower(rightSpeed);
        rightDrive.setPower(leftSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f / %5.2f", error, steer);
        telemetry.addData("Speed Steer.", "%5.2f, %5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          positive error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;
        double currentAngle;
        double clockwiseAngle;
        double counterClockwiseAngle;

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
        //while (robotError > 180)  robotError -= 360;
        //while (robotError <= -180) robotError += 360;

        //return robotError;

        // Get current angle and adjust to be in range of 0 - 360 (CCW positive) IMU gives angle in +/- 180 angle from original angle
        currentAngle = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
        if (currentAngle < 0)
            currentAngle += 360;

        clockwiseAngle = currentAngle - targetAngle;
        if (clockwiseAngle < 0)
            clockwiseAngle += 360;

        counterClockwiseAngle = targetAngle - currentAngle;
        if (counterClockwiseAngle < 0)
            counterClockwiseAngle += 360;

        telemetry.addData("Target Angle: ", "%5.2f", targetAngle);
        telemetry.addData("Current Angle: ", "%5.2f", currentAngle);
        telemetry.addData("CW Angle: ", "%5.2f", clockwiseAngle);
        telemetry.addData("CCW Angle: ", "%5.2f", counterClockwiseAngle);
        telemetry.update();
        leftLateral.setPower(0);
        rightLateral.setPower(0);
        //sleep(100);

        if (counterClockwiseAngle <= clockwiseAngle)
            return counterClockwiseAngle;               // Turn Left (positive)
        else
            return -clockwiseAngle;                     // Turn Right (negative)
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        double steer = error * PCoeff;
        if (steer < 0.0 && steer > -MIN_TURN_SPEED)
            steer = -MIN_TURN_SPEED;
        else if (steer > 0.0 && steer < MIN_TURN_SPEED)
            steer = MIN_TURN_SPEED;
        return Range.clip(error * PCoeff, -TURN_SPEED, TURN_SPEED);
    }
}



