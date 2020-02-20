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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left front drive motor:        "leftFrontDrive"
 * Motor channel:  Right front drive motor:        "rightFrontDrive"
 * Motor channel:  Left rear drive motor:        "leftRearDrive"
 * Motor channel:  Right rear drive motor:        "rightRearDrive"
 */
public class RobotHardware
{
    /* Public OpMode members. */
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  leftRearDrive   = null;
    public DcMotor  rightRearDrive  = null;
    public DcMotor  liftMotor = null;

    public Servo capStoneServo = null;
    public Servo blockFlippingServo = null;
    public Servo blockGrabbingServo = null;
    public Servo leftFoundationServo = null;
    public Servo rightFoundationServo = null;
    public Servo capStoneHolder = null;

    BNO055IMU imu;
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;
    ColorSensor frontColorSensor;
    ColorSensor frontLeftColorSensor;
    DistanceSensor leftDistanceSensor;
    DistanceSensor rightDistanceSensor;
    DistanceSensor rearDistanceSensor;
    DistanceSensor frontDistanceSensor;
    DistanceSensor frontLeftDistanceSensor;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    //Hardware constants
    public static final int AMBIENT_LIGHT_MODIFIER = 0;
    public static final int LEFT_AMBIENT_LIGHT_MODIFIER = 0;
    public static final double THRESHOLD_PERCENT = .8;
    public static final int[] blockDistance = new int[] {0, 91, 74, 54, 31, 12, 91};
    public static final int[] blockTravelDistance = new int[] {0, 87, 95, 59};

    public static final double TELEOPDEADZONE = 0.05;
    public static final double NOTTURBOFACTOR = 0.5;
    public static final int CLICKS_PER_INCH = 45;
    public static final int STRAFE_CLICKS_PER_INCH = 48;

    public static final double SKYSTONE_SERVO_UP = 1;
    public static final double SKYSTONE_SERVO_DOWN_AUTO = 0.29;

    public static final double CAPSTONE_SERVO_IN = 0.75;
    public static final double CAPSTONE_SERVO_OUT = 0.05;
    public static final double CAPSTONE_HOLDER_RELEASE = 0.4;
    public static final double CAPSTONE_HOLDER_GRAB = 0.8;

    public static final double BLOCK_SERVO_GRAB = 1.0;
    public static final double BLOCK_SERVO_RELEASE = 0.45;

    public static final double LIFT_BLOCK_SERVO_UP = 0;
    public static final double LIFT_BLOCK_SERVO_DOWN = 0.6;
    public static final double LIFT_BLOCK_BARLEY_UP = 0.45;

    public final double HIGH_TURN_POWER = 0.3;
    public final double MEDIUM_TURN_POWER = 0.08;
    public final double LOW_TURN_POWER = 0.04;

    public final int RED_THRESHOLD = 175;
    public final int BLUE_THRESHOLD = 140;

    public static final double RIGHT_FOUNDATION_SERVO_UP = 0.67;
    public static final double LEFT_FOUNDATION_SERVO_UP = 0.2;
    public static final double RIGHT_FOUNDATION_SERVO_DOWN = 0;
    public static final double LEFT_FOUNDATION_SERVO_DOWN = 0.83;
    public static final double RIGHT_FOUNDATION_SERVO_INIT = 0.75;
    public static final double LEFT_FOUNDATION_SERVO_INIT = 0;


    /* Constructor */
    public RobotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive  = hwMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hwMap.get(DcMotor.class, "rightRearDrive");

        liftMotor = hwMap.get(DcMotor.class, "liftMotor");


        capStoneServo = hwMap.get(Servo.class, "capStoneServo");
        blockFlippingServo = hwMap.get(Servo.class, "stoneFlippingServo");
        blockGrabbingServo = hwMap.get(Servo.class, "blockGrabbingServo");
        rightFoundationServo = hwMap.get(Servo.class, "rightFoundationServo");
        leftFoundationServo = hwMap.get(Servo.class, "leftFoundationServo");
        capStoneHolder = hwMap.get(Servo.class, "capStoneHolder");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        colorSensor = hwMap.colorSensor.get("colorSensor");
        distanceSensor = hwMap.get(DistanceSensor.class, "colorSensor");
        frontColorSensor = hwMap.get(ColorSensor.class, "frontColorSensor");
        frontDistanceSensor = hwMap.get(DistanceSensor.class, "frontColorSensor");
        frontLeftColorSensor = hwMap.get(ColorSensor.class, "frontLeftColorSensor");
        frontLeftDistanceSensor = hwMap.get(DistanceSensor.class, "frontLeftColorSensor");
        leftDistanceSensor = hwMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hwMap.get(DistanceSensor.class, "rightDistanceSensor");
        rearDistanceSensor = hwMap.get(DistanceSensor.class, "rearDistanceSensor");


        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        liftMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        capStoneServo.setPosition(CAPSTONE_SERVO_IN);
        capStoneHolder.setPosition(CAPSTONE_HOLDER_GRAB);

        blockFlippingServo.setPosition(LIFT_BLOCK_SERVO_UP);
        blockGrabbingServo.setPosition(BLOCK_SERVO_RELEASE);

        rightFoundationServo.setPosition(RIGHT_FOUNDATION_SERVO_INIT);
        leftFoundationServo.setPosition(LEFT_FOUNDATION_SERVO_INIT);
    }

    public void teleopInit(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        leftRearDrive  = hwMap.get(DcMotor.class, "leftRearDrive");
        rightRearDrive = hwMap.get(DcMotor.class, "rightRearDrive");
        liftMotor = hwMap.get(DcMotor.class, "liftMotor");


        capStoneServo = hwMap.get(Servo.class, "capStoneServo");
        blockFlippingServo = hwMap.get(Servo.class, "stoneFlippingServo");
        blockGrabbingServo = hwMap.get(Servo.class, "blockGrabbingServo");
        rightFoundationServo = hwMap.get(Servo.class, "rightFoundationServo");
        leftFoundationServo = hwMap.get(Servo.class, "leftFoundationServo");
        capStoneHolder = hwMap.get(Servo.class, "capStoneHolder");

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        liftMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetEncoder () {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setupDriveTrain () {
        resetEncoder();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void runUsingEncoder () {
        resetEncoder();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stop () {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        Acceleration gravity = imu.getGravity();
        return angles.firstAngle;
    }

}

