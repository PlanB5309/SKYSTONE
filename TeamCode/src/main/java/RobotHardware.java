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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;

import java.net.PortUnreachableException;

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
    public DcMotor  leftIntakeMotor = null;
    public DcMotor  rightIntakeMotor = null;
    public DcMotor  liftMotor = null;

    public Servo skyStoneClaw = null;
    public Servo capStoneServo = null;
    public Servo blockFlippingServo = null;
    public Servo blockTurningServo = null;
    public Servo blockGrabbingServo = null;
    public Servo blockKickerServo = null;
    public Servo leftFoundationServo = null;
    public Servo rightFoundationServo = null;

    BNO055IMU imu;
    ColorSensor colorSensor;
    DistanceSensor distanceSensor;
    ColorSensor sideColorSensor;
    DistanceSensor sideDistanceSensor;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    //Hardware constants
    public static final double SKYSTONECLAW_INIT    =  0.45 ;
    public static final double SKYSTONECLAW_ACTIVATE    =  0.15 ;
    public static final double TELEOPDEADZONE = 0.05;
    public static final double NOTTURBOFACTOR = 0.5;
    public static final int CLICKS_PER_INCH = 45;
    public static final int STRAFE_CLICKS_PER_INCH = 48;
    public static final double INTAKE_WHEEL_SPEED = 0.7;

    public static final double SKYSTONE_SERVO_UP = 1;
    public static final double SKYSTONE_SERVO_DOWN_AUTO = 0.29;
    public static final double SKYSTONE_SERVO_DOWN_TELEOP = 0.29;

    public static final double CAPSTONE_SERVO_IN = 0.49;
    public static final double CAPSTONE_SERVO_OUT = 0.8;

    public static final double BLOCK_SERVO_GRAB = 0.15;
    public static final double BLOCK_SERVO_RELEASE = 0.43;

    public static final double LIFT_BLOCK_SERVO_START = 1;
    public static final double LIFT_BLOCK_SERVO_TOP = 0.52;

    public static final double BLOCK_TURNING_SERVO_IN = 0;
    public static final double BLOCK_TURNING_SERVO_OUT = 0.73;

    public static final double KICKER_STANDARD_POSITION = 0.50;
    public static final double KICKER_IN_POSITION = 0.25;
    public static final double KICKER_OUT_POSITION = 0.8;

    public final double HIGH_TURN_POWER = 0.15;
    public final double LOW_TURN_POWER = 0.06;

    public final int RED_THRESHOLD = 175;
    public final int BLUE_THRESHOLD = 140;
    public final int SKYSTONE_COLOR_THRESHOLD = 500;

    public static final double RIGHT_FOUNDATION_SERVO_UP = 0.1;
    public static final double LEFT_FOUNDATION_SERVO_UP = 0.52;
    public static final double RIGHT_FOUNDATION_SERVO_DOWN = 0.63;
    public static final double LEFT_FOUNDATION_SERVO_DOWN = 0.17;





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

        leftIntakeMotor = hwMap.get(DcMotor.class, "leftIntakeMotor");
        rightIntakeMotor = hwMap.get(DcMotor.class, "rightIntakeMotor");
        liftMotor = hwMap.get(DcMotor.class, "liftMotor");


        skyStoneClaw  = hwMap.get(Servo.class, "skyStoneClaw");
        capStoneServo = hwMap.get(Servo.class, "capStoneServo");
        blockFlippingServo = hwMap.get(Servo.class, "stoneFlippingServo");
        blockTurningServo = hwMap.get(Servo.class, "blockTurningServo");
        blockGrabbingServo = hwMap.get(Servo.class, "blockGrabbingServo");
        blockKickerServo = hwMap.get(Servo.class, "blockKickerServo");
        rightFoundationServo = hwMap.get(Servo.class, "rightFoundationServo");
        leftFoundationServo = hwMap.get(Servo.class, "leftFoundationServo");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();//meow

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        colorSensor = hwMap.colorSensor.get("colorSensor");
        distanceSensor = hwMap.get(DistanceSensor.class, "colorSensor");
        sideColorSensor = hwMap.get(ColorSensor.class,"sideColorSensor");
        sideDistanceSensor = hwMap.get(DistanceSensor.class, "sideColorSensor");

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);
        liftMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        skyStoneClaw.setPosition(SKYSTONE_SERVO_UP);
        capStoneServo.setPosition(CAPSTONE_SERVO_OUT);

        blockFlippingServo.setPosition(LIFT_BLOCK_SERVO_START);
        blockTurningServo.setPosition(BLOCK_TURNING_SERVO_IN);
        blockGrabbingServo.setPosition(BLOCK_SERVO_RELEASE);
        blockKickerServo.setPosition(KICKER_STANDARD_POSITION);

        rightFoundationServo.setPosition(RIGHT_FOUNDATION_SERVO_UP);
        leftFoundationServo.setPosition(LEFT_FOUNDATION_SERVO_UP);
    }

    public void setupDriveTrain () {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runUsingEncoder () {
    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

}

