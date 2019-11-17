package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Main TeleOp", group = "Iterative Opmode")

public class MainTeleOp extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor arm = null;
    private Servo clamp = null;
    private boolean was2A = true;
    private boolean was2X = true;
    private Servo left;
    private Servo right;
    DigitalChannel digitalTouch;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        arm = hardwareMap.get(DcMotor.class, "arm");
        clamp = hardwareMap.get(Servo.class, "clamp");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        // digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        // digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(-100);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clamp.setPosition(0);

        left.setPosition(0);
        right.setPosition(0);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // driving
        double forward = gamepad1.left_stick_y;
        double sideways = -gamepad1.left_stick_x;
        double rotation = -gamepad1.right_stick_x;

        mecanumDrive(forward, sideways, rotation);


        // arm
        arm.setPower(0.6);

        if (gamepad2.x && !was2X) {
            if (arm.getTargetPosition() == -2600){
                arm.setTargetPosition(-100);
            } else {
                arm.setTargetPosition(-2600);
            }
        }


        if (gamepad2.a && !was2A) {
            if (clamp.getPosition() == 0.2) {
                clamp.setPosition(0.8);
            } else {
                clamp.setPosition(0.2);
            }
        }

        if (gamepad2.start) {
            left.setPosition(0);
            right.setPosition(1);
        }
        if (gamepad2.back) {
            left.setPosition(1);
            right.setPosition(0);
        }


        was2A = gamepad2.a;
        was2X = gamepad2.x;
    }


    public void mecanumDrive(double forward, double sideways, double rotation) {
        double leftBackPower = Range.clip(forward - sideways + rotation, -1, 1);
        double rightFrontPower = Range.clip(forward - sideways - rotation, -1, 1);
        double leftFrontPower = Range.clip(forward + sideways + rotation, -1, 1);
        double rightBackPower = Range.clip(forward + sideways - rotation, -1, 1);


        // Send calculated power to wheels
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}


