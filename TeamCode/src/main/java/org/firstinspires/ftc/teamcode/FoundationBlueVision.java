package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Disabled
@Autonomous(name = "Blue Vision_")
public class FoundationBlueVision extends EncoderDrive {

    private DcMotor arm = null;

    private Servo clamp = null;


    VuforiaLocalizer vuforia;
    VuforiaTrackables trackables;
    VuforiaTrackable template;
    VuforiaTrackableDefaultListener mark;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        arm = hardwareMap.get(DcMotor.class, "arm");

        clamp = hardwareMap.get(Servo.class, "clamp");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right = hardwareMap.get(Servo.class, "right");
        left = hardwareMap.get(Servo.class, "left");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        right.setDirection(Servo.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(-100);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clamp.setPosition(0);

        right.setDirection(Servo.Direction.REVERSE);

        right.scaleRange(0, 0.25);
        left.scaleRange(0.7, 1);

        initVuforia();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        forward(0.15, 6);

        sleep(1000);

        encoderDrive(0.2, 6, 6, 3);
        /*while (!markVisible()) {
            sideways(0.15,2);
            sleep(300);
        }


        double target = 90;

        while (Math.abs(target - markPos()) > 5) {
            double power = (target - markPos()) * 0.001;

            leftFront.setPower(power);
            rightFront.setPower(-power);
            leftBack.setPower(-power);
            rightBack.setPower(power);
        }
        arm.setTargetPosition(-2600);
        clamp.setPosition(1);
        encoderDrive(0.3,18, 18, 5);
        clamp.setPosition(0);
        arm.setTargetPosition(-100);
        encoderDrive(-0.3,24, 24, 5);
        encoderSideways(-0.5, 84, 84, 5);
        arm.setTargetPosition(-100);
        clamp.setPosition(1);
        arm.setTargetPosition(-2600);
        clamp.setPosition(0);
        touchSensor(0.4);
        servo(1);
        encoderDrive(-0.3, 10, 10, 5);
        encoderSideways(-0.6,18, 18, 5);*/
    }


    private void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey =
                "AeTSKkn/////AAABmXK540vJ4k8muhR6D7aoeZpbnFSenqf9a+poNXj4KY56UyTsbTrSeHqrNBi7hJweC+rEjfGiSPfJ813Az57QwdTyLzth/JgNKh3BfGz7OcgIaqCMLwDZf+BAEjFYuX2j5bKUNN/+kCrWp8AUvbPpcPHmGnnwJ7ABzmsazba+tMgSs3rcA3AvezaOGOMDwIiG71ouwN3mKOvybsDNf+2jzMCn1tywUqn3teDCGzKjV2ZeqJW9Qt2wjrvY2sI3MS176buUh/H04Da5FDj+6Dg3/fZtsIlsrVu2fAvepwWvgiCprGf6i9Q4oLBryNgCLDfpMx9EXBWAE4D5hzyBsoFITU0z6zUO+e8b6rJ0xQQr7dbV";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        trackables = this.vuforia.loadTrackablesFromAsset("Skystone");
        template = trackables.get(0);

        mark = (VuforiaTrackableDefaultListener) template.getListener();
        trackables.activate();
    }

    private boolean markVisible() {
        return mark.isVisible();
    }

    private double markPos() {
        OpenGLMatrix pose = mark.getPose();

        VectorF trans = pose.getTranslation();
        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double tX = trans.get(0);
        double tY = trans.get(1);
        double tZ = trans.get(2);

        double rX = rot.firstAngle;
        double rY = rot.secondAngle;
        double rZ = rot.thirdAngle;

        return tX;
    }

    /*public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy())) {
                idle();
            }

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    } */

    /*public void touchSensor(double power){
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);


        while (digitalTouch.getState() == true && opModeIsActive()){
            sleep(1);
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    } */

    /* public void forward(double power, double inches) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);

        int start = leftBack.getCurrentPosition();

        double distance = inches/(4*Math.PI)*1440;

        while (Math.abs(leftBack.getCurrentPosition() - start) < distance && opModeIsActive()){
            telemetry.addData("position", leftBack.getCurrentPosition());
            telemetry.update();
            sleep(1);
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    } */

    /*public void servo(int servoPosition){
        left.setPosition(servoPosition);
        right.setPosition(servoPosition);

    }*/
    /*public void sideways(double power, double inches){
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(power);

        int start = leftBack.getCurrentPosition();

        double distance = inches/(4*Math.PI)*1440;

        while (Math.abs(leftBack.getCurrentPosition() - start) < distance && opModeIsActive()){
            telemetry.addData("position", leftBack.getCurrentPosition());
            telemetry.update();
            sleep(1);
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    } */
    /*
    public void encoderSideways(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFront.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBack.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(newLeftFrontTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy())) {
                idle();
            }

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    } */
}

