import lejos.nxt.*;
import lejos.nxt.addon.TiltSensor;
import lejos.util.Delay;

public class Shooter {
    private final static int POWER_MAX = 100;
    private final static int LEFT_LIMIT_TO_ZERO = 180;
    private final static int RIGHT_LIMIT_TO_ZERO = 90;
    private final static int PID_LOOP_WAIT = 10;
    private final static int PID_P = 2;
    private final static int PID_D = 5;

    private NXTRegulatedMotor leftMotor = Motor.A;
    private NXTRegulatedMotor rightMotor = Motor.C;

    private int leftZero, rightZero, leftTarget, rightTarget;

    public static void main(String[] args) {
        Shooter shooter = new Shooter();
        shooter.goToStartPosition();
        TaskMovementMotors taskMovementMotors = new TaskMovementMotors(shooter);
        taskMovementMotors.start();
        Delay.msDelay(5000);
        ReadAccelSensor accelSensor = new ReadAccelSensor(shooter);
        accelSensor.start();
    }

    public int getLeftTarget() {
        return leftTarget;
    }

    public int getRightTarget() {
        return rightTarget;
    }

    public void setLeftTarget(int leftTarget) {
        this.leftTarget = leftTarget;
    }

    public void setRightTarget(int rightTarget) {
        this.rightTarget = rightTarget;
    }

    public NXTRegulatedMotor getLeftMotor() {
        return leftMotor;
    }

    public NXTRegulatedMotor getRightMotor() {
        return rightMotor;
    }

    public int getLeftZero() {
        return leftZero;
    }

    public int getRightZero() {
        return rightZero;
    }

    public static class ReadAccelSensor extends Thread {
        private final Shooter shooter;
        private final TiltSensor accelSensor;

        public ReadAccelSensor(Shooter shooter) {
            super();
            this.shooter = shooter;
            this.accelSensor = new TiltSensor(SensorPort.S2);
        }

        @Override
        public void run() {
            int x, y, z;

            int h, v;
            int eLT, eRT;

            while (!Button.ESCAPE.isPressed()) {
                x = accelSensor.getXTilt();
                y = accelSensor.getYTilt();
                z = accelSensor.getZTilt();

                LCD.drawString("x : " + x, 0, 1);
                LCD.drawString("y : " + y, 0, 2);
                LCD.drawString("z : " + z, 0, 3);

                // Set Horizontal value (map 1 to 1 of y value)
                h = y;
                // Limit horizontal value
                if (h > 100)
                    h = 100;
                if (h < -100)
                    h = -100;

                // Set vertical value (map sensor z range 80 to -20 to range 0 to 100)
                v = 60 - z;
                if (v > 100)
                    v = 100;
                if (v < -0)
                    v = 0;

                // Calc left and right target, do not do math directly with
                // target variable as compiler may set transient values to variable
                // which can affect servo task
                eLT = (v + h) / 2;
                eRT = (v - h) / 2;
                shooter.setLeftTarget(eLT);
                shooter.setRightTarget(eRT);

//                LCD.drawString("target h : " + h, 0, 1);
//                LCD.drawString("target v : " + v, 0, 2);

                Delay.msDelay(10);
            }
        }

    }

    public static class TaskMovementMotors extends Thread {
        int errLeftPrev, errRightPrev;
        int eLeft, eRight;
        int errLeft, errRight;
        int pwrLeft, pwrRight;

        private NXTRegulatedMotor leftMotor;
        private NXTRegulatedMotor rightMotor;
        private int leftZero, rightZero, leftTarget, rightTarget;

        public TaskMovementMotors(Shooter shooter) {
            this.leftMotor = shooter.getLeftMotor();
            this.rightMotor = shooter.getRightMotor();
            this.leftZero = shooter.getLeftZero();
            this.rightZero = shooter.getRightZero();
        }

        @Override
        public void run() {
            eLeft = leftMotor.getPosition() - leftZero;
            eRight = rightMotor.getPosition() - rightZero;

            errLeft = leftTarget - eLeft;
            errRight = rightTarget - eRight;

            while (!Button.ESCAPE.isPressed()) {
                Delay.msDelay(PID_LOOP_WAIT);
                errLeftPrev = errLeft;
                errRightPrev = errRight;

                eLeft = leftMotor.getPosition() - leftZero;
                eRight = rightMotor.getPosition() - rightZero;

                errLeft = leftTarget - eLeft;
                errRight = rightTarget - eRight;

                pwrLeft = errLeft * PID_P + (errLeft - errLeftPrev) * PID_D;
                pwrRight = errRight * PID_P + (errRight - errRightPrev) * PID_D;

                if (pwrLeft < -POWER_MAX) pwrLeft = -POWER_MAX;
                if (pwrLeft > POWER_MAX) pwrLeft = POWER_MAX;
                if (pwrRight < -POWER_MAX) pwrRight = -POWER_MAX;
                if (pwrRight > POWER_MAX) pwrRight = POWER_MAX;

                leftMotor.setSpeed(pwrLeft);
                rightMotor.setSpeed(pwrRight);
                leftMotor.forward();
                rightMotor.forward();
            }
        }

    }

    private void goToStartPosition() {
        goToStartPosition(leftMotor, true, "Left", 1);
        goToStartPosition(rightMotor, false, "Right", 2);
        leftZero = leftMotor.getPosition() - LEFT_LIMIT_TO_ZERO;
        rightZero = rightMotor.getPosition() + RIGHT_LIMIT_TO_ZERO;

        leftTarget = 0;
        rightTarget = 0;
    }

    private void goToStartPosition(NXTRegulatedMotor motor, boolean forward, String name, int display) {
        motor.setSpeed(40);
        motor.resetTachoCount();

        int curPosition = motor.getTachoCount();
        int prevPosition;

        if (forward) {
            motor.forward();
        } else {
            motor.backward();
        }

        boolean done = false;
        while (!done) {
            Delay.msDelay(100);
            prevPosition = curPosition;
            curPosition = motor.getTachoCount();
            if ((forward && !(curPosition > prevPosition)) || (!forward && !(curPosition < prevPosition))) {
                motor.stop();
                done = true;
                LCD.drawString("Done with " + name, 0, display);
            } else {
                LCD.drawString(name + " : " + curPosition, 0, display);
            }
        }
    }
}
