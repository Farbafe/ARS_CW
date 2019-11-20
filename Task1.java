import java.io.IOException;
import java.util.Stack;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.video.Video;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
public class Task1 {
    private static final int WIDTH = 160;
    private static final int HEIGHT = 120;
    //private static final int NUM_PIXELS = WIDTH * HEIGHT;

    private static int [][] luminanceFrame = new int [HEIGHT][WIDTH];
    private static int threshold = 90;
    private static RegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
    private static RegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.D);
    private static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2); //colorSensor
    enum RobotMovement {
        LEFT,
        STRAIGHT,
        RIGHT,
        REVERSE,
        TURN,
        RIGHT_REVERSE,
        LEFT_REVERSE,
        NONE
    }
    private static RobotMovement robotMovement = RobotMovement.NONE;

    public Task1() {
        // Initialize luminance frame
        for (int x=0; x<WIDTH; x += 1) {
            for (int y=0; y<HEIGHT; y += 1) {
                luminanceFrame[y][x] = 0;
            }
        }
    }

    public static void main(String[] args) throws IOException  {

        EV3 ev3 = (EV3) BrickFinder.getLocal();
        Video video = ev3.getVideo();
        video.open(WIDTH, HEIGHT);
        byte[] frame = video.createFrame();
        int blackTotal, blackLeft, blackCentre, blackRight, blackLeftWide, blackRightWide, greyArea, greyCount = 0, greyLoopCount = 0, whiteLoopCount = 0;
        boolean isAllWhite = false;
        colorSensor.setCurrentMode("Ambient");
        float[] sample = new float[colorSensor.sampleSize()];

        while (Button.ESCAPE.isUp()) {
            --greyLoopCount;
            if (greyLoopCount < 0) {
                greyLoopCount = 0;
                colorSensor.fetchSample(sample, 0);
                if (sample[0] < 0.37 && sample[0] > 0.34) {
                    Delay.msDelay(40);
                    colorSensor.fetchSample(sample, 0);
                    if (sample[0] < 0.37 && sample[0] > 0.34) {
                        goStraight();
                        Delay.msDelay(100);
                        stop();
                        colorSensor.fetchSample(sample, 0);
                        if (sample[0] < 0.37 && sample[0] > 0.34) {
                            Delay.msDelay(400);
                            Sound.beep();
                            greyLoopCount = 95;
                        }
                    }
                }
            }
            
            blackTotal = 0;
            blackLeftWide = 0;
            blackLeft = 0;
            blackCentre = 0;
            blackRight = 0;
            blackRightWide = 0;
            greyArea = 0;
            video.grabFrame(frame);

            // Create a frame of luminance values
            extractLuminanceValues(frame);   

//            dispFrame();
            
            // y 18 to y 60 + 18 (change to 2 or 3 layers?) todo
            for (int y = 18; y <= HEIGHT / 2 + 18; y += 2) {
                for (int x = 0; x <= 5; ++x) {
                    if (luminanceFrame[y][x] < threshold) {
                        ++blackLeftWide;
                    }
                }
                for (int x = WIDTH / 16 * 5; x <= WIDTH / 16 * 5 + 2; ++x) {
                    if (luminanceFrame[y][x] < threshold) {
                        ++blackLeft;
                    }
                }
                for (int x = WIDTH / 2 - 1; x <= WIDTH / 2 + 1; ++x) {
                    if (luminanceFrame[y][x] < threshold) {
                        ++blackCentre;
                    }
                }
                for (int x = WIDTH / 16 * 11 - 2; x <= WIDTH / 16 * 11; ++x) {
                    if (luminanceFrame[y][x] < threshold) {
                        ++blackRight;
                    }
                }
                for (int x = 154; x <= 159; ++x) {
                    if (luminanceFrame[y][x] < threshold) {
                        ++blackRightWide;
                    }
                }
            }
            // todo mark current position as a vector based on last position vector
            // as a junction if blackLeft and/or blackRight is low and 

            if ((robotMovement == RobotMovement.LEFT || robotMovement == RobotMovement.RIGHT) && blackCentre >= 9) {
                continue; // i.e keep turning until the centre is less black
            }
            
            blackTotal = blackLeftWide + blackLeft + blackCentre + blackRight + blackRightWide;
            if (blackTotal <= 9) {
                if (sample[0] > 0.37 || sample[0] < 0.34) {                    
                    Sound.twoBeeps();
                    if (isAllWhite) {
                        turnAround();                    
                        isAllWhite = false;
                    }
                    else {
                        isAllWhite = true;
                        goStraight();
                        Delay.msDelay(350); // if delay doesn't work then use
                        // x0 = currTachoCount, while tachoCount < x + x0
                        // or a while loop counter (just like greyCount's implementation)
                        continue;
                    }
                }
            }
            isAllWhite = false;
            
            if (blackCentre <= 9) {
                if (robotMovement == RobotMovement.RIGHT) {
                    stop();
                    motorRight.rotate(20, false);
                }
                else if (robotMovement == RobotMovement.LEFT) {
                    stop();
                    motorLeft.rotate(20, false);
                }
                goStraight();
            }
            else if (blackRight <= 27) {
                if (robotMovement != RobotMovement.RIGHT) {
                    Delay.msDelay(20);
                    turnRight();
                }
            }
            else if (blackLeft <= 27) {
                if (robotMovement != RobotMovement.LEFT) {
                    Delay.msDelay(20);
                    turnLeft();
                }
            }            
        }
        video.close();
    }

    public static void extractLuminanceValues(byte [] frame) {
        int x,y;
        int doubleWidth = 2*WIDTH; // y1: pos 0; u: pos 1; y2: pos 2; v: pos 3.
        int frameLength = frame.length;
        for(int i=0;i<frameLength;i+=2) {
            x = (i / 2) % WIDTH;
            y = i / doubleWidth;
            //luminanceFrame[y][x] = 2*Math.abs((int) frame[i]);
            luminanceFrame[y][x] = frame[i] & 0xFF;           
        }
    }

    public static void dispFrame() {
        for (int y=0; y<HEIGHT; y++) {
            for (int x=0; x<WIDTH; x++) {
                if (luminanceFrame[y][x] <= threshold) {
                    LCD.setPixel(x, y, 1);
                }
                else {
                    LCD.setPixel(x, y, 0);
                }    
            }
        }
    }

    public static void setDisp(int y0, int x0, int height, int width, int color) {
        for (int y = y0; y < height; ++y) {
            for (int x = x0; x < width; ++x) {
                LCD.setPixel(x, y, color);
            }
        }
    }

    public static void stop() {
        motorRight.stop(true);
        motorLeft.stop(true);
    }    

    public static void turnRight() {
        motorLeft.setSpeed(180);
        motorRight.setSpeed(45);
        motorLeft.forward();
        motorRight.backward();
        robotMovement = RobotMovement.RIGHT;
    }

    public static void turnLeft() {
        motorLeft.setSpeed(45);
        motorRight.setSpeed(180);
        motorLeft.backward();
        motorRight.forward();
        robotMovement = RobotMovement.LEFT;
    }

    public static void goStraight() {
        motorLeft.setSpeed(180);
        motorRight.setSpeed(180);
        motorRight.forward();
        motorLeft.forward();
        robotMovement = RobotMovement.STRAIGHT;
    }

    public static void goBackward() {
        motorLeft.setSpeed(150);
        motorRight.setSpeed(150);
        motorRight.backward();
        motorLeft.backward();
        robotMovement = RobotMovement.REVERSE;
    }

    public static void turnAround() {
        motorLeft.setSpeed(180);
        motorRight.setSpeed(180);
        motorRight.rotate(369, true);
        motorLeft.rotate(-369, false);
        goStraight();
        Delay.msDelay(80);
        robotMovement = RobotMovement.TURN;
    }

    public static void turnRightReversed() {
        motorLeft.setSpeed(180);
        motorRight.setSpeed(45);
        motorLeft.backward();
        motorRight.forward();
        robotMovement = RobotMovement.RIGHT_REVERSE;
    }

    public static void turnLeftReversed() {
        motorLeft.setSpeed(45); 
        motorRight.setSpeed(180);
        motorLeft.forward();
        motorRight.backward();
        robotMovement = RobotMovement.LEFT_REVERSE;
    }
}
