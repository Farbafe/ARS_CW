import java.io.IOException;
import java.util.Stack;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.video.Video;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
public class Task1 {
    private static boolean isGreyDetectorMode = true;

    private static final int WIDTH = 160;
    private static final int HEIGHT = 120;
    //private static final int NUM_PIXELS = WIDTH * HEIGHT;
    
    private static int [][] luminanceFrame = new int [HEIGHT][WIDTH];
    private static int threshold = 90;
    private static RegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
    private static RegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.D);
    enum RobotMovement {
        LEFT,
        STRAIGHT,
        RIGHT
    }
    private static RobotMovement robotMovement = RobotMovement.STRAIGHT;
     
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
        int blackLeft, blackRight, greyArea;
            
        while (Button.ESCAPE.isUp()) {
            blackLeft = 0;
            blackRight = 0;
            greyArea = 0;
            video.grabFrame(frame);
                       
            // Create a frame of luminance values
            extractLuminanceValues(frame);   
            
            // Adjust threshold?
            if (Button.UP.isDown()) {
                threshold +=5;
                if (threshold > 255)
                    threshold = 255;
            }
            else if (Button.DOWN.isDown()) { // if down is pushed down and hold, does threshold keep changing? - need to check, maybe that's why we were getting weird results 
                threshold -=5;
                if (threshold < 0)
                    threshold = 0;
            }
            
            if (Button.RIGHT.isDown()) {
                isGreyDetectorMode = !isGreyDetectorMode;
                stop();
                Delay.msDelay(1000);
            }
            
            if (isGreyDetectorMode) {
                for (int y = HEIGHT / 2 - 5; y < HEIGHT / 2 + 5; ++y) {
                    for (int x = WIDTH / 2 - 5; x < WIDTH / 2 + 5; ++x) {
                        if (luminanceFrame[y][x] < threshold && luminanceFrame[y][x] > (threshold - 20)) {
                            ++greyArea;
                            if (Button.LEFT.isDown()) {
                                System.out.println("");
                                System.out.println("");
                                System.out.println("");
                                System.out.println("");
                                System.out.println("");
                                System.out.println("   greyArea:  " + greyArea);
                                System.out.println("   threshold: " + threshold);
                            }
                            else {
                                LCD.setPixel(x + 10, y, 1);
                            }
                        }
                        else {
                            --greyArea;
                            if (Button.LEFT.isDown()) {
                                System.out.println("");
                                System.out.println("");
                                System.out.println("");
                                System.out.println("");
                                System.out.println("");
                                System.out.println("   greyArea:  " + greyArea);
                                System.out.println("   threshold: " + threshold);
                            }
                            else {
                                LCD.setPixel(x + 10, y, 0);
                            }
                        }
                    }
                }
                if (greyArea > 0) {
                    Sound.beep();
                }
//                setDisp(0, WIDTH-10, 10, WIDTH, greyArea > 0 ? 1 : 0);
                continue;
            }
            else {
                dispFrame(); // todo we should remove dispFrame for Task1 (to reduce computation needed)
            }
            
            for (int y = 0; y <= HEIGHT / 2; y += 2) {
                for (int x = WIDTH / 3; x <= WIDTH / 3 + 2; ++x) {
                    if (luminanceFrame[y][x] < threshold) {
                        ++blackLeft;
                    }
                }
                for (int x = WIDTH / 3 * 2 - 2; x <= WIDTH / 3 * 2; ++x) {
                    if (luminanceFrame[y][x] < threshold) {
                        ++blackRight;
                    }
                } 
            }
            
            //Four Conditions
            if (blackLeft >= blackRight * 0.9 && blackLeft <= blackRight * 1.1 && (blackLeft <= 24)) {
                if (robotMovement != RobotMovement.RIGHT) {
                    turnRight();
                }                
                // todo have condition, if this was preceded by
                // an all white region for a bit, then turn exactly 
                // 180 degrees and follow the path. This means it
                // has reached the end of the maze, and the end of
                // the white square perimeter.
            }
            else if (blackLeft >= blackRight * 0.9 && blackLeft <= blackRight * 1.1) {
                if (robotMovement != RobotMovement.STRAIGHT) {
                    if (robotMovement == RobotMovement.LEFT) {
                        motorRight.setSpeed(180);
                        motorRight.rotate(-70, false);
                    }
                    else {
                        motorLeft.setSpeed(180);
                        motorLeft.rotate(-70, false);
                    }
                    goStraight();
                }
            }
            else if (blackLeft > blackRight) {
                if (robotMovement != RobotMovement.RIGHT) {
                    Delay.msDelay(100);
                    turnRight();
                }
            }
            else if (blackLeft < blackRight) {
                if (robotMovement != RobotMovement.LEFT) {
                    Delay.msDelay(100);
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
        motorLeft.setSpeed(45); // todo if needed we can multiply by normalisedTimeTurning which will be a variable between 0 and 1
        motorRight.setSpeed(180); // try 140, it will turn slower so it'll give the robot more time to react after turning
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
}