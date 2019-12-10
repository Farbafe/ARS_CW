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
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.video.Video;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.Move;
import lejos.utility.Delay;
public class Task4 {
    private static final int WIDTH = 160;
    private static final int HEIGHT = 120;

    private static int [][] luminanceFrame = new int [HEIGHT][WIDTH];
    private static int threshold = 90;
    private static int turnAroundMultiplier = 1;
    private static RegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.A);
    private static RegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.D);
    private static EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1); //UltrasonicSensor
    private static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
    private static EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);

    enum RobotMovement {
        LEFT,
        STRAIGHT,
        RIGHT,
        REVERSE,
        TURN_AROUND,
        RIGHT_REVERSE,
        LEFT_REVERSE,
        NONE
    }
    private static RobotMovement robotMovement = RobotMovement.NONE;

    public Task4() {
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
        
        colorSensor.setCurrentMode("Ambient");
        gyroSensor.setCurrentMode("Angle");
        ultrasonicSensor.enable();
        ultrasonicSensor.getDistanceMode();
        float[] colorSample = new float[colorSensor.sampleSize()];
        float[] gyroSample = new float[gyroSensor.sampleSize()];        
        float[] sampleUltrasonic = new float[ultrasonicSensor.sampleSize()];
        
        Movement[] movements = new Movement[50]; // ArrayList (or any expandable grouping of variables) is preferable but Array is simpler
        Movement[] movementsGrey = new Movement[3];
        Movement movementBuffer;
        RobotMovement previousMovement;
        int movementIndex = 0;
        boolean isReturningToBase = false, isReturningToBase2 = false;
        
        int blackTotal, blackLeft, blackCentre, blackRight, blackLeftWide, blackRightWide,
        greyLoopCount = 0, greyAreaCount = 0, greyAreaDetectedCount = 0,
        ultrasonicLoopCount = 0,
        rightJunctionLoopCount = 0, leftJunctionLoopCount = 0,
        straightLoopCount = 0, blackLoopCount = 0;
        // xxxLoopCount variables are incremented, decremented or set to specific values  
        // when some reading is above or below a threshold or another condition is met.
        // If the xxxLoopCount is then above a threshold some action takes place.
        
        Sound.beep(); // indicate readiness and wait for user interaction
        Button.waitForAnyPress();
        
        goStraight(); // going straight for 2 seconds ensures that the robot enters the maze
        Delay.msDelay(2000);
        while (Button.ESCAPE.isUp()) {        
            ultrasonicSensor.fetchSample(sampleUltrasonic, 0);
            if (sampleUltrasonic[0] < 0.14) {
                if (ultrasonicLoopCount < 0) {
                    ultrasonicLoopCount = 0;
                }
                ++ultrasonicLoopCount;
                if (ultrasonicLoopCount > 1) {
                    goBackward();
                    Delay.msDelay(350);
                    turnAround(); // this is a blocking function
                    goBackward();
                    Delay.msDelay(150);
                    stop();
                    continue;
                }
            }
            --ultrasonicLoopCount;

            if (greyAreaDetectedCount < 3) {
                --greyLoopCount;
                if (greyLoopCount < 0) {
                    --greyAreaCount;
                    colorSensor.fetchSample(colorSample, 0);
                    if (colorSample[0] < 0.37 && colorSample[0] > 0.27) {
                        if (greyAreaCount < 0) {
                            greyAreaCount = 0;
                        }
                        greyAreaCount += 2;
                        if (greyAreaCount > 5) {
                            previousMovement = robotMovement;
                            stop();
                            ++greyAreaDetectedCount;
                            // todo check if same grey area detected before  
                            Delay.msDelay(1000);
                            Sound.beep();
                            greyLoopCount = 88; // This prevents detecting the same 
                            // grey area again. I.e. it gives the robot enough time 
                            // to leave the grey area.
                            if (greyAreaDetectedCount == 3) {
                                do { // re-orient the robot to face base
                                    motorLeft.rotate(-12, true);
                                    motorRight.rotate(12, false);
                                    gyroSensor.fetchSample(gyroSample, 0);                
                                  } while (Math.abs(gyroSample[0]) <= 10);
                                  turnAround(); // this is a blocking function
                                  goStraight();
                                  isReturningToBase = true;
                                  continue;
                            }
                            goBackward(); // this ensures that the robot follows the previous movement
                            Delay.msDelay(150);  
                            if (previousMovement == RobotMovement.STRAIGHT) {
                                goStraight();
                            }
                            else if (previousMovement == RobotMovement.LEFT) {
                                turnLeft();
                            }
                            else if (previousMovement == RobotMovement.RIGHT) {
                                turnRight();
                            }    
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
            
            // Grab and store frame from webcam
            video.grabFrame(frame);
            // Create a frame of luminance values
            extractLuminanceValues(frame);               

            // Analyse the web-cam readings using 5 (over a span of 10) vertical readings
            // and 3 consecutive horizontal readings over 5 different parts of the frame.
            for (int y = 42; y <= 51; y += 2) {
                for (int x = 5; x <= 7; ++x) {
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
                for (int x = 154; x <= 156; ++x) {
                    if (luminanceFrame[y][x] < threshold) {
                        ++blackRightWide;
                    }
                }
            }

            if ((robotMovement == RobotMovement.LEFT || robotMovement == RobotMovement.RIGHT) && blackCentre >= 3) {
                continue; // i.e keep turning until the centre is less black
            }

            blackTotal = blackLeftWide + blackLeft + blackCentre + blackRight + blackRightWide;
            if (isReturningToBase == true) {
                if (blackTotal == 0) {
                    isReturningToBase2 = true;                    
                }
            }
            if (blackTotal >= 72) {
                if (isReturningToBase2 == true) {
                    stop();
                }
                // The following code makes the robot go backward if the area ahead is black.
                // However, if it has already gone backwards then it will turn around.
                if (blackLoopCount < 0) {
                    goBackward();
                    Delay.msDelay(250);
                    blackLoopCount = 20;
                    continue;
                }
                turnAround(); // this is a blocking function
                goStraight();
                Delay.msDelay(20);
                continue;
            }        
            else {
                --blackLoopCount;
                turnAroundMultiplier = 1;
            }

            if (blackLeftWide <= 10) {
                if (leftJunctionLoopCount < 0) {
                    leftJunctionLoopCount = 0;
                }
                ++leftJunctionLoopCount;
            }
            else {
                --leftJunctionLoopCount;           
            }
            if (blackRightWide <= 10) {
                if (rightJunctionLoopCount < 0) {
                    rightJunctionLoopCount = 0;
                }
                ++rightJunctionLoopCount;
            }
            else {
                --rightJunctionLoopCount;           
            }

            Button.LEDPattern(0); // for visual purposes only - no LED if no junction detected

            if (blackCentre <= 4) {
                if (straightLoopCount > 50) {
                    if (rightJunctionLoopCount > 2) {
                        Button.LEDPattern(1); // greeen
                        rightJunctionLoopCount = 0;
                        goStraight();
                        Delay.msDelay(220);
                        turnRight();
                        Delay.msDelay(1000);
                        goBackward();
                        Delay.msDelay(150);
                        straightLoopCount = 0;
                        continue;
                    }
                    if (leftJunctionLoopCount > 2) {             
                        Button.LEDPattern(2); // red
                        leftJunctionLoopCount = 0;
                        goStraight();
                        Delay.msDelay(220);
                        turnLeft();
                        Delay.msDelay(1000);
                        goBackward();
                        Delay.msDelay(150);
                        straightLoopCount = 0;
                        continue;
                    }
                }
                ++straightLoopCount;
                goStraight();                
            }
            else if (blackRight <= 10) {
                straightLoopCount = 0;
                if (robotMovement != RobotMovement.RIGHT) {
                    Delay.msDelay(220); // the web-cam sees too far ahead so this adjusts for that
                    turnRight();
                    // save current movement and add to array
                    gyroSensor.fetchSample(gyroSample, 0);
                    movementBuffer = new Movement(motorLeft.getTachoCount(), motorRight.getTachoCount(), gyroSample[0]);
                    movements[movementIndex++] = movementBuffer;
                    resetTachoCounts();
                }
            }
            else if (blackLeft <= 10) {
                straightLoopCount = 0;
                if (robotMovement != RobotMovement.LEFT) {
                    Delay.msDelay(220); // the web-cam sees too far ahead so this adjusts for that
                    turnLeft();
                    // save current movement and add to array
                    gyroSensor.fetchSample(gyroSample, 0);
                    movementBuffer = new Movement(motorLeft.getTachoCount(), motorRight.getTachoCount(), gyroSample[0]);
                    movements[movementIndex++] = movementBuffer;
                    resetTachoCounts();
                }
            }
            else { // last resort just in case really, robot should not reach this condition
                turnAround();
            }
        }
        video.close();
        colorSensor.close();
        gyroSensor.close();
        ultrasonicSensor.close();
        motorLeft.close();
        motorRight.close();
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

    // the following 2 functions are no longer in use and are only helpful when debugging
//    public static void dispFrame() {
//        for (int y=0; y<HEIGHT; y++) {
//            for (int x=0; x<WIDTH; x++) {
//                if (luminanceFrame[y][x] <= threshold) {
//                    LCD.setPixel(x, y, 1);
//                }
//                else {
//                    LCD.setPixel(x, y, 0);
//                }    
//            }
//        }
//    }
//
//    public static void setDisp(int y0, int x0, int height, int width, int color) {
//        for (int y = y0; y < height; ++y) {
//            for (int x = x0; x < width; ++x) {
//                LCD.setPixel(x, y, color);
//            }
//        }
//    }

    public static void stop() {
        motorRight.stop(true);
        motorLeft.stop(true);
        robotMovement = RobotMovement.NONE;
    }    

    // attempts to turn in place but not too much because some turns
    // benefit from not turning in place
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
        motorLeft.setSpeed(180);
        motorRight.setSpeed(180);
        motorRight.backward();
        motorLeft.backward();
        robotMovement = RobotMovement.REVERSE;
    }

    public static void turnAround() {
        goBackward();
        Delay.msDelay(150);
        motorLeft.setSpeed(180);
        motorRight.setSpeed(180);
        motorRight.rotate(400 - turnAroundMultiplier * 15, true); // each time turn a little bit less, multiplier is reset elsewhere
        motorLeft.rotate(-400 + turnAroundMultiplier * 15, false);
        ++turnAroundMultiplier;
        stop();
        Delay.msDelay(250); // gives time for the web-cam to readjust to a sudden change in light intensity
        // the RobotMovement.TURN_AROUND is no longer used
    }

    // The following functions were tested to traverse backwards to the starting position.
    // However, it was found that not being able to detect the path and correct any errors
    // was far too inaccurate to be used.
//    public static void turnRightReversed() {
//        motorLeft.setSpeed(180);
//        motorRight.setSpeed(45);
//        motorLeft.backward();
//        motorRight.forward();
//        robotMovement = RobotMovement.RIGHT_REVERSE;
//    }
//
//    public static void turnLeftReversed() {
//        motorLeft.setSpeed(45); 
//        motorRight.setSpeed(180);
//        motorLeft.forward();
//        motorRight.backward();
//        robotMovement = RobotMovement.LEFT_REVERSE;
//    }

    public static void resetTachoCounts() {
        motorLeft.resetTachoCount();
        motorRight.resetTachoCount();
    }
    
     // An attempt at creating a junction class. In the end, movement was not accurate enough
     // to correctly determine the position of a junction.
//    enum JunctionType {
//        LEFT_RIGHT,
//        LEFT,
//        RIGHT
//    }
//    
//    class Junction {
//        private int movementIndex;
//        private JunctionType junctionType;
//        boolean isAllOptionsTravelled = false;
//
//        public Junction(int movementIndex, JunctionType junctionType) {
//            this.movementIndex = movementIndex;
//            this.junctionType = junctionType;
//        }
//
//        public int getMovementIndex() {
//            return this.movementIndex;
//        }
//
//        public JunctionType getJunctionType() {
//            return this.junctionType;
//        }
//    }
}

class Movement {
    // The idea is that before every turn and when junctions/grey areas
    // are detected a movement is created. Therefore, we would have an 
    // array of movements that describe how the robot moved from start 
    // till now. The array is limited to 50 movements at the moment which
    // should be sufficient for the tasks however a dynamic array is
    // preferable in further improvements.
    // Simple tests show that the basic idea works but it is too difficult
    // to implement on a bigger scale. Perhaps this class should be 
    // simplified via an interface? Or maybe use leJOS built-in packages.
    public Movement position; // the movement that absolutely describes this movement from origin
    private int leftTacho = 0;
    private int rightTacho = 0;
    private int distance;
    private int bearing = 0; // movement started with this bearing
    public Movement parentMovement = null;

    public Movement(int leftTacho, int rightTacho, float bearing) {
        this.leftTacho = leftTacho;
        this.rightTacho = rightTacho;
        this.bearing = (int) bearing;
        this.distance = (int) ((this.leftTacho + this.rightTacho) * 2 * 3.14159 * 1.52 / 360); // in cm
        this.position = this.sum(this.parentMovement);
    }

    public Movement(int leftTacho, int rightTacho, float bearing, Movement parentMovement) {
        this.leftTacho = leftTacho;
        this.rightTacho = rightTacho;
        this.bearing = (int) bearing;
        this.distance = (int) ((this.leftTacho + this.rightTacho) * 2 * 3.14159 * 1.52 / 360); // in cm
        this.parentMovement = parentMovement;
        this.position = this.sum(this.parentMovement);
    }

    public void setBearing(int bearing) {
        this.bearing = bearing;
    }

    public int getBearing() {
        return this.bearing;
    }

    public int getLeftTachoCount() {
        return this.leftTacho;
    }

    public float getRightTachoCount() {
        return this.rightTacho;
    }

    public float getMotorLeftSpeed() {
        return this.leftTacho / this.rightTacho;
    }

    public float getMotorRightSpeed() {
        return this.rightTacho / this.leftTacho;
    }

    public Movement reverseMovement() {
        return new Movement(this.rightTacho, this.leftTacho, 360 - this.bearing);
    }

    public Movement reverseMovement(Movement movement) {
        return new Movement(movement.rightTacho, movement.leftTacho, 360 - movement.bearing);
    }

    public boolean isSameMovement(Movement movement, boolean isReversed) {
        if (isReversed) {
            if ((Math.abs(movement.leftTacho - this.rightTacho) < 100) || (movement.leftTacho >= this.rightTacho * 0.9 && movement.leftTacho <= this.rightTacho * 1.1) && ((Math.abs(movement.rightTacho - this.leftTacho) < 100) || (movement.rightTacho >= this.leftTacho * 0.9 && movement.rightTacho <= this.leftTacho * 1.1))) {
                return false;
            }
        }
        else {
            if ((Math.abs(movement.leftTacho - this.leftTacho) < 100) || (movement.leftTacho >= this.leftTacho * 0.9 && movement.leftTacho <= this.leftTacho * 1.1) && ((Math.abs(movement.rightTacho - this.rightTacho) < 100) || (movement.rightTacho >= this.rightTacho * 0.9 && movement.rightTacho <= this.rightTacho * 1.1))) {
                return true;
            }
        }
        return false;
    }

    public int splitDistanceIntoEqualTachoCounts() {
        int tachoCount = (int) (this.distance * 360 / 2 / 3.14159 / 2.75 / 2);
        return tachoCount;
    }

    public Movement sum(Movement movement) {
        if (movement == null) {
            return this;
        }
        double x1 = this.leftTacho * Math.cos(this.bearing);
        double x2 = movement.getLeftTachoCount() * Math.cos(movement.getBearing());
        double y1 = this.leftTacho * Math.sin(this.bearing);
        double y2 = movement.getLeftTachoCount() * Math.sin(movement.getBearing());
        double bearing = Math.atan2(y1 + y2, x1 + x2);
        double distance = Math.sqrt((x1+x2)*(x1+x2)+((y1+y2)*(y1+y2)));
        Movement sum = new Movement((int)distance, (int)distance, (float)bearing);
        return sum;
    }
    
    public boolean isSamePosition(Movement movement) {
        if ((Math.abs(this.position.leftTacho - movement.position.leftTacho) < 500) && (Math.abs(this.position.rightTacho - movement.position.rightTacho) < 500) && (Math.abs(this.position.bearing - movement.position.bearing) < 72)) {
            return true;
        }
        return false;
    }
}
