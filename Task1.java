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
public class Task1 {
    private static final int WIDTH = 160;
    private static final int HEIGHT = 120;
    //private static final int NUM_PIXELS = WIDTH * HEIGHT;

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
       
        colorSensor.setCurrentMode("Ambient");
        gyroSensor.setCurrentMode("Angle");
        ultrasonicSensor.enable();
        ultrasonicSensor.getDistanceMode();
        float[] colorSample = new float[colorSensor.sampleSize()];
        float[] gyroSample = new float[gyroSensor.sampleSize()];        
        float[] sampleUltrasonic = new float[ultrasonicSensor.sampleSize()];
//        Movement[] movements = new Movement[50]; // ArrayList is preferable but Array is simpler
//        Movement[] movementsGrey = new Movement[3];
//        Movement movementBuffer;
        RobotMovement previousMovement;
        boolean isGoingBack = false;
        boolean isGoingBack2 = false;
        
        int blackTotal, blackLeft, blackCentre, blackRight, blackLeftWide, blackRightWide,
        greyLoopCount = 0, greyAreaCount = 0, greyAreaDetectedCount = 0,
        rightJunctionCount = 0, leftJunctionCount = 0, straightCount = 0, blackCount = 0;
        Sound.beep();
        Button.waitForAnyPress();
        goStraight();
        Delay.msDelay(2000);
        while (Button.ESCAPE.isUp()) {        
            ultrasonicSensor.fetchSample(sampleUltrasonic, 0);
            if (sampleUltrasonic[0] < 0.14) {
                goBackward();
                Delay.msDelay(360);
                turnAround();
                goBackward();
                Delay.msDelay(150);
                stop();
                Delay.msDelay(150);
                continue;
            }

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
                        if (greyAreaCount >= 5) {
                            previousMovement = robotMovement;
                            stop();
                            ++greyAreaDetectedCount;
                            // check if same grey area?
                            Delay.msDelay(1000);
                            Sound.beep();
                            greyLoopCount = 88;     
                            
                            

                            if (greyAreaDetectedCount == 3) {
                                do {
                                    motorLeft.rotate(-12, true);
                                    motorRight.rotate(12, false);
                                    gyroSensor.fetchSample(gyroSample, 0);                
                                  } while (Math.abs(gyroSample[0]) <= 10);
                                  turnAround();
                                  goStraight();
                                  isGoingBack = true;
                                  continue;
                            }
                            
                            

                          goBackward();
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
//
//            gyroSensor.fetchSample(gyroSample, 0); // only do this when needed

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

            // 2 outer loops
            // one wiht a rnage of 60 increment by 2
            // and one with a lesser range but increment by 1
            for (int y = 42; y <= 51; y += 2) { // 22, 57 // 27, 52
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
            if (isGoingBack == true) {
                if (blackTotal == 0) { // tests for all white, then test for all black?
                    isGoingBack2 = true;                    
                }
//                continue;
            }
            if (blackTotal >= 72) {
                if (isGoingBack2 == true) {
                    stop();
                }
                if (blackCount < 0) {
                    goBackward();
                    Delay.msDelay(250);
                    blackCount = 20;
                    continue;
                }
                turnAround();
                stop();
                Delay.msDelay(500);
                goStraight();
                Delay.msDelay(20);
                continue;
            }        
            else {
                --blackCount;
                turnAroundMultiplier = 1;
            }
            

            if (blackLeftWide <= 10) {
                if (leftJunctionCount < 0) {
                    leftJunctionCount = 0;
                }
                ++leftJunctionCount;
            }
            else {
                --leftJunctionCount;           
            }
            if (blackRightWide <= 10) {
                if (rightJunctionCount < 0) {
                    rightJunctionCount = 0;
                }
                ++rightJunctionCount;
            }
            else {
                --rightJunctionCount;           
            }

                        Button.LEDPattern(0);



            //            if (leftJunctionCount > 2) {
            //                if (robotMovement == RobotMovement.LEFT || robotMovement == RobotMovement.RIGHT) {
            //                    continue;
            //                }
            //                Button.LEDPattern(2);
            //                leftJunctionCount = 0;
            //                Delay.msDelay(100);
            //                turnLeft();
            //                Delay.msDelay(200);
            //                continue;
            //                // create left junction
            //            }


            if (blackCentre <= 4) {
                // disable right and left junction counts
                // see if it can move in tight areas properly
                // then add the left and right junction counts
                // then perfect it so that it matches
                
                
                if (straightCount > 50) {
                    if (leftJunctionCount > 2) {             
                        Button.LEDPattern(1);
                        leftJunctionCount = 0;
                        goStraight();
                        Delay.msDelay(450);
                        turnLeft();
                        Delay.msDelay(1000);
                        goBackward();
                        Delay.msDelay(150);
                        straightCount = 0;
                        continue;
                    }
                if (rightJunctionCount > 2) {             
                    Button.LEDPattern(1);
                    rightJunctionCount = 0;
                    goStraight();
                    Delay.msDelay(450);
                    turnRight();
                    Delay.msDelay(1000);
                    goBackward();
                    Delay.msDelay(150);
                    straightCount = 0;
                    continue;
//                    turnRight();
                    // grab frames here and process until centre is less black?
                    // create right junction
                } // todo junctions
//                if (leftJunctionCount > 1) {             
//                    Button.LEDPattern(2);
//                    leftJunctionCount = 0;
//                    goStraight();
//                    Delay.msDelay(600);
//                    turnRightReversed();
//                    Delay.msDelay(600);
//                    goBackward();
//                    Delay.msDelay(150);
//                    continue;
//                }
                
                }
                
                
                ++straightCount;
                goStraight();                
            }
            else if (blackLeft <= 10) {
                straightCount = 0;
                if (robotMovement != RobotMovement.LEFT) {
                    Delay.msDelay(220);
                    turnLeft();
                }
            }        else if (blackRight <= 10) {
                straightCount = 0;
                if (robotMovement != RobotMovement.RIGHT) {
                    Delay.msDelay(220);
                    turnRight();
                    // save current movement
                    //                    gyroSensor.fetchSample(gyroSample, 0);
                    //                    movementBuffer = new Movement(motorLeft.getTachoCount(), motorRight.getTachoCount(), gyroSample[0]);
                    //                    resetTachoCounts();
                }
            }     
            else { // this is needed in case the left, centre and right are black but the wides are not black?
                turnAround();
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
        robotMovement = RobotMovement.NONE;
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
        motorRight.rotate(400 - turnAroundMultiplier * 15, true);
        motorLeft.rotate(-400 + turnAroundMultiplier * 15, false);
        ++turnAroundMultiplier;
        stop();
        Delay.msDelay(250); // let camera adjust        
        //        goStraight();
        //        Delay.msDelay(40);
        //        robotMovement = RobotMovement.TURN_AROUND;
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

    public static void resetTachoCounts() {
        motorLeft.resetTachoCount();
        motorRight.resetTachoCount();
    }

    enum JunctionType {
        LEFT_RIGHT,
        LEFT,
        RIGHT
    }
    // should it backtrack after second junction detection? or do the whole maze and then decide?

    class Junction {
        private int movementIndex;
        private JunctionType junctionType;
        boolean isAllOptionsTravelled = false;

        public Junction(int movementIndex, JunctionType junctionType) {
            this.movementIndex = movementIndex;
            this.junctionType = junctionType;
        }

        public int getMovementIndex() {
            return this.movementIndex;
        }

        public JunctionType getJunctionType() {
            return this.junctionType;
        }
    }
}

class Movement {
    //
    // Any turning event should create a new movement
    // unless the turning is after less than 20 tacho counts?
    // this ensures that each vector is a straight line
    // as for turning vectors... we ignore for now.
    // there is no need to know a turn
    // we just need to know the straight paths

    //
    // this means that we will have gaps in our movement
    // but it is fine?
    // what we care about is the ifStraight when turnin then make movement and reset tacho
    //

    // this whole class just needs starting bearing, ending bearing and magnitude
    // can we calculate radius of curvature if we have left and right tachos? i think we can
    public Movement position; // the movement that absolutely describes this movement
    private int leftTacho = 0;
    private int rightTacho = 0;
    private int distance;
    private int bearing = 0; // movement started with this bearing (should be changed to start bearing and endbearing?)
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
        // get x1 x2 y1 y2
        double x1 = this.leftTacho * Math.cos(this.bearing);
        double x2 = movement.getLeftTachoCount() * Math.cos(movement.getBearing());
        double y1 = this.leftTacho * Math.sin(this.bearing);
        double y2 = movement.getLeftTachoCount() * Math.sin(movement.getBearing());
        double bearing = Math.atan2(y1 + y2, x1 + x2);
        double place = Math.sqrt((x1+x2)*(x1+x2)+((y1+y2)*(y1+y2)));
//        int tachoCount = this.splitDistanceIntoEqualTachoCounts() + movement.splitDistanceIntoEqualTachoCounts();
//        int x = (int) (this.leftTacho * Math.cos(this.bearing) + movement.getLeftTachoCount() * Math.cos(movement.getBearing()));
//        int y = (int) (this.leftTacho * Math.sin(this.bearing) + movement.getLeftTachoCount() * Math.sin(movement.getBearing()));
//        int bearing = (int) Math.atan2(y, x);
//        int bearing = (this.bearing + movement.bearing) % 360;
        // make better
        // make all double
        Movement sum = new Movement((int)place, (int)place, (float)bearing);
        return sum;
    }
    
    public boolean isSamePosition(Movement movement) {
//        this.position = this.getAbsolutePosition();
//        movement.position = movement.getAbsolutePosition();
        if ((Math.abs(this.position.leftTacho - movement.position.leftTacho) < 500) && (Math.abs(this.position.rightTacho - movement.position.rightTacho) < 500) && (Math.abs(this.position.bearing - movement.position.bearing) < 72)) {
            return true;
        }
        return false;
    }
}
