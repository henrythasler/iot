 /*
 *  Program for visualizing rotation data about the three IMU axes as calculated by the MPU6050 DMP
 *  and the complementary filter.
 */

import processing.serial.*;

Serial  myPort;
String  portName = "/dev/ttyUSB0"; // Index of serial port in list (varies by computer)
int     lf = 10;       //ASCII linefeed
String  inString;      //String for testing serial communication

// x,y coordinates of circles 1, 2, 3 (pitch, roll, yaw)
int cx[] = {150, 450, 750};
int cy[] = {200, 200, 200};

// circle diameters
int d   = 200; 

// Data [ax, ay, az, gx, gy, gz, mx, my, mz, yaw, pitch, roll, rate] 
float data[] = new float[13];


/*
 * Draws a line of length len, centered at x, y at the specified angle
 */
void drawLine(int x, int y, int len, float angle) {
  pushMatrix();
  translate(x, y);
  rotate(angle);
  line(-len/2, 0, len/2, 0);
  popMatrix();
}


void setup() {
  
  // Set up the main window
  size(900, 400);
  //background(0);
  
  // Set up serial port access
  //  println("in setup");
  println(" Connecting to -> " + portName);
  myPort = new Serial(this, portName, 115200);
  myPort.clear();
  myPort.bufferUntil(lf);
}

void draw() {
  
  background(0);
  
  // Draw the three background circles
  noStroke();
  fill(225);
  for (int i = 0; i < 3; i++) {
    ellipse(cx[i], cy[i], d, d);
  }
  
  // Draw the lines representing the angles
  for (int i = 0; i < 3; i++) {
    strokeWeight(3);
    stroke(255, 0, 0);
    drawLine(cx[i], cy[i], d, radians(data[9+i]));

    strokeWeight(1);
    stroke(128, 128, 128);
    drawLine(cx[i], cy[i], d, 0);
    drawLine(cx[i], cy[i], d, radians(90));
  }
  
  // Draw the explanatory text
  textSize(20);
  fill(255);
  text("MPU 9250", 10, 20);
  
 
  fill(255);
  text("Yaw", cx[0]-25, 75);
  text("Pitch", cx[1]-22, 75);
  text("Roll", cx[2]-22, 75);
  
  fill(255, 0, 0);
  text(String.format("%5.2f", data[9]), cx[0]-22, 320);
  text(String.format("%5.2f", data[10]), cx[1]-22, 320);
  text(String.format("%5.2f", data[11]), cx[2]-22, 320);

  fill(#EBDF02);
  text(String.format("%5.0f Hz", data[12]), 800, 20);  // update rate

  textSize(14);
  fill(#2EBDF0);
  text(String.format("ax=%5.2f", data[0]), cx[0]-22, 340);
  text(String.format("ay=%5.2f", data[1]), cx[1]-22, 340);
  text(String.format("az=%5.2f", data[2]), cx[2]-22, 340);

  fill(#F02EBD);
  text(String.format("gx=%5.2f", data[3]), cx[0]-22, 360);
  text(String.format("gy=%5.2f", data[4]), cx[1]-22, 360);
  text(String.format("gz=%5.2f", data[5]), cx[2]-22, 360);

  fill(#BDF02E);
  text(String.format("mx=%5.2f", data[6]), cx[0]-22, 380);
  text(String.format("my=%5.2f", data[7]), cx[1]-22, 380);
  text(String.format("mz=%5.2f", data[8]), cx[2]-22, 380);
 
}


/*
 *  Read and process data from the serial port
 */
void serialEvent(Serial p) {
  inString = myPort.readString();
  
  try {
    // Parse the data
    //println(inString);
    String[] dataStrings = split(inString, ',');
    for (int i = 0; i < dataStrings.length; i++) {
      data[i] = float(dataStrings[i]);
    }
  } catch (Exception e) {
    println("Caught Exception");
  }
}
