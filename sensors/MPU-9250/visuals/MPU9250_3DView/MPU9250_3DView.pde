import processing.serial.*;

Serial  myPort;
String  portName = "/dev/ttyUSB0"; // Index of serial port in list (varies by computer)
int     lf = 10;       //ASCII linefeed
String  inString;      //String for testing serial communication
int     calibrating;

// Data [ax, ay, az, gx, gy, gz, mx, my, mz, yaw, pitch, roll, rate] 
float data[] = new float[13];
float rate = 0;

float timestamp = 0, lastTimestamp=0;
float rxrate = 0;

void setup() { 
  //  size(640, 360, P3D); 
  size(1400, 800, P3D);
  surface.setResizable(true);

  noStroke();
  colorMode(RGB, 256);


  println(" Connecting to -> " + portName);
  myPort = new Serial(this, portName, 115200);
  myPort.clear();
  myPort.bufferUntil(lf);
} 

void draw() { 
  background(32);
  lights();

  // Tweak the view of the rectangles
  int distance = 150;
  int x_rotation = 90;
  int z_rotation = 90;

  //Show combined data
  pushMatrix();
  translate(3*width/6, 2*height/3, -distance);
  rotateX(radians(-data[10] - x_rotation));
  rotateY(radians(-data[11]));
  rotateZ(radians(-data[9] - z_rotation));

  fill(255);
  box(500, 800, 50);

  fill(255, 0, 0);

  pushMatrix();
  translate(-350, 0, 0);
  sphere(30);  
  popMatrix();

  box(700, 20, 20);

  fill(0, 255, 0);
  pushMatrix();
  translate(0, -450, 0);
  sphere(30);  
  popMatrix();

  box(20, 900, 20);

  fill(#2EBDF0);
  pushMatrix();
  translate(0, 0, -100);
  sphere(30);  
  popMatrix();
  box(20, 20, 200);
  popMatrix();

  textSize(20);
  fill(255, 0, 0);
  text(String.format("Roll = %5.2f°", data[11]), 420, 20);
  fill(0, 255, 0);
  text(String.format("Pitch = %5.2f°", data[10]), 220, 20);
  fill(#2EBDF0);
  text(String.format("Yaw = %5.2f°", data[9]), 20, 20);

  fill(#EBDF02);
  text(String.format("%5.0f Hz / %3.0f Hz", rate, rxrate), 800, 20);  // update rate
} 

void serialEvent(Serial p) {
  inString = myPort.readString();

  try {
    // Parse the data
    //println(inString);
    String[] dataStrings = split(inString, ',');
    for (int i = 0; i < dataStrings.length; i++) {
      data[i] = float(dataStrings[i]);
    }    
    rate = (rate*0.99 + data[12]*.01);

    timestamp = millis()/1000.;
    rxrate = rxrate*0.9 + (1./(timestamp - lastTimestamp))*0.1;
    lastTimestamp = timestamp;
  } 
  catch (Exception e) {
    println(inString);
    //    println("Caught Exception");
  }
}
