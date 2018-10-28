import controlP5.*;
import processing.serial.*;

ControlP5 cp5;
Chart yawChart, pitchChart, rollChart;

Serial  myPort;
String  portName = "/dev/ttyUSB0"; // Index of serial port in list (varies by computer)
int     lf = 10;       //ASCII linefeed
String  inString;      //String for testing serial communication
int     calibrating;

// Data [ax, ay, az, gx, gy, gz, mx, my, mz, yaw, pitch, roll, rate, lin_ax, lin_ay, lin_az] 
float data[] = new float[16];
float rate = 0;

float timestamp = 0, lastTimestamp=0;
float rxrate = 0;

void setup() { 
  surface.setResizable(true);

  size(1400, 800, P3D);
  //fullScreen(P3D);

  noStroke();
  colorMode(RGB, 256);
  
  PFont font;
  font = loadFont("Monospaced.bold-16.vlw");
  textFont(font, 12);
  textAlign(LEFT, TOP);
  
  println(" Connecting to -> " + portName);
  myPort = new Serial(this, portName, 115200);
  myPort.clear();
  myPort.bufferUntil(lf);
  
  // setup charts
  cp5 = new ControlP5(this);
  yawChart = cp5.addChart("Yaw")
               .setCaptionLabel("Yaw")
               .setPosition(20, 30)
               .setSize(200, 100)
               .setRange(-180, 180)
               .setView(Chart.LINE)
               .setStrokeWeight(1.5)
               .setColorCaptionLabel(color(140))
               ;
  yawChart.addDataSet("data");
  yawChart.setData("data", new float[200]);
  

  pitchChart = cp5.addChart("Pitch")
               .setCaptionLabel("Pitch")
               .setPosition(240, 30)
               .setSize(200, 100)
               .setRange(-90, 90)
               .setView(Chart.LINE)
               .setStrokeWeight(1.5)
               .setColorCaptionLabel(color(140))
               ;
  pitchChart.addDataSet("data");
  pitchChart.setData("data", new float[200]);

  rollChart = cp5.addChart("Roll")
               .setCaptionLabel("Roll")
               .setPosition(460, 30)
               .setSize(200, 100)
               .setRange(-180, 180)
               .setView(Chart.LINE)
               .setStrokeWeight(1.5)
               .setColorCaptionLabel(color(140))
               ;
  rollChart.addDataSet("data");
  rollChart.setData("data", new float[200]);

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
  text(String.format("Roll = %5.2f°", data[11]), 460, 10);
  fill(0, 255, 0);
  text(String.format("Pitch = %5.2f°", data[10]), 240, 10);
  fill(#2EBDF0);
  text(String.format("Yaw = %5.2f°", data[9]), 20, 10);

  textSize(14);
  fill(#EBDF02);
  text(String.format("Filter: %5.0f Hz, GUI: %3.0f Hz", rate, rxrate), 20, height - 20);  // update rate
  
  textSize(16);
  fill(#EE6600);
  text(String.format("ax:%4.1f g", data[0]), 680, 30);
  text(String.format("ay:%4.1f g", data[1]), 680, 50);
  text(String.format("az:%4.1f g", data[2]), 680, 70);

  fill(#F02EBD);
  text(String.format("gx:%4.1f °/s", data[3]), 810, 30);
  text(String.format("gy:%4.1f °/s", data[4]), 810, 50);
  text(String.format("gz:%4.1f °/s", data[5]), 810, 70);

  fill(#BDF02E);
  text(String.format("mx:%3.0f µT", data[6]), 950, 30);
  text(String.format("my:%3.0f µT", data[7]), 950, 50);
  text(String.format("mz:%3.0f µT", data[8]), 950, 70);
/*  
  fill(#BDF02E);
  text(String.format("ax:%3.2f m/s2", data[13]), 1250, 30);
  text(String.format("ay:%3.2f m/s2", data[14]), 1250, 50);
  text(String.format("az:%3.2f m/s2", data[15]), 1250, 70);  
  */
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
    
    yawChart.push("data", data[9]);
    pitchChart.push("data", data[10]);
    rollChart.push("data", data[11]);
  } 
  catch (Exception e) {
    println(inString);
    //    println("Caught Exception");
  }
}
