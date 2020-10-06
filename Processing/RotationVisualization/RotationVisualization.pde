/*
    Arduino and ADXL345 Accelerometer - 3D Visualization Example 
     by Dejan, https://howtomechatronics.com
*/
// --- Libraries ---
import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;

import toxi.geom.*;
import toxi.geom.mesh.*;

import toxi.processing.*;

TriangleMesh mesh;
ToxiclibsSupport gfx;


// --- Teensy Attitude Estimation ---
Serial teensy_port;
String data="";
float roll, pitch, yaw;



void setup() 
{
  size (960, 640, P3D);
  // --- STL import ---
  mesh=(TriangleMesh)new STLReader().loadBinary(sketchPath("rocket.stl"),STLReader.TRIANGLEMESH);
  //mesh=(TriangleMesh)new STLReader().loadBinary(sketchPath("mesh-flipped.stl"),STLReader.TRIANGLEMESH).flipYAxis();
  gfx=new ToxiclibsSupport(this);
  
  
  // --- Setup the Serial Communication ---
  teensy_port = new Serial(this, "COM6", 9600); // starts the serial communication
  teensy_port.bufferUntil('\n');
  
  

}
void draw() 
{  
  translate(width/2, height/2, 0);
  background(33);
  textSize(22);
  text("Roll: " + str(roll) + "     Pitch: " + str(pitch) + "     Yaw: " + str(yaw), -100, 265);
  // Rotate the object
  //rotateX(radians(roll));
  //rotateY(radians(pitch));
  //rotateZ(radians(yaw));
  
  rotateX(radians(roll));
  rotateY(radians(yaw));
  rotateZ(radians(-pitch));
  
  // 3D 0bject
  textSize(30);  
  fill(230, 0, 33);
  box (386, 40, 200); // Draw box
  textSize(25);
  fill(255, 255, 255);
  text("yasss", -183, 10, 101);
  
  gfx.origin(new Vec3D(),200);
  //gfx.mesh(mesh,false,10); 
 
  //delay(10);
  //println("ypr:\t" + angleX + "\t" + angleY); // Print the values to check whether we are getting proper values

}




// --- Read data from the Serial Port ---
void serialEvent (Serial myPort) 
{ 
  // reads the data from the Serial Port up to the character '.' and puts it into the String variable "data".
  data = myPort.readStringUntil('\n');
  // if you got any bytes other than the linefeed:
  if (data != null) {
    data = trim(data);
    // split the string at "/"
    String items[] = split(data, '\t');
    if (items.length > 1) {
      //--- Roll,Pitch in degrees
      roll = float(items[0]);
      pitch = float(items[1]);
      yaw = float(items[2]);
    }
  }
}
