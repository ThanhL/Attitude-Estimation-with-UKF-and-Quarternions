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
float qw, qx, qy, qz;

// --- Quaternion ---
void apply_quaternion_rotation(float qw, float qx, float qy, float qz)
{ 
  // Since processing has coordinate system following left handed rule we need to convert
  // quaternions to left-hand-rule representations before applying matrix rotation
  // Left-handed coordinate conversion: https://www.codeproject.com/Tips/1240454/How-to-Convert-Right-Handed-to-Left-Handed-Coordin
  // Processing P3D coordinate system: https://processing.org/tutorials/p3d/
  // Left-handed conversion of right-handed quaternion 
  float lh_qw = qw;
  float lh_qx = -qx;
  float lh_qy = -qz;
  float lh_qz = -qy;
  
  // Using the quaternion coordinates calculate the rotation matrix to be applied by processing
  // Rotation matrix from quaternion: https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
  // Note that apply matrix accepts a homogenous transform matrix, so we add the rotation matrix
  // of the quaternion with a [0,0,0] translation vector for homogenous transform matrix.
  applyMatrix(1 - 2*lh_qy*lh_qy - 2*lh_qz*lh_qz, 2*lh_qx*lh_qy - 2*lh_qz*lh_qw, 2*lh_qx*lh_qz + 2*lh_qy*lh_qw, 0.0,
              2*lh_qx*lh_qy + 2*lh_qz*lh_qw, 1 - 2*lh_qx*lh_qx - 2*lh_qz*lh_qz, 2*lh_qy*lh_qz - 2*lh_qx*lh_qw, 0.0,
              2*lh_qx*lh_qz - 2*lh_qy*lh_qw, 2*lh_qy*lh_qz + 2*lh_qx*lh_qw, 1 - 2*lh_qx*lh_qx - 2*lh_qy*lh_qy, 0.0,
              0.0, 0.0, 0.0,  1.0);
  
  
  // If processing followed right hand rule, we would just use applyMatrix without the need to convert the quaternions 
  // to left-handed with the following code.
  //applyMatrix(  1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw, 0.0,
  //              2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw, 0.0,
  //              2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy, 0.0,
  //              0.0, 0.0, 0.0,  1.0);
}


// --- Processing main ---
void setup() 
{
  //size (960, 640, P3D);
  //// --- STL import ---
  //mesh=(TriangleMesh)new STLReader().loadBinary(sketchPath("rocket.stl"),STLReader.TRIANGLEMESH);
  ////mesh=(TriangleMesh)new STLReader().loadBinary(sketchPath("mesh-flipped.stl"),STLReader.TRIANGLEMESH).flipYAxis();
  //gfx=new ToxiclibsSupport(this);
  
  
  //// --- Setup the Serial Communication ---
  //teensy_port = new Serial(this, "COM6", 9600); // starts the serial communication
  //teensy_port.bufferUntil('\n');
  
  // Define dimensions of the processing visualization
  size (960, 640, P3D);

  // --- Setup the Serial Communication ---
  teensy_port = new Serial(this, "COM6", 9600); // starts the serial communication
  teensy_port.bufferUntil('\n');

}


void draw() 
{
  background(33);
  translate(width/2,height/2,0);
  
  // Quaternion values displayed on the processing visualuazation of orientation
  textSize(22);
  text("qw: " + str(qw) + "     qx: " + str(qx) + "     qy: " + str(qy) + "     qz: " + str(qz), -100, 265);
  
  // Apply quaternion rotation
  apply_quaternion_rotation(qw, qx, qy, qz);

  // 3D Object to be drawn to visualize rotation
  textSize(30);  
  fill(230, 0, 33);
  box (386, 40, 200); // Draw box
  textSize(25);
  fill(255, 255, 255);
  text("yasss", -183, 10, 101);
  
  
}




// --- Read data from the Serial Port ---
void serialEvent (Serial myPort) 
{ 
  // reads the data from the Serial Port up to the character '.' and puts it into the String variable "data".
  data = myPort.readStringUntil('\n');
  // if you got any bytes other than the linefeed:
  if (data != null) 
  {
    data = trim(data);
    // split the string at "/"
    String items[] = split(data, '\t');
    if (items.length > 1) 
    {
      ////--- Roll,Pitch in degrees
      //roll = float(items[0]);
      //pitch = float(items[1]);
      //yaw = float(items[2]);
    
      // --- quaternion
      qw = float(items[0]);
      qx = float(items[1]);
      qy = float(items[2]);
      qz = float(items[3]);
    }
  }
}
