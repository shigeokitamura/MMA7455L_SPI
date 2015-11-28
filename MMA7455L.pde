import processing.serial.*;

Serial serial;

void setup() {
  size(256, 256);
  //frameRate(30);
  serial = new Serial(this, "/dev/cu.usbmodem1411", 9600);
}

void draw() {

  while(serial.available() > 0) {
      background(255);
    String str = serial.readStringUntil('\n');
    if(str != null) {
      String str2[] = split(str, ", ");
      fill(0);
      textSize(30);
      text("x: " + str2[0], 10, 50);
      text("y: " + str2[1], 10, 80);
      text("z: " + str2[2], 10, 110);
    }
  }
}
