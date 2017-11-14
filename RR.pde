import processing.serial.*;
PrintWriter output;

Serial arduino;
void setup(){
  arduino = new Serial(this, "/dev/cu.usbserial-DN02B3P7",115200);
  output = createWriter("values.txt");
}

int val;
void draw(){
  if(arduino.available()>0){
    String input = arduino.readString();
    print(input);
    output.print(input);
  }
}

void keyPressed() {
  output.flush(); // Writes the remaining data to the file
  output.close(); // Finishes the file
  exit(); // Stops the program
}