import processing.serial.*;
import controlP5.*;

Serial myPort;          
String selectedPort;    
String sensorData = ""; 
float[] rpsData;        
int graphWidth;         
int graphHeight;        
float currentRPS = 0;   // Variabel untuk menyimpan RPS terkini
int currentPulses = 0;  // Variabel untuk menyimpan jumlah pulses
int currentPWM = 0;     // Variabel untuk menyimpan nilai PWM

ControlP5 cp5;
Textfield kpField, kiField, kdField, setpointField;
Button setPIDButton, setSetpointButton, startMotorButton;
DropdownList portList;

// State control
boolean isMotorRunning = false;

// Warna
color bgColor = color(240, 240, 245);
color primaryColor = color(33, 150, 243);
color textColor = color(33, 33, 33);
color inputBgColor = color(255, 255, 255);
color startButtonColor = color(76, 175, 80);  // Green for start
color stopButtonColor = color(244, 67, 54);   // Red for stop

void setup() {
  size(1080, 750);  // Increased height to accommodate label spacing
  surface.setTitle("Motor PID Control");
  textMode(SHAPE);
  
  // Inisialisasi CP5
  cp5 = new ControlP5(this);
  cp5.setColorBackground(primaryColor)
     .setColorForeground(color(33, 150, 243, 150))
     .setColorActive(color(33, 150, 243, 220));
  
  // Pengaturan font
  PFont p = createFont("Arial", 12);
  cp5.setFont(p);
  
  // Dropdown pilih port
  portList = cp5.addDropdownList("Port Selection")
               .setPosition(20, 30)
               .setWidth(250)
               .setItemHeight(30)
               .setBarHeight(35)
               .setCaptionLabel("Select Serial Port");
  
  // daftar isi port
  String[] ports = Serial.list();
  for (int i = 0; i < ports.length; i++) {
    portList.addItem(ports[i], i);
  }
  
  // Tunning PID
  int startY = 200;
  int spacing = 60;  // Increased spacing for better visibility
  
  // Proportional (Kp)
  fill(textColor);
  textSize(14);
  textAlign(LEFT);
  text("Proportional (Kp):", 20, startY);
  kpField = cp5.addTextfield("KP")
               .setPosition(40, 105)
               .setWidth(100)
               .setHeight(13)
               .setColor(textColor)
               .setColorBackground(inputBgColor)
               .setColorForeground(primaryColor)
               .setAutoClear(true);
               
  // Integral (Ki)
  text("Integral (Ki):", 20, startY + spacing);
  kiField = cp5.addTextfield("KI")
               .setPosition(40, spacing + 95)
               .setWidth(100)
               .setHeight(15)
               .setColor(textColor)
               .setColorBackground(inputBgColor)
               .setColorForeground(primaryColor)
               .setAutoClear(true);
               
  // Derivative (Kd)
  text("Derivative (Kd):", 20, startY + 2*spacing);
  kdField = cp5.addTextfield("KD")
               .setPosition(40, 2*spacing + 85)
               .setWidth(100)
               .setHeight(13)
               .setColor(textColor)
               .setColorBackground(inputBgColor)
               .setColorForeground(primaryColor)
               .setAutoClear(true);
  
  // Tombol Set PID
  setPIDButton = cp5.addButton("SendPIDValues")
                    .setPosition(20, 3*spacing + 70)
                    .setWidth(250)
                    .setHeight(40)
                    .setCaptionLabel("Set PID Parameters");
  
  // Setpoint
  text("Setpoint (RPS):", 20, startY + 4*spacing);
  setpointField = cp5.addTextfield("Setpoint")
                     .setPosition(20, 4*spacing + 100)
                     .setWidth(100)
                     .setHeight(13)
                     .setColor(textColor)
                     .setColorBackground(inputBgColor)
                     .setColorForeground(primaryColor)
                     .setAutoClear(true);
  
  // Tombol Set Setpoint
  setSetpointButton = cp5.addButton("SendSetpoint")
                         .setPosition(20, 5*spacing + 80)
                         .setWidth(250)
                         .setHeight(40)
                         .setCaptionLabel("Set Setpoint");
  
  // Tombol start motor
  startMotorButton = cp5.addButton("ToggleMotor")
                        .setPosition(20, 8*spacing)
                        .setWidth(250)
                        .setHeight(40)
                        .setCaptionLabel("Start Motor (Clockwise)")
                        .setColorBackground(startButtonColor);
  
  // Inisialisasi Grafik
  graphWidth = width - 320;  // Adjusted for sidebar 
  graphHeight = height - 100;
  rpsData = new float[graphWidth];
}

void draw() {
  background(bgColor);
  fill(255);
  noStroke();
  rect(0, 0, 290, height);
  
  // Membaca serial data
  if (myPort != null) {
    while (myPort.available() > 0) {
      char inChar = myPort.readChar();
      if (inChar == '\n') {
        processSensorData(sensorData);
        sensorData = "";
      } else {
        sensorData += inChar;
      }
    }
  }
  
  // Gambar Grafik
  pushMatrix();
  translate(300, 50); 
  drawGraph();
  popMatrix();
  
  // headers sidebar
  fill(primaryColor);
  textSize(16);
  textAlign(LEFT);
  text("Serial Port", 20, 20);
  text("PID Control", 20, 90);
  
  // Labels untuk input PID Tunning
  text("KP:", 10, 115);
  text("KI:", 10, 165);
  text("KD:", 10, 215);
  text("Setpoint:", 10, 320);
  
  // Display Informasi
  displayMotorInfo();
}

// Fungsi untuk menampilkan informasi motor
void displayMotorInfo() {
  fill(textColor);
  textSize(14);
  textAlign(LEFT);
  
  // Header
  text("Motor Information", 20, 560);
  
  // Garis pembatas
  stroke(200);
  line(10, 570, 280, 570);
  
  // Tampilan RPS
  text("Current RPS:", 20, 600);
  text(nf(currentRPS, 0, 2), 200, 600);
  
  // Tampilan Pulses
  text("Total Pulses:", 20, 630);
  text(currentPulses, 200, 630);
  
  // Tampilan PWM
  text("PWM Value:", 20, 660);
  text(currentPWM, 200, 660);
  
  // Status Motor
  text("Motor Status:", 20, 690);
  fill(isMotorRunning ? color(76, 175, 80) : color(244, 67, 54));
  text(isMotorRunning ? "Running" : "Stopped", 200, 690);
}

// Port selection event
void controlEvent(ControlEvent event) {
    if (event.isFrom(portList)) {
        int selectedIndex = int(event.getValue());
        selectedPort = Serial.list()[selectedIndex];

        // Tutup port jika terbuka
        if (myPort != null) {
            myPort.stop();
            myPort = null; // Pastikan myPort direset
        }

        // Membuka port baru
        try {
            myPort = new Serial(this, selectedPort, 9600);
            println("Connected to port: " + selectedPort);
        } catch (Exception e) {
            println("Error connecting to port: " + e.getMessage());
        }
    }
}


// Mengirim Nilai PID ke Arduino
void SendPIDValues() {
  if (myPort != null) {
    try {
      float kp = float(kpField.getText());
      float ki = float(kiField.getText());
      float kd = float(kdField.getText());
      
      // Format: PID,kp,ki,kd
      String pidCommand = "PID," + kp + "," + ki + "," + kd + "\n";
      myPort.write(pidCommand);
      println("Sent PID values: " + pidCommand);
    } catch (Exception e) {
      println("Error sending PID values: " + e.getMessage());
    }
  }
}

// Mengirim Setpoint ke Arduino
void SendSetpoint() {
  if (myPort != null) {
    try {
      float setpoint = float(setpointField.getText());
      
      // Format: SETPOINT,value
      String setpointCommand = "SETPOINT," + setpoint + "\n";
      myPort.write(setpointCommand);
      println("Sent Setpoint: " + setpointCommand);
    } catch (Exception e) {
      println("Error sending Setpoint: " + e.getMessage());
    }
  }
}

// Tombol Nyala / Mati motor
void ToggleMotor() {
    if (myPort != null) {
        isMotorRunning = !isMotorRunning;

        if (isMotorRunning) {
            // Start motor dengan PWM default (misalnya 0)
            myPort.write("START,CW\n");
            currentPWM = 0; // Reset nilai PWM di GUI
            myPort.write("PWM,0\n"); // Kirim nilai PWM default ke Arduino
            startMotorButton.setCaptionLabel("Stop Motor")
                             .setColorBackground(stopButtonColor);
            println("Motor started clockwise with PWM = 0");
        } else {
            // Stop motor
            myPort.write("STOP\n");
            currentPWM = 0; // Reset nilai PWM di GUI
            startMotorButton.setCaptionLabel("Start Motor (Clockwise)")
                             .setColorBackground(startButtonColor);
            println("Motor stopped");
        }
    }
}

// Proses data
void processSensorData(String data) {
    try {
        String[] values = split(data.trim(), ',');
        if (values.length == 3) {
            currentPulses = int(values[0]);
            currentRPS = float(values[1]);
            currentPWM = int(values[2]);

            // Sinkronisasi PWM saat motor berhenti
            if (!isMotorRunning) {
                currentPWM = 0;
            }

            println("Pulses: " + currentPulses + ", RPS: " + currentRPS + ", PWM: " + currentPWM);

            // Shift data to the left and add new RPS value
            for (int i = 0; i < rpsData.length - 1; i++) {
                rpsData[i] = rpsData[i + 1];
            }
            rpsData[rpsData.length - 1] = currentRPS;
        }
    } catch (Exception e) {
        println("Error processing data: " + e.getMessage());
    }
}


void drawGraph() {
  fill(250);
  stroke(200);
  rect(0, 0, graphWidth, graphHeight);
  stroke(220);
  strokeWeight(1);
  
  // Vertikal grid
  int verticalGridLines = 10;
  for (int i = 1; i <= verticalGridLines; i++) {
    float x = map(i, 0, verticalGridLines, 0, graphWidth);
    line(x, 0, x, graphHeight);
  }
  
  // Horizontal grid
  int horizontalGridLines = 5;
  float maxRPS = 200.0; // Maximum RPS skala
  for (int i = 0; i <= horizontalGridLines; i++) {
    float y = map(i, 0, horizontalGridLines, graphHeight, 0);
    line(0, y, graphWidth, y);
    
    // Label Sumbu Y
    fill(textColor);
    textAlign(RIGHT, CENTER);
    textSize(10);
    text(nf(i * (maxRPS / horizontalGridLines), 0, 1), -10, y);
  }
  
  // Garis untuk setpoint
  float setpointValue;
  try {
    setpointValue = float(setpointField.getText());
  } catch (Exception e) {
    setpointValue = 0;
  }
  
  stroke(color(255, 0, 0, 150)); // Semi-transparent red
  strokeWeight(2);
  float setpointY = map(constrain(setpointValue, 0, maxRPS), 0, maxRPS, graphHeight, 0);
  line(0, setpointY, graphWidth, setpointY);
  
  // Garis untuk RPS
  stroke(primaryColor);
  strokeWeight(2);
  noFill();
  beginShape();
  for (int i = 0; i < rpsData.length; i++) {
    float y = map(constrain(rpsData[i], 0, maxRPS), 0, maxRPS, graphHeight, 0);
    vertex(i, y);
  }
  endShape();
  
  // Menambahkan label
  fill(textColor);
  textSize(12);
  textAlign(LEFT, TOP);
  text("Rotations Per Second (RPS)", 10, -20);
  
  // Tampilan Sumbu Y
  textAlign(CENTER, TOP);
  for (int i = 1; i <= verticalGridLines; i++) {
    float x = map(i, 0, verticalGridLines, 0, graphWidth);
    text(i + " sec", x, graphHeight + 10);
  }
}
