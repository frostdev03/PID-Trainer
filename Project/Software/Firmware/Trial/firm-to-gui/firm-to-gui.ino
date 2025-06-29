#include <Wire.h>
#include <AS5600.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

AS5600 as5600;

const char* ssid = "ESP32_Sensor_Dashboard"; 
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); 

// uint16_t last_raw_angle = 0;
// float last_angle_deg = 0.0;
String lastSerialMessage = "Tidak ada pesan.";

const char htmlContent[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="id">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Real-Time Dashboard</title>
    <style>
        body { font-family: Arial, sans-serif; background-color: #f0f2f5; margin: 0; padding: 20px; display: flex; justify-content: center; align-items: center; min-height: 100vh; }
        .container { background-color: white; padding: 30px; border-radius: 10px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); text-align: center; max-width: 400px; width: 100%; }
        h1 { color: #333; }
        #ws-status { font-size: 0.8em; padding: 5px 10px; border-radius: 15px; color: white; }
        .connected { background-color: #28a745; }
        .disconnected { background-color: #dc3545; }
        .sensor-data { margin-top: 20px; font-size: 1.2em; }
        .data-label { font-weight: bold; color: #555; }
        .data-value { color: #007BFF; font-family: 'Courier New', Courier, monospace; font-size: 1.5em; display: block; margin-top: 5px; }
        .message-box { margin-top: 30px; border-top: 1px solid #eee; padding-top: 20px; }
        .message-value { color: #28a745; font-style: italic; font-size: 1.2em; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Dashboard Real-Time <span id="ws-status" class="disconnected">Offline</span></h1>
        <div class="sensor-data">
            <p class="data-label">Posisi Sudut Mentah:</p>
            <span id="raw_angle" class="data-value">---</span>
            
            <p class="data-label" style="margin-top: 25px;">Derajat:</p>
            <span id="angle_deg" class="data-value">--- &deg;</span>
        </div>
        <div class="message-box">
            <p class="data-label">Pesan dari Serial Monitor:</p>
            <span id="serial_message" class="message-value">Menunggu pesan...</span>
        </div>
    </div>
    <script>
        // Alamat WebSocket, menggunakan hostname halaman untuk fleksibilitas
        var gateway = `ws://${window.location.hostname}/ws`;
        var websocket;

        // Fungsi untuk memulai koneksi WebSocket
        function initWebSocket() {
            console.log('Mencoba terhubung ke WebSocket...');
            websocket = new WebSocket(gateway);
            websocket.onopen = onOpen;
            websocket.onclose = onClose;
            websocket.onmessage = onMessage;
        }

        // Saat koneksi berhasil dibuka
        function onOpen(event) {
            console.log('Koneksi WebSocket berhasil dibuka.');
            document.getElementById('ws-status').innerHTML = 'Online';
            document.getElementById('ws-status').className = 'connected';
        }

        // Saat koneksi ditutup
        function onClose(event) {
            console.log('Koneksi WebSocket ditutup.');
            document.getElementById('ws-status').innerHTML = 'Offline';
            document.getElementById('ws-status').className = 'disconnected';
            setTimeout(initWebSocket, 2000); // Coba sambungkan lagi setelah 2 detik
        }

        // Saat menerima pesan dari server
        function onMessage(event) {
            console.log('Menerima pesan:', event.data);
            var myObj = JSON.parse(event.data);
            document.getElementById("raw_angle").innerHTML = myObj.raw;
            document.getElementById("angle_deg").innerHTML = myObj.deg + " &deg;";
            document.getElementById("serial_message").innerHTML = myObj.message;
        }
        
        // Panggil fungsi untuk memulai koneksi saat halaman dimuat
        window.addEventListener('load', initWebSocket);
    </script>
</body>
</html>
)rawliteral";

// websocket handler
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u terhubung dari %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u terputus\n", client->id());
      break;
    case WS_EVT_DATA:
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // pin default GPIO 21 (SDA) dan 22 (SCL)

  as5600.begin();
  // if (!as5600.isConnected()) {
  //   Serial.println("Error: Sensor AS5600 tidak terdeteksi! ");
  //   while (1);
  // }
  Serial.println("Sensor AS5600 terdeteksi. Putar poros motor untuk melihat perubahan nilai.");
    // (0-4095)
  uint16_t startPos = as5600.rawAngle();
  as5600.setZPosition(startPos);

  Serial.println("\nMembuat Access Point...");
  Serial.print("Nama (SSID): ");
  Serial.println(ssid);
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("Alamat IP AP: ");
  Serial.println(myIP); 

  ws.onEvent(onEvent);
  server.addHandler(&ws);

  // server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
  //   // coba kirim string
  //   String json = "{\"raw\":" + String(last_raw_angle) + 
  //                 ", \"deg\":" + String(last_angle_deg, 2) + 
  //                 ", \"message\":\"" + lastSerialMessage + "\"}"; 
  //   request->send(200, "application/json", json);
  // });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", htmlContent);
  });
    
  server.begin();
}

void loop() {

if (Serial.available() > 0) {
    String receivedText = Serial.readStringUntil('\n');
    receivedText.trim(); 

    if (receivedText.length() > 0) {
      lastSerialMessage = receivedText; 
      Serial.print("Pesan baru dikirim ke web: ");
      Serial.println(lastSerialMessage);
    }
  }

  float raw_angle = as5600.rawAngle();
  float angle_deg = (raw_angle/4095) * 360;

  String json = "{\"raw\":" + String(raw_angle) + 
                ", \"deg\":" + String(angle_deg, 2) + 
                ", \"message\":\"" + lastSerialMessage + "\"}";

  ws.textAll(json);
  //cleanup
  ws.cleanupClients();

  // Serial.print("Posisi Sudut Mentah: ");
  // Serial.println(currentPos);
  // Serial.print(" || Derajat: ");
  // Serial.print(currentPos/4095 * 360);

  delay(100); 
}