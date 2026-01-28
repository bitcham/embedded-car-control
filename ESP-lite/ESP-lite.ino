#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <FS.h>

const char* ssid = "Titenet-IoT";         // Wi-Fi network name
const char* password = "7kDtaphg";        // Wi-Fi password

ESP8266WebServer server(80);              // Web server on port 80

void setup() {
  Serial.begin(115200);        // For debugging via USB
  Serial1.begin(115200);       // Serial1 for communication with Arduino Mega

  // Initialize SPIFFS
  if (!SPIFFS.begin()) {
    Serial.println("Error mounting SPIFFS");
    return;
  }

  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());

  // Setup static web files
  server.serveStatic("/", SPIFFS, "/index.html");
  server.serveStatic("/style.css", SPIFFS, "/style.css");
  server.serveStatic("/script.js", SPIFFS, "/script.js");
  server.serveStatic("/favicon.ico", SPIFFS, "/favicon.png");

  // Movement command URLs
  server.on("/forwards5", [](){ handleMove(5); });
  server.on("/forwards20", [](){ handleMove(20); });
  server.on("/backwards5", [](){ handleMove(-5); });
  server.on("/backwards20", [](){ handleMove(-20); });

  // Compass command URL
  server.on("/compass", handleCompass);

  // Handle 404 not found
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();  // Handle incoming requests
}

// Handle 404 not found URLs
void handleNotFound() {
  server.send(404, "text/plain", "404: Not Found");
}

// Handle movement commands
void handleMove(int distance) {
  String cmd = "dist:" + String(distance);
  Serial1.println(cmd);          // Send to Arduino Mega
  Serial.println(cmd);           // Debug output
  server.send(200, "text/plain", "OK");
}



// Handle compass commands
void handleCompass() {
  if (server.hasArg("value")) {
    String valueString = server.arg("value");
    String cmd = "deg:" + valueString;
    Serial1.println(cmd);        // Send to Arduino Mega
    Serial.println(cmd);         // Debug output
  }
  server.send(200, "text/plain", "OK");
}
