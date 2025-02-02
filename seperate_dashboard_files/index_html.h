#ifndef INDEX_HTML_H
#define INDEX_HTML_H

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>ESP8266 Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    /* Styles will be inserted here */
  </style>
</head>
<body>
  <h1>ESP8266 Dashboard</h1>
  <p>Welcome to your ESP8266 Web Server</p>
  <button class="button" onclick="toggleLED()">Toggle LED</button>
  <p id="ledStatus">LED Status: OFF</p>

  <script>
    // JavaScript will be inserted here
  </script>
</body>
</html>
)rawliteral";

#endif