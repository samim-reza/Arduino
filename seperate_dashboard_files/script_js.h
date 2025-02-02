#ifndef SCRIPT_JS_H
#define SCRIPT_JS_H

const char SCRIPT_JS[] PROGMEM = R"rawliteral(
function toggleLED() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("ledStatus").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/toggle", true);
  xhttp.send();
}
)rawliteral";

#endif