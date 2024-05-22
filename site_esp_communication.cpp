#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "iPhone Couto";        // Substitua com o nome da sua rede WiFi
const char* password = "coutinho22";   // Substitua com a senha da sua rede WiFi

WebServer server(80);

void handleRoot() {
  server.send(200, "text/html",
    "<!DOCTYPE html>"
    "<html>"
    "<head>"
    "<title>Coordenadas</title>"
    "<style>"
    "body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }"
    "h1 { color: #333; }"
    "form { margin-bottom: 20px; }"
    "input[type=text] { padding: 10px; margin: 10px; width: 100px; }"
    "button { padding: 10px 20px; background-color: #28a745; color: white; border: none; cursor: pointer; }"
    "button:hover { background-color: #218838; }"
    "#myChart { max-width: 600px; margin: 0 auto; }"
    "</style>"
    "<script src=\"https://cdn.jsdelivr.net/npm/chart.js\"></script>"
    "</head>"
    "<body>"
    "<h1>Coordenadas</h1>"
    "<form id=\"positionForm\">"
    "X: <input type=\"text\" id=\"x\" name=\"x\" placeholder=\"X\"><br>"
    "Y: <input type=\"text\" id=\"y\" name=\"y\" placeholder=\"Y\"><br>"
    "Theta: <input type=\"text\" id=\"theta\" name=\"theta\" placeholder=\"Theta\"><br>"
    "<button type=\"button\" onclick=\"sendPosition()\">Enviar</button>"
    "</form>"
    "<canvas id=\"myChart\" width=\"400\" height=\"400\"></canvas>"
    "<script>"
    "let positions = { x: [0], y: [0] };"
    "const ctx = document.getElementById('myChart').getContext('2d');"
    "let myChart = new Chart(ctx, {"
    "  type: 'line',"
    "  data: {"
    "    labels: positions.x.map((_, i) => i),"
    "    datasets: [{"
    "      label: 'trajeto',"
    "      data: positions.x.map((x, i) => ({ x: positions.x[i], y: positions.y[i] })),"
    "      borderColor: 'rgba(75, 192, 192, 1)',"
    "      borderWidth: 2,"
    "      fill: false,"
    "      tension: 0.1"
    "    }]"
    "  },"
    "  options: {"
    "    scales: {"
    "      x: {"
    "        type: 'linear',"
    "        position: 'bottom'"
    "      },"
    "      y: {"
    "        beginAtZero: true"
    "      }"
    "    }"
    "  }"
    "});"
    "function sendPosition() {"
    "  const x = parseFloat(document.getElementById('x').value);"
    "  const y = parseFloat(document.getElementById('y').value);"
    "  const theta = parseFloat(document.getElementById('theta').value);"
    "  fetch(`/command?x=${encodeURIComponent(x)}&y=${encodeURIComponent(y)}&theta=${encodeURIComponent(theta)}`)"
    "    .then(response => response.text())"
    "    .then(data => {"
    "      alert(data);"
    "      positions.x.push(x);"
    "      positions.y.push(y);"
    "      updateChart();"
    "    })"
    "    .catch(error => {"
    "      console.error('Erro:', error);"
    "    });"
    "}"
    "function updateChart() {"
    "  myChart.data.labels = positions.x.map((_, i) => i);"
    "  myChart.data.datasets[0].data = positions.x.map((x, i) => ({ x: positions.x[i], y: positions.y[i] }));"
    "  myChart.update();"
    "}"
    "</script>"
    "</body>"
    "</html>"
  );
}

void handleCommand() {
  if (server.hasArg("x") && server.hasArg("y") && server.hasArg("theta")) {
    String x = server.arg("x");
    String y = server.arg("y");
    String theta = server.arg("theta");
    // Processar os comandos aqui
    Serial.printf("Recebido - X: %s, Y: %s, Theta: %s\n", x.c_str(), y.c_str(), theta.c_str());
    server.send(200, "text/plain", "Comando recebido: X=" + x + " Y=" + y + " Theta=" + theta);
  } else {
    server.send(400, "text/plain", "Parâmetros insuficientes");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Serial.print("Conectando ao WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Conectado ao WiFi");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/command", handleCommand);

  server.begin();
  Serial.println("Servidor iniciado");
}

void loop() {
  server.handleClient();
}
