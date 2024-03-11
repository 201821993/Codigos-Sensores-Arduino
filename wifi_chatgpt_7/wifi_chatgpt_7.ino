#include <ArduinoHttpClient.h>
#include <WiFi.h>

// Configuración de la red WiFi
char ssid[] = "netis_70F8D0";
char password[] = "carlos4523";

// Configuración del servidor
const char* serverAddress = "192.168.166.103";
const int serverPort = 3001;  // Puerto para la conexión HTTP

void setup() {
  Serial.begin(115200);

  // Conéctate a la red Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a la red WiFi...");
  }

  Serial.println("Conectado a la red WiFi");
}

void loop() {
  // Datos que deseas enviar
  String postData = "parametro1=valor1&parametro2=valor2";

  // Crea una instancia de HttpClient
  WiFiClient wifi;
  HttpClient http = HttpClient(wifi, serverAddress, serverPort);

  // Construye la URL completa
  String url = "http://192.168.166.103/api/showData";  // No es necesario especificar la dirección completa si ya la especificaste en la configuración del cliente

  // Realiza la solicitud POST
  http.post(url, "application/x-www-form-urlencoded", postData);

  // Obtiene y muestra la respuesta del servidor
  String response = http.responseBody();
  Serial.print("Respuesta del servidor: ");
  Serial.println(response);

  // Cierra la conexión
  http.stop();

  delay(5000);  // Espera 5 segundos antes de la siguiente solicitud
}
