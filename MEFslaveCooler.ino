// Solucion taller estacion - codigo esclavo

/////////////////////////////////////////////////////////// WIFI

// Librerias requeridas
#include <SPI.h>
#include <WiFi101.h>

// Variables para la conexión WiFi
char ssid[] = "SSID-ISA";     // Network SSID (name)
char pass[] = "isa2023-1";     // Network password
int status = WL_IDLE_STATUS;  //Network status

void printWifiStatus() {
  // Muestra el nombre de la red WiFi
  Serial.print("[WIFI]\tConectado a SSID: ");
  Serial.println(WiFi.SSID());

  // Muestra la dirección IP
  IPAddress ip = WiFi.localIP();
  Serial.print("[WIFI]\tDirección IP: ");
  Serial.println(ip);

  // Muestra la intensidad de señal:
  long rssi = WiFi.RSSI();
  Serial.print("[WIFI]\tIntensidad de señal (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void WiFiInit() {
  // Le damos tiempo al shield WiFi de iniciar:
  delay(1000);

  // Verificamos la presencia del shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("[WIFI]\tShield not present");
    while (true); // No continuamos
  }

  String fv = WiFi.firmwareVersion();
  if (fv <= "1.1.0" )
    Serial.println("[WIFI]\tPor favor actualizar el firmware");

  // Intentando conectarse al WiFi:
  while (status != WL_CONNECTED) {
    Serial.print("[WIFI]\tIntento de connexión al SSID: ");
    Serial.println(ssid);
    // Conexión a una red por WPA/WPA2.
    // Cambiar la siguiente linea si usa un WIFI abierto o red WEP:
    status = WiFi.begin(ssid, pass);

    // Esperando 1 segundo para conexión:
    delay(1000);
  }
  // Estamos conectados, mostramos la información de la conexión:
  printWifiStatus();
}

/////////////////////////////////////////////////// SENDING DATA

#include <ArduinoHttpClient.h>

// Constantes
const unsigned long postingInterval = 2 * 1000; // Intervalo mínmo para el envio de datos 2*1000 ms
const unsigned int sensorCount = 3;             // Numero de datos a enviar

// Variables
char* host = "isa.requestcatcher.com"; // No incluir el https://
char* path = "/post";                  // Path
int   port = 80;                       // Puerto

WiFiClient wifi;
HttpClient client = HttpClient(wifi, host, port);

String response;
int statusCode = 0;

void POST_send(int sensorCount, char* sensorNames[], float values[]) {
  String contentType = "application/json";
  String postData = "";
  String request = String(path) + "?";
  for (int idx = 0; idx < sensorCount; idx++) {
    if (idx != 0) request += "&";
    request += sensorNames[idx];
    request += "=";
    request += values[idx];
  }
  client.post(request, contentType, postData);
  statusCode = client.responseStatusCode();
  response = client.responseBody();
  Serial.print("status-code: ");
  Serial.println(statusCode);
}

char* nameArray[] = {"temperature", "humidity", "battery"}; // Nombres de las variables en la nube
float sensorValues[sensorCount];        // Vector de valores
unsigned long lastConnectionTime = 0;   // Marca temporal para el ultimo envio de datos
float T = 0, H = 0, B = 0; //Var Environment Temperature (t) as float

void POST() {
  sensorValues[0] = T;
  sensorValues[1] = H;
  sensorValues[2] = B;
  if ((millis() - lastConnectionTime) > postingInterval)
  {
    digitalWrite(2,HIGH);
    Serial.print("[SEND]\tEnviando datos > ");
    POST_send(sensorCount, nameArray, sensorValues);  // REST call
    lastConnectionTime = millis();  // Actualizamos la marca temporal
    digitalWrite(2,LOW);
  }
}

///////////////////////////////////////////////////////// SENSOR

#include "DHT.h"

#define DHTTYPE DHT11 //Use DHT11 sensor variant
#define SBAT 0  //Battery sensor connected to Arduino analog pin 0
#define DHTPIN 9  //DHT sensor connected to Arduino digital pin 2

const unsigned long tDHTmeas = 200; //Time to measure DHT11 (200 ms)

DHT dht(DHTPIN, DHTTYPE); //DHT object var
unsigned long tiniDHT = 0;  //Timing for DHT11 measurements

void readDHT() {  //Function that reads DHT sensor and also battery level
  if ((millis() - tiniDHT) > tDHTmeas) {
    B = analogRead(SBAT) * 5.0 / 1023.0; //Acquire current battery level voltage
    Serial.print("[DATA]\tMedidas : {Battery Voltage: ");
    Serial.print(B);
    T = dht.readTemperature();
    Serial.print(", Temperature: ");
    Serial.print(T);
    H = dht.readHumidity();
    Serial.print(", Humidity: ");
    Serial.print(H);
    Serial.println(" }");
    tiniDHT = millis(); //Reset the timing for measuring DHT11
  }
}

/////////////////////////////////////////////////////// ACTUADOR

#define COOLER 8  // Ventilador en el pin 8
#define LR   6    // LED Rojo   en el pin 6
#define LG 3      // LED Verde  en el pin 3
#define LB  2     // LED Azul   en el pin 2 

void verificarActuadores() {
  pinMode(COOLER, OUTPUT);
  digitalWrite(COOLER, HIGH);
  delay(1000);
  digitalWrite(COOLER, LOW);
  pinMode(LR, OUTPUT);
  digitalWrite(LR, HIGH);
  delay(1000);
  digitalWrite(LR, LOW);
  pinMode(LG, OUTPUT);
  digitalWrite(LG, HIGH);
  delay(1000);
  digitalWrite(LG, LOW);
  pinMode(LB, OUTPUT);
  digitalWrite(LB, HIGH);
  delay(1000);
  digitalWrite(LB, LOW);
}

/////////////////////////////////////////////////// MEF Parpadeo

#define ROFF 0
#define RON  1

const unsigned long p = 500;

int eParpadeo = ROFF;
bool P = 0;
unsigned long tr = 0;

void MEF_Parpadeo() {
  switch (eParpadeo) {
    case ROFF:
      digitalWrite(LR, LOW);
      if ( (millis() - tr > p) && P) {
        Serial.println("[MEF]\tParpadeo ON");
        eParpadeo = RON;
        tr = millis();
      }
      break;
    case RON:
      digitalWrite(LR, HIGH);
      if ( (millis() - tr > p) || !P) {
        Serial.println("[MEF]\tParpadeo OFF");
        eParpadeo = ROFF;
        tr = millis();
      }
      break;
  }
}

//////////////////////////////////////////////////// MEF Sistema

#define ESLEEP 0
#define EWARN  1

int eSistema = ESLEEP;
char msg = '\0';

char readSerial() {
  char charSerial = '\0';
  if (Serial.available() > 0) { //Check if there are bytes sent from PC
    charSerial = Serial.read(); //Store in charSerial the incoming char
    Serial.flush(); //Clean Serial port buffer
  }
  if (Serial1.available() > 0) { //Check if there are bytes sent from XBee
    charSerial = Serial1.read(); //Store in readCommXBee the incoming char
    Serial1.flush(); //Clean Serial1 port buffer
  }
  return charSerial;
}

void MEF_Sistema() {
  msg = readSerial();
  switch (eSistema) {
    case ESLEEP:
      P = 0;
      if (msg == 'W') {
        Serial.println("[MEF]\tSistema en Alerta");
        eSistema = EWARN;
      }
      break;
    case EWARN:
      P = 1;
      readDHT();
      POST();
      if (msg == 'O') {
        Serial.println("[MEF]\tSistema en Espera");
        eSistema = ESLEEP;
      }
      break;
  }
}

/////////////////////////////////////////////////// MEF COOLER

#define COOLERON 0
#define COOLEROFF  1


int eCooler = COOLEROFF;

void MEF_Cooler() {
  switch (eCooler) {
    case COOLEROFF:
      digitalWrite(COOLER, LOW);
      if (P) {
        Serial.println("[MEF]\tCooler ON");
        eCooler = COOLERON;
      }
      break;
    case COOLERON:
      digitalWrite(COOLER, HIGH);
      if (!P) {
        Serial.println("[MEF]\tCooler OFF");
        eCooler = COOLEROFF;
      }
    
      break;
  }
}



//////////////////////////////////////////////////////// ARDUINO

void setup() {

  // Comunicación
  Serial.begin(9600);
  verificarActuadores();
  WiFiInit(); // Iniciando la conexión WIFI

}

void loop() {
  MEF_Sistema();
  MEF_Parpadeo();
  MEF_Cooler();
}
