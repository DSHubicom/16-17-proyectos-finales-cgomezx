#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define LED_GPIO 2 //LED interno
#define TEMPERATURA_GPIO A0 //Temperatura (temperature sensor)
#define RELAY_GPIO 5 //Relé conectado al brasero 

/************************* Parámetros de conexión red WiFi WiFi *********************************/
#define WLAN_SSID       "TELEPORTE.es 0D8_927750750"
#define WLAN_PASS       "RqnMiaPe@#76:01+"

/************************* Parámatros de conexión Servidor MQTT (e.g., Adafruit.io) *********************************/
#define AIO_SERVER      "192.168.2.107"
#define AIO_SERVERPORT  1883                   //8883 para SSL

/************ Variables para cliente WiFi y cliente MQTT ******************/
// Crea un cliente ESP8266
WiFiClient client; //usar: WiFiClientSecure client; para cliente SSL

// Crea el cliente MQTT
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT);

/****************************** Canales (Feeds) ***************************************/
// La ruta cambia según el servidor MQTT usado, para Adafruit.io: <username>/feeds/<feedname>
// Nos subscribiremos al canal "canalonoff"
Adafruit_MQTT_Subscribe canalbrasero = Adafruit_MQTT_Subscribe(&mqtt, "salon/brasero");
Adafruit_MQTT_Subscribe canalhoraactual = Adafruit_MQTT_Subscribe(&mqtt, "fecha/hora");
Adafruit_MQTT_Subscribe canalminutoactual = Adafruit_MQTT_Subscribe(&mqtt, "fecha/minuto");
Adafruit_MQTT_Subscribe canaldiaactual = Adafruit_MQTT_Subscribe(&mqtt, "fecha/dia_de_la_semana");

// Publicaremos en un canal el valor del sensor de temperatura, el canal se llamara "temperatura"
Adafruit_MQTT_Publish canaltemperatura = Adafruit_MQTT_Publish(&mqtt, "salon/temperatura");
Adafruit_MQTT_Publish canalbraseropub = Adafruit_MQTT_Publish(&mqtt, "salon/brasero");

// Variable para el calculo de la temperatura para su envío
int a;
float temperatura;
int B=3975;                  //B value of the thermistor
float resistance;

int releencendido;

//Variables de las horas importantes para el encendido/apagado del brasero
int horalevantarsefinde=11,minutolevantarsefinde=00,horaacostarsefinde=23,minutoacostarsefinde=59;
int horalevantarselaboral=7,minutolevantarselaboral=30,horaacostarselaboral=22,minutoacostarselaboral=45;
int horamarcharsetrabajo=8,minutomarcharsetrabajo=15,horallegartrabajo=19,minutollegartrabajo=30;

//Variable para el control de errores de envío al publicar
unsigned int errorPublicacion;

/***************************************************************/
void setup() {
  Serial.begin(115200);

  digitalWrite(RELAY_GPIO, LOW);
  //Configuramos los pines como entradas o salidas
  pinMode(TEMPERATURA_GPIO, INPUT);
  pinMode(LED_GPIO, OUTPUT);
  pinMode(RELAY_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, HIGH); //encendemos el LED interno al comienzo

  //Conectamos a la WiFi
  WIFI_connect();

  // Nos subscribimos a los canales MQTT que nos interesan
  mqtt.subscribe(&canalbrasero);
  mqtt.subscribe(&canalhoraactual);
  mqtt.subscribe(&canalminutoactual);
  mqtt.subscribe(&canaldiaactual);
}


void loop() {
  // MQTT_connect() sirve tanto para la primera conexión como para en caso de producirse una desconexión, volver a conectar (por eso se llama en cada iteracción del loop)
  MQTT_connect();


  //ESP8266 soporta comunicación bidireccional, pero no simultaneamente: si está enviando no puede recibir y si está recibiendo no puede enviar
  //así que en este caso primero nos ponderemos a escuchar (los canales a los que estemos subscritos que nos interesen) y después a publicar (en los canales correspondientes)

  //****** SUBSCRIPCION
  //Vemos si hay mensajes por leer en los canales que nos interesan, como pueden ser varios, y en este caso sólo nos interesa el canalonoff,
  //filtramos los mensajes con "if (subscription == &canalonoff)"
  //En caso de publicarse algo en el tiempo que estamos pendientes de las subscripciones (50000ms en este caso) lo leeríamos.
  //Como ejemplo, si el mensaje recibido es ON, encendemos el LED, en caso contrario lo apagamos
  Adafruit_MQTT_Subscribe *subscription;
  uint16_t diaactual=0, horaactual=0, minutoactual=0;
  int estadorele=0;
  int salirbucle=0;
  int dialaboral=0;
  while ((subscription = mqtt.readSubscription(10000)) && salirbucle<3) {
    if (subscription == &canaldiaactual){
      diaactual = atoi((char *)canaldiaactual.lastread);
      if (diaactual<6){
        dialaboral=1;
      }
      salirbucle++;
    }
    if (subscription == &canalhoraactual) {
      horaactual = atoi((char *)canalhoraactual.lastread);
      salirbucle++;
    }
    if (subscription == &canalminutoactual) {
      minutoactual = atoi((char *)canalminutoactual.lastread);
      salirbucle++;
    }
  }

  Serial.print("Hora = ");
  Serial.print(horaactual);
  Serial.print(":");
  Serial.println(minutoactual);
  

  //****** PUBLICACION
  //Calculo el valor a enviar (pasándolo a %)
    a=analogRead(TEMPERATURA_GPIO);
    resistance=(float)(1023-a)*10000/a; //get the resistance of the sensor;
    temperatura=1/(log(resistance/10000)/B+1/298.15)-273.15;//convert to temperature via datasheet&nbsp;
    Serial.print("La temperatura es ");
    Serial.println(temperatura);

  
  if (dialaboral==1){ //si es día laboral
    Serial.println("Día de diario");
    if(hora_a_minutos(horaactual,minutoactual)>=(hora_a_minutos(horalevantarselaboral,minutolevantarselaboral)-15) && hora_a_minutos(horaactual,minutoactual)<hora_a_minutos(horamarcharsetrabajo,minutomarcharsetrabajo) ){
      digitalWrite(RELAY_GPIO, HIGH);
      estadorele=1;
    }else if (hora_a_minutos(horaactual,minutoactual)>=(hora_a_minutos(horallegartrabajo,minutollegartrabajo)-20) && hora_a_minutos(horaactual,minutoactual)<hora_a_minutos(horaacostarselaboral,minutoacostarselaboral)){
      digitalWrite(RELAY_GPIO, HIGH); 
      estadorele=1;
    }else{
      digitalWrite(RELAY_GPIO, LOW);
      estadorele=0;
    }
  }else{ //si es fin de semana
    Serial.println("Fin de semana");
    if(hora_a_minutos(horaactual,minutoactual)>=hora_a_minutos(horalevantarsefinde,minutolevantarsefinde) && hora_a_minutos(horaactual,minutoactual)<hora_a_minutos(horaacostarsefinde,minutoacostarsefinde) ){
      if (temperatura<22){
        Serial.println("Menor de 22");
        digitalWrite(RELAY_GPIO, HIGH);
        estadorele=1;
        releencendido=HIGH;
        Serial.println(releencendido);
      }else if (temperatura>26){
        Serial.println("Mayor de 26");
        digitalWrite(RELAY_GPIO, LOW);
        estadorele=0;
        releencendido=LOW;
        Serial.println(releencendido);
      }else{
        Serial.println("Entre 22 y 26");
        Serial.println(releencendido);
        //digitalWrite(RELAY_GPIO, releencendido);
        if(releencendido==0){
          digitalWrite(RELAY_GPIO, LOW);
          estadorele=0;
        }else if (releencendido==1){
          digitalWrite(RELAY_GPIO, HIGH);
          estadorele=1;
        }
      }
    } else{
      digitalWrite(RELAY_GPIO, LOW);
    }
  }
  
  //Si no hay error de publicación, la función devuelve 0, sino el código de error correspondiente (sólo interesante para debug)
  if (! (errorPublicacion = canaltemperatura.publish(temperatura))) {
    Serial.print("Error de publicación (error ");
    Serial.print(errorPublicacion);
    Serial.println(")");
  }

  
  if (estadorele==1){
    if (! (errorPublicacion = canalbraseropub.publish("ON"))) {
      Serial.print("Error de publicación (error ");
      Serial.print(errorPublicacion);
      Serial.println(")");
    }
  }else {
    if (! (errorPublicacion = canalbraseropub.publish("OFF"))) {
      Serial.print("Error de publicación (error ");
      Serial.print(errorPublicacion);
      Serial.println(")");
    }    
  }
  

}

int hora_a_minutos (int hora, int minuto){
  return (hora*60)+minuto;
}

// Funcción para conectar inicialmente y reconectar cuando se haya perdido la conexión al servidor MQTT
void MQTT_connect() {
  int8_t ret;

  if (!mqtt.connected()) {

    Serial.println("Conectando al servidor MQTT... ");

    uint8_t intentos = 3;
    while ((ret = mqtt.connect()) != 0) { // connect devuelve 0 cuando se ha conectado correctamente y el código de error correspondiente en caso contrario
      Serial.println(mqtt.connectErrorString(ret)); // (sólo interesante para debug)
      Serial.println("Reintentando dentro de 3 segundos...");
      mqtt.disconnect();
      delay(3000);  // esperar 3 segundos
      if (! intentos--) { //decrementamos el número de intentos hasta que sea cero
        while (1);  // El ESP8266 no soporta los while(1) sin código dentro, se resetea automáticamente, así que estamos forzando un reset
          //Si no quisieramos que se resetease, dentro del while(1) habría que incluir al menos una instrucción, por ejemplo delay(1); o yield();
      }
    }
    Serial.println("MQTT conectado");
  }
}

void WIFI_connect() {
  // Poner el módulo WiFi en modo station (este modo permite de serie el "light sleep" para consumir menos
  // y desconectar de cualquier red a la que pudiese estar previamente conectado
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(2000);

  // Conectamos a la WiFi
  Serial.println("Conectando a la red WiFi");

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) { //Nos quedamos esperando hasta que conecte
    delay(500);
    Serial.print(".");
  }
  Serial.println(" conectado.");
}


/***************************************************
  Adafruit MQTT Library ESP8266 Example
  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino
  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
  Modificaciones de programación estructurada, traducción y añadidos por Marino Linaje
 ****************************************************/
