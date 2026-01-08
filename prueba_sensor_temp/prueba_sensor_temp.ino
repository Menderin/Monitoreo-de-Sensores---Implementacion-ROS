#include <OneWire.h>
#include <DallasTemperature.h>

// El pin donde conectaste el cable de datos del sensor
const int oneWireBus = 4; 

// Configurar una instancia oneWire para comunicarse con cualquier dispositivo OneWire
OneWire oneWire(oneWireBus);

// Pasar la referencia oneWire a la librería Dallas Temperature
DallasTemperature sensors(&oneWire);

void setup() {
  // Iniciar monitor serial para ver los resultados en la PC
  Serial.begin(115200);
  
  // Iniciar la librería del sensor
  sensors.begin();
  
  Serial.println("Esperando lecturas del sensor DS18B20...");
}

void loop() {
  // 1. Enviar el comando para que todos los sensores en el bus lean la temperatura
  // (Esto toma un poco de tiempo)
  // Serial.print("Solicitando temperaturas...");
  sensors.requestTemperatures(); 
  
  // 2. Obtener la temperatura en Celsius
  // Usamos el índice 0 porque solo tienes un sensor conectado.
  // Si tuvieras más en el mismo cable, usarías 1, 2, etc.
  float temperaturaC = sensors.getTempCByIndex(0);

  // 3. Verificar si la lectura es válida
  // El sensor devuelve -127.00 si hay un error de conexión o falta la resistencia
  if(temperaturaC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: No se pudo leer el sensor (Revisa cables y resistencia pull-up)");
  } else {
    Serial.print("Temperatura: ");
    Serial.print(temperaturaC);
    Serial.println(" ºC");
  }
  
  // Esperar 2 segundos antes de la siguiente lectura
  delay(5000);
}