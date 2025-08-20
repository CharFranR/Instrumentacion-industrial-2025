#include <Arduino.h>
#include <ESP32Servo.h>
#include <Ultrasonic.h>

// ====== Pines ======
const int pinRele = 2;
const int pinEcho = 5;
const int pinTrig = 18;
const int lm35Pin = 34;
const int potPin = 35;
const int pinVentilador = 27;
const int bombaPin = 12;
const int pinServo = 13;
const int pinCaudal = 14;
int entero   = 42;
bool bool1   = true;
bool bool2   = false;


// ====== Pines NUEVOS ======
const int ledRojo = 25;
const int ledVerde = 33;
const int ledAzul = 32;
const int ledAmarillo = 26;

const int btnVerde = 23;    // Marcha
const int btnRojo = 17;     // Paro
const int btnAzul = 16;     // Servo abrir
const int btnAmarillo = 19; // Servo cerrar

// ====== Variables Globales ======
float nivel = 12;
float temperatura = 0;
float caudal_Lmin = 0;
float volumen_L = 0;

int setTemp=40;
int setNivel=30;

int estadoVentilador = 0;
bool condicionTempEstable = false;
unsigned long tiempoInicioTemp = 0;

bool releEstado = false;
bool bombaEstado = false;
bool servoEstado = true;
volatile uint32_t pulseCount = 0;

bool sistemaActivo = false; // Sistema en marcha/parado

float calibrationFactor = 4.5;

Ultrasonic ultrasonic(pinTrig, pinEcho);
Servo servoMotor;

// ====== Anti-rebote ======
unsigned long lastDebounce[4] = {0, 0, 0, 0};
const unsigned long debounceDelay = 50;

// ====== Funciones ======

void tareaLeerSerial(void *pvParameters) {
  while (true) {
    leerComandos();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void leerComandos() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // elimina espacios y saltos de línea

    int comaIndex = input.indexOf(',');
    if (comaIndex > 0) {
      String tempStr = input.substring(0, comaIndex);
      String nivelStr = input.substring(comaIndex + 1);

      setTemp = tempStr.toInt();
      setNivel = nivelStr.toInt();
    }
  }
}
void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

void Rele(bool estado) {
  digitalWrite(pinRele, estado ? HIGH : LOW);
  releEstado = estado;
}

float leerTemperatura() {
  int adcValue = analogRead(lm35Pin);
  float voltage = adcValue * (5.0 / 4095.0);
  return voltage * 100.0;
}

void Bomba(bool estado) {
  if (estado) {
    int potValue = analogRead(potPin);
    int freq = map(potValue, 0, 4095, 10, 1000);
    ledcChangeFrequency(bombaPin, freq, 8);

    int dutyCycle;
    if (freq <= 512) {
      dutyCycle = freq * 255 / 512;
    } else {
      dutyCycle = 128;
    }

    ledcWrite(bombaPin, dutyCycle);
    bombaEstado = true;
  } else {
    ledcWrite(bombaPin, 0);
    bombaEstado = false;
  }
}

void Ventilador() {
  digitalWrite(pinVentilador, estadoVentilador ? HIGH : LOW);
}

void moverServoPorEstado(bool abierto) {
  servoEstado = abierto;
  servoMotor.write(abierto ? 0 : 180);
}

// ====== Lectura de botones ======
bool leerBoton(int pin, int index) {
  bool estado = false;
  if (digitalRead(pin) == LOW && (millis() - lastDebounce[index]) > debounceDelay) {
    lastDebounce[index] = millis();
    estado = true;
  }
  return estado;
}

void leerBotones() {
  // Marcha
  if (leerBoton(btnVerde, 0)) {
    sistemaActivo = true;
    digitalWrite(ledRojo, LOW); // Apagar rojo al iniciar
  }

  // Paro
  if (leerBoton(btnRojo, 1)) {
    sistemaActivo = false;
    Rele(false);
    Bomba(false);
    estadoVentilador = 0;
    Ventilador();
    moverServoPorEstado(false);
    digitalWrite(ledRojo, HIGH); // Encender rojo en paro
  }

  // Control servo manual
  if (leerBoton(btnAzul, 2) && sistemaActivo) {
    moverServoPorEstado(true);
  }

  if (leerBoton(btnAmarillo, 3) && sistemaActivo) {
    moverServoPorEstado(false);
  }
}

// ====== Control LEDs ======
void actualizarLeds() {
  digitalWrite(ledVerde, sistemaActivo ? HIGH : LOW); // Verde siempre en marcha
  digitalWrite(ledAmarillo, releEstado ? HIGH : LOW);
  digitalWrite(ledAzul, bombaEstado ? HIGH : LOW);
}

// ====== Tareas ======
void tareaServoMovimiento(void *pvParameters) {
  bool ultimoEstado = servoEstado;
  while (true) {
    if (servoEstado != ultimoEstado) {
      servoMotor.write(servoEstado ? 180 : 0);
      ultimoEstado = servoEstado;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void tareaUltrasonico(void *pvParameters) {
  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);
  while (true) {
    if (!sistemaActivo) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    digitalWrite(pinTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(pinTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrig, LOW);
    long duration = pulseIn(pinEcho, HIGH, 30000);
    if (duration == 0) nivel = -1;
    else nivel = (duration / 2.0) * 0.0343;
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void tareaTemperatura(void *pvParameters) {
  while (true) {
    if (sistemaActivo) {
      temperatura = leerTemperatura();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void tareaCaudalimetro(void *pvParameters) {
  pinMode(pinCaudal, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinCaudal), pulseCounter, RISING);
  unsigned long oldTime = millis();
  while (true) {
    if (!sistemaActivo) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    if ((millis() - oldTime) >= 1000) {
      detachInterrupt(digitalPinToInterrupt(pinCaudal));
      caudal_Lmin = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
      volumen_L += (caudal_Lmin / 60.0);
      pulseCount = 0;
      oldTime = millis();
      attachInterrupt(digitalPinToInterrupt(pinCaudal), pulseCounter, RISING);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void tareaLecturaSerial(void *pvParameters) {
  String entrada = "";
  
  while (true) {
    while (Serial.available() > 0) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') { 
        // Procesar la cadena solo si no está vacía
        if (entrada.length() > 0) {
          int comaIndex = entrada.indexOf(',');
          if (comaIndex > 0) {
            String tempStr = entrada.substring(0, comaIndex);
            String nivelStr = entrada.substring(comaIndex + 1);

            int nuevaTemp = tempStr.toInt();
            int nuevoNivel = nivelStr.toInt();

            // Solo actualizamos si son valores positivos
            if (nuevaTemp > 0 && nuevoNivel > 0) {
              setTemp = nuevaTemp;
              setNivel = nuevoNivel;

              Serial.print("Nuevo setpoint de temperatura: ");
              Serial.println(setTemp);
              Serial.print("Nuevo setpoint de nivel: ");
              Serial.println(setNivel);
            } else {
              Serial.println("⚠ Valores inválidos recibidos");
            }
          } else {
            Serial.println("⚠ Formato inválido, use: temp,nivel");
          }
          entrada = ""; // Limpiar buffer
        }
      } 
      else {
        entrada += c; // Acumular datos
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void tareaControl(void *pvParameters) {
  while (true) {
    leerBotones();
    if (!sistemaActivo) {
      actualizarLeds();
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    if (nivel >= 45) {
      Bomba(true);
      Rele(false);
    } else if (nivel < setNivel) {
      Bomba(false);
      Rele(true);
    }

    if (temperatura > setTemp) {
      Rele(false);
    }
    if (temperatura > (setTemp+2)) {
      estadoVentilador = 1;
    } else if (temperatura < setTemp) {
      estadoVentilador = 0;
    }

    if (temperatura >= (setTemp-2) && temperatura <= (setTemp+2)) {
      if (!condicionTempEstable) {
        condicionTempEstable = true;
        tiempoInicioTemp = millis();
      }
      if (millis() - tiempoInicioTemp >= 120000) {
        moverServoPorEstado(true);
      }
    } else {
      condicionTempEstable = false;
      tiempoInicioTemp = 0;
    }

    if (nivel >= setNivel) {
      moverServoPorEstado(false);
    }

    Ventilador();
    actualizarLeds();
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}



void tareaImpresion(void *pvParameters) {
  while (true) {
    float a=(((50-nivel)/50)*100);
    Serial.print(temperatura, 2); Serial.print(", ");
    Serial.print(caudal_Lmin, 1); Serial.print(", ");
    Serial.print(nivel, 1); Serial.print(", ");
    Serial.print(entero);    Serial.print(", ");
    Serial.print(releEstado ? "true" : "false"); Serial.print(", ");
    Serial.println(bombaEstado ? "true" : "false");
    
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}



// ====== Setup ======
void setup() {
  Serial.begin(115200);

  pinMode(pinRele, OUTPUT);
  pinMode(pinVentilador, OUTPUT);
  pinMode(potPin, INPUT);
  pinMode(bombaPin, OUTPUT);
  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);

  // Pines nuevos
  pinMode(ledRojo, OUTPUT);
  pinMode(ledVerde, OUTPUT);
  pinMode(ledAzul, OUTPUT);
  pinMode(ledAmarillo, OUTPUT);
  pinMode(btnVerde, INPUT_PULLUP);
  pinMode(btnRojo, INPUT_PULLUP);
  pinMode(btnAzul, INPUT_PULLUP);
  pinMode(btnAmarillo, INPUT_PULLUP);

  digitalWrite(ledRojo, LOW);
  digitalWrite(ledVerde, LOW);
  digitalWrite(ledAzul, LOW);
  digitalWrite(ledAmarillo, LOW);

  servoMotor.attach(pinServo);
  servoMotor.write(180);

  if (!ledcAttach(bombaPin, 500, 8)) {
    Serial.println("Error al configurar PWM para bomba");
  }

  xTaskCreate(tareaServoMovimiento, "ServoMovimiento", 2048, NULL, 1, NULL);
  xTaskCreate(tareaUltrasonico, "Medir Ultrasonico", 2048, NULL, 1, NULL);
  xTaskCreate(tareaTemperatura, "Medir Temperatura", 2048, NULL, 1, NULL);
  xTaskCreate(tareaCaudalimetro, "Medir Caudal", 2048, NULL, 1, NULL);
  xTaskCreate(tareaLecturaSerial, "Lectura Serial", 2048, NULL, 1, NULL);
  xTaskCreate(tareaControl, "Control General", 4096, NULL, 1, NULL);
  xTaskCreate(tareaImpresion, "Impresion Estado", 2048, NULL, 1, NULL);
  xTaskCreate(tareaLeerSerial, "Leer Serial", 2048, NULL, 1, NULL);
}

void loop() {
  // FreeRTOS controla todo
}