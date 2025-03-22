#include <Arduino.h>
#include <InfluxDbClient.h>
#include <WiFi.h>

#define SENSOR 34
#define SERVO 26

const char* SSID             = "Home";
const char* PASSWORD         = "2aw6d13z";
const char* INFLUXDB_HOST    = "http://10.0.0.177:8086";
const char* INFLUXDB_DB_NAME = "pid";

void send_to_influxdb(float distance, float error, float setpoint, float p,
                      float i, float d, float angle);

InfluxDBClient client(INFLUXDB_HOST, INFLUXDB_DB_NAME);

Point sensor("measurement");

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\nConnecting to: " + String(SSID));
    WiFi.begin(SSID, PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect!");
    }

    client.setConnectionParamsV1(INFLUXDB_HOST, INFLUXDB_DB_NAME, "", "");

    if (client.validateConnection()) {
        Serial.print("Conectado ao InfluxDB: ");
        Serial.println(client.getServerUrl());
    } else {
        Serial.print("Falha na conexão ao InfluxDB: ");
        Serial.println(client.getLastErrorMessage());
    }
}

void loop() {
    send_to_influxdb(random(10, 80), random(-30, 30), 20, random(30, 35),
                     random(-1, 1), random(-100, 100), random(90, 180));
    delay(2000);
}

void send_to_influxdb(float distance, float error, float setpoint, float p,
                      float i, float d, float angle) {
    sensor.clearFields();

    sensor.addField("distance", distance);
    sensor.addField("error", error);
    sensor.addField("setpoint", setpoint);
    sensor.addField("p", p);
    sensor.addField("i", i);
    sensor.addField("d", d);
    sensor.addField("angle", angle);

    if (client.writePoint(sensor)) {
        Serial.println("Dados enviados com sucesso!");
    } else {
        Serial.print("Falha no envio de dados: ");
        Serial.println(client.getLastErrorMessage());
    }
}

float get_distance(int n) {
    long sum = 0;
    for (int i = 0; i < n; i++) {
        sum = sum + analogRead(SENSOR);
    }
    float adc_value = sum / n;
    float adc       = adc_value * (3.3 / 4095.0);
    return 27.728 * pow(adc, -1.2045);
}