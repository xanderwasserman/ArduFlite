#ifndef ARDUFLITEMQTTTELEMETRY_H
#define ARDUFLITEMQTTTELEMETRY_H

#include <PubSubClient.h>

#include "src/telemetry/TelemetryData.h"

extern SemaphoreHandle_t telemetryMutex;
extern TelemetryData telemetryData;

class ArduFliteMqttTelemetry {
public:
    ArduFliteMqttTelemetry(PubSubClient& client, float frequencyHz);
    void begin();

private:
    static void telemetryTask(void* pvParameters);
    void publish(const TelemetryData& data);

    PubSubClient& mqttClient;
    float intervalMs;
};

#endif
