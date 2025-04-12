```bash
docker run -d \
  --rm \
  --name mosquitto \
  -p 1883:1883 \
  -v $(pwd)/mosquitto.conf:/mosquitto/config/mosquitto.conf \
  eclipse-mosquitto:latest

```

```bash
docker run -d \
  --restart=unless-stopped \
  --name mosquitto \
  -p 1883:1883 \
  -v $(pwd)/mosquitto.conf:/mosquitto/config/mosquitto.conf \
  eclipse-mosquitto:latest


```