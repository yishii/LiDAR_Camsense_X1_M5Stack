#include <M5Stack.h>

#define LIDARSerial Serial1

typedef struct
{
  uint16_t x;
  uint16_t y;
} Point_t;

typedef enum
{
  STATE_WAIT_HEADER = 0,
  STATE_READ_HEADER,
  STATE_READ_PAYLOAD,
  STATE_READ_DONE
} State_t;

typedef struct {
  uint8_t header0;
  uint8_t header1;
  uint8_t header2;
  uint8_t header3;
  uint16_t rotation_speed;
  uint16_t angle_begin;
  uint16_t distance_0;
  uint8_t reserved_0;
  uint16_t distance_1;
  uint8_t reserved_1;
  uint16_t distance_2;
  uint8_t reserved_2;
  uint16_t distance_3;
  uint8_t reserved_3;
  uint16_t distance_4;
  uint8_t reserved_4;
  uint16_t distance_5;
  uint8_t reserved_5;
  uint16_t distance_6;
  uint8_t reserved_6;
  uint16_t distance_7;
  uint8_t reserved_7;
  uint16_t angle_end;
  uint16_t crc;
} __attribute__((packed)) LidarPacket_t;

const uint8_t header[] = { 0x55, 0xaa, 0x03, 0x08 };

void setup(void) {
  M5.begin();
  // M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.printf("Start\n");
  Serial.begin(115200);

  LIDARSerial.begin(115200, SERIAL_8N1, 21, 22);

  delay(10);

  Serial.printf("Start\n");
}

uint16_t convertDegree(uint16_t input)
{
  return (input - 40960) / 64;
}

uint16_t convertSpeed(uint16_t input)
{
  return input / 64;
}

void remapDegrees(uint16_t minAngle, uint16_t maxAngle, uint16_t *map)
{
  int16_t delta = maxAngle - minAngle;
  if (maxAngle < minAngle) {
    delta += 360;
  }

  if ((map == NULL) || (delta < 0)) {
    return;
  }
  for (int32_t cnt = 0; cnt < 8; cnt++)
  {
    map[cnt] = minAngle + (delta * cnt / 7);
    if (map[cnt] >= 360) {
      map[cnt] -= 360;
    }
  }
}

void plotDistanceMap(uint16_t* degrees, uint16_t* distances)
{
  int32_t i;
  uint32_t x, y;
  static Point_t pointCloud[360];      // 360度分の点群

  for (i = 0; i < 8; i++) {
    M5.Lcd.drawPixel(pointCloud[degrees[i]].x, pointCloud[degrees[i]].y, BLACK);
    if (distances[i] < 1000) {
      x = cos((1.f * PI * degrees[i]) / 180) * (distances[i] / 10) + 160;
      y = sin((1.f * PI * degrees[i]) / 180) * (distances[i] / 10) + 120;

      M5.Lcd.drawPixel(x, y, WHITE);

      pointCloud[degrees[i]].x = x;
      pointCloud[degrees[i]].y = y;

    }
  }
}

void loop() {
  static State_t state;
  static uint32_t counter;
  static uint8_t payload[64];

  if (LIDARSerial.available())
  {
    uint8_t data = LIDARSerial.read();
    switch (state)
    {
      case STATE_WAIT_HEADER:
        if (data == header[0]) {
          counter++;
          payload[0] = data;
          state = STATE_READ_HEADER;
        } else {
          //printf("?? (%02X) Please do LiDAR power cycle\n", data);
          LIDARSerial.flush();
        }
        break;
      case STATE_READ_HEADER:
        if (data == header[counter]) {
          payload[counter] = data;
          counter++;
          if (counter == sizeof(header)) {
            state = STATE_READ_PAYLOAD;
          }
        } else {
          counter = 0;
          state = STATE_WAIT_HEADER;
        }
        break;
      case STATE_READ_PAYLOAD:
        payload[counter] = data;
        counter++;
        if (counter == sizeof(LidarPacket_t)) {
          state = STATE_READ_DONE;
        }
        break;
      case STATE_READ_DONE:
        LidarPacket_t* packet = (LidarPacket_t*)payload;
        {
          uint16_t degree_begin;
          uint16_t degree_end;
          degree_begin = convertDegree(packet->angle_begin);
          degree_end = convertDegree(packet->angle_end);
          if ((degree_begin < 360) && (degree_end < 360)) {
            printf("%3drpm %5d - %5d\n", convertSpeed(packet->rotation_speed), convertDegree(packet->angle_begin), convertDegree(packet->angle_end));
            uint16_t map[8];
            uint16_t distances[8];
            remapDegrees(degree_begin, degree_end, map);
            distances[0] = packet->distance_0;
            distances[1] = packet->distance_1;
            distances[2] = packet->distance_2;
            distances[3] = packet->distance_3;
            distances[4] = packet->distance_4;
            distances[5] = packet->distance_5;
            distances[6] = packet->distance_6;
            distances[7] = packet->distance_7;
            plotDistanceMap(map, distances);
          }
        }
        // M5.Lcd.setCursor(0, 0);
        // M5.Lcd.printf("Speed : %d rpm  \n", convertSpeed(packet->rotation_speed));        state = STATE_WAIT_HEADER;
        counter = 0;
        state = STATE_WAIT_HEADER;
        break;
    }
  }
}
