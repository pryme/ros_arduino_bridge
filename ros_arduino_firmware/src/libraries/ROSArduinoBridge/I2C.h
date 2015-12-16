#ifndef __I2C_H__
#define __I2C_H__

#define USE_I2C

#define I2C_MAX_SENT_BYTES 3

volatile unsigned char i2c_buffer[] = {
  // 0x00 - 0x07 Version, read-only
  '2', '0', '1', '5', '1', '2', '1', '1',
  // 0x08 - 0x0f VendorId, read-only
  'm', 'w', '4', '6', 'd', ' ', ' ', ' ',
  // 0x10 - 0x17 DeviceId, read-only
  'P', 'i', 'A', '*', 'H', 'a', 't', ' ',
  // space
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  // 0x20 - 0x2f
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  // 0x30 - 0x3f
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  // 0x40 Command byte, write-only
  0x00,
  // space
  0x00, 0x00, 0x00,
  // 0x44 - 0x47 left encoder, LSB first
  0x00, 0x00, 0x00, 0x00,
  // 0x48 - 0x4b right encoder, LSB first
  0x00, 0x00, 0x00, 0x00,
  // 0x4c - 0x4d left speed, LSB first
  0x00, 0x00,
  // 0x4e - 0x4f right speed, LSB first
  0x00, 0x00,
  // 0x50 - 0x51 Kp, LSB first
  0x00, 0x00,
  // 0x52 - 0x53 Ki, LSB first
  0x00, 0x00,
  // 0x54 - 0x55 Kd, LSB first
  0x00, 0x00,
  // 0x56 - 0x57 Ko, LSB first
  0x00, 0x00,
  // space
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

unsigned char i2c_received_buffer[I2C_MAX_SENT_BYTES];

static void i2c_update_encoder_buffer(unsigned char cmd) {
  long e = readEncoder((cmd - 0x44) / 4);

  i2c_buffer[cmd    ] =  e        & 0xff;
  i2c_buffer[cmd + 1] = (e >>  8) & 0xff;
  i2c_buffer[cmd + 2] = (e >> 16) & 0xff;
  i2c_buffer[cmd + 3] = (e >> 24) & 0xff;
}

static void i2c_request_event() {
  unsigned char cmd = i2c_received_buffer[0];
  unsigned char len = 1;

  if (cmd < 0x40) {
    cmd = (cmd / 8) * 8;
    len = 8;
  }
  else if (cmd < 0x44) {
    // Just one byte useless values
  }
  else if (cmd < 0x4c) {
    cmd = (cmd / 4) * 4;
    len = 4;
  }
  else if (cmd < sizeof(i2c_buffer)) {
    cmd = (cmd / 2) * 2;
    len = 2;
  }

  if (len > 2) {
    Wire.write(len);
  }

  Wire.write((unsigned char *)(i2c_buffer + i2c_received_buffer[0]), len);
}

static void i2c_receive_event(int bytes_received) {
  unsigned char cmd;
  unsigned char len;

  for (int a = 0; a < I2C_MAX_SENT_BYTES; a++) {
    i2c_received_buffer[a] = 0;
  }

  for (int a = 0; a < bytes_received; a++) {
    if ( a < I2C_MAX_SENT_BYTES) {
      i2c_received_buffer[a] = Wire.read();
    }
    else {
      Wire.read();  // if we receive more data then allowed just throw it away
    }
  }

  cmd = i2c_received_buffer[0] = i2c_received_buffer[0] % sizeof(i2c_buffer);

  if (bytes_received == 1) {
    if (cmd == 0x44 || cmd == 0x48) {
       i2c_update_encoder_buffer(cmd);
    }
  }

  if (bytes_received > 1) {
    // We got a write/process.
    if (cmd == 0x40) {
      // We got a command
      i2c_buffer[cmd] = i2c_received_buffer[1];
    }
    else if (cmd == 0x4c || cmd == 0x4e ||
        cmd == 0x50 || cmd == 0x52 || cmd == 0x54 || cmd == 0x56) {
      // two bytes expected
      i2c_buffer[cmd] = i2c_received_buffer[1];
      i2c_buffer[cmd + 1] = i2c_received_buffer[2];
    }
  }
}


void initI2c() {
  Wire.begin(0x42);
  Wire.onReceive(i2c_receive_event);
  Wire.onRequest(i2c_request_event);

  // Kp
  i2c_buffer[0x50] = Kp & 0xff;
  i2c_buffer[0x51] = (Kp >> 8) & 0xff;
  // Ki
  i2c_buffer[0x52] = Ki & 0xff;
  i2c_buffer[0x53] = (Ki >> 8) & 0xff;
  // Kd
  i2c_buffer[0x54] = Kd & 0xff;
  i2c_buffer[0x55] = (Kd >> 8) & 0xff;
  // Ko
  i2c_buffer[0x56] = Ko & 0xff;
  i2c_buffer[0x57] = (Ko >> 8) & 0xff;
}

void runI2c() {
  unsigned char cmd = i2c_buffer[0x40];

  i2c_buffer[0x40] = 0;

  if (cmd > 0) {
    Serial.print("i2c_handle_command("); Serial.println(cmd);
  }

  switch(cmd) {
  case READ_ENCODERS:
    // not needed, just read the i2c area
    break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (i2c_buffer[0x4c] == i2c_buffer[0x4d] == i2c_buffer[0x4e] == i2c_buffer[0x4f] == 0) {
      setMotorSpeeds(0, 0);
      moving = 0;
    }
    else {
      moving = 1;
    }
    leftPID.TargetTicksPerFrame = (int)i2c_buffer[0x4d] << 8 | ((int)i2c_buffer[0x4c] & 0xff);
    rightPID.TargetTicksPerFrame = (int)i2c_buffer[0x4f] << 8 | ((int)i2c_buffer[0x4e] & 0xff);
    break;
  case 'u':  // UPDATE_PID
    Kp = (int)i2c_buffer[0x51] << 8 | ((int)i2c_buffer[0x50] & 0xff);
    Kd = (int)i2c_buffer[0x53] << 8 | ((int)i2c_buffer[0x52] & 0xff);
    Ki = (int)i2c_buffer[0x55] << 8 | ((int)i2c_buffer[0x54] & 0xff);
    Ko = (int)i2c_buffer[0x57] << 8 | ((int)i2c_buffer[0x56] & 0xff);
    break;
  }
}

#endif
