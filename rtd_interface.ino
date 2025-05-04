
#include <SPI.h>
#include <math.h>

#define CS_PIN 10 // Chip Select connected to Digital Pin 10
//MISO => SDO //Arduino 12
//MOSI => SDI //Arduino 11
//SCK => SCK //Arduino 13

// --- MAX31865 Register Definitions ---
#define MAX31865_REG_CONFIG       0x00
#define MAX31865_REG_RTD_MSB      0x01
#define MAX31865_REG_RTD_LSB      0x02
#define MAX31865_REG_HFAULT_MSB   0x03
#define MAX31865_REG_HFAULT_LSB   0x04
#define MAX31865_REG_LFAULT_MSB   0x05
#define MAX31865_REG_LFAULT_LSB   0x06
#define MAX31865_REG_FAULT_STATUS 0x07

#define MAX31865_CONFIG_WRITE_ADDR  0x80 
#define MAX31865_CONFIG_VBIAS       (1 << 7)
#define MAX31865_CONFIG_CONV_AUTO   (1 << 6)
#define MAX31865_CONFIG_CONV_1SHOT  (1 << 5)
#define MAX31865_CONFIG_WIRE_4      (1 << 4)
#define MAX31865_CONFIG_WIRE_3      (1 << 0)
#define MAX31865_CONFIG_FAULT_DET_MASK (3 << 2)
#define MAX31865_CONFIG_FAULT_DET_NONE (0 << 2)
#define MAX31865_CONFIG_FAULT_DET_AUTO (1 << 2)
#define MAX31865_CONFIG_FAULT_DET_MAN1 (2 << 2)
#define MAX31865_CONFIG_FAULT_DET_MAN2 (3 << 2)
#define MAX31865_CONFIG_FAULT_CLEAR   (1 << 1) 
#define MAX31865_CONFIG_FILTER_50HZ   (1 << 0) 

#define RREF 400.0f
#define RTD_A 3.9083e-3f
#define RTD_B -5.775e-7f

typedef enum {
    RTD_TYPE_PT100,
    RTD_TYPE_PT1000,
    RTD_TYPE_UNKNOWN
} RtdType;

RtdType current_rtd_type = RTD_TYPE_UNKNOWN; 
SPISettings max31865_spisettings(4000000, MSBFIRST, SPI_MODE1);

void max31865_select() {
  SPI.beginTransaction(max31865_spisettings);
  digitalWrite(CS_PIN, LOW);
}

void max31865_deselect() {
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

uint8_t max31865_read_register(uint8_t reg_addr) {
  max31865_select();
  SPI.transfer(reg_addr & 0x7F);
  uint8_t value = SPI.transfer(0xFF);
  max31865_deselect();
  return value;
}

void max31865_write_register(uint8_t reg_addr, uint8_t value) {
  max31865_select();
  SPI.transfer(reg_addr | MAX31865_CONFIG_WRITE_ADDR);
  SPI.transfer(value);
  max31865_deselect();
}

void rtd_sensor_init(RtdType rtd_type_to_use) {
  current_rtd_type = rtd_type_to_use;
  if (current_rtd_type == RTD_TYPE_UNKNOWN) {
    Serial.println("ERROR: RTD Type Unknown during sensor init!");
    return; 
  }

  uint8_t config_val = MAX31865_CONFIG_VBIAS |
                      MAX31865_CONFIG_WIRE_4 | 
                      MAX31865_CONFIG_FAULT_CLEAR | 
                      MAX31865_CONFIG_FILTER_60HZ; 
  max31865_write_register(MAX31865_REG_CONFIG, config_val);
  delay(10);
  uint8_t cfg_read = max31865_read_register(MAX31865_REG_CONFIG);
  Serial.print("Config Register after Init: 0x"); Serial.println(cfg_read, HEX);
  uint8_t fault_status = max31865_read_register(MAX31865_REG_FAULT_STATUS);
  Serial.print("Fault Status after Init: 0x"); Serial.println(fault_status, HEX);
}

uint16_t rtd_read_raw_adc(uint8_t* fault_detected) {
  if (current_rtd_type == RTD_TYPE_UNKNOWN) {
    if(fault_detected) *fault_detected = 0xFF;
    return 0;
  }

  uint8_t config_read = max31865_read_register(MAX31865_REG_CONFIG);
  config_read |= MAX31865_CONFIG_CONV_1SHOT;
  config_read |= MAX31865_CONFIG_VBIAS;    
  max31865_write_register(MAX31865_REG_CONFIG, config_read);

  delay(70); 
  uint8_t rtd_msb = max31865_read_register(MAX31865_REG_RTD_MSB);
  uint8_t rtd_lsb = max31865_read_register(MAX31865_REG_RTD_LSB);

  uint8_t fault_status = max31865_read_register(MAX31865_REG_FAULT_STATUS);
  if (fault_detected != NULL) {
    *fault_detected = fault_status;
  }

  if (fault_status != 0) {
    return 0;
  }
  uint16_t adc_raw_16bit = ((uint16_t)rtd_msb << 8) | rtd_lsb;
  uint16_t adc_raw_15bit = adc_raw_16bit >> 1;

  return adc_raw_15bit;
}

float rtd_convert_adc_to_temp(uint16_t adc_raw) {
  if (current_rtd_type == RTD_TYPE_UNKNOWN || adc_raw == 0) { 
    return NAN;
  }

  float resistance = ((float)adc_raw * RREF) / 32768.0f;

  float r0;
  if (current_rtd_type == RTD_TYPE_PT100) {
    r0 = 100.0f;
  } else { 
    r0 = 1000.0f;
  }

  float rt_div_r0 = resistance / r0; 
  if (resistance <= 0 || rt_div_r0 < 0.1) { 
    Serial.print("Warning: Implausible resistance calculated: "); Serial.println(resistance);
    return NAN;
  }

  float temp;
  if (rt_div_r0 >= 1.0f) {
    float discriminant = (RTD_A * RTD_A) - (4.0f * RTD_B * (1.0f - rt_div_r0));
    if (discriminant < 0.0f) {
      Serial.println("Warning: Negative discriminant (T>=0)");
      return NAN;
    }
    temp = (-RTD_A + sqrt(discriminant)) / (2.0f * RTD_B);
  }
  else {
    temp = (rt_div_r0 - 1.0f) / RTD_A;
  }

  return temp;
}

RtdType auto_detect_rtd_type() {
  Serial.println("Attempting RTD Auto-Detection...");

  uint8_t initial_config = max31865_read_register(MAX31865_REG_CONFIG);
  max31865_write_register(MAX31865_REG_CONFIG, initial_config | MAX31865_CONFIG_VBIAS | MAX31865_CONFIG_FAULT_CLEAR);
  delay(20);
  uint8_t fault = 0;
  uint16_t adc_raw = rtd_read_raw_adc(&fault);

  if (fault != 0 || adc_raw == 0) { 
    Serial.print("Fault during RTD type detection! Fault code: 0x"); Serial.println(fault, HEX);
    return RTD_TYPE_UNKNOWN;
  }

  float resistance = ((float)adc_raw * RREF) / 32768.0f;
  Serial.print("Resistance measured for detection: "); Serial.print(resistance, 2); Serial.println(" Ohms");

  if (resistance < 500.0f && resistance > 10.0f) { 
      Serial.println("==> Detected PT100");
      return RTD_TYPE_PT100;
  } else if (resistance >= 500.0f) {
      Serial.println("==> Detected PT1000");
      return RTD_TYPE_PT1000;
  } else {
      Serial.println("==> Detection failed: Resistance out of expected range.");
      return RTD_TYPE_UNKNOWN;
  }
}


void setup() {
  SPI.begin();
  Serial.begin(115200);
  while (!Serial && millis() < 4000);
  Serial.println("");
  Serial.println("MAX31865 RTD Sensor Test - Arduino");

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); 
  SPI.begin();

  // Auto-Detect rtd type
  RtdType detected_type = auto_detect_rtd_type();

  rtd_sensor_init(detected_type);
  if (current_rtd_type == RTD_TYPE_UNKNOWN) {
    Serial.println("RTD Initialization Failed. Check connections & type detection. Halting.");
    while(1);
  }

  Serial.println("Initialization Complete. Starting readings...");
  Serial.println("---------------------------------------------");
}

void loop() {
  uint8_t fault_code = 0;
  uint16_t raw_adc = rtd_read_raw_adc(&fault_code);

  if (fault_code == 0) {
      float temperature = rtd_convert_adc_to_temp(raw_adc);

      if (!isnan(temperature)) {
          Serial.print("Raw ADC: "); Serial.print(raw_adc);
          Serial.print("\t Resistance: "); Serial.print(((float)raw_adc * RREF) / 32768.0f, 2); Serial.print(" Ohms");
          Serial.print("\t Temperature: "); Serial.print(temperature, 2); 
          Serial.println(" C");
      } else {
          Serial.println("Temperature calculation error (NaN).");
      }
  } else {
      Serial.print("RTD Read Fault Detected! Fault Code: 0x"); Serial.println(fault_code, HEX);
      uint8_t config_val = max31865_read_register(MAX31865_REG_CONFIG);
      max31865_write_register(MAX31865_REG_CONFIG, config_val | MAX31865_CONFIG_FAULT_CLEAR);
      Serial.println("(Fault cleared, trying again next cycle)");
  }

  delay(1000); 
}
