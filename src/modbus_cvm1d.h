
#ifndef MODBUS_CVM1D_H
#define MODBUS_CVM1D_H


/*
Necesario para comunicar con CVM-1D
Serial.begin(19200);
#ifdef DEBUG_SERIAL1
Serial1.begin(19200);
#endif

*/

#include <Arduino.h>
#include <ModbusMaster.h>

#define MAX485_DE      D1
#define MAX485_RE_NEG  D2

uint32_t t_last_tx=0;

// instantiate ModbusMaster object
ModbusMaster node;

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}
void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}


void modbus_setup()
{

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 115200 baud

  // Modbus slave ID 1


  node.begin(3, Serial);
  // node.begin(3, Serial2);
  // Callbacks allow us to configure the RS485 transceiver correctly

  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);


}


void modbus_loop()
{
  // escritura de los registros e configuración desde 044C

  // Primario Tensión          1(Dec)          00000001 (Hex)
  node.setTransmitBuffer(0x00, 0x0000);
  node.setTransmitBuffer(0x01, 0x0001);

  // Secundario Tensión       1(Dec)          0001 (Hex)
  node.setTransmitBuffer(0x02, 0x0001);

  // Primario de Corriente    50 (Dec)     32 (Hex)
  node.setTransmitBuffer(0x03, 0x0032);

  // Sin uso
  node.setTransmitBuffer(0x04, 0x0000);

  // Cálculo de armónicos      00 Respecto el Valor Eficaz
  node.setTransmitBuffer(0x05, 0x0000);

  node.writeMultipleRegisters(0x044C, 6);

  // lectura de los registros e configuración desde 044C
  uint8_t result;
  result = node.readInputRegisters(0x044C, 6);
  Serial.println("");
  Serial.println("lectura de los registros e configuración desde 044C");

  Serial.println(result);
  if (result == node.ku8MBSuccess)
  {
    Serial.println("lectura de los registros ES VALIDA");
    Serial.print("0x00: ");
    Serial.println(node.getResponseBuffer(0x00));
    Serial.print("0x01: ");
    Serial.println(node.getResponseBuffer(0x01));
    Serial.print("0x02: ");
    Serial.println(node.getResponseBuffer(0x02));
    Serial.print("0x03: ");
    Serial.println(node.getResponseBuffer(0x03));
    Serial.print("0x04: ");
    Serial.println(node.getResponseBuffer(0x04));
    Serial.print("0x05: ");
    Serial.println(node.getResponseBuffer(0x05));
  }
  else {Serial.println("la lectura de registros NO es CORRECTA");}

/*
  result = node.readInputRegisters(0x044C, 6);
  Serial.println("");

  Serial.println(result);
  if (result == node.ku8MBSuccess)
  {
    Serial.print("0x00: ");
    Serial.println(node.getResponseBuffer(0x00));
    Serial.print("0x01: ");
    Serial.println(node.getResponseBuffer(0x01));

    Serial.print("0x02: ");
    Serial.println(node.getResponseBuffer(0x02));
    Serial.print("0x03: ");
    Serial.println(node.getResponseBuffer(0x03));

    // String name_value;
    // name_value += "intensidad:";
    name_value += String(node.getResponseBuffer(0x03));
    Serial.println(name_value);


    uint32_t current_time= millis();
    if (wifi_mode == WIFI_MODE_STA || wifi_mode == WIFI_MODE_AP_AND_STA)
      {
        if(emoncms_apikey != 0 && ((current_time - t_last_tx) > 40000))
          {
            t_last_tx = current_time;
            emoncms_publish(name_value);

          }
      }


    Serial.print("0x04: ");
    Serial.println(node.getResponseBuffer(0x04));
    Serial.print("0x05: ");
    Serial.println(node.getResponseBuffer(0x05));

    // Serial.println(node.getResponseBuffer(0x04)/100.0f);
    // Serial.print("Vload: ");
    // Serial.println(node.getResponseBuffer(0xC0)/100.0f);
    //Serial.print("Pload: ");
    //Serial.println((node.getResponseBuffer(0x0D) +
    //                 node.getResponseBuffer(0x0E) << 16)/100.0f);
  }
  else
  {
    Serial.println("Conexion no establecida");
  }
*/
}

#endif // MODBUS_CVM1D_H
