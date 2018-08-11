
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

#define MAX485_DE      D5
#define MAX485_RE_NEG  D6

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

uint32_t compemento_a_dos(uint16_t high_16, uint16_t low_16){

  uint32_t both_32 = 0x00000000;
  uint32_t sign_32 = 0x00000000;
  both_32 = ((uint32_t)high_16) << 16;
  both_32 = both_32 | (uint32_t)low_16;
  sign_32 = both_32 & 0x80000000;
  if (sign_32) {
    Serial.println("El signo es negativo");
  }
  Serial.print("unidos los dos registros de uint16_t: ");
  Serial.println((unsigned long)both_32,HEX);
  Serial.print("elsigno de de registro_32: ");
  Serial.println((unsigned long)sign_32,HEX); // 0 o 80000000
  return both_32;

}


void modbus_loop()
{
  Serial.println(" ----------------------------- ");
  Serial.println("prueba con variables de 32 bits");
  Serial.println(" ----------------------------- ");

  int espacio = sizeof(uint16_t);
  Serial.print("bits de un uint16_t: ");
  Serial.println(espacio);

  espacio = sizeof(uint32_t);
  Serial.print("bits de un uint32_t: ");
  Serial.println(espacio);

  espacio = sizeof(unsigned long);
  Serial.print("bits de un unsigned long: ");
  Serial.println(espacio);

  espacio = sizeof(long);
  Serial.print("bits de un long: ");
  Serial.println(espacio);

  uint32_t registro_32 = 0x0;
  uint32_t signo_32 = 0x80000000;
  uint16_t registro_16h = 0xAAAA;
  uint16_t registro_16l = 0xBBBB;

  Serial.println("_32 mayor numero con signo en complenento a 2: ");
  Serial.println(signo_32); //2147483648
  Serial.println(signo_32,HEX);


  Serial.println("_32 mayor numero: ");
  uint32_t mayor_32 = 0xFFFFFFFF;
  Serial.println(mayor_32); //2147483648
  Serial.println(mayor_32,HEX);

  Serial.println((unsigned long)registro_32);
  registro_32 = ((uint32_t)registro_16h) << 16;
  registro_32 = registro_32 | (uint32_t)registro_16l;
  signo_32 = registro_32 & 0x80000000;

  Serial.print("unidos los dos registros de uint16_t: ");
  Serial.println((unsigned long)registro_32,HEX);
  Serial.print("elsigno de de registro_32: ");
  Serial.println((unsigned long)signo_32,HEX);

  Serial.println(" ----------------------------- ");
  Serial.println("OTRA varible de 32 bits");
  Serial.println(" ----------------------------- ");

  registro_16h = 0x8AAA;
  registro_16l = 0xBB0B;
  registro_32 = ((uint32_t)registro_16h) << 16;
  registro_32 = registro_32 | (uint32_t)registro_16l;
  signo_32 = registro_32 & 0x80000000;
  Serial.print("unidos los dos registros de uint16_t: ");
  Serial.println((unsigned long)registro_32,HEX);
  Serial.print("elsigno de de registro_32: ");
  Serial.println((unsigned long)signo_32,HEX); // 0 o 80000000

  Serial.println(" ----------------------------- ");
  Serial.println("OTRA varible POR FUNCION");
  Serial.println(" ----------------------------- ");

  registro_16h = 0x7AAA;
  registro_16l = 0xBB0B;
  uint32_t test = compemento_a_dos(registro_16h, registro_16l);


  // escritura de los registros e configuración desde 044C

  // Primario Tensión          1(Dec)          00000001 (Hex)
  node.setTransmitBuffer(0x00, 0x0000);
  node.setTransmitBuffer(0x01, 0x0001);

  // Secundario Tensión       1(Dec)          0001 (Hex)
  node.setTransmitBuffer(0x02, 0x0001);

  // Primario de Corriente    50(Dec) 32 Hex) 5000(Dec) 1388(Hex)
  node.setTransmitBuffer(0x03, 0x0032);

  // Sin uso
  node.setTransmitBuffer(0x04, 0x0000);

  // Cálculo de armónicos      00 Respecto el Valor Eficaz
  node.setTransmitBuffer(0x05, 0x0000);

  node.writeMultipleRegisters(0x044C, 6);

  // lectura de los registros e configuración desde 044C
  uint8_t result;
  result = node.readInputRegisters(0x044C, 6);
  // result = node.readHoldingRegisters(0x000A, 6);
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


  delay(10000);
  result = node.readInputRegisters(0x03E8, 3);
  // result = node.readHoldingRegisters(0x000A, 6);
  Serial.println("");
  Serial.println("lectura de los PARAMETROS desde 03E8");

  Serial.println(result);
  if (result == node.ku8MBSuccess)
  {
    Serial.println("lectura de los registros ES VALIDA");
    Serial.print("0x00: ");
    Serial.println(node.getResponseBuffer(0x00),HEX);
    Serial.print("0x01: ");
    Serial.println(node.getResponseBuffer(0x01),HEX);
    Serial.print("0x02: ");
    Serial.println(node.getResponseBuffer(0x02),HEX);

  }
  else {Serial.println("la lectura de PARAMETROS NO es CORRECTA");}



  //Necesario para darle tiempo al equipo de CIRCUTOR
  delay(10000);


  result = node.readInputRegisters(0x0000, 30);
  Serial.println("");
  Serial.println("lectura de los registros de energía desde 0x0000");
  Serial.println(result);
  if (result == node.ku8MBSuccess)
  {
    Serial.print("0x00:Vr: ");
    Serial.println(node.getResponseBuffer(0x00),HEX);
    Serial.print("0x01:Vr: ");
    Serial.println(node.getResponseBuffer(0x01),HEX);
    Serial.print("0x02:Irms: ");
    Serial.println(node.getResponseBuffer(0x02));
    Serial.print("0x03:Irms: ");
    Serial.println(node.getResponseBuffer(0x03));
    Serial.print("0x04:Pa: ");
    Serial.println(node.getResponseBuffer(0x04));
    Serial.print("0x05:Pa: ");
    Serial.println(node.getResponseBuffer(0x05));
    Serial.print("0x06:Pr: ");
    Serial.println(node.getResponseBuffer(0x06),HEX);
    Serial.print("0x07:Pr: ");
    Serial.println(node.getResponseBuffer(0x07),HEX);
    Serial.print("0x08:Pf ");
    Serial.println(node.getResponseBuffer(0x08),HEX);
    Serial.print("0x09:Pf ");
    Serial.println(node.getResponseBuffer(0x09),HEX);

    Serial.print("0x0A:Vs: ");
    Serial.println(node.getResponseBuffer(0x0A));
    Serial.print("0x0B:Vs: ");
    Serial.println(node.getResponseBuffer(0x0B));
    Serial.print("0x0C: ");
    Serial.println(node.getResponseBuffer(0x0C));
    Serial.print("0x0D: ");
    Serial.println(node.getResponseBuffer(0x0D));
    Serial.print("0x0E: ");
    Serial.println(node.getResponseBuffer(0x0E));
    Serial.print("0x0F: ");
    Serial.println(node.getResponseBuffer(0x0F));
    Serial.print("0x10: ");
    Serial.println(node.getResponseBuffer(0x10));
    Serial.print("0x11: ");
    Serial.println(node.getResponseBuffer(0x11));
    Serial.print("0x12: ");
    Serial.println(node.getResponseBuffer(0x12));
    Serial.print("0x13: ");
    Serial.println(node.getResponseBuffer(0x13));
    Serial.print("0x14: ");
    Serial.println(node.getResponseBuffer(0x14));
    Serial.print("0x15:Vt: ");
    Serial.println(node.getResponseBuffer(0x15));
    Serial.print("0x16: ");
    Serial.println(node.getResponseBuffer(0x16));
    Serial.print("0x17: ");
    Serial.println(node.getResponseBuffer(0x17));
    Serial.print("0x18: ");
    Serial.println(node.getResponseBuffer(0x18));
    Serial.print("0x19: ");
    Serial.println(node.getResponseBuffer(0x19));
    Serial.print("0x1A: ");
    Serial.println(node.getResponseBuffer(0x1A));
    Serial.print("0x1B: ");
    Serial.println(node.getResponseBuffer(0x1B));
    Serial.print("0x1C: ");
    Serial.println(node.getResponseBuffer(0x1C));
    Serial.print("0x1D: ");
    Serial.println(node.getResponseBuffer(0x1D));
    Serial.print("0x1F: ");
    Serial.println(node.getResponseBuffer(0x1F));


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
