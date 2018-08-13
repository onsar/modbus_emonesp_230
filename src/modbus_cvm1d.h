
#ifndef MODBUS_CVM1D_H
#define MODBUS_CVM1D_H

#include <Arduino.h>
#include <ModbusMaster.h>

#define MAX485_DE      D5
#define MAX485_RE_NEG  D6

// ************************************
// **  MATRIZ DE CONFIGURACION ********
// ************************************

/*
DATA:    [=====     ]  52.4% (used 42916 bytes from 81920 bytes)
PROGRAM: [=         ]  10.3% (used 432488 bytes from 4194304 bytes)
*/

#define NUMBER_OF_REGISTERS 24


#define PARAMETROS_LIST "V_1","A_1","Kw_1","Kvar_1","PF_1",\
                        "V_2","A_2","Kw_2","Kvar_2","PF_2",\
                        "V_3","A_3","Kw_3","Kvar_3","PF_3",\
                        "Kw_III","KvarL_III","KvarC_III","Cos_III","PFIII",\
                        "Hz","V12","V23","V31"

#define FACTOR_LIST 10,1000,1,1,100,\
                    10,1000,1,1,100,\
                    10,1000,1,1,100,\
                    1,1,1,100,100,\
                    10,10,10,10


String registro_parametros[] = {PARAMETROS_LIST};
int registro_factor[] = {FACTOR_LIST};
// registro de resultado para transmitir al servidor
float registro_tx[24];
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
// ************************************
// *******    MODBUS_SETUP      *******
// ************************************

void modbus_setup()
{
  Serial.println("_modbus_setup_");
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 115200 baud
  // Modbus slave ID 3
  node.begin(3, Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);


  Serial.println("_setTransmitBuffer_044C_");
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

  Serial.println("_writeMultipleRegisters_044C_");
  node.writeMultipleRegisters(0x044C, 6);

  Serial.println("_readInputRegisters_044C_");
  uint8_t result;
  result = node.readInputRegisters(0x044C, 6);
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
  else {Serial.println("la lectura NO es CORRECTA");}

  Serial.println("_readInputRegisters_03E8_");
  delay(10000);
  result = node.readInputRegisters(0x03E8, 3);
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
  else {Serial.println("la lectura NO es CORRECTA");}

}
// ************************************
// *******    _FUNCIONES_      ********
// ************************************


long two_register_to_long(uint16_t high_16, uint16_t low_16){

  Serial.println("_two_register_to_long_");
  uint32_t both_32 = 0x00000000;
  uint32_t sign_32 = 0x00000000;
  long both_long = 0;

  both_32 = ((uint32_t)high_16) << 16;
  both_32 = both_32 | (uint32_t)low_16;
  sign_32 = both_32 & 0x80000000;
  both_long = (long)both_32;

  Serial.print("HEX: ");
  Serial.print((unsigned long)both_32,HEX);
  Serial.print(" - sign: ");
  Serial.print((unsigned long)sign_32,HEX); // 0 o 80000000
  Serial.print(" - long: ");
  Serial.println(both_long);
  return both_long;
}

void result_to_register(int n){
  Serial.println("_result_to_tx_register_");

  long both_long = 0;
  both_long = two_register_to_long(node.getResponseBuffer(n*2),node.getResponseBuffer((n*2)+1));
  registro_tx[n] = (float)both_long/(float)registro_factor[n];
  Serial.println(n);

  Serial.print("both_long --> ");
  Serial.println(both_long);
  Serial.print("media que se almacena para tx: ");
  Serial.println(registro_tx[n]);
}

void modbus_loop()
{
  // sizeof(uint16_t) --> 2
  // sizeof(uint32_t) --> 4
  // sizeof(unsigned long) --> 4
  // sizeof(long) --> 4
  // Serial.println(mayor_32) --> 2147483648

  Serial.println("_modbus_loop_");
  Serial.println("_readInputRegisters_");
  delay(10000);  //Necesario para darle tiempo al equipo de CIRCUTOR
  uint8_t result;
  result = node.readInputRegisters(0x0000, (30));
  Serial.println(result);

  if (result == node.ku8MBSuccess)
  {

    for (int i =0; i < (15-1); i++) {
      result_to_register(i);
    }

    for (int i = 0; i < (15-1); i++) {
      Serial.print(registro_parametros[i]);
      Serial.print(" ---> ");
      Serial.println(registro_tx[i]);
    }

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

    Serial.print("0x14:Vt ");
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

  }
  else {Serial.println("la lectura de registros NO es CORRECTA");}

}

#endif // MODBUS_CVM1D_H
