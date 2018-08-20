
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

#define REGISTERS_TO_READ 15

#define PARAMETER_LIST "V_1","A_1","Kw_1","Kvar_1","PF_1",\
                       "V_2","A_2","Kw_2","Kvar_2","PF_2",\
                       "V_3","A_3","Kw_3","Kvar_3","PF_3",\
                       "Kw_III","KvarL_III","KvarC_III","Cos_III","PFIII",\
                       "Hz","V12","V23","V31"

#define TRANSMISSION_LIST "Kw_1","Kvar_1",\
                          "Kw_2","Kvar_2",\
                          "Kw_3","Kvar_3"

#define FACTOR_LIST 10,1000,1,1,100,\
                    10,1000,1,1,100,\
                    10,1000,1,1,100,\
                    1,1,1,100,100,\
                    10,10,10,10




String array_parameters[] = {PARAMETER_LIST};
String transmission_list[] = {TRANSMISSION_LIST};
int factor_list[] = {FACTOR_LIST};
int tx_mark[NUMBER_OF_REGISTERS];

int number_of_parameters = 0;
int number_of_tx_list = 0;
// registro de resultadoS para transmitir al servidor
float tx_values[24];
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

void result_to_register(int n, int first){
  Serial.println("_result_to_tx_register_");

  long both_long = 0;
  both_long = two_register_to_long(node.getResponseBuffer(n*2),node.getResponseBuffer((n*2)+1));
  tx_values[n+first] = (float)both_long/(float)factor_list[n+first];

  Serial.print(n+first);
  Serial.print("  both_long: ");
  Serial.print(both_long);
  Serial.print(" - float: ");
  Serial.println(tx_values[n+first]);
}


void default_value_to_register(){
  Serial.println("_default_value_to_register_");
  for (int i =0; i < (number_of_parameters); i++) {
    tx_values[i] = 1.12;
    Serial.print("tx_values[] ");
    Serial.print(i);
    Serial.print(" -> ");
    Serial.println(tx_values[i]);
  }
}

void parameter_list_mark_to_tx(){
  //tamaño de todos los punteros / Tamaño de un puntero
  Serial.println("_parameter_list_mark_to_tx_");

  number_of_parameters = sizeof(array_parameters)/sizeof(array_parameters[0]);
  number_of_tx_list = sizeof(transmission_list)/sizeof(transmission_list[0]);
  Serial.print("number_of_parameters   :");
  Serial.println(number_of_parameters);
  Serial.print("number_of_tx_list   :");
  Serial.println(number_of_tx_list);

  for (int pl = 0; pl < number_of_parameters; pl++) {         //parameter_list
    tx_mark[pl] = 0;
    for (int tl = 0; tl < number_of_tx_list; tl++) {    //transmission_list
      int r = array_parameters[pl].equalsIgnoreCase(transmission_list[tl]);
      if (r) {
        tx_mark[pl] = 1;
        Serial.print("realtion_pl_tl   :");
        Serial.print(pl);
        Serial.print(" = ");
        Serial.println(tl);
        break;
      }
    }
  }
}

String compose_msj_to_tx() {
  Serial.println("_compose_msj_to_tx_");
  Serial.print("number_of_parameters : ");
  Serial.println(number_of_parameters);
  String message_to_tx_ = "";
  for (int i = 0; i < number_of_parameters; i++) {
    if (tx_mark[i]) {
      message_to_tx_ += array_parameters[i];
      message_to_tx_ += ":";
      message_to_tx_ += tx_values[i];
      message_to_tx_ += ",";
      // Serial.println(message_to_tx_);
      // Serial.println(message_to_tx_.length());
    }
  }
  if (message_to_tx_.endsWith(",")) {
    message_to_tx_.remove((message_to_tx_.length()-1));
  }
  return message_to_tx_;
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

  Serial.println("");
  Serial.println("_readInputRegisters_044C_");
  delay(10000);
  uint8_t result;
  result = node.readInputRegisters(0x044C, 6);
  Serial.print("result = ");
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


  parameter_list_mark_to_tx();


}

void modbus_loop()
{
  // sizeof(uint16_t) --> 2
  // sizeof(uint32_t) --> 4
  // sizeof(unsigned long) --> 4
  // sizeof(long) --> 4
  // Serial.println(mayor_32) --> 2147483648

  Serial.println("_modbus_loop_");


  // ***********************************************************
  // prueba para guardar los parametros que deben ser trasmitidos
  // ***********************************************************



  Serial.println("_readInputRegisters_1_");
  delay(10000);  //Necesario para darle tiempo al equipo de CIRCUTOR
  uint8_t result;

  result = node.readInputRegisters(0x0000, (REGISTERS_TO_READ*2));
  // result = node.readInputRegisters(0x001E, (18));
  Serial.println(result);

  if (result == node.ku8MBSuccess)
  {

    for (int i =0; i < (REGISTERS_TO_READ); i++) {
      result_to_register(i,0);
    }

    for (int i = 0; i < (REGISTERS_TO_READ); i++) {
      Serial.print(array_parameters[i]);
      Serial.print(" ---> ");
      Serial.println(tx_values[i]);
    }
  }
  else {
    Serial.println("la lectura NO es CORRECTA");
    default_value_to_register();
  }

  Serial.println("_readInputRegisters_2_");
  delay(10000);  //Necesario para darle tiempo al equipo de CIRCUTOR

  result = node.readInputRegisters(0x001E, (18));
    // result = node.readInputRegisters(0x001E, (18));
  Serial.println(result);

  if (result == node.ku8MBSuccess)
  {
    for (int i =0; i < (9); i++) {
    result_to_register(i,15);
    }
    for (int i = 15; i < (24); i++) {
      Serial.print(array_parameters[i]);
      Serial.print(" ---> ");
      Serial.println(tx_values[i]);
    }
  }
  else {
    Serial.println("la lectura NO es CORRECTA");
    default_value_to_register();
  }

  String msj_to_tx = compose_msj_to_tx();
  Serial.println(msj_to_tx);

}

#endif // MODBUS_CVM1D_H
