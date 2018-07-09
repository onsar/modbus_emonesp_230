/*
 * -------------------------------------------------------------------
 * EmonESP Serial to Emoncms gateway
 * -------------------------------------------------------------------
 * Adaptation of Chris Howells OpenEVSE ESP Wifi
 * by Trystan Lea, Glyn Hudson, OpenEnergyMonitor
 * All adaptation GNU General Public License as below.
 *
 * -------------------------------------------------------------------
 *
 * This file is part of OpenEnergyMonitor.org project.
 * EmonESP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * EmonESP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with EmonESP; see the file COPYING.  If not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "emonesp.h"
#include "config.h"
#include "wifi.h"
#include "web_server.h"
#include "ota.h"
#include "input.h"
#include "emoncms.h"
#include "mqtt.h"
// 485-------------------------------------------------------------------
#include "modbus_cvm1d.h"



// 485-------------------------------------------------------------------


// -------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------
void setup() {
  delay(2000);

  Serial.begin(19200);
#ifdef DEBUG_SERIAL1
  Serial1.begin(19200);
#endif

  DEBUG.println();
  DEBUG.print("EmonESP ");
  DEBUG.println(ESP.getChipId());
  DEBUG.println("Firmware: "+ currentfirmware);

  // Read saved settings from the config
  config_load_settings();

  // Initialise the WiFi
  DEBUG.println("wifi_setup");
  wifi_setup();

  // Bring up the web server

  DEBUG.println("web_server_setup");
  web_server_setup();

  // Start the OTA update systems
  // ota_setup();

  DEBUG.println("Server started");

  delay(100);


// 485------
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 115200 baud

  // Modbus slave ID 1

  DEBUG.println("node.begin");
  node.begin(1, Serial);
  // Callbacks allow us to configure the RS485 transceiver correctly

  DEBUG.println("preTransmission");
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

// 485------

} // end setup

// 485------
// bool state = true;
// 485------

// -------------------------------------------------------------------
// LOOP
// -------------------------------------------------------------------

void loop()
{

  // ota_loop();
  web_server_loop();
  wifi_loop();
  modbus_loop();

  /*
  String input = "";
  boolean gotInput = input_get(input);

  if (wifi_mode == WIFI_MODE_STA || wifi_mode == WIFI_MODE_AP_AND_STA)
  {
    if(emoncms_apikey != 0 && gotInput) {
      emoncms_publish(input);
    }
    if(mqtt_server != 0)
    {
      mqtt_loop();
      if(gotInput) {
        mqtt_publish(input);
      }
    }
  }
*/
// 485------


  delay(2000);
  // node.preTransmission(preTransmission);
  // preTransmission();
  // Serial.println("preTransmission");
  // delay(10000);

  // node.postTransmission(postTransmission);
  // postTransmission();
  // Serial.println("postTransmission");
  // delay(10000);

// 485------


} // end loop
