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
#include "modbus_cvm1d.h"



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

  modbus_setup();

  delay(100);

} // end setup


void loop()
{

  // ota_loop();
  web_server_loop();
  wifi_loop();



//   String input = "";
//  boolean gotInput = input_get(input);

  if (( millis() - t_last_tx) > 10000) {
    t_last_tx = millis();
    if (wifi_mode == WIFI_MODE_STA || wifi_mode == WIFI_MODE_AP_AND_STA){
      if(emoncms_apikey != 0) {
        String msj_to_tx_ = modbus_loop();
        if (msj_to_tx_ != "nook") {
          emoncms_publish(msj_to_tx_);
        }
      }
    }
  }




  // timet to tx modbus information
  // Serial.print("_millis()_");
  // Serial.println(millis()/1000);

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
