#pragma once
#include "SinglePortModule.h"
#include <OneWire.h> //Paul Stoffregen's OneWire library
/**
 * A simple example module that just replies with "Message received" to any message it receives.
 */

extern OneWire oneWire;

class MiasModule : public SinglePortModule
{
public:
  /** Constructor
   * name is for debugging output
   */
  MiasModule() : SinglePortModule("text", meshtastic_PortNum_TEXT_MESSAGE_APP)
  {
    digitalWrite(36, HIGH); // Turn of Vext
  }

protected:
  /** For reply module we do all of our processing in the (normally optional)
   * want_replies handling
   */
  virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;
  // Internal method to fetch one wire temperature
  float getTemperature(byte addr[8]);
};