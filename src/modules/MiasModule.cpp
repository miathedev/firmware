#include "MiasModule.h"
#include "MeshService.h"
#include "configuration.h"
#include "main.h"
#include <OneWire.h> //Paul Stoffregen's OneWire library
#include <assert.h>

OneWire oneWire(4);

// Outdoor sensor: 28 ff d9 dc 93 16 5 43
byte addr_water[8] = {0x28, 0xAA, 0x9A, 0x80, 0x13, 0x13, 0x02, 0xF5};
// Indoor sensor: 28 aa 9a 80 13 13 2 f5
byte addr_air[8] = {0x28, 0xFF, 0xD9, 0xDC, 0x93, 0x16, 0x05, 0x43};

// param: addr - address of the sensor
float MiasModule::getTemperature(byte addr_selected[8])
{
    LOG_INFO("Starting temperature sensor\n");

    byte i;
    byte present = 0;
    byte type_s;
    byte data[9];
    byte addr[8];
    float celsius, fahrenheit;

    // if (!oneWire.search(addr))
    // {
    //     LOG_INFO("No more addresses\n");
    //     oneWire.reset_search();
    //     delay(250);
    //     return 1024;
    // }

    LOG_INFO("Searching for sensor\n");

    while (true)
    {
        if (!oneWire.search(addr))
        {
            LOG_INFO("No more addresses\n");
            oneWire.reset_search();
            delay(250);
            return 1024;
        }
        LOG_INFO("Found sensor with address: ");
        for (i = 0; i < 8; i++)
        {
            LOG_INFO("%x ", addr[i]);
        }
        // Continue if addr matches
        if (memcmp(addr, addr_selected, 8) == 0)
        {
            LOG_INFO("Found sensor\n");
            break;
        }
    }

    LOG_INFO("ROM =");
    for (i = 0; i < 8; i++)
    {
        LOG_INFO(" %x", addr[i]);
    }

    if (OneWire::crc8(addr, 7) != addr[7])
    {
        LOG_INFO("CRC is not valid!\n");
        return 1024;
    }
    Serial.println();

    // the first ROM byte indicates which chip
    switch (addr[0])
    {
    case 0x10:
        LOG_INFO("  Chip = DS18S20\n");
        type_s = 1;
        break;
    case 0x28:
        LOG_INFO("  Chip = DS18B20\n");
        type_s = 0;
        break;
    case 0x22:
        LOG_INFO("  Chip = DS1822\n");
        type_s = 0;
        break;
    default:
        LOG_INFO("Device is not a DS18x20 family device.\n");
        return 1024;
    }

    oneWire.reset();
    oneWire.select(addr);
    oneWire.write(0x44, 1); // start conversion, with parasite power on at the end

    delay(1000); // maybe 750ms is enough, maybe not
    // we might do a oneWire.depower() here, but the reset will take care of it.

    present = oneWire.reset();
    oneWire.select(addr);
    oneWire.write(0xBE); // Read Scratchpad

    LOG_INFO("  Data = %x\n", present);
    for (i = 0; i < 9; i++)
    { // we need 9 bytes
        data[i] = oneWire.read();
        LOG_INFO("  Data = %x\n", data[i]);
    }

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s)
    {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10)
        {
            // "count remain" gives full 12 bit resolution
            raw = (raw & 0xFFF0) + 12 - data[6];
        }
    }
    else
    {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00)
            raw = raw & ~7; // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20)
            raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40)
            raw = raw & ~1; // 11 bit res, 375 ms
                            //// default is 12 bit resolution, 750 ms conversion time
    }
    celsius = (float)raw / 16.0;
    fahrenheit = celsius * 1.8 + 32.0;
    LOG_INFO("  Temperature = %f Celsius, %f Fahrenheit\n", celsius, fahrenheit);
    oneWire.reset_search();
    return celsius;
};

ProcessMessage MiasModule::handleReceived(const meshtastic_MeshPacket &mp)
{
#ifdef DEBUG_PORT
    auto &p = mp.decoded;
    LOG_INFO("Received a msg from=0x%0x, id=0x%x, msg=%.*s\n", mp.from, mp.id, p.payload.size, p.payload.bytes);
#endif

    // We only store/display messages destined for us.
    // Keep a copy of the most recent text message.
    // devicestate.rx_text_message = mp;
    // devicestate.has_rx_text_message = true;
    if (p.payload.size <= 40)
    {

        char *request = new char[40];
        sprintf(request, "%s", p.payload.bytes);
        if (strcmp(request, "ping") == 0)
        {
            char *message = new char[120];
            sprintf(message, "Pong response, got ping from: 0x%0x with rssi: %d, snr: %f, hops_away: %d, hops_start: %d, mqtt_path: %d", mp.from, (int)mp.rx_rssi, mp.rx_snr, (unsigned int)(mp.hop_start - mp.hop_limit), (unsigned int)(mp.hop_start), mp.via_mqtt);
            meshtastic_MeshPacket *presponse = allocDataPacket();
            presponse->want_ack = false;
            presponse->decoded.payload.size = strlen(message);
            presponse->hop_limit = 7;
            presponse->to = mp.from; // Send back to where it came from
            // presponse->channel = mp.channel; // Use same channel
            memcpy(presponse->decoded.payload.bytes, message, presponse->decoded.payload.size);
            LOG_INFO("Sending message id=%d, dest=%x, msg=%.*s\n", presponse->id, presponse->to, presponse->decoded.payload.size, presponse->decoded.payload.bytes);
            // service->sendToMesh(presponse);
            service->sendToMesh(presponse, RX_SRC_LOCAL, true);
            delete[] message;
        }
        else if (strcmp(request, "temp") == 0)
        {
            LOG_INFO("Starting temperature sensor\n");
            digitalWrite(36, LOW);
            delay(1000);
            float air_temp = getTemperature(addr_air);
            digitalWrite(36, HIGH);

            delay(1000);

            digitalWrite(36, LOW);
            delay(1000);
            float water_temp = getTemperature(addr_water);
            digitalWrite(36, HIGH);

            char *message = new char[60];
            sprintf(message, "Air temp: %f, Water temp: %f", air_temp, water_temp);
            LOG_INFO("Preparing to send message");
            meshtastic_MeshPacket *presponse = allocDataPacket();
            presponse->want_ack = false;
            presponse->decoded.payload.size = strlen(message);
            presponse->hop_limit = 7;
            presponse->to = mp.from; // Send back to where it came from
            // presponse->channel = mp.channel; // Use same channel
            LOG_INFO("Copying message");
            memcpy(presponse->decoded.payload.bytes, message, presponse->decoded.payload.size);
            LOG_INFO("Sending message id=%d, dest=%x, msg=%.*s\n", presponse->id, presponse->to, presponse->decoded.payload.size, presponse->decoded.payload.bytes);
            // service->sendToMesh(presponse);
            service->sendToMesh(presponse, RX_SRC_LOCAL, true);
            delete[] message;
        }
        // Return available commands
        else if (strcmp(request, "help") == 0)
        {
            char *message = new char[120];
            sprintf(message, "Available commands: ping, temp, help");
            meshtastic_MeshPacket *presponse = allocDataPacket();
            presponse->want_ack = false;
            presponse->decoded.payload.size = strlen(message);
            presponse->hop_limit = 7;
            presponse->to = mp.from; // Send back to where it came from
            // presponse->channel = mp.channel; // Use same channel
            memcpy(presponse->decoded.payload.bytes, message, presponse->decoded.payload.size);
            LOG_INFO("Sending message id=%d, dest=%x, msg=%.*s\n", presponse->id, presponse->to, presponse->decoded.payload.size, presponse->decoded.payload.bytes);
            // service->sendToMesh(presponse);
            service->sendToMesh(presponse, RX_SRC_LOCAL, true);
            delete[] message;
        }
    }
    return ProcessMessage::CONTINUE; // Let others look at this message also if they want
}