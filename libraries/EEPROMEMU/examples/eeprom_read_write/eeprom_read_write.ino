#include <Arduino.h>
#include <EEPROMEMU.h>

struct MyObject
{
    float field1;
    byte field2;
    char name[10];
};

void secondTest();

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting emulated flash EEPROM test!");

    EEPROMEMU.begin();
    //erase.
    Serial.println("Erasing emulated flash EEPROM beforehand.");
    for (size_t i = 0; i < sizeof(float) + sizeof(MyObject); i++)
    {
        EEPROMEMU.write(i, 0xff);
    }
    EEPROMEMU.commit();

    float f = 123.456f; //Variable to store in emulated EEPROM.
    int eeAddress = 0;  //Location we want the data to be put.

    //One simple call, with the address first and the object second.
    EEPROMEMU.put(eeAddress, f);

    Serial.println("Written float data type! (value = " + String(f) + ")");

    /** Put is designed for use with custom structures also. **/

    //Data to store.
    MyObject customVar = {
        3.14f,
        65,
        "Working!"
    };

    eeAddress += sizeof(float); //Move address to the next byte after float 'f'.

    EEPROMEMU.put(eeAddress, customVar);
    unsigned long micros_start = micros();
    EEPROMEMU.commit();
    unsigned long micros_end = micros();

    Serial.println("Written custom data type in " + String(micros_end - micros_start) + " microseconds!");
    Serial.println("Now retrieving values.");

    secondTest();
}

void secondTest()
{
    Serial.print("Read float from flash emulated EEPROM: ");
    float f = 0.00f;   //Variable to store data read from flash emulated EEPROM.
    int eeAddress = 0; //flash emulated EEPROM address to start reading from

    //Get the float data from the  flash emulatedEEPROM at position 'eeAddress'
    EEPROMEMU.get(eeAddress, f);
    Serial.println(f, 3); //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.

    eeAddress = sizeof(float); //Move address to the next byte after float 'f'.

    MyObject customVar; //Variable to store custom object read from EEPROM.
    EEPROMEMU.get(eeAddress, customVar);

    Serial.println("Read custom object from flash emulated EEPROM: ");
    Serial.println(customVar.field1);
    Serial.println(customVar.field2);
    Serial.println(customVar.name);

    if (strcmp(customVar.name, "Working!") == 0)
    {
        Serial.println("== Test passed: Writing + reading string field ==");
        /* yes, exact float compare is wanted -- must be equal to what we wrote*/
        if (customVar.field1 == 3.14f)
        {
            Serial.println("== Test passed: Writing + reading float field ==");
            if (customVar.field2 == 65)
            {
                Serial.println("== Test passed: Writing + reading int field ==");
                Serial.println("=== ALL TESTS PASSED ===");
            }
            else
            {
                Serial.println("== Test failed: Retrived not the same int ==");
            }
        }
        else
        {
            Serial.println("== Test failed: Retrived not the same float ==");
        }
    }
    else
    {
        Serial.println("== Test failed: Retrived not the same string ==");
    }
}

void loop()
{
    Serial.println("Test done.");
    delay(20000);
}