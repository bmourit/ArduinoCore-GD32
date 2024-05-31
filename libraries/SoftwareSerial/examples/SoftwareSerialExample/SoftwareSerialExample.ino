#include  <SoftwareSerial.h>

SoftwareSerial mySerial(10, 9);  //RX(D2),TX(D3)

void setup()
{
    mySerial.begin(115200);
    mySerial.print("hello arduino");
    mySerial.print("The startup code appears to work");
}

// the loop function runs over and over again forever
void loop()
{

}
