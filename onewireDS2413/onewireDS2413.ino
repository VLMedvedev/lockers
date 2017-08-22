#include <OneWire.h>
#include <EEPROM.h>

#include <AltSoftSerial.h>
AltSoftSerial BarcodeScaner; // RX-8, TX-9

#define LEN_ID_PACKET  (14)
#define LEN_SERIAL  (8)

//SoftwareSerial BarcodeScaner(A0, A1); // RX, TX
char strBarcode[LEN_ID_PACKET];
String barcode = "";
String buff = "";

#define DS2413_ONEWIRE_PIN  (4)

#define DS2413_FAMILY_ID    0x3A
#define DS2413_ACCESS_READ  0xF5
#define DS2413_ACCESS_WRITE 0x5A
#define DS2413_ACK_SUCCESS  0xAA
#define DS2413_ACK_ERROR    0xFF

OneWire oneWire(DS2413_ONEWIRE_PIN);
byte address[LEN_SERIAL] = { 0, 0, 0, 0, 0, 0, 0, 0 };
/*
  typedef struct
  {
  String SerialNumberUnit = "3A1234567";
  String ID_paket = "LF202929536CN";
  byte StatusUnit;
  }  record_type;
*/


struct record_type
{
  byte SerialNumberUnit[LEN_SERIAL];
  byte ID_paket[LEN_ID_PACKET];
  byte StatusUnit;
}  ;

struct onewire_address_type
{
  byte SerialNumberUnit[LEN_SERIAL];
}  ;

String PassWord[32];

byte len_eeAddress = 24;

//const byte secret[16] = "01234567890ABCDEF";
const char serial[11] = "0123ABCDEF";

#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

/********************************************************/
void printBytes(byte* addr, byte count, bool newline = 0)
{
  for (byte i = 0; i < count; i++)
  {
    Serial.print(addr[i] >> 4, HEX);
    Serial.print(addr[i] & 0x0f, HEX);
    Serial.print(" ");
  }
  if (newline)
  {
    Serial.println();
  }
}

void set_str (char* strrez, char* strset, byte len)
{
  for (byte i = 0; i <= len; i++)
  {
    strrez[i] = strset[i];
  }
}
/*******************************************************************/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Wait 5 sec..");

  Serial.println("Tests ..");
  byte count = 1;

  
    onewire_address_type onewire_address[24];
    count = test_1_wire(onewire_address);
    Serial.print( count ); Serial.println(" ");
    test_ee_count(count);

    for (byte i = 0; i <= count; i++)
    {
      printBytes(onewire_address[i].SerialNumberUnit, LEN_SERIAL);
      //Serial.print( i );
      Serial.println(" ");
      byte index = get_eeIndexFromSerialNumberUnit (onewire_address[i].SerialNumberUnit );
      Serial.print( index );
      Serial.println(" ");
      if ( index == 0xFF)
      {
        put_eeSerialNumberUnitToIndex(onewire_address[i].SerialNumberUnit, i);
      }

      delay(200);
    }
  
  eeprom_test(count);

  //put_box_table ();
  //greate_passwd_table ();

  // set the data rate for the SoftwareSerial port
  BarcodeScaner.begin(9600);
  Serial.println("Tests ended");


}


void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("Arbaiten !!!");
  //delay(3000);
  char c;
  char endstr = 0xD;
  char retstr = 0xA;

  if (BarcodeScaner.available()) {
    c = BarcodeScaner.read();
    buff = String (buff + c);
    //Serial.print(c);   Serial.print(" ");   Serial.println(c, HEX);
    if (c == endstr || c == retstr)
    {
      /*
        Serial.println(buff.length());
        Serial.println(buff);
        Serial.println(c, HEX);
      */
      if (buff.length() >= 2)
      {
        barcode = buff;
        buff = "";
        buff.trim();
        //Serial.print(" bc - ");
        Serial.print(barcode);
        byte bc[LEN_ID_PACKET];
        //set_str (char* strrez, char* strset, byte len);
        for (byte j = 0; j < LEN_ID_PACKET; j++)
        {
          bc[j] = barcode[j];
        }
        put_eeID_paketToIndex (bc, 0);
      }
      else
      {
        //Serial.println(" buff = 0 ");
      }
    }
  }
}

void get_eeSerialNumberUnitFromIndex (byte* addr, byte index) {
  record_type record;
  int eeAddress = index * len_eeAddress;
  EEPROM.get(eeAddress, record);
  for (byte i = 0; i < LEN_SERIAL; i++)
  {
    addr[i] = record.SerialNumberUnit[i];
  }
}

void put_eeSerialNumberUnitToIndex (byte* addr, byte index) {
  record_type record;
  int eeAddress = index * len_eeAddress;
  for (byte i = 0; i < LEN_SERIAL; i++)
  {
    record.SerialNumberUnit[i] = addr[i] ;
  }
  EEPROM.put(eeAddress, record);
}

void get_eeID_paketFromIndex (byte* addr, byte index) {
  record_type record;
  Serial.print(" in addr"); 
   printBytes(addr, LEN_ID_PACKET, true);
  int eeAddress = index * len_eeAddress;
  EEPROM.get(eeAddress, record);
  for (byte i = 0; i < LEN_ID_PACKET; i++)
  {
    addr[i] = record.ID_paket[i];
    //Serial.print(" in "); Serial.print(addr[i], HEX); Serial.print(" out "); Serial.print(record.ID_paket[i] , HEX); Serial.println(); 
   }
   Serial.print("get eeaddr  "); Serial.print(eeAddress); Serial.print(" index "); Serial.print(index); Serial.println(); 
  printBytes(addr, LEN_ID_PACKET, true);
}

void put_eeID_paketToIndex (byte* addr, byte index) {
  record_type record;
  int eeAddress = index * len_eeAddress; 
  for (byte i = 0; i < LEN_ID_PACKET; i++)
  {
    record.ID_paket[i] = byte(addr[i]) ;
    //Serial.print(" in "); Serial.print(addr[i], HEX); Serial.print(" out "); Serial.print(record.ID_paket[i] , HEX); Serial.println(); 
  }
  EEPROM.put(eeAddress, record);

  Serial.print("put eeaddr  "); Serial.print(eeAddress); Serial.print(" index "); Serial.print(index); Serial.println(); 
  printBytes(addr, LEN_ID_PACKET, true);

  byte barcod[LEN_ID_PACKET];
  get_eeID_paketFromIndex (barcod, index);
  Serial.print("test put ");
  printBytes(barcod, LEN_ID_PACKET, true);
  
}

byte get_eeStatusUnitFromIndex (byte index) {
  record_type record;
  int eeAddress = index * len_eeAddress;
  EEPROM.get(eeAddress, record);
  byte StatusUnit = record.StatusUnit;
  return StatusUnit;
}

void put_eeStatusUnitToIndex (byte StatusUnit, byte index) {
  record_type record;
  int eeAddress = index * len_eeAddress;
  record.StatusUnit = StatusUnit;
  EEPROM.put(eeAddress, record);
}

byte get_eeIndexFromID_paket (byte* addr) {
  byte index = 0xFF; //no search
  record_type record;
  byte count;
  int eeAddress = 1022;
  EEPROM.get(eeAddress, count);
  if (count >= 32)
  {
    return index;
  }

  //Serial.print(" addr ");
  //  printBytes(addr, LEN_SERIAL, true);

  boolean yes = false;
  for (byte i = 0; i <= count; i++)
  {
    byte eeaddr[LEN_SERIAL];
    get_eeID_paketFromIndex (eeaddr, i);
    //Serial.print(" eeaddr ");
    //printBytes(eeaddr, LEN_SERIAL, true);

    for (byte j = 0; j < LEN_ID_PACKET; j++)
    {
      if (addr[j] != eeaddr[j])
      {
        yes = false;
        //Serial.print(" no "); Serial.print(i); Serial.print(" "); Serial.print(j); Serial.print(" "); Serial.print(addr[j], HEX); Serial.print(" - "); Serial.println(record.SerialNumberUnit[j], HEX);
        break;
      }
      else
      {
        yes = true;
        //Serial.print(" yes "); Serial.print(i); Serial.print(" "); Serial.print(j); Serial.print(" "); Serial.print(addr[j], HEX); Serial.print(" - "); Serial.println(record.SerialNumberUnit[j], HEX);
      }
    }

    if (yes == true)
    {
      index = i;
      break;
    }
    //Serial.print(" index "); Serial.print(index); Serial.print(" yes "); Serial.print(yes);
  }
  //Serial.print(" index "); Serial.println(index);
  return index;
} //

byte get_eeIndexFromSerialNumberUnit (byte* addr) {
  byte index = 0xFF; //no search
  record_type record;
  byte count;
  int eeAddress = 1022;
  EEPROM.get(eeAddress, count);
  if (count >= 32)
  {
    return index;
  }

  //Serial.print(" addr ");
  //printBytes(addr, LEN_SERIAL, true);

  boolean yes = false;
  for (byte i = 0; i <= count; i++)
  {
    byte eeaddr[LEN_SERIAL];
    get_eeSerialNumberUnitFromIndex (eeaddr, i);
    //Serial.print(" eeaddr ");
    //printBytes(eeaddr, LEN_SERIAL, true);

    for (byte j = 0; j < LEN_SERIAL; j++)
    {
      if (addr[j] != eeaddr[j])
      {
        yes = false;
        //Serial.print(" no "); Serial.print(i); Serial.print(" "); Serial.print(j); Serial.print(" "); Serial.print(addr[j], HEX); Serial.print(" - "); Serial.println(record.SerialNumberUnit[j], HEX);
        break;
      }
      else
      {
        yes = true;
        //Serial.print(" yes "); Serial.print(i); Serial.print(" "); Serial.print(j); Serial.print(" "); Serial.print(addr[j], HEX); Serial.print(" - "); Serial.println(record.SerialNumberUnit[j], HEX);
      }
    }

    if (yes == true)
    {
      index = i;
      break;
    }
    //Serial.print(" index "); Serial.print(index); Serial.print(" yes "); Serial.print(yes);
  }
  //Serial.print(" index "); Serial.println(index);
  return index;
} //

void test_ee_count(byte count) {
  int eeAddress = 1022;
  byte eeValue;

  EEPROM.get(eeAddress, eeValue);

  if (count != eeValue)
  {
    EEPROM.put(eeAddress, count);
    //Serial.println(" Not eq ser "); Serial.print(eeValue, HEX); Serial.print(ser, HEX);
  }
} // test_count


void eeprom_test(byte count) {
  byte addr[LEN_SERIAL] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  byte barcod[LEN_ID_PACKET];
  
    byte bc[LEN_ID_PACKET] = "RB298973247SG";
    put_eeID_paketToIndex (bc, 0x00);
    put_eeStatusUnitToIndex (0xA0, 0x00);
Serial.print("test write ");
  get_eeID_paketFromIndex (barcod, 0x00);
  printBytes(barcod, LEN_ID_PACKET, true);
    
    byte bc1[LEN_ID_PACKET] = "RB299157466SG";
    put_eeID_paketToIndex (bc1, 0x01);
    put_eeStatusUnitToIndex (0xA3, 0x01);

   get_eeID_paketFromIndex (barcod, 0x01);
  printBytes(barcod, LEN_ID_PACKET, true);

  //delay(100);

  for (byte i = 0; i <= count; i++)
  {
    //addr[0] = 0;
    Serial.println(" rec "); Serial.println(count, HEX); Serial.println(i, HEX);
    get_eeSerialNumberUnitFromIndex (addr, i);
    printBytes(addr, LEN_SERIAL, true);
    get_eeID_paketFromIndex(barcod, i);
    printBytes(barcod, LEN_ID_PACKET, true);
    byte StatusUnit = get_eeStatusUnitFromIndex (i);
    Serial.println(StatusUnit, HEX);

    //Serial.println(" rec "); Serial.println(count, HEX); Serial.println(i, HEX);
  }
  //Serial.println("Tests ended");

} //end eeprom_test


/***************************************1-Wire***************************/


byte read(void)
{
  bool ok = false;
  byte results;

  oneWire.reset();
  oneWire.select(address);
  oneWire.write(DS2413_ACCESS_READ);

  results = oneWire.read();                 /* Get the register results   */
  ok = (!results & 0x0F) == (results >> 4); /* Compare nibbles            */
  results &= 0x0F;                          /* Clear inverted values      */

  oneWire.reset();

  // return ok ? results : -1;
  return results;
}

bool write(byte state)
{
  byte ack = 0;

  /* Top six bits must '1' */
  state |= 0xFC;

  oneWire.reset();
  oneWire.select(address);
  oneWire.write(DS2413_ACCESS_WRITE);
  oneWire.write(state);
  oneWire.write(~state);                    /* Invert data and resend     */
  ack = oneWire.read();                     /* 0xAA=success, 0xFF=failure */
  if (ack == DS2413_ACK_SUCCESS)
  {
    oneWire.read();                          /* Read the status byte      */
  }
  oneWire.reset();

  return (ack == DS2413_ACK_SUCCESS ? true : false);
}

byte test_1_wire(onewire_address_type * onewire_address)
{

  byte i = 0;
  byte t = 0;

  Serial.println(F("Looking for a DS2413 on the bus"));

  /* Try to find a device on the bus */
  oneWire.reset_search();
  delay(500);

  while (oneWire.search(address))
  {
    /* Check the CRC in the device address */
    if (OneWire::crc8(address, 7) != address[7])
    {
      Serial.println(F("Invalid CRC!"));
      if (t >= 3)
      {
        i++;
      }
      else
      {
        i = 0;
        oneWire.reset_search();
      }
      t++;
    }
    else
    {
      for (byte j = 0; j < LEN_SERIAL; j++)
      {
        onewire_address[i].SerialNumberUnit[j] = address[j];
      }

      i++;
    }
    delay(500);
  }

  /* Make sure we have a DS2413 */
  /*
    if (address[0] != DS2413_FAMILY_ID)
    {
     printBytes(address, 8);
     Serial.println(F(" is not a DS2413!"));
     //while (1);
    }

  */
  //test_wire1() ;

  return i - 1;
}

void test_wire1()
{
  /* Read */

  byte state = read();
  if (state == -1)
    Serial.println(F("Failed reading the DS2413"));
  else
    Serial.println(state, BIN);

  Serial.println();

  /* Write */
  /*
    bool ok = false;
    ok = write(0x3);
    if (!ok) Serial.println(F("Wire failed"));
    delay(1000);
    ok = write(0x0);
    if (!ok) Serial.println(F("Wire failed"));
    delay(1000);
  */
}

