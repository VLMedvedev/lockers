#include <OneWire.h>
#include <EEPROM.h>

#include <AltSoftSerial.h>
AltSoftSerial BarcodeScaner; // RX-8, TX-9
/*
  #define MAX_INDEX  (byte(32))
  #define LEN_ID_PACKET  (byte(14))
  #define LEN_SERIAL_NUMBER  (byte(8))
  #define LEN_STATUS_UNIT  (byte(1))
  #define LEN_PASSWD  (byte(6))
  #define ADDRESS_SERIAL_NUMBER_ARRAY  (0)
  #define ADDRESS_ID_PACKET_ARRAY  (int(MAX_INDEX*LEN_SERIAL_NUMBER))
  #define ADDRESS_STATUS_UNIT_ARRAY  (int(ADDRESS_ID_PACKET_ARRAY+MAX_INDEX*LEN_ID_PACKET))
  #define ADDRESS_PASSWD_ARRAY  (int(ADDRESS_STATUS_UNIT_ARRAY+MAX_INDEX*LEN_STATUS_UNIT))
  #define ADDRESS_COUNT_BOX  (int(1022))
*/

const byte MAX_INDEX = 32;
const byte LEN_ID_PACKET = 14;
const byte LEN_SERIAL_NUMBER = 8;
const byte LEN_STATUS_UNIT = 1;
const byte LEN_PASSWD = 6;
const int ADDRESS_SERIAL_NUMBER_ARRAY = 0;
const int ADDRESS_ID_PACKET_ARRAY = MAX_INDEX * LEN_SERIAL_NUMBER;
const int ADDRESS_STATUS_UNIT_ARRAY = ADDRESS_ID_PACKET_ARRAY + MAX_INDEX * LEN_ID_PACKET;
const int ADDRESS_PASSWD_ARRAY = ADDRESS_STATUS_UNIT_ARRAY + MAX_INDEX * LEN_STATUS_UNIT;
const int ADDRESS_COUNT_BOX = 1022;

byte count_box;

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
byte address[LEN_SERIAL_NUMBER] = { 0, 0, 0, 0, 0, 0, 0, 0 };

struct onewire_address_type
{
  byte SerialNumberUnit[LEN_SERIAL_NUMBER];
}  ;

//const byte secret[16] = "01234567890ABCDEF";
const char serial_postomat[11] = "0123ABCDEF";

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

void array_to_array (byte* array_out, byte* array_in, byte len)
{
  for (byte i = 0; i <= len; i++)
  {
    array_out[i] = array_in[i];
  }
}
/*******************************************************************/

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Wait 5 sec..");

  Serial.println("Tests ..");
  byte count = 0;
  /*
    for (int i = 0 ; i < EEPROM.length() ; i++) {
      EEPROM.write(i, 0);
    }
  */
  onewire_address_type onewire_address[24];
  count = test_1_wire(onewire_address);
  //Serial.print( count ); Serial.println(" ");
  test_ee_count(count);
/*
  for (byte i = 0; i < count; i++)
  {
    printBytes(onewire_address[i].SerialNumberUnit, LEN_SERIAL_NUMBER,1);
    //Serial.print( i );
    //Serial.println(" ");
    byte index = get_ee_index_an_string_array (onewire_address[i].SerialNumberUnit, LEN_SERIAL_NUMBER, ADDRESS_SERIAL_NUMBER_ARRAY) ;
    //Serial.print( index );
    //Serial.println(" ");
    if ( index == 0xFF)
    {
      put_srting_array_from_eeprom_to_index(onewire_address[i].SerialNumberUnit, LEN_SERIAL_NUMBER, ADDRESS_SERIAL_NUMBER_ARRAY, i );
    }

    delay(200);
  }
*/
  //eeprom_test();

  //put_box_table ();
  //greate_passwd_table ();

  // set the data rate for the SoftwareSerial port
  BarcodeScaner.begin(9600);
  Serial.println("Tests ended");
}


void loop()
{
  // put your main code here, to run repeatedly:
  //Serial.println("Arbaiten !!!");
  //delay(3000);
  char c;
  char endstr = 0xD;
  char retstr = 0xA;
  

  if (BarcodeScaner.available()) {
    c = BarcodeScaner.read();
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
          //Serial.print(barcode[j]);
        }
        //j++;
        //printBytes(bc, LEN_ID_PACKET, true);
        //bc[j]=0x00;
        //put_srting_array_from_eeprom_to_index(bc, LEN_ID_PACKET, ADDRESS_ID_PACKET_ARRAY, 1 );
        byte bar[LEN_ID_PACKET];
        //get_srting_array_from_eeprom_an_index(bar, LEN_ID_PACKET, ADDRESS_ID_PACKET_ARRAY, 1 );
        //printBytes(bar, LEN_ID_PACKET, true);
        byte index = get_ee_index_an_string_array (bc, LEN_ID_PACKET, ADDRESS_ID_PACKET_ARRAY );
        Serial.print(" ind "); Serial.println(index);
      }
    }
    else
    {
      buff = String (buff + c);
    }


  }
}


void get_srting_array_from_eeprom_an_index(byte* out_string, byte len_string, int address_array, byte index )
{
  int eeaddress = address_array;
  eeaddress += index * len_string;
  if (len_string <= 1)
  {
    out_string = EEPROM.read(eeaddress);
    //Serial.print(out_string[0], HEX); Serial.print(" g-0- ");
  }
  else
  {
    for (byte i = 0; i < len_string; i++)
    {
      eeaddress += i;
      out_string[i] = EEPROM.read(eeaddress);
      //Serial.print(out_string[i], HEX); Serial.print(" g- ");
    }
  }
  //Serial.print(" get in_string "); Serial.print(len_string); Serial.print(" index "); Serial.print(index); Serial.print(" eeaddr "); Serial.print(eeaddress); Serial.println("  ");
  //printBytes(out_string, len_string, true);

} // end get_srting_array_from_eeprom_an_index

void put_srting_array_from_eeprom_to_index(byte* in_string, byte len_string, int address_array, byte index )
{
  //byte ee_string_array[len_string];
  int eeaddress = address_array;
  eeaddress += index * len_string;
  if (len_string <= 1)
  {
    EEPROM.write(eeaddress, in_string);
    //Serial.print(in_string[0], HEX); Serial.print(" p-0- ");
  }
  else
  {
    for (byte i = 0; i < len_string; i++)
    {
      eeaddress += i;
      EEPROM.write(eeaddress, in_string[i]);
      // Serial.print(in_string[i], HEX); Serial.print(" p- ");
    }
  }

  Serial.print(" put in_string "); Serial.print(len_string); Serial.print(" index "); Serial.print(index); Serial.print(" eeaddr "); Serial.print(eeaddress); Serial.println("  ");
  printBytes(in_string, len_string, true);
} // end put_srting_array_from_eeprom_to_index

byte set_byte_array_from_eeprom_an_index(int address_array, byte index )
{
  int eeaddress = address_array;
  eeaddress += index;

  byte out_byte = EEPROM.read(eeaddress);
  //Serial.print(out_byte, HEX); Serial.print(" gb-0- ");
  return out_byte;

} // end get_byte_array_from_eeprom_an_index

void set_byte_array_from_eeprom_an_index(byte in_byte, int address_array, byte index )
{
  int eeaddress = address_array;
  eeaddress += index;

  EEPROM.write(eeaddress, in_byte);
  //Serial.print(in_byte, HEX); Serial.print(" sb-0- ");
} // end set_byte_array_from_eeprom_an_index

byte get_ee_index_an_string_array (byte* in_string, byte len_string, int address_array )
{
  byte index = 0xFF; //no search
  byte eearray[len_string];

  //Serial.print(" in_string ");Serial.print(len_string);Serial.print("  "); Serial.print(count_box);Serial.print("  ");
  //printBytes(in_string, len_string, true);

  boolean yes = false;
  for (byte i = 0; i < count_box; i++)
  {
    get_srting_array_from_eeprom_an_index(eearray, len_string, address_array, i);
    //Serial.print(" eearray ");
    //printBytes(eearray, len_string, true);

    for (byte j = 0; j < len_string; j++)
    {
      if (in_string[j] != eearray[j])
      {
        yes = false;
        // Serial.print(" no "); Serial.print(i); Serial.print(" "); Serial.print(j); Serial.print(" "); Serial.print(in_string[j], HEX); Serial.print(" - "); Serial.println(eearray[j], HEX);
        break;
      }
      else
      {
        yes = true;
        // Serial.print(" yes "); Serial.print(i); Serial.print(" "); Serial.print(j); Serial.print(" "); Serial.print(in_string[j], HEX); Serial.print(" - "); Serial.println(eearray[j], HEX);
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
} // get_ee_index_an_string_array

void test_ee_count(byte count)
{
  int eeAddress = ADDRESS_COUNT_BOX;
  byte eeValue;

  count_box = count;
  eeValue = EEPROM.read(eeAddress);

  if (count != eeValue)
  {
    EEPROM.write(eeAddress, count);
    //Serial.println(" Not eq count "); Serial.print(eeValue, HEX); Serial.print(count, HEX);
  }
  //  Serial.println(" Not eq count "); Serial.print(eeValue, HEX); Serial.print(count, HEX);
} // test_count


void eeprom_test()
{
  byte addr[LEN_SERIAL_NUMBER] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  byte barcod[LEN_ID_PACKET] = { 0, 0, 0, 0, 0, 0, 0, 0 , 0, 0, 0, 0, 0, 0}; ;

  byte bc[LEN_ID_PACKET] = "RB298973247SG";
  byte index = 0;
  byte status_unit = 0xC4;
  put_srting_array_from_eeprom_to_index(bc, LEN_ID_PACKET, ADDRESS_ID_PACKET_ARRAY, index );
  set_byte_array_from_eeprom_an_index(status_unit, ADDRESS_STATUS_UNIT_ARRAY, index );
  //put_srting_array_from_eeprom_to_index(bc, LEN_PASSWD, ADDRESS_PASSWD_ARRAY, index );

  byte bc1[LEN_ID_PACKET] = "RB299157466SG";
  put_srting_array_from_eeprom_to_index(bc1, LEN_ID_PACKET, ADDRESS_ID_PACKET_ARRAY, index + 1 );
  set_byte_array_from_eeprom_an_index(status_unit + 1, ADDRESS_STATUS_UNIT_ARRAY, index + 1 );

  //delay(100);

  for (byte i = 0; i < count_box; i++)
  {

    //Serial.println(" rec "); Serial.println(count_box, HEX); Serial.println(i, HEX);
    get_srting_array_from_eeprom_an_index(addr, LEN_SERIAL_NUMBER, ADDRESS_SERIAL_NUMBER_ARRAY, i );
    printBytes(addr, LEN_SERIAL_NUMBER, true);

    get_srting_array_from_eeprom_an_index(barcod, LEN_ID_PACKET, ADDRESS_ID_PACKET_ARRAY, i );
    printBytes(barcod, LEN_ID_PACKET, true);

    byte StatusUnit = set_byte_array_from_eeprom_an_index( ADDRESS_STATUS_UNIT_ARRAY, i );
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
      for (byte j = 0; j < LEN_SERIAL_NUMBER; j++)
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

  return i;
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

