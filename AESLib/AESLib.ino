#include <AESLib.h>
/*
  void aes128_enc_single(const uint8_t* key, void* data);
  void aes128_dec_single(const uint8_t* key, void* data);
*/

uint8_t key[] = {55, 129, 2, 45, 4, 115, 6, 7, 211, 9, 10, 11, 192, 13, 14, 15};
char passwd[7] = "123456" ;
String pass_wd;

/********************************************************/
void printBytes(uint8_t* addr, uint8_t count, bool newline = 0)
{
  for (uint8_t i = 0; i < count; i++)
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

void set_passwd(uint8_t* data, uint8_t* pass)
{
  uint8_t j = 0; // counter summ byte
  uint8_t i = 0; //counter main
  uint8_t k = 0; // counter pass
  uint8_t p = 0; // pointer
  boolean f_and = false; // and or
  boolean f_rol = false; // rol ror
  uint8_t data_byte;
  uint32_t pass_num = 0;

  p = key[0] % 16;
  if (key[0] > 128)
  {
    f_rol = true;
  }
  if (key[1] > 128)
  {
    f_and = true;
  }

 // Serial.println(p, HEX);
  //   Serial.print(" ");

  while ( i < 16 )
  {
    //delay(100);

    data_byte = data[p];
    /*
    Serial.print(data_byte, HEX);
    Serial.print(" key ");
    Serial.print(key[i], HEX);
    Serial.print(" ");
    Serial.print(data_byte, HEX);
    Serial.print("  ");
    Serial.print(data_byte, HEX);
    Serial.print("  ");
    Serial.print(" i - ");
    Serial.print(i, HEX);
    Serial.print(" p - ");
    Serial.println(p, HEX);
    */
    if (f_and)
    {
      data_byte = data_byte | key[i];
    }
    else
    {
      data_byte = data_byte & key[i];
    }
    data_byte = data_byte ^ key[i];

    pass_num = pass_num + data_byte;

    if (!f_rol)
    {
  //    Serial.print(" ror - ");
      p++;
      if (p >= 16)
      {
        p = 0;
      }
    }
    else
    {
  //    Serial.print(" rol - ");
      p--;
   //   Serial.print(" p - ");
    //  Serial.println(p, HEX);
      if (p == 255)
      {
        p = 15;
      }
   //   Serial.print(" p - ");
   //   Serial.println(p, HEX);
    }

    i++;
    j++;

    //   Serial.print(pass_num);
    //   Serial.print(" ");

    if ( j >= 3 )
    {
      pass[k] = char(48 + pass_num % 9);
      k++;
      pass_num = 0;
      j = 0;
    }
  }
  pass[k] = 48 + pass_num % 9;
}



unsigned long calk_password (uint8_t* ID_box)
{
  /*
    record_type record;
    int eeAddress = 0;
    uint8_t sbyte; //secret byte
    uint8_t mb; //master 4 bit byte
    uint8_t sb; //slave 4 bite bite
    uint8_t adr = 0; //addres secret
    uint8_t pb; // pas byte
    uint8_t passwd[6];
    eeAddress = eeAddress + ID_box * len_eeAddress;
    Serial.print(eeAddress);
    Serial.print(" id ");
    //EEPROM.get(eeAddress, record);
    //byte ID_paket = record.ID_paket[14];
    uint8_t ID_paket[14] = "LF202929536CN";
    uint8_t ID[14];
    ID[14] = ID_paket[14];

    //Serial.println(ID_paket);
    printBytes(ID_paket, 14);
    for (int i = 0; i <= 5; i++)
    {
    sbyte = secret[adr];
    Serial.print(" sbyte ");
    Serial.println(sbyte);
    Serial.print(" adr - ");
    Serial.println(adr, HEX);
    mb = sbyte & B00001111;
    sb = sbyte & B11110000;
    adr = ( ID[0] >> mb ) + sb;
    pb = sb % 9;
    Serial.print(" adr + ");
    Serial.println(adr, HEX);

    Serial.print(" pb ");
    Serial.println(pb, HEX);
    passwd[i] = pb;
    }
    unsigned long passwdint = int(passwd);
    Serial.print(passwdint);
    Serial.println(" pass ");

    return passwd;
  */
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(" ");

  char data[] = "LR234567890123RS"; //16 chars == 16 bytes
  aes128_enc_single(key, data);
  Serial.print("encrypted:");
  Serial.println(data);
  printBytes(data, 16, 1);
  set_passwd(data, passwd);
  printBytes(passwd, 7, 1);
  pass_wd = passwd;
  Serial.print("pass:");
  Serial.println(passwd);
  Serial.println(pass_wd);
  aes128_dec_single(key, data);
  Serial.print("decrypted:");
  Serial.println(data);
  set_passwd(data, passwd);
  printBytes(passwd, 7, 1);
  pass_wd = passwd;
  Serial.print("pass:");
  Serial.println(passwd);
  Serial.println(pass_wd);
}

void loop() {
  // put your main code here, to run repeatedly:


  delay(3000);
}


