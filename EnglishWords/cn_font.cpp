#include "Arduino.h"
#include "cn_font.h"
#include<string.h>

uint8_t get_cn_font_one(const char *str, uint8_t *buff)
{
  uint8_t section_addr, bit_addr, charnum;
  uint32_t offset = 0;
  charnum = 0;
//  Serial.printf("输入0x%x,0x%x", str[0], str[1]);
  if (str[0] != '\0') {
    section_addr = str[0];
    if (section_addr > 0xA0) // chinese
    {
      bit_addr = str[1];
      offset = (section_addr - 0xa1) * 94 + (bit_addr - 0xa1);
//      Serial.printf("偏移:%d,%d", ((section_addr - 0xa1) * 94 + (bit_addr - 0xa1) + 1), offset);
//      Serial.println("");
      memcpy(buff, gb2312Code + offset * 72, 72);
      charnum = 2;
    }
    else if (section_addr < 0xA1) //English
    {
      memcpy(buff, asciiCode + section_addr * 48, 48);
      charnum = 1;
    }
  }
  return charnum;
}
