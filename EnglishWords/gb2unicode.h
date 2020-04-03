#ifndef GB2UNICODE_H
#define GB2UNICODE_H

int strUnicode2GB(const unsigned char *strSourcer, unsigned char *strDest, int n);
int strGB2Unicode(const unsigned char *str, unsigned char *dst, int n);
int enc_utf8_to_unicode(char *utf8, uint8_t* unicode, uint16_t utf8_len);
int utf8StrToGB2312(
  char *gbStr,        /* Output GB2312 chars */
  char* utf8Str,        /* Input Utf-8 chars */
  uint8_t nBytes            /* Size of input Utf-8 chars */
);
#endif
