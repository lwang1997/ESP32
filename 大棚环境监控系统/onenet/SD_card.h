#define SD_CARD_h
#define SD_CARD_h

void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void createDir(fs::FS &fs, const char * path);
void removeDir(fs::FS &fs, const char * path);
void readFile(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void deleteFile(fs::FS &fs, const char * path);
void testFileIO(fs::FS &fs, const char * path);

#define BITMAP_h

//listDir(SD, "/", 0);
//createDir(SD, "/mydir");
//listDir(SD, "/", 0);
//removeDir(SD, "/mydir");
//listDir(SD, "/", 2);
//writeFile(SD, "/hello.txt", "Hello ");
//appendFile(SD, "/hello.txt", "World!\n");
//readFile(SD, "/hello.txt");
//deleteFile(SD, "/foo.txt");
//renameFile(SD, "/hello.txt", "/foo.txt");
//readFile(SD, "/foo.txt");
//testFileIO(SD, "/test.txt");
