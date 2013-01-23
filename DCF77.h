
uint8_t dcf77Bits[] = { 1,2,4,8,10,20,40 };
uint8_t minuteBits[] = { 1,2,4,8,10,20,40 };
uint8_t stundeBits[] = { 1,2,4,8,10,20 };
uint8_t kalendertagBits[] = { 1,2,4,8,10,20 };
 uint8_t fehler;


struct time {
  uint8_t sekunde;
  uint8_t minute;
  uint8_t stunde;
  uint8_t kalendertag;
  uint8_t wochentag;
  uint8_t kalendermonat;
  uint8_t kalenderjahr;
};
void init(void);
extern struct time Zeit;
