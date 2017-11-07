# superSecretProjectNumber29
### https://datasheets.maximintegrated.com/en/ds/DS3231.pdf __page 11__ -> pin adresses

### main.c in ```int main(void)```; ```while(1)``` loop: 
```
printf("weekday: %i%i\n", (xBuffer[3]&(0xF<<4))>>4, xBuffer[3]&0xF);
printf("%i%i : %i%i : %i%i\n\n",
      (xBuffer[2]&(0xF<<4))>>4 ,xBuffer[2]&0xF, 
      (xBuffer[1]&(0xF<<4))>>4,xBuffer[1]&0xF, 
      (xBuffer[0]&(0xF<<4))>>4,xBuffer[0]&0xF);
      continue;
```
