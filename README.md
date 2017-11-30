#Smart alarm on STM32 with RTC and...

## info:
* https://datasheets.maximintegrated.com/en/ds/DS3231.pdf __page 11__ -> pin adresses

# RTC
## pins:
* SCL -> PB6
* SDA -> PB7
## code:
main.c in ```int main(void)```; ```while(1)``` loop: 
```
HAL_I2C_Mem_Read(&hi2c1, 0x68*2, 0, 1, xBuffer, 4, 500);
printf("weekday: %i\n", weekday);
printf("%i%i : %i%i : %i%i\n\n", dhrs, hrs, dmin, min, dsec, sec);
continue;
```
usage of data:

``` weekday    ```  weekday in range 1-7

``` dhrs, hrs  ```  actual hour = dhrs*10 + hrs

``` dmin, min  ```  actual minutes = dmin*10 + min

``` dsec, sec  ```  actual seconds = dsec*10 + sec

if you are really interested, these lines contain actual addresses in RTC:
```
sec       ==	xBuffer[0]&0xF
dsec      ==	(xBuffer[0]&(0xF<<4))>>4
min       ==	xBuffer[1]&0xF
dmin      == 	(xBuffer[1]&(0xF<<4))>>4
hrs       == 	xBuffer[2]&0xF
dhrs      == 	(xBuffer[2]&(0xF<<4))>>4
weekday   ==    xBuffer[3]&0xF
```
