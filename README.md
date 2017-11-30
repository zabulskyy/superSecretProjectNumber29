# Smart alarm on STM32 with RTC and...

# RTC

## info:
* https://datasheets.maximintegrated.com/en/ds/DS3231.pdf __page 11__ -> pin adresses


## pins:
* SCL -> PB6
* SDA -> PB7

## read data:
main.c in ```int main(void)```; ```while(1)``` loop: 
```
HAL_I2C_Mem_Read(&hi2c1, 0x68*2, 0, 1, xBuffer, 4, 500);
printf("weekday: %i\n", read_weekday);
printf("%i%i : %i%i : %i%i\n\n", read_dhrs, read_hrs, read_dmin, read_min, read_dsec, read_sec);
continue;
```
Usage of data:

``` read_day    ```  weekday in range 1-7

``` read_dhrs, read_hrs  ```  actual hour = dhrs*10 + hrs

``` read_dmin, read_min  ```  actual minutes = dmin*10 + min

``` read_dsec, read_sec  ```  actual seconds = dsec*10 + sec

If you are really interested, these lines contain actual addresses in RTC:
```
read_sec       ==	xBuffer[0]&0xF
read_dsec      ==	(xBuffer[0]&(0xF<<4))>>4
read_min       ==	xBuffer[1]&0xF
read_dmin      == 	(xBuffer[1]&(0xF<<4))>>4
read_hrs       == 	xBuffer[2]&0xF
read_dhrs      == 	(xBuffer[2]&(0xF<<4))>>4
read_day       ==    xBuffer[3]&0xF
```

## write data
Find these lines:
```
  /*****************/
  /*write here data you want to write to alarm*/
  int alarm_day = 1;  // Monday
  
  int alarm_dhrs = 0;
  int alarm_hrs = 7;
  
  int alarm_dmin = 3;
  int alarm_min = 0;
  /*****************/
```
Documentation is all it says. In this instance alarm will sound at Monday, 7:30
