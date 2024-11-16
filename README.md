#### 1)-Overview : 
Simple system to display time/date & temperature in a circular manner.
#### 2)-Components : 
- PIC18F452
- LM35
- MAX6902
#### 3)-Working Concept :
i'm using LM35 analog temperature sensor and using spi protocol to fetch time/date from MAX6902 RTC  to display the time/date & temperature in a circular manner
i have used a timer_0 where each overflows triggers an interrupts that dispalys either time/date or temperature based on the state of the flag.
