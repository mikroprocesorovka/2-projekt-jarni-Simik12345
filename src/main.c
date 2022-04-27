#include "stm8s.h"
#include "milis.h"
#include "stm8_hd44780.h"
#include "stdio.h"
#include "delay.h"
#include "spse_stm8.h"



void init_enc(void);
void process_enc(void);
void init_timer(void);
void display_time(void);


void ADC_init(void);

void init_spi(void);
void test(uint8_t* data, uint16_t delka);

#define pocet_LED 24 

// test pattern for (8 RGB LED ring) - values in lowest brightness
uint8_t colors[pocet_LED*2]={
	0x01,0x00,0x00, // B
	0x00,0x01,0x00, // R
	0x00,0x00,0x01, // G
	0x01,0x01,0x01, //
	0x01,0x01,0x01, //
	0x01,0x01,0x01, //
	0x01,0x01,0x01, //
	0x01,0x01,0x01, //
};
uint8_t* colors2;
uint32_t posun_posledne=0;
uint32_t refresh_posledne=0;
float brightness = 1;
uint16_t posun = 0;
//colors2 = colors + 3;
#define L_PATTERN 0b01110000 // 3x125ns (8MHZ SPI)
#define H_PATTERN 0b01111100 // 5x125ns (8MHZ SPI), first and last bit must be zero (to remain MOSI in Low between frames/bits)

#define readA   GPIO_ReadInputPin(GPIOC,GPIO_PIN_4)//definování pinů enkodéru
#define readB 	GPIO_ReadInputPin(GPIOD,GPIO_PIN_3)
#define readC   GPIO_ReadInputPin(GPIOE,GPIO_PIN_5)

#define refresh_rate	50	//ms; mezičas obnovení jasu

char text[24];				//promenna pro zapis

uint16_t minule=7;  		//pomocná proměnná pro pozici enkodéru
uint16_t pause_minule=0;	//pomocná proměnná pro tlačítko
uint16_t ADC_value = 0;

uint16_t x=0;                //režim
uint16_t jas_min=50;
uint16_t jas_max=100;
uint16_t posun_rychlost=100;
uint16_t intensity=10;
uint16_t start_intensity=1;
uint16_t below_TH=0;
const uint8_t symbol1[8]={  //symbol
    0b00100,
    0b01010,
    0b10001,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000
};
void main(void){
CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1); // 16MHz z interního RC oscilátoru
GPIO_Init(GPIOE,GPIO_PIN_4,GPIO_MODE_IN_FL_NO_IT);	//tlačítko na kitu
init_milis();	// milis kvuli delay_ms()
init_enc();     // inicializace vstupu enkodéru
lcd_init();     // inicializace displeje
init_spi();		//inicializace SPI pro LED
lcd_store_symbol(0,symbol1); //uložit symbol
init_timer();   // spustí tim3 s poerušením každé 2ms
ADC_init();
enableInterrupts(); // není nutné, protože tuto funkci voláme v init_milis()

for(uint8_t i=0;i<(pocet_LED);i++){     //duplikovat array pro jednodušší posun
                colors[i+pocet_LED]=(colors[i]);
}
lcd_clear();    //reset displeje
ADC_value = ADC_get(ADC2_CHANNEL_2); // do adc_value ulož výsledek převodu vstupu ADC_IN2 (PB2)
display_time();

    while (1){
		if (GPIO_ReadInputPin(GPIOE,GPIO_PIN_4)==RESET){	//reset displeje protože blbnou kontakty
			lcd_init();
		}
        if(milis()-refresh_posledne>=refresh_rate){
            ADC_value = ADC_get(ADC2_CHANNEL_2); // do adc_value ulož výsledek převodu vstupu ADC_IN2 (PB2)
            refresh_posledne=milis();
            if(ADC_value >= jas_min*10 && ADC_value < jas_max*10){
                brightness = (float)((ADC_value-(jas_min*10))*intensity/(1024) + start_intensity);
            }
            if(ADC_value < jas_min*10){
				if(below_TH==0){
					brightness = (float)0;
				}
				if(below_TH==1){
					brightness = (float)(intensity/(1024) + start_intensity);
				}
            }
			if(ADC_value >= jas_max*10){
				brightness = (float)(((jas_max*10)-(jas_min*10))*intensity/(1024) + start_intensity);
			}
            for(uint8_t i=0;i<(pocet_LED);i++){     //posunutí
                colors2[i]=(int)(colors[i+3*posun]*brightness);
            }
            test(colors2,pocet_LED);
        }
        if((milis()-posun_posledne)>=posun_rychlost){
            posun++;
            if(posun>=(pocet_LED/3)){
                posun = 0;
            }
			for(uint8_t i=0;i<(pocet_LED);i++){     //posunutí
                colors2[i]=(int)(colors[i+3*posun]*brightness);
            }
            posun_posledne=milis();
			test(colors2,pocet_LED);
        }
    }
}

 INTERRUPT_HANDLER(TIM3_UPD_OVF_BRK_IRQHandler, 15) //interrupt pro vstupy na ekodéru
 {
     TIM3_ClearITPendingBit(TIM3_IT_UPDATE);
     process_enc();
 }
 
// takes array of LED_number * 3 bytes (RGB per LED)
void test(uint8_t* data, uint16_t length){
    uint8_t mask;
    disableInterrupts(); // can be omitted if interrupts do not take more then about ~25us
    while(length){   // for all bytes from input array
    length--;
    mask=0b10000000; // for all bits in byte
    while(mask){
    while(!(SPI->SR & SPI_SR_TXE)); // wait for empty SPI buffer
    if(mask & data[length]){ // send pulse with coresponding length ("L" od "H")
        SPI->DR = H_PATTERN;
    }else{
        SPI->DR = L_PATTERN;
    }
    mask = mask >> 1;
    }
    }
    enableInterrupts();
    while(SPI->SR & SPI_SR_BSY); // wait until end of transfer - there should come "reset" (>50us in Low)
}
void init_spi(void){
    // Software slave managment (disable CS/SS input), BiDirectional-Mode release MISO pin to general purpose
    SPI->CR2 |= SPI_CR2_SSM | SPI_CR2_SSI | SPI_CR2_BDM | SPI_CR2_BDOE; 
    SPI->CR1 |= SPI_CR1_SPE | SPI_CR1_MSTR; // Enable SPI as master at maximum speed (F_MCU/2, there 16/2=8MHz)
}
void ADC_init(void){
    // na pinech/vstupech ADC_IN2 (PB2) a ADC_IN3 (PB3) vypneme vstupní buffer
    ADC2_SchmittTriggerConfig(ADC2_SCHMITTTRIG_CHANNEL2,DISABLE);
    ADC2_SchmittTriggerConfig(ADC2_SCHMITTTRIG_CHANNEL3,DISABLE);
    // nastavíme clock pro ADC (16MHz / 4 = 4MHz)
    ADC2_PrescalerConfig(ADC2_PRESSEL_FCPU_D4);
    // volíme zarovnání výsledku (typicky vpravo, jen vyjmečně je výhodné vlevo)
    ADC2_AlignConfig(ADC2_ALIGN_RIGHT);
    // nasatvíme multiplexer na některý ze vstupních kanálů
    ADC2_Select_Channel(ADC2_CHANNEL_2);
    // rozběhneme AD převodník
    ADC2_Cmd(ENABLE);
	// počkáme než se AD převodník rozběhne (~7us)
    ADC2_Startup_Wait();
}
void init_timer(void){  //zapnout interrupt
    TIM3_TimeBaseInit(TIM3_PRESCALER_16,1999);
    TIM3_ITConfig(TIM3_IT_UPDATE, ENABLE);
    TIM3_Cmd(ENABLE);
}
void init_enc(void){    //povolit piny pro enkodér
    GPIO_Init(GPIOE,GPIO_PIN_5,GPIO_MODE_IN_PU_NO_IT); //enkodér
    GPIO_Init(GPIOC,GPIO_PIN_6,GPIO_MODE_IN_PU_NO_IT); //enkodér
    GPIO_Init(GPIOC,GPIO_PIN_7,GPIO_MODE_IN_PU_NO_IT); //tlačítko enkodéru
}
void process_enc(void){ //enkodér
    if(readA == RESET && readB == RESET){   //detekce rozepnutého stavu
        minule=0;
    }
    if(readA != RESET && readB != RESET){   //detekce sepnutého stavu
        minule=1;
    }
    if((minule==1 && readA!=RESET && readB == RESET) ||(minule==0 && readA==RESET && readB != RESET)){  //logika pro otáčení clockwise
        minule=7;   //zapsat posunutí, ochrana proti opakování funkce
        if(x==0){
			posun_rychlost++;
			
		}
		if(x==1){
			if(jas_min<100 && jas_min<jas_max){
				jas_min++;
			}
		}
		if(x==2){
			if(jas_max<100){
				jas_max++;
			}
		}
		if(x==3){
			intensity++;
		}
		if(x==4){
			start_intensity++;
		}
		if(x==5){
			below_TH++;
			if(below_TH>1){
				below_TH=0;
			}
		}
		display_time(); //přepsat cifry
    }
    if((minule==1 && readA==RESET && readB != RESET)||(minule==0 && readA!=RESET && readB == RESET)){   //logika pro otáčení counterclockwise
        minule=7;   //zapsat posunutí, ochrana proti opakování funkce
		if(x==0){
			if(posun_rychlost>0){
				posun_rychlost--;
			}
			
		}
		if(x==1){
			if(jas_min>0){
				jas_min--;
			}
		}
		if(x==2){
			if(jas_max>0 && jas_max > jas_min){
				jas_max--;
			}
		}
		if(x==3){
			if(intensity > 0){
				intensity--;
			}
		}
		if(x==4){
			if(start_intensity > 0){
				start_intensity--;
			}
		}
		if(x==5){
			below_TH--;
			if(below_TH>1){
				below_TH=1;
			}
		}
		display_time(); //přepsat cifry
          
    }
    if(readC==RESET && pause_minule==0){    //detekce zmáčknutí enkodéru
        pause_minule=1; //pomocná proměnná aby se neopakovala funkce
		x++;
		if(x>5){
			x=0;
		}
		display_time();
    }
    if(readC!=RESET && pause_minule==1){    //reset pomocné proměnné
        pause_minule=0;
    }
}
void display_time(void){    //přepsat cifry
    disableInterrupts();
    
	//DEBUG VĚCI
	lcd_gotoxy(0,1);
	sprintf(text,"%4i",(int)brightness); // poiprav text na displej
    lcd_puts(text); // vypiš poipravený text
    lcd_gotoxy(5,1);
    sprintf(text,"%4i",ADC_value); // poiprav text na displej
    lcd_puts(text); // vypiš poipravený text
    lcd_gotoxy(11,1);
    sprintf(text,"%i",ADC_value/200); // poiprav text na displej
    lcd_puts(text); // vypiš poipravený text
    lcd_gotoxy(12,1);
    lcd_puts("."); // vypiš poipravený text
    lcd_gotoxy(13,1);
    sprintf(text,"%i",ADC_value/20%10); // poiprav text na displej
    lcd_puts(text); // vypiš poipravený text
    lcd_gotoxy(14,1);
    sprintf(text,"%i",ADC_value/2%10); // poiprav text na displej
    lcd_puts(text); // vypiš poipravený text
    lcd_gotoxy(15,1);
    lcd_puts("V"); // vypiš poipravený text


	lcd_gotoxy(0,0);
	lcd_puts("                ");
	if(x==0){
		lcd_gotoxy(0,0);
    	lcd_puts("Rychlost:"); // vypiš poipravený text
		lcd_gotoxy(10,0);
		sprintf(text,"%4i",posun_rychlost); // poiprav text na displej
		lcd_puts(text); // vypiš poipravený text
		lcd_gotoxy(14,0);
    	lcd_puts("ms"); // vypiš poipravený text
	}
	if(x==1){
		lcd_gotoxy(0,0);
    	lcd_puts("Min TH.:"); // vypiš poipravený text
		lcd_gotoxy(12,0);
		sprintf(text,"%3i",jas_min); // poiprav text na displej
		lcd_puts(text); // vypiš poipravený text
		lcd_gotoxy(15,0);
    	lcd_puts("%"); // vypiš poipravený text
	}
	if(x==2){
		lcd_gotoxy(0,0);
    	lcd_puts("Max TH.:"); // vypiš poipravený text
		lcd_gotoxy(12,0);
		sprintf(text,"%3i",jas_max); // poiprav text na displej
		lcd_puts(text); // vypiš poipravený text
		lcd_gotoxy(15,0);
    	lcd_puts("%"); // vypiš poipravený text
	}
	if(x==3){
		lcd_gotoxy(0,0);
    	lcd_puts("Intensity:"); // vypiš poipravený text
		lcd_gotoxy(13,0);
		sprintf(text,"%3i",intensity); // poiprav text na displej
		lcd_puts(text); // vypiš poipravený text
	}
	if(x==4){
		lcd_gotoxy(0,0);
    	lcd_puts("Start Int.:"); // vypiš poipravený text
		lcd_gotoxy(13,0);
		sprintf(text,"%3i",start_intensity); // poiprav text na displej
		lcd_puts(text); // vypiš poipravený text
	}
	if(x==5){
		lcd_gotoxy(0,0);
    	lcd_puts("Pod min.TH:"); // vypiš poipravený text
		if(below_TH==0){
			lcd_gotoxy(13,0);
			lcd_puts("off");
		}
		if(below_TH==1){
			lcd_gotoxy(14,0);
			lcd_puts("on");
		}
	}

    enableInterrupts();
}
#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/