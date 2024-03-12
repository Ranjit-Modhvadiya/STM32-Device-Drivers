#include "stm32f303xe.h"

#define LCD (0xff<<4)
#define RS (1<<1)
#define RW (1<<2)
#define EN (1<<3)

void delay_fv(int x,int y);
void lcd_display(unsigned int x);
void cmd (unsigned int x);
void lcd_ini();
//void lcd_str(unsigned char *x);

int main()
{

char a = 'A';
//unsigned char b[] = {"I am Ranjit"};

RCC->AHBENR = 0x000060014;
GPIOA->MODER = 0xA8555555;
GPIOA->OTYPER = 0x00000000;
GPIOA->OSPEEDR = 0x0C000000;
GPIOA->PUPDR = 0X64000000;

while(1){

lcd_ini();

cmd(0x80);
lcd_display(a);
//lcd_str(a);

//lcd_str(b);
}
}

void delay_fv( int x, int y)
{
int i;
int j;

for(i=0;i<x;i++){
for(j=0;j<y;j++);
}
}

void lcd_display(unsigned int x) //at bit 0 to 7;x= 0x41 = 01000001
{
GPIOA->ODR = 0x00000FF0; //bits 8 to 18 as 00000000
x =(x<<4); //at bit 8 to 15;x= 0x41 = 01000001
GPIOA->ODR = x; // bits 8 to 18 as 01000001
GPIOA->ODR = 0X00000008; //RS=1 for data
delay_fv(100,10);
GPIOA->BRR = 0X00000008;
}
void cmd (unsigned int x)
{
GPIOA->ODR = 0x00000FF0;
x =(x<<8);
GPIOA->ODR = x;
GPIOA->ODR = 0X0000000C;
delay_fv(100,10);
GPIOA->ODR = 0X00000008;
}

void lcd_ini()
{
cmd(0X38);
cmd(0x80);
cmd(0X0E);
cmd(0X01);
cmd(0X06);
}

/*void lcd_str(unsigned char x[])
{
int i;
for(i=0;x[i]!='\0';i++)
{
lcd_display(x[i]);
}
}*/
