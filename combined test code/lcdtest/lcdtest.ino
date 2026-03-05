/*****************************************
 * name: I2C LCD2004
 * function: You should now see your I2C LCD2004 display "Hello,world!","IIC/I2C LCD2004"
 * "20 cols, 4 rows","www.sunfounder.com"
 ********************************/
//Email:service@sunfounder.com
//Website:www.sunfounder.com

/********************************/
// Include necessary libraries
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Initialize the LCD object, set the LCD I2C address to 0x27 for a 20x4 display
LiquidCrystal_I2C lcd(0x27, 20, 4);  

/*********************************************************/
void setup()
{
  lcd.init();              // Initialize the LCD
  lcd.backlight();         // Turn on the backlight
  
  // Set cursor to the top left corner and print the string on the first row
  lcd.setCursor(0, 0);     
  lcd.print("    Hello, world!    "); 
  
  // Move to the second row and print the string
  lcd.setCursor(0, 1);     
  lcd.print("   IIC/I2C LCD2004  "); 
  
  // Move to the third row and print the string
  lcd.setCursor(0, 2);     
  lcd.print("  20 cols, 4 rows   "); 
  
  // Move to the fourth row and print the string
  lcd.setCursor(0, 3);     
  lcd.print(" www.sunfounder.com ");
}
/*********************************************************/
void loop() 
{
  // Empty loop
}
/************************************************************/
