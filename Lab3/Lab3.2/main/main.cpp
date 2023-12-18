#include <stdio.h>
#include "DFRobot_LCD.h"


extern "C" void app_main(void)
{
	DFRobot_LCD lcd(16,2);
	
	lcd.init();
	while(true) {
		
		lcd.setRGB(0,0,255);
		lcd.setCursor(0,0);
		lcd.printstr("Hello CSE121!");
		lcd.setCursor(0,1);
		lcd.printstr("Duo");
	}


}
