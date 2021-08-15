/*
 * LCD driver for the ILI9225 TFT LCD chips. 
 * 
 * This driver works with OTM2201A, and ILI9926 controller chips.
 * 
 * Feel free to use, change, distribute this code as desired. Use under
 * GPLv3 open-source license.
 * 
 * File:   ILI9225.c
 * Author: tommy
 *
 * Created on 2 July 2019, 7:32 PM
 */


#include "ILI9225.h"


#define ABS(x)   ((x) > 0 ? (x) : -(x))


/*
 * Writes a byte to SPI without changing chip select (CSX) state.
 * Called by the write_command() and write_data() functions which
 * control these pins as required.
 */
void spi_write(uint8_t* data, uint32_t size) {


	HAL_SPI_Transmit_DMA(&hspi2, data, size);
}

/*
 * Writes a data byte to the display. Pulls CS low as required.
 */
void lcd_write_data(uint8_t* data, uint32_t size) {

    HAL_GPIO_WritePin(CMD_PORT, CMD_PIN, GPIO_PIN_SET);



    while(size > 0) {

    	HAL_GPIO_WritePin(CSX_PORT, CSX_PIN, GPIO_PIN_RESET);
    	uint16_t chunk_size = size > 32768 ? 32768 : size;
            spi_write(data, chunk_size);

            data += chunk_size;
            size -= chunk_size;
        }

}

/*
 * Writes a command byte to the display
 */
void lcd_write_command(uint8_t data) {
    //Pull the command AND chip select lines LOW
    HAL_GPIO_WritePin(CMD_PORT, CMD_PIN, GPIO_PIN_RESET);
    //CSX = 0;
    HAL_GPIO_WritePin(CSX_PORT, CSX_PIN, GPIO_PIN_RESET);
    spi_write(&data, 1);
    //Return the control lines to HIGH

}

/*
 * Writes data to a specific register.
 * Accepts a 16-bit register address, and 16-bits of data.
 */
void lcd_write_register(unsigned int reg, unsigned int data) {
    //Write each register byte, and each data byte seperately.

    lcd_write_command(reg >> 8); //regH
    lcd_write_command(reg & 0xFF); //regL
    uint8_t data_buff[] = {data >> 8, data & 0xFF};
    lcd_write_data(data_buff, sizeof(data_buff));

}

/*
 * Swaps two 16-bit integers
 */
void swap_int(int *num1, int *num2) {
    int temp = *num2;
    *num2 = *num1;
    *num1 = temp;
}

/*
 * Swaps two 8-bit integers
 */
void swap_char(uint8_t *num1, uint8_t *num2) {
    char temp = *num2;
    *num2 = *num1;
    *num1 = temp;
}


/*
 * Initialisation routine for the LCD
 * I got this from the one of the ebay sellers which make them.
 * From Open-Smart
 */
void lcd_init() {
    
    //SET control pins for the LCD HIGH (they are active LOW)
    HAL_GPIO_WritePin(CSX_PORT, CSX_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CMD_PORT, CMD_PIN, GPIO_PIN_RESET); //Data / command select, the datasheet isn't clear on that.
    HAL_GPIO_WritePin(RESX_PORT, RESX_PIN, GPIO_PIN_SET); //RESET pin HIGH
    
    //Cycle reset pin
    HAL_GPIO_WritePin(RESX_PORT, RESX_PIN, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(RESX_PORT, RESX_PIN, GPIO_PIN_SET);
    HAL_Delay(500);
    
    lcd_init_command_list();
    
}

/**
 * This is the magic initialisation routine. Supplied by Open-Smart
 * who sell cheap modules on eBay.
 * This routine works with OTM2201A and ILI9225.
 */
void lcd_init_command_list(void)
{
    
    lcd_write_register(ILI9225_POWER_CTRL1, 0x0000); // Set SAP,DSTB,STB
    lcd_write_register(ILI9225_POWER_CTRL2, 0x0000); // Set APON,PON,AON,VCI1EN,VC
    lcd_write_register(ILI9225_POWER_CTRL3, 0x0000); // Set BT,DC1,DC2,DC3
    lcd_write_register(ILI9225_POWER_CTRL4, 0x0000); // Set GVDD
    lcd_write_register(ILI9225_POWER_CTRL5, 0x0000); // Set VCOMH/VCOML voltage
    
    HAL_Delay(10);
    
    lcd_write_register(ILI9225_POWER_CTRL2, 0xFFFF); // EVERYTHING ON
    lcd_write_register(ILI9225_POWER_CTRL3, 0x7000); // Set BT,DC1,DC2,DC3
    lcd_write_register(ILI9225_POWER_CTRL4, 0x006F); // Set GVDD   /*007F 0088 */
    lcd_write_register(ILI9225_POWER_CTRL5, 0x495F); // Set VCOMH/VCOML voltage
    lcd_write_register(ILI9225_POWER_CTRL1, 0x0F00); // Set SAP,DSTB,STB
    
    HAL_Delay(10);
    

    lcd_write_register(ILI9225_POWER_CTRL2, 0xFFFF); // Set APON,PON,AON,VCI1EN,VC

    HAL_Delay(50);


    lcd_write_register(ILI9225_DRIVER_OUTPUT_CTRL, 0x011C); // set the display line number and display direction
    lcd_write_register(ILI9225_LCD_AC_DRIVING_CTRL, 0x0100); // set 1 line inversion
    lcd_write_register(ILI9225_ENTRY_MODE, 0x1038); // set GRAM write direction and BGR=1. 0x1030
    lcd_write_register(ILI9225_DISP_CTRL1, 0x0000); // Display off
    lcd_write_register(ILI9225_BLANK_PERIOD_CTRL1, 0x0202); // set the back porch and front porch (2 lines, minimum)
    lcd_write_register(ILI9225_FRAME_CYCLE_CTRL, 0x0000); // set the clocks number per line
    lcd_write_register(ILI9225_INTERFACE_CTRL, 0x0000); // CPU interface
    lcd_write_register(ILI9225_OSC_CTRL, 0x0F01); // Set Osc
    lcd_write_register(ILI9225_VCI_RECYCLING, 0x0000); // Set VCI recycling
    lcd_write_register(ILI9225_RAM_ADDR_SET1, 0xDB00); // RAM Address
    lcd_write_register(ILI9225_RAM_ADDR_SET2, 0x00AF); // RAM Address

    /* Set GRAM area */
    lcd_write_register(ILI9225_GATE_SCAN_CTRL, 0x0000); 
    lcd_write_register(ILI9225_VERTICAL_SCROLL_CTRL1, 0x00DB);
    lcd_write_register(ILI9225_VERTICAL_SCROLL_CTRL2, 0x0000);
    lcd_write_register(ILI9225_VERTICAL_SCROLL_CTRL3, 0x0000);
    lcd_write_register(ILI9225_PARTIAL_DRIVING_POS1, 0x00DB); //0x0000
    lcd_write_register(ILI9225_PARTIAL_DRIVING_POS2, 0x0000);
    lcd_write_register(ILI9225_HORIZONTAL_WINDOW_ADDR1, 0x00AF);// 0x00AF
    lcd_write_register(ILI9225_HORIZONTAL_WINDOW_ADDR2, 0x0000); //0x0000
    lcd_write_register(ILI9225_VERTICAL_WINDOW_ADDR1, 0x00DB); //0x00DB
    lcd_write_register(ILI9225_VERTICAL_WINDOW_ADDR2, 0x0000);

    /* Set GAMMA curve */
    lcd_write_register(ILI9225_GAMMA_CTRL1, 0x0000); 
    lcd_write_register(ILI9225_GAMMA_CTRL2, 0x0808); 
    lcd_write_register(ILI9225_GAMMA_CTRL3, 0x080A); 
    lcd_write_register(ILI9225_GAMMA_CTRL4, 0x000A); 
    lcd_write_register(ILI9225_GAMMA_CTRL5, 0x0A08); 
    lcd_write_register(ILI9225_GAMMA_CTRL6, 0x0808); 
    lcd_write_register(ILI9225_GAMMA_CTRL7, 0x0000); 
    lcd_write_register(ILI9225_GAMMA_CTRL8, 0x0A00); 
    lcd_write_register(ILI9225_GAMMA_CTRL9, 0x0710); 
    lcd_write_register(ILI9225_GAMMA_CTRL10, 0x0710); 

    lcd_write_register(ILI9225_DISP_CTRL1, 0x0012); 

    HAL_Delay(50);
    
    lcd_write_register(ILI9225_DISP_CTRL1, 0x1017);

    
}

/*
 * Draws a single pixel to the LCD at position X, Y, with 
 * Colour.
 * 
 * 28 bytes per pixel. Use it wisely.
 */
void draw_pixel(char x, char y, unsigned int colour) {

    
    //Set the x, y position that we want to write to
    set_draw_window(x, y, x+1, y+1);
    uint8_t data[] = {colour >> 8, colour & 0xFF};
    lcd_write_data(data, sizeof(data));

}

/*
 * Fills a rectangle with a given colour
 */
void fill_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, unsigned int colour) {
	uint16_t delta_x, delta_y;
	delta_x = ABS(x2 - x1);
	delta_y = ABS(y2 - y1);


    
    HAL_GPIO_WritePin(CSX_PORT, CSX_PIN, GPIO_PIN_RESET);
    set_draw_window(x1, y1, x2, y2);

    uint32_t data_size = delta_x * delta_y;
    //TODO: если при подключении какой то новой периферии контроллер начинает зависать - увеличить делитель в SQUARE_QUARTER
    const uint16_t SQUARE_QUARTER = (WIDTH) * (HEIGHT) / 16;
    uint16_t data_color_size = SQUARE_QUARTER;
    
    uint16_t data_color[SQUARE_QUARTER];
    
	for (uint16_t i = 0; i < data_color_size; i++) {
		data_color[i] = ((colour & 0xFF00) >> 8) | ((colour & 0xFF) << 8);
	}
	while (data_size > 0)
	{
			// делим заливку массива data_color на порции chunk_size размером не более четвертинки
			// логика: chunk_size равен прямоугольнику data_size, если он меньше четвертинки, а
			// если прямоугольник больше четвертинки, то chunk_size равен четвертинке
			// при следующем проходе цикла четвертинка из заливаемого массива вычитается

			uint16_t chunk_size = data_size > SQUARE_QUARTER ? SQUARE_QUARTER : data_size;

			lcd_write_data((uint8_t*)data_color, 2 * chunk_size);

			data_color_size += chunk_size;
			data_size -= chunk_size;
	}

}

/*
 * Sets the X,Y position for following commands on the display.
 * Should only be called within a function that draws something
 * to the display.
 * 
 * NOTE: This is 26 bytes. Use it sparingly (see draw_bitmap())
 */
void set_draw_window(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2) {
    
    //Check that the values are in order
    if(x2 < x1)
        swap_char(&x2, &x1);
    if(y2 < y1)
        swap_char(&y2, &y1);

    lcd_write_register(ILI9225_HORIZONTAL_WINDOW_ADDR1,y2);//x2
    lcd_write_register(ILI9225_HORIZONTAL_WINDOW_ADDR2,y1);//x1

    lcd_write_register(ILI9225_VERTICAL_WINDOW_ADDR1,x2);//y2
    lcd_write_register(ILI9225_VERTICAL_WINDOW_ADDR2,x1);//y1

    lcd_write_register(ILI9225_RAM_ADDR_SET1,y1);//x1
    lcd_write_register(ILI9225_RAM_ADDR_SET2,x1);//y1

    lcd_write_command(0x00);
    lcd_write_command(0x22);
}

/*
 * Draws a single char to the screen.
 * Called by the various string writing functions like print().
 * 
 * NOTE:
 * This sends approx. 800 bytes per char to the LCD, but it does preserver
 * the background image. Use the draw_fast_char() function where possible.
 */
void draw_char(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor)
{
	uint32_t i, b, j;

	    set_draw_window(x, y, x+font.width-1, y+font.height-1);

	    for(i = 0; i < font.height; i++)
	    {
	        b = font.data[(ch - 32) * font.height + (font.height - i - 1)];
	        for(j = 0; j < font.width; j++)
	        {
	            if((b << j) & 0x8000)
	            {
	                uint8_t data[] = { color >> 8, color & 0xFF };
	                lcd_write_data(data, sizeof(data));
	            } else
	            {
	                uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
	                lcd_write_data(data, sizeof(data));
	            }
	        }
	    }
}



/*
 * Writes a string to the display as an array of chars at position x, y with 
 * a given colour and size.
 */
void draw_string(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor)
{
    
	while(*str)
	{
	        if(x + font.width >= WIDTH)
	        {
	            x = 0;
	            y -= font.height;
	            if(y - font.height >= HEIGHT)
	            {
	                break;
	            }

	            if(*str == ' ')
	            {
	                // skip spaces in the beginning of the new line
	                str++;
	                continue;
	            }
	        }

	        draw_char(x, y, *str, font, color, bgcolor);
	        x += font.width;
	        str++;
	    }

}

/*
 * Draws a bitmap by directly writing the byte stream to the LCD.
 * 
 * So the scaling is done strangely here because writing individual pixels 
 * has an overhead of 26 bytes each.
 */
void draw_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data)
{
    if((x >= WIDTH) || (y >= HEIGHT)) return;
    if((x + w - 1) >= WIDTH) return;
    if((y + h - 1) >= HEIGHT) return;

//    ILI9341_Select();
    set_draw_window(x, y, x+w-1, y+h-1);
    lcd_write_data((uint8_t*) data, sizeof(uint16_t)*w*h);
}
