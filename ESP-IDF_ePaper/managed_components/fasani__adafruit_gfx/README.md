# Adafruit GFX Library
This is my try to port AdafruitGFX to ESP-IDF. 
It was not a light task since is developed to work with Arduino framwork. That's why I had to also add some of that libraries, keeping it to a minimum level but still not the ideal:

	Print.cpp
	Print.h
	Printable.h
	WString.cpp
	WString.h

This is currently used in my project [Cale-idf](https://github.com/martinberlin/cale-idf) and the [ESP-IDF epaper component](https://github.com/martinberlin/CalEPD).
If someone knows C++ OOP better than me maybe is a better way to implement this without this libraries. The good thing is that this is working and might be a good start if you want to use fonts or geometric functions in ESP-IDF like drawCircle, drawPixel, etc.
To implement print / println after extending Adafruit GFX you should add this functions. Eg. in an epaper driver implementation
   // epd.h

    class Epd : public virtual Adafruit_GFX
    {
    public:
    Epd();
     // Should include 
    void drawPixel(int16_t x, int16_t y, uint16_t color);  
    // Extending Print owns write that is a virtual member
    size_t write(uint8_t);
    // Epd print 
    void print(const std::string& text);
    void println(const std::string& text);
    }

    // epd.cpp  Constructor and sample methods implemented
    Epd::Epd():Adafruit_GFX(GxGDEW0213I5F_WIDTH, GxGDEW0213I5F_HEIGHT){
      printf("Epd() constructor extends Adafruit_GFX(%d,%d)\n",
      GxGDEW0213I5F_WIDTH, GxGDEW0213I5F_HEIGHT); 
      // Start your display IO,etc. 
    }

    size_t Epd::write(uint8_t v){
       Adafruit_GFX::write(v);
      return 1;
    }

    void Epd::print(const std::string& text){
      for(auto c : text) {
        write(uint8_t(c));
      }
    }

    void Epd::println(const std::string& text){
      for(auto c : text) {
        write(uint8_t(c));
      }
      write(10); // newline
    }

This is the core graphics library for all our displays, providing a common set of graphics primitives (points, lines, circles, etc.). It needs to be paired with a hardware-specific library for each display device we carry (to handle the lower-level functions).

Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
BSD license, check license.txt for more information.
All text above must be included in any redistribution.

Recent Arduino IDE releases include the Library Manager for easy installation. Otherwise, to download, click the DOWNLOAD ZIP button, uncompress and rename the uncompressed folder Adafruit_GFX. Confirm that the Adafruit_GFX folder contains Adafruit_GFX.cpp and Adafruit_GFX.h. Place the Adafruit_GFX library folder your <arduinosketchfolder>/Libraries/ folder. You may need to create the Libraries subfolder if its your first library. Restart the IDE.

# Useful Resources

- Image2Code: This is a handy Java GUI utility to convert a BMP file into the array code necessary to display the image with the drawBitmap function. Check out the code at ehubin's GitHub repository: https://github.com/ehubin/Adafruit-GFX-Library/tree/master/Img2Code

- drawXBitmap function: You can use the GIMP photo editor to save a .xbm file and use the array saved in the file to draw a bitmap with the drawXBitmap function. See the pull request here for more details: https://github.com/adafruit/Adafruit-GFX-Library/pull/31

- 'Fonts' folder contains bitmap fonts for use with recent (1.1 and later) Adafruit_GFX. To use a font in your Arduino sketch, #include the corresponding .h file and pass address of GFXfont struct to setFont(). Pass NULL to revert to 'classic' fixed-space bitmap font.

- 'fontconvert' folder contains a command-line tool for converting TTF fonts to Adafruit_GFX .h format.
	
## Related projects
	
- [Cale-idf](https://github.com/martinberlin/cale-idf)
- [CalEPD](https://github.com/martinberlin/CalEPD) the eink component for the ESP32 Espressif IDF framework
