# Intellivision controller to USB adapter firmware

A firmware to convert Intellivision controllers to USB, for AVR micro-controllers.

## Project homepage

Schematic and additional information are available on the project homepage:

English: [Intellivision controller to USB adapter project](http://www.raphnet.net/electronique/intellivusb/index_en.php)
French: [Adaptation d'une manette Intellivision à USB](http://www.raphnet.net/electronique/intellivusb/index.php)

## Supported micro-controllers

Currently supported micro-controllers:

* Atmega8
* Atmega168

Adding support for other micro-controllers should be easy, as long as the target has enough
IO pins, enough memory (flash and SRAM) and is supported by V-USB.

## Built with

* [avr-gcc](https://gcc.gnu.org/wiki/avr-gcc)
* [avr-libc](http://www.nongnu.org/avr-libc/)
* [gnu make](https://www.gnu.org/software/make/manual/make.html)

## License

This project is licensed under the terms of the GNU General Public License, version 2.

## Acknowledgments

* Thank you to Objective development, author of [V-USB](https://www.obdev.at/products/vusb/index.html) for a wonderful software-only USB device implementation.
