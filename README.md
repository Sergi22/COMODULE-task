# COMODULE-task
				Steps to use the GPS example
1.	Load and unpack SDK 14.2 to your machine
2.	Download main.c and put it into a copy of folder 
examples/ble_peripheral/ble_app_uart.
3.	Use nRF studio to erase flash and then softdevice version 6.1.1 to the target.
4.	Use Keil or SES to compile project and download it to the target device.
5.	In nRF connect new device with name GPSDATA should appear. It should advertise “Hello” if you choose format Text.
6.	To send commands to device use Putty. Set it up to the COM port assigned to
Microcontroller, baud rate is 115200, data bits 8, stop bit 1, Parity: none, flow control: none. Choose terminal settings and select “force on” in both lines of “line discripline options”.
7.	Open connection and try to send data.

Unfortunately, data that is send is limited and in text field only 4 first symbols are visible. Was not able to find solution to that problem.
