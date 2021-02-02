# LinuxSerialBridge

  The Linux Serial Bridge includes the following deliverables:
  
   1. wilcsbridge_sdio.c
   2. wilcsbridge_spi.c
   3. wilc_sbridge.h
   4. wilc3000_ble_burst_firmware.h
   5. sbridge_app.c
    

## Kernel Compilation with Serial Bridge Driver
  Perform the following to compile the kernel with Serial Bridge driver.

   1. Get the kernel source from Linux4wilc GitHub repository using the following command:
      ```
        $ git clone https://github.com/linux4wilc/linux-at91.git'
        $ cd linux-at91'
      ```    
   2. Create the kernel using sama5_defconfig defconfig file, using the following command:
       ``` 
        $ make ARCH=arm sama5_defconfig'
       ``` 
   3. Add the wilcsbridge driver in kernel's directory `$ "/drivers/staging/"` as provided in the release package "src"
   folder, using the following command:
     ``` 
      $ cp -rf ../src/wilcsbridge ../linux-at91/drivers/staging/
     ``` 
   4. To include the wilcsbridge driver in the kernel build, relevant Kconfig and Makefile must be changed as
   following:
     ``` 
      $ vi linux-at91/drivers/staging/Kconfig
     ``` 
   Add the following:
     ``` 
        source "drivers/staging/wilcsbridge/Kconfig"
        $ vi linux-at91/drivers/staging/Makefile
     ``` 
   Add the following:
      ```
        obj-$(CONFIG_WILCSBRIDGE) += wilcsbridge/
      ```  
   5. To include the wilcsbridge driver in the kernel build, modify the default configuration using the `S“menuconfig”`
   parameter in "Device Drivers > staging Driver > Select wilcsbridge SPI/SDIO" and select as modules.
     ``` 
      $ make ARCH=arm menuconfig
     ``` 
   6. Save and exit from the menuconfig configuration page.
   7. Modify the Device Tree files for SDIO or SPI for ATWILC1000/ATWILC3000. Modify the files for UART
   interface for ATWILC3000 only. Sample Device Tree is based on the SAMA5D4_Xplained Pro board. User
   must modify the Device Tree node based on the used MPU. Mainly, the GPIO changes for CHIP_EN and
   RESETN pin detials must be added for SPI interface on wilc_sdio node of mmc1 and UART for BLE interface.
   
   - For SDIO Interface:
        ```
          For SD/MMC node:
            mmc1: mmc@fc000000 {
            pinctrl-names = "default";
            pinctrl-0 = <&pinctrl_mmc1_clk_cmd_dat0 &pinctrl_mmc1_dat1_3>;
            vmmc-supply = <&vcc_mmc1_reg>;
            vqmmc-supply = <&vcc_3v3_reg>;
            non-removable;
            status = "okay";
              wilc_sdio@0{
              compatible = "microchip,wilc1000", "microchip,wilc3000";
              irq-gpios = <&pioC 27 0>;
              reset-gpios = <&pioB 28 0>;
              chip_en-gpios = <&pioC 30 0>;
              status = "okay";
              reg = <0>;
              bus-width = <4>;
              };
            };
        ``` 
        
   - For SPI Interface:
        ```
          spi1: spi@fc018000 {
          cs-gpios = <&pioB 21 0>;
          status = "okay";
            wilc_spi@0 {
            compatible = "microchip,wilc1000", "microchip,wilc3000";
            spi-max-frequency = <48000000>;
            irq-gpios = <&pioB 26 0>;
            reset-gpios = <&pioE 21 0>;
            chip_en-gpios = <&pioB 27 0>;
            reg = <0>;
            status = "okay";
            };
          };
        ``` 
   - For ATWILC3000 BLE UART interface:         

        ```
          usart4: serial@fc010000 {
          atmel,use-dma-rx;
          atmel,use-dma-tx;
          status = "okay";
          };
        ``` 
   8. To build the kernel image and Device Tree binary file .dtb, generate the build using the following commands:
       ```
        $ make ARCH=arm CROSS_COMPILE=...../arm-linux-gnueabihf- zImage modules dtbs
       ``` 
      Kernel build generates the zImage, `$at91-sama5d4_xplained.dtb`, `$wilcsridge-sdio.ko`, and/or `$wilcsbridge-spi.ko` files.

## Flashing Binaries in SAMA5D4 Xplained Pro Board
  Perform the following steps to flash binaries in the SAMA5D4 Xplained Pro board:

   1. Download the demo binary package from linux4wilc GitHub repository, using the following commands:
       ```  
        $ git clone https://github.com/linux4wilc/wilc_demo.git
        $ tar -xvf wilc_sama5d4_linux_demo_4.9_kernel.zip
        $ cd wilc_sama5d4_linux_demo_4.9_kernel
       ```        
   2. Copy the Linux kernel `$"arch/arm/boot/zImage"` and `$"arch/arm/boot/dts/at91-sama5d4_xplained.dtb"` files to demo binary.
   3. Install the sam-ba 2.18 or 3.2.x version in Windows or Linux platform.
   4. For Linux sam-ba and 32 bit or sam-ba_64 and 64 bit, add the path in the shell script file
       ```  
        "$ demo_linux_nandflash.sh".
       ```
   5. Add the jumper at JP7 and connect to the PC via the USB port at J11.
   6. Reset the board to enter in to "ROMboot" and remove the jumper JP7.
   7. Execute demo_linux_nandflash.bat (for Windows®) or `$demo_linux_nandflash.sh` (for Linux).
   8. After successful flashing, copy the modules `$"drivers/staging/wilcsbridge/wilcsbridge-sdio.ko"` and `$"drivers/staging/wilcsbridge/wilcsbridge-spi.ko"`   based on the used interface. For more details, refer the [ATWILC Linux User Guide](http://ww1.microchip.com/downloads/en/DeviceDoc/ATWILC1000-ATWILC3000-Wi-Fi-Link-Controller-Linux-User-Guide-DS70005328B.pdf).

## Compiling the Sbridge Application
  Perform the following steps to compile the Sbridge Application:
   1. Serial Bridge application must be cross compiled against the target device, using the following command:
       ```  
        $ ...../arm-linux-gnueabihf-gcc -o sbridge_app sbridge_app.c
       ```
   **Note:** ARM GCC compiler must be same as the compiler which is used for Linux kernel compilation.
   2. Export the ARM GCC compiler path or provide the absolute path.
  
## Output Modules
  The following are the list of output modules:
   1. `$wilcsbridge-spi.ko`, for SPI interface.
   2. `$wilcsbridge-sdio.ko`, for SDIO interface.
   3. sbridge_app, for serial bridge application with MAC address read/write feature.
    
## Steps to Run Serial Bridge Application to Connect with MCHPRT2 Tool using SPI Interface
  The following are the list of steps to run Serial Bridge application to connect with MCHPRT2 tool using SPI interface:
   1. Enter `$ $ insmod wilcsbridge-spi.ko`
   2. Navigate to `$ $ ./sbridge_app spi 230400 ttyS5`
   3. Connect the FTDI cable with SAMA5D4 EXT2 pin 11 (Tx) and pin 12 (Rx).
   4. Run the MCHPRT2 tool and follow the user guide to run the tool.
    
## Steps to Run Serial Bridge Application to Connect with MCHPRT2 Tool using SDIO Interface
  The following are the list of steps to run Serial Bridge application to connect with MCHPRT2 tool using SDIO interface:
   1. Enter `$ $ insmod wilcsbridge-sdio.ko`
   2. Navigate to `$ $ ./sbridge_app sdio 230400 ttyS5` > interface and set the sdio, buadrate - 230400, usrt console - ttyS5 with RECORDER_MODE macro enabled.
   3. Connect the FTDI cable with SAMA5D4 EXT2 pin 11 (Tx) and pin 12 (Rx).
   4. Run the MCHPRT2 tool and follow the user guide to run the tool.  

## Steps to Run Serial Bridge Application to Connect with HCI Tool for ATWILC3000 BLE UART Interface
  The following are the list of steps to run Serial Bridge application to connect with HCI tool for ATWILC3000 BLE UART interface:
   1. Enter `$ $ insmod wilcsbridge-sdio.ko / insmod wilcsbridge-spi.ko`
   2. Navigate to `$ $ ./sbridge_app uart 230400 ttyS5 ttyS2` > interface and set the sdio, buadrate - 230400, usrt console - ttyS5 with RECORDER_MODE macro enabled.
   3. Connect the FTDI cable with SAMA5D4 EXT2 pin 11 (Tx) and pin 12 (Rx) for Sbridge connection.
   4. Connect the ATWILC3000 BLE TX pin to SAMA5D4 UART4 Rx pin (SAMA5D4 Xplained Pro board J19 pin 1 - PE27).
   5. Connect the ATWILC3000 BLE RX pin to SAMA5D4 UART4 Tx pin (SAMA5D4 Xplained Pro board J19 pin 2 - PE26).
   6. Run the HCI_COMMAND tool and follow the user guide to run the tool.

## Steps to Run MAC Read/Write using Sbridge Application
  The following are the list of steps to run MAC Read/Write using Sbridge application:
   1. Load the provided driver module in the package (wilcsbridge-spi.ko) using the following command:
         ```
         $ insmod wilcsbridge-spi.ko
         ```
   2. To run the application file, the following four parameters are required:
        - Protocol (valid argument:spi) 
        - Serial console baud rate (valid argument:115200)
        - ttyS5
        
   **Note:** This parameter does not have impact on this application. This is used for compatibility with Serial Bridge application, which can be merged with this application in the future.
    
   3. To enter the MAC address write mode followed by hex value MAC address `$"mac"` string is used (valid argument:6 bytes of hexadecimal number) as following:
        ```    
        $ ./sbridge_app spi 230400 ttyS5 mac f8f005f431f2
        ```

## Steps to Run Frequency Offset Write using Sbridge Application:

  The following are the list of steps to run Frequency offset Write using Sbridge application:
   1. Load the provided driver module in the package (wilcsbridge-spi.ko) using the following command:
         ```
         $ insmod wilcsbridge-spi.ko
         ```
   2. To run the application file, the following four parameters are required:
        - Protocol (valid argument:spi) 
        - Serial console baud rate (valid argument:115200)
        - ttyS5   
        
   **Note:** This parameter does not have impact on this application. This is used for compatibility with Serial Bridge application, which can be merged with this application in the future.
	
   3. `$ $ "xooff"` string will be used to enter the frequency or XO offset mode followed by offset value in ppm   
   4. Frequency offset value (valid argument: in ppm vlaue in floating point number Ex: 4.03 or 3.25 etc)
      ```
        $ ./sbridge_app  spi 230400 ttyS5 xooff 4.03
      ```  
	 
	 
## Steps to Run MAC write and frequency offset value combinedly using sbridge application:

  The following are the list of steps to write the Frequency offset with MAC address using Sbridge application:
   1. Load the provided driver module in the package (wilcsbridge-spi.ko) using the following command:
         ```
         $ insmod wilcsbridge-spi.ko
         ```
   2. To run the application file, the following four parameters are required:
        - Protocol (valid argument:spi) 
        - Serial console baud rate (valid argument:115200)
        - ttyS5
        
   **Note:** This parameter does not have impact on this application. This is used for compatibility with Serial Bridge application, which can be merged with this application in the future.
    
   3. `$"mac"` string will be used to enter the MAC address write mode followed by hex value MAC address	   
   4. MAC address(valid argument:6 bytes of hexadecimal number)
	 5. `$"xooff"` string will be used to enter the frequency or XO offset mode followed by offset value in ppm   
   6. Frequency offset value (valid argument: in ppm vlaue in floating point number Ex: 4.03 or 3.25 etc)
        ```
        $ ./sbridge_app  spi 230400 ttyS5 mac f8f005f431f2 xooff 4.03
        ```

## UART Configuration for Serial Bridge Application
   In SAMA5D4 Xplained Pro board, UART0 is used for Serial Bridge communication. By default UART0 is disabled. nable the UART0 in Device Tree .dtsi file.
   For the UART0 `$"/dev/ttyS5"` device is created in the Serial Bridge application. This is also used to transmit and receive the data from the GUI tool.
 
   **Note:** For other MPU's use any available UART and respective `$"/dev/ttyxx"` device.
  
## Pin Configuration for SAMA5D4 with ATWILC1000 SPI Interface
   The Linux Serial Bridge driver acts as a reference to the SAMA5D4 Xplained pro board. EXT1 header pin configuration is used for SPI interface, Rest,   Chip_En, and IRQ.
   
   ### Pin Configuration for SAMA5D4 with ATWILC1000 SPI Interface
      ```
        **Description**    **EXT1 of SAMA5D4 Xplained Pro Board**
        SPI MISO    -        17 pin
        SPI MOSI    -        16 pin
        SPI SCK     -        18 pin
        SPI SS      -        15 pin
        RESET       -        5 pin
        CHIP_EN     -        10 pin
        IRQ         -        9 pin (optional for Serial bridge application)
       ```
  
   **Note:** The SPI interface pins are configured using Device Tree of the Linux kernel. The RESET and CHIP_EN pins are configured in the `$"wilc_spi.c"` driver itself using GPIO numbers.
  
## Test Procedure
   1. Flash the Linux kernel image in to the SAMA5D4 MPU. For more details, refer [ATWILC Linux User Guide](http://ww1.microchip.com/downloads/en/DeviceDoc/ATWILC1000-ATWILC3000-Wi-Fi-Link-Controller-Linux-User-Guide-DS70005328B.pdf).
   2. Connect the ATWILC1000 SPI interface board to EXT1 or SD board to SDIO interface connector.
   3. Connect the FTDI cable Rx - Yellow (ET2 header - pin 12) and Tx - Orange (ET2 header - pin 11) and Windows PC for GUI tool communication.
   4. Follow the MCHPRT2 user guide for production and validation.   
   
For more information on Microchip Linux Serial Bridge and MCHPRT, visit [Microchip ATWILC Linux Serial Bridge Application](http://ww1.microchip.com/downloads/en/Appnotes/00003397A.pdf).
