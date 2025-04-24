```ASCII
     _    __  __ ____         ____ ____    _____          _                
    / \  |  \/  / ___|       / ___| __ )  |__  /___ _ __ | |__  _   _ _ __ 
   / _ \ | |\/| \___ \ _____| |   |  _ \    / // _ \ '_ \| '_ \| | | | '__|
  / ___ \| |  | |___) |_____| |___| |_) |  / /|  __/ |_) | | | | |_| | |   
 /_/   \_\_|  |_|____/       \____|____/  /____\___| .__/|_| |_|\__, |_|   
                                                   |_|          |___/   
```

## Software Deployment
To install the necessary software on your Ubuntu system, follow these steps:

The used Ubuntu version for this project was **Ubuntu 24.04**
1. **Check and update your Ubuntu version**:
    ```sh
    lsb_release -a
    sudo apt update
    sudo apt upgrade
    ```

2. **Install Zephyr**:

    Follow the [Zephyr Project's Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html) to set up the Zephyr environment.
    The following verisons were used for this project:
    - Zephyr: **4.1.0**
        ```sh
        cat ~/zephyrproject/zephyr/VERSION
        ```
    - West: **1.3.0** (west -V)
        ```sh
        cd ~/zephyrproject/zephyr/
        west -V
        ```
    - Zephyr SDK: **0.17.0**

3. **Clone the repository**:
    ```sh
    git clone https://github.com/hackerluca/BMS_zephyr.git
    cd BMS_zephyr
    ```
4. **Build the project**:
    ```sh
    west build -p always -b nucleo_l432kc <PATH>/BMS_zephyr/BMS_l432kc/
    ```

5. **install STMCubeProgrammer**
    Install [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)

6. **install st-link-tools**
    ```sh
    sudo apt install stlink-tools
    ```
    After installing stlink tools you can check for devices with:
    ```sh
    st-info --probe
    ```
    This should detect the microcontroller. When a device is detected you should be able to flash the software.

7. **Flash the firmware**:
    ```sh
    west flash
    ```

## Activation and Configuration Sequence Diagram
The following sequence diagram illustrates the activation and configuration process, detailing the interactions between components during system initialization.

```mermaid
sequenceDiagram
    participant ecu as ECU
    participant ivt-s as Isabellenhütte
    participant nucleo as AMS-CB
    participant adbms as AMS-MB's
    participant gpio as GPIO's

    nucleo->>gpio: SET all relais FALSE
    nucleo->>gpio: READ sdc
    gpio->>nucleo: SDC state
    nucleo->>ivt-s: Init config
    nucleo->>adbms: WRITE config REGISTER
    nucleo->>adbms: READ config REGISTER
    adbms->>nucleo: REIGSTER state
    nucleo->>nucleo: DIFF write and read AMS-MB's
    nucleo->>nucleo: check CRC AMS-MB's
    nucleo->>adbms: READ data
    adbms->>nucleo: DATA state
    nucleo->>nucleo: check DATA
    nucleo->>gpio: WRITE sdc TRUE
    nucleo->>ecu: send HV-CIRCUIT ready

```


## Loops Sequence Diagram
The following sequence diagram illustrates the interactions between various components, including their communication loops and timings.

```mermaid
sequenceDiagram
    participant laptop as Laptop
    participant ecu as ECU
    participant ivt-s as Isabellenhütte
    participant nucleo as AMS-CB
    participant adbms as AMS-MB's
    participant gpio as GPIO's

    ecu->>nucleo: 8 bytes
    loop <100ms
    nucleo->>+adbms: send Voltage and Temp.
    adbms->>-nucleo: Voltage and Temp. data
    end
    nucleo->>ecu: 8 bytes
    nucleo->>laptop: all data
    loop 100ms
    ivt-s->>nucleo: 6 bytes
    end
    loop ?ms
    nucleo-->>gpio: read
    end

```
## Class Diagram
The class diagram shows the different classes used and their depencies to each other. As a first instance only public function are declared to get an overall view of the program.

```mermaid
classDiagram

    class main

    class Can_Bus{
        int BMS_CAN_INIT(void);
        int send_CAN(uint32_t address, uint8_t *TxBuffer);
        int send_CAN_IVT_nbytes(uint32_t address, uint8_t *TxBuffer, uint8_t length);
        int send_data2ECU(uint16_t GPIO_Input);
        int ISA_IVT_Init(void);
    }

    class spiMB{
        int spi_wakeup_adbms1818();
        extern uint16_t spi_generate_pec(const uint8_t data[], size_t len);
        extern int spi_read_voltages(uint16_t *data_buffer);
        extern int spi_read_temp(uint16_t *data_buffer);
        extern uint16_t spi_read_adbms_temp();
        extern int spi_set_discharge_cell_x(uint32_t *data_buffer);
        uint16_t generatePEC(uint8_t data[], size_t len);
        void spi_wakeup();
        int spi_adbms1818_hw_init();
    }

    class battery{
        int spi_wakeup_adbms1818();
        extern uint16_t spi_generate_pec(const uint8_t data[], size_t len);
        extern int spi_read_voltages(uint16_t *data_buffer);
        extern int spi_read_temp(uint16_t *data_buffer);
        extern uint16_t spi_read_adbms_temp();
        extern int spi_set_discharge_cell_x(uint32_t *data_buffer);
        uint16_t generatePEC(uint8_t data[], size_t len);
        void spi_wakeup();
        int spi_adbms1818_hw_init();
    }

    class shutdowncircuit{
        extern Battery_StatusTypeDef refresh_sdc();
        extern void sdc_set_relays(uint8_t CAN_Data);
    }

    class serialmonitor{
        void SerialMonitor(uint8_t* data, uint16_t size);
    }



    main ..> battery : uses
    main ..> Can_Bus : uses
    main ..> serialmonitor : uses
    main ..> shutdowncircuit : uses
    battery ..> spiMB : uses
    Can_Bus ..> shutdowncircuit : uses
    battery ..> shutdowncircuit : uses

```

