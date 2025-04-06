# BMS_zephyr

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
        HAL_StatusTypeDef Read_Voltages(uint16_t *data_buffer);
        HAL_StatusTypeDef Read_Temp(uint16_t *data_buffer);
        uint16_t read_ADBMS_Temp();
        HAL_StatusTypeDef set_DCCx(uint32_t *cells_to_balance);
        HAL_StatusTypeDef ADBMS_HW_Init();
    }

    class battery{
        uint8_t get_battery_status_code(uint16_t GPIO_Input);
        void battery_reset_error_flags();
        uint8_t get_battery_error_code();
        void set_battery_error_flag(uint8_t mask);
        uint8_t volt2celsius(uint16_t volt_100uV);
        Battery_StatusTypeDef SDC_reset();
        void check_SDC_Feedback(uint32_t input_data);
        Battery_StatusTypeDef check_battery();
        void set_relays(uint8_t CAN_Data);
        void charging(uint32_t input_data);
        void set_time_per_measurement(uint16_t time_ms);
    }

    class serialmonitor{
        void SerialMonitor(uint8_t* data, uint16_t size);
    }

    class shutdowncircuit

    main ..> battery : uses
    main ..> Can_Bus : uses
    main ..> serialmonitor : uses
    main ..> shutdowncircuit : uses
    battery ..> spiMB : uses

```