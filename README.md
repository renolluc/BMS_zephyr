# BMS_zephyr

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
