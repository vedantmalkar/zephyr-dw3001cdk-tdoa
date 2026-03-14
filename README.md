# Zephyr Firmware for UWB Localization (DW3000 / DWM3001C)

Custom firmware built on **Zephyr RTOS** for **Ultra-Wideband (UWB) based indoor localization** using the **DW3000 / DWM3001C platform**.

The goal of this project is to develop a **fully customizable localization firmware stack** supporting:

- **DS-TWR (Double-Sided Two-Way Ranging)**
- **TDoA (Time Difference of Arrival)**

This firmware runs on **anchor and tag nodes** in a UWB network and aims to enable **high-accuracy and scalable positioning systems**.

---

# Getting Started

This project uses **Nordic's nRF Connect SDK (based on Zephyr RTOS)** for firmware development.

If you are new to nRF Connect, follow the official tutorial:

https://academy.nordicsemi.com/courses/nrf-connect-sdk-fundamentals/

The tutorial explains how to:

- Install the nRF Connect SDK
- Set up the toolchain
- Configure VS Code extensions
- Build Zephyr applications

---

# Using This Repository

The main firmware logic is located in:

`src/main.c`

Open this file to modify or extend the firmware behavior such as:

- ranging logic
- packet transmission and reception
- synchronization experiments
- protocol modifications

For API references and documentation related to the **DW3000 software stack**, refer to the documentation inside:

`docs/`

The software guide inside this folder describes the available API functions for:

- configuring the UWB radio
- transmitting and receiving packets
- extracting timestamps
- controlling radio parameters

---

# Building and Flashing

After making changes:

1. Open the project in **VS Code with the nRF Connect extension installed**
2. Build the firmware using the **nRF Connect build system**
3. Flash the firmware to the board using **nRF Connect**

Typical workflow:

1. Modify firmware in `src/main.c`
2. Build the project
3. Flash the firmware to the DWM3001C board

---

# Background

UWB radios estimate distance by measuring **time-of-flight of radio signals**, allowing **centimeter-level localization accuracy**.

Two main localization approaches are used.

### DS-TWR

Two devices exchange packets and compute distance using round-trip timing.

- No global clock synchronization required  
- High accuracy  
- Less scalable for large tag networks  

### TDoA

A tag broadcasts a packet that is received by multiple anchors.

Each anchor records a **precise reception timestamp**, and the **difference between timestamps** is used to compute the tag position.

- Highly scalable  
- Requires **nanosecond-level clock synchronization**

---

# Previous Work

Before developing custom firmware, we implemented a working localization system using **Qorvo’s official firmware**.

Repository:  
https://github.com/Lakshyaa1/Localisation-using-UWB-chips

This implementation achieved:

- **~3 cm localization accuracy**
- Multi-anchor localization
- Real-time positioning

However, the vendor firmware limited deeper experimentation.

---

# Why Custom Firmware?

The official firmware introduced several limitations.

### Closed Firmware Stack

The core firmware cannot be modified, preventing experimentation with:

- radio behavior
- packet structures
- ranging protocols
- synchronization mechanisms

### Anchor Limit

The vendor firmware supports a **maximum of 8 anchors**, limiting scalability.

### Limited Radio Control

Important parameters such as:

- transmission power  
- packet timing  
- PHY configuration  

cannot be easily modified.

To overcome these limitations, we are implementing a **fully customizable firmware stack on Zephyr RTOS**.

---

# Synchronization Experiments

TDoA localization requires **extremely precise clock synchronization between anchors**.

### RS-485 Synchronization

Initial experiments used **RS-485** for synchronization between anchors.

While useful for:

- configuration
- anchor coordination
- data transfer

it cannot achieve **nanosecond-level synchronization** due to:

- UART serialization delays
- hardware propagation delays
- RTOS interrupt jitter

Since radio signals travel **~30 cm per nanosecond**, even small timing errors cause large localization errors.

### Ethernet Synchronization

Current work focuses on **Ethernet-based synchronization**, which offers:

- lower latency
- more deterministic timing
- better suitability for nanosecond-level synchronization

---

# Current Progress

- **DS-TWR implemented for a single anchor–tag pair**
- Currently extending DS-TWR to **multi-anchor ranging**
- Developing TDoA firmware
- Development of **Ethernet-based time synchronization**

---

# Project Goal

The final goal is to build an **open UWB localization firmware stack** supporting:

### DS-TWR

- Anchor ↔ tag ranging  
- Distance estimation  
- Multi-anchor localization  

### TDoA

- Synchronized anchor networks  
- High precision timestamp capture  
- Scalable multi-tag tracking  

---

# Hardware

- Qorvo **DWM3001C / DW3000 UWB modules**
- Ethernet synchronization network
