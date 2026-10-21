This repository has all of the firmware code (i.e. code that the firmware subteam is responsible for).

Please create a subdirectory for each embedded device (Teensy on Short Circuit, Teensy on NAND, etc.)

Below is a description of each of the codebases currently in use.

# lora_rtcm_broadcast

## What it runs on

This firmware runs on a Teensy connected to a SX1276 module that can broadcast in the 900 MHz band at up to 1 Watt.

This Teensy is connected via serial to a Raspberry Pi that's sending it RTK corrections to be broadcast out.

```
-------------         -------------         --------------
|  ZED F9P  |         | Raspberry |         |            |
|    GPS    |   -->   |    Pi     |   -->   |   Teensy   |
|           |         |           |         |            |
-------------         -------------         --------------
```

This hardware is located in the Carnegie Tech Radio Club's radio shack, on the 4th floor of Hammerschlag Hall.  Additionally, we must include the W3VC callsign in our broadcast.

## What it does
This firmware makes it possible for the Teensy to receive RTK corrections via serial from the Raspberry Pi.  The Teensy also interfaces with an SX1276 module to broadcast out that data at around 1W of power.

## An additional note about this hardware

The specific module we're using is [SparkFun's LoRa 1W Breakout - 915M30S](https://www.sparkfun.com/lora-1w-breakout-915m30s.html).  At the time of writing this, SparkFun lists this as an experimental product, and not guaranteed for long-term production.  But that's ok because this module is just a very simple breakout board for the E19-915M30S module, made by Ebyte.

# lora_rtcm_listen

## What it runs on

## What it does

# nand_encoder

## What it runs on

## What it does

# nand_teensy

This is the firmware that runs on the Teensy 4.1 onboard our buggy NAND.

# short_circuit_shell

## What it runs on

I don't quite remember at the moment, but given that it's just one `.ino` file, it's probably some Arduino IDE-compatible board.

## What it does

Controlls the RGB LEDs attached to Short Circuit's shell.

# short_circuit_teensy

This is the firmware that runs on the Teensy 4.1 onboard our buggy Short Circuit.