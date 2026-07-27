# BenchPilot

**Bench-test your Raymarine autopilot or replace the display on your boat.**

BenchPilot turns an ESP32 touchscreen display (ESP32-2432S028R CYD) into a SeaTalk 1 bus monitor and wireless control head. Connect it to a Raymarine SmartPilot S3 course computer on the bench or on the water. It generates heading data, decodes autopilot status, and lets you push buttons just like the real ST6002, all through a 320x240 color touchscreen or a WiFi web dashboard on your phone.

![BenchPilot](docs/benchpilot.png)

## What you can do

- **Replace your display.** Plug it into the existing SeaTalk bus and get heading, rudder, depth, wind, speed, and GPS on the touchscreen with no wiring changes needed.
- **Control from anywhere.** The built-in WiFi web dashboard works on any phone, tablet, or laptop. Adjust course or change modes without reaching for the helm.
- **Monitor.** See all SeaTalk traffic in real time, on-screen or remotely.
- **Simulate.** Built-in heading generator ("FluxSim") sends fake heading datagrams so you can test the autopilot's response on the bench without a compass.
- **Log.** Every bus datagram is saved to the SD card for later analysis.
- **Update.** Flash new firmware over WiFi (OTA) with no cables needed.

## What you need

- ESP32-2432S028R "CYD" display board
- BenchPilot interface PCB (see pcb/ folder for Fritzing files)
- 12V power supply (or the SeaTalk bus itself)
- Any SeaTalk 1 device, such as SmartPilot S3, ST50/ST60 instruments, etc.

## Getting started

1. Build the interface circuit (pcb/ or docs/seatalk_circuit.txt)
2. Copy `src/wifi_config.example.h` to `src/wifi_config.h`, add your WiFi
3. Build and upload with PlatformIO:
   ```
   pio run -t upload
   ```
4. Connect to the BenchPilot WiFi access point or join your network
5. Open http://benchpilot.local in your browser

## License

GNU General Public License v3.0
