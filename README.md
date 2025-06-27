# ESP32 Matter light

<b>REPRODUCTION STEPS</b>

Open a terminal window on your mac.

```
docker pull espressif/esp-matter:latest
```
```
git clone --recursive https://github.com/jonkofee/esp32-matter-ledstrip.git
```
```
docker run -it -v ./:/code -w /code espressif/esp-matter:latest
```
```
idf.py set-target esp32
```
```
idf.py menuconfig
```
```
idf.py build
```
Open a new terminal window on your mac.
```
esptool.py -p /dev/tty.usbserial-110 erase_flash
```
```
esptool.py -p /dev/tty.usbserial-110 --chip esp32 -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 4MB --flash_freq 40m 0x1000 build/bootloader/bootloader.bin 0xc000 build/partition_table/partition-table.bin 0x1d000 build/ota_data_initial.bin 0x20000 build/light.bin
```
- Replace `/dev/tty.usbserial-01FD1166` with your USB port.
```
screen /dev/tty.usbserial-110 115200
```
- Replace `/dev/tty.usbserial-01FD1166` with your USB port.

esptool.py -p /dev/tty.usbserial-110 --chip esp32 -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 4MB --flash_freq 40m 0x1000 build/bootloader/bootloader.bin 0xc000 build/partition_table/partition-table.bin 0x1d000 build/ota_data_initial.bin 0x20000 build/light.bin
