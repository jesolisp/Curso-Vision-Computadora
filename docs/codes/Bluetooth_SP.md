The following are instructions for connecting a Bluetooth device for serial communication on Arch Linux using BlueZ 5.31.


## Prerequisites

The following packages are required:

* `bluez-deprecated-tools`

## Pair

1. Start daemon: `systemctl enable --now bluetooth.service`
1. Pair using `bluetoothctl`:

   ```
   power on
   agent on
   scan on
   ... wait ...
   scan off
   pair <dev>
   ```

1. Create serial device: `sudo rfcomm bind rfcomm0 MAC_address_of_Bluetooth_device`

For example, Unicycle-Robot with ESP32 `sudo rfcomm bind rfcomm0 A8:42:E3:48:49:9A`

You should now have `/dev/rfcomm0`.


## Unpair

1. Remove serial device: `rfcomm release 0`
1. Unpair using `bluetoothctl`:

   ```
   remove <dev>
   power off
   ```

## Troubleshooting

Check `rfkill list` to make sure that the Bluetooth device is not blocked.



