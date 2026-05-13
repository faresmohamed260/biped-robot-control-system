$cli = 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
$url = 'https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json'

& $cli core update-index --additional-urls $url
& $cli core install esp32-bluepad32:esp32@4.1.0 --additional-urls $url
