. ~/esp/v5.3.1/esp-idf/export.sh
export IDF_TARGET=esp32h2
if [ "$1" == "-e" ]; then
    idf.py -p /dev/ttyACM0 erase-flash
else
    idf.py -p /dev/ttyACM0 flash
fi

