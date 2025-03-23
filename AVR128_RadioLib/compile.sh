rm -rf build
rm *.lst *.map
arduino-cli compile -e --fqbn DxCore:megaavr:avrdb
