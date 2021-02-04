if [ -d "/media/$USER/RPI-RP2" ]; then
  cp ./cmake-build/rotary_inverted_pendulum.uf2 /media/$USER/RPI-RP2
else
  echo "PICO not connected in upload mode"
fi