rm -rf target ; rsync -rp  --exclude '../shared/target' ../shared pi@192.168.0.179:/home/pi/lora ; rsync -rp * pi@192.168.0.179:/home/pi/lora/rx
