ARCH = arm-none-eabi-

rom.bin: loader.bin EtherZ/EtherZ,ffa IDEWedge/IDEWedge,ffa makerom.py
	python ./makerom.py > rom.bin

loader.bin: loader
	$(ARCH)objcopy -O binary $< $@

loader: loader.S
	$(ARCH)gcc -nostartfiles -nostdlib -o $@ $<
