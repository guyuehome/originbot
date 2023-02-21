mount -o,remount rw /
echo 8 >/proc/sys/kernel/printk
echo 112 >/sys/class/gpio/export
echo out >/sys/class/gpio/gpio112/direction
echo 1 >/sys/class/gpio/gpio112/value
echo 118 >/sys/class/gpio/export
echo out >/sys/class/gpio/gpio118/direction
echo 1 >/sys/class/gpio/gpio118/value


modprobe -r es7210
modprobe -r es8156
modprobe -r hobot-i2s-dma
modprobe -r hobot-cpudai
modprobe -r hobot-snd-7210 snd_card=5


modprobe es7210
modprobe es8156
modprobe hobot-i2s-dma
modprobe hobot-cpudai
modprobe hobot-snd-7210 snd_card=5
