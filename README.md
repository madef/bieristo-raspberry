Installation guide
=================

## Install dependencies
```
sudo apt update
sudo apt install supervisor git python3 python3-pip spi-tools python3-spidev
```

## Install python dependency
```
sudo pip3 install adafruit-circuitpython-max31865
```

## Configure system
```
sudo dtparam spi=on
```

## Copy the source (of the projet)
```
git clone https://github.com/madef/bieristo-raspberry.git bieristo; cd bieristo
```

## Configure supervisor
```
sudo cp supervisord.conf /etc/supervisor/conf.d/bieristo.conf
sudo systemctl reload supervisor.service
```

## Active spi
```
sudo sed -i 's/#dtparam=spi=on/dtparam=spi=on/g' /boot/config.txt
```

## Create config file.

On bieristo.com, create a new card.
Clic the icon « copy » to copy the configuration.

From the bash of your raspberry
```
echo '<copy here the configuration>' >> config/app.json
```

## Reboot
```
sudo reboot
```
