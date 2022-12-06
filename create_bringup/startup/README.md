# To install, first check all paths in scripts are correct then:

sudo cp shoebot.service /lib/systemd/system

sudo systemctl enable shoebot.service

# To start

sudo systemctl start shoebot.service

# To stop

sudo systemctl stop shoebot.service

# To check the status

systemctl status shoebot.service
