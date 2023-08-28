# ConnectSSh_RaspPi4

## Raspberry Pi OS

Instalação do OS:
```
$ sudo apt install rpi-imager
```

## Fix the SSH

Install the OpenSSH server:
```
$ sudo apt install openssh-server
```

## Check the SSH Service Status
```
$ sudo systemctl status sshd
```
![](https://static1.makeuseofimages.com/wordpress/wp-content/uploads/2023/01/status-of-ssh-server.jpg?q=50&fit=crop&w=1500&dpr=1.5)


## Check the SSH Port
```
$ sudo systemctl status sshd
```
![](https://static1.makeuseofimages.com/wordpress/wp-content/uploads/2023/01/check-ssh-port.jpg?q=50&fit=crop&w=1500&dpr=1.5)


## Ifconfig
Install:
```
$ sudo apt-get install net-tools
```

ifconfig:
```
$ ifconfig
```
![image](https://github.com/HerickDallagnol/ConnectSSh_RaspPi4/assets/140270456/a24e2e51-322d-49db-8346-cdf0f41d5a37)

save "inet"

## Connect 
```
$ ssh ubuntu@[IPinet]
  password da rasp
```
![image](https://github.com/HerickDallagnol/ConnectSSh_RaspPi4/assets/140270456/40533f9b-f92b-4fa1-ae85-96e2fc18c1a7)
)
