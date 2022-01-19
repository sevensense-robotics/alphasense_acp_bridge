# Installing packages from APT

To install the Alphasense Communication Protocol (ACP) driver on ROS-focal and ROS-noetic systems with amd64 or arm64 architecture we recommend installation over APT.

```
# Add the Sevensense PGP key to make this machine trust Sevensense's packages.
curl -Ls https://deb.7sr.ch/pubkey.gpg | sudo apt-key add -

# Add the Sevensense APT repository to the list of known sources.
echo "deb [arch=$(dpkg --print-architecture)] https://deb.7sr.ch/alphasense focal main" \
          | sudo tee /etc/apt/sources.list.d/sevensense.list
```

This should result in a file `/etc/apt/sources.list.d/sevensense.list` with the following content.

```
deb [arch=amd64] https://devel.deb.7sr.ch/alphasense focal main
```
