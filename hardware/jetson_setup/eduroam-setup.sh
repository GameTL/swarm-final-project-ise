nmcli device

sudo nmcli con add \
    type wifi \
    con-name "eduroam" \
    ssid "eduroam" \
    wifi-sec.key-mgmt wpa-eap \
    802-1x.eap peap \
    802-1x.phase2-auth mschapv2 \
    802-1x.identity "your_username@your.edu" \
    802-1x.password "your_password"

sudo nmcli connection modify eduroam 802-1x.identity "your_new_username@your.edu"
sudo nmcli connection modify eduroam 802-1x.password "your_new_password"

sudo nmcli connection up eduroam

sudo nano /etc/wpa_supplicant/wpa_supplicant.conf

# Add this configuration:
network={
    ssid="eduroam"
    key_mgmt=WPA-EAP
    eap=PEAP
    identity="your_username@your.edu"
    password="your_password"
    phase2="auth=MSCHAPV2"
    ca_cert="/etc/ssl/certs/ca-certificates.crt"
}

sudo systemctl restart NetworkManager