#!/bin/bash

# Check if script is run as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root (use sudo)"
    exit 1
fi

# Prompt for credentials
echo "Enter your eduroam username (including @domain.edu):"
read USERNAME

echo "Enter your eduroam password:"
read -s PASSWORD
echo ""

# Stop NetworkManager to ensure clean setup
systemctl stop NetworkManager

# Create wpa_supplicant configuration
cat > /etc/wpa_supplicant/wpa_supplicant.conf << EOF
network={
    ssid="eduroam"
    key_mgmt=WPA-EAP
    eap=PEAP
    identity="${USERNAME}"
    password="${PASSWORD}"
    phase2="auth=MSCHAPV2"
    ca_cert="/etc/ssl/certs/ca-certificates.crt"
}
EOF

# Secure the configuration file
chmod 600 /etc/wpa_supplicant/wpa_supplicant.conf

# Create NetworkManager connection
nmcli con add \
    type wifi \
    con-name "eduroam" \
    ssid "eduroam" \
    wifi-sec.key-mgmt wpa-eap \
    802-1x.eap peap \
    802-1x.phase2-auth mschapv2 \
    802-1x.identity "${USERNAME}" \
    802-1x.password "${PASSWORD}"

# Restart NetworkManager
systemctl restart NetworkManager

# Try to connect
echo "Attempting to connect to eduroam..."
nmcli connection up eduroam

# Check connection status
if nmcli -t -f GENERAL.STATE connection show eduroam | grep -q "activated"; then
    echo "Successfully connected to eduroam!"
else
    echo "Connection attempt failed. Here's the current status:"
    nmcli connection show eduroam
    echo ""
    echo "You can try these troubleshooting steps:"
    echo "1. Run 'nmcli connection show eduroam' for detailed status"
    echo "2. Check logs with 'journalctl -u NetworkManager'"
    echo "3. Try manual connection with 'sudo nmcli connection up eduroam'"
fi

# Cleanup
# Clear variables containing sensitive information
USERNAME=""
PASSWORD=""
