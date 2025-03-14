echo 'SUBSYSTEM=="vchiq",MODE="0666"' > /etc/udev/rules.d/99-camera.rules
cp -r /etc/apt/trusted.gpg.d/ ./trusted