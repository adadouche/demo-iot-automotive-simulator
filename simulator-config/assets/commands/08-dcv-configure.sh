#!/bin/bash

pip3 install crudini

systemctl stop dcvserver

cp /etc/dcv/dcv.conf /etc/dcv/dcv.conf.original

# https://docs.aws.amazon.com/dcv/latest/adminguide/enable-quic.html
crudini --set /etc/dcv/dcv.conf "security" "no-tls-strict" "true"

crudini --set /etc/dcv/dcv.conf "connectivity" "enable-quic-frontend" "false"
# crudini --set /etc/dcv/dcv.conf "connectivity" "enable-datagrams-display" "always-off"

crudini --set /etc/dcv/dcv.conf "connectivity" "quic-listen-endpoints" "['0.0.0.0:8443', '[::]:8443']"
crudini --set /etc/dcv/dcv.conf "connectivity" "quic-port" "8443"
crudini --set /etc/dcv/dcv.conf "connectivity" "web-listen-endpoints" "['0.0.0.0:8443', '[::]:8443']"
crudini --set /etc/dcv/dcv.conf "connectivity" "web-port" "8443"

# crudini --set /etc/dcv/dcv.conf "session-management" "create-session" "false"
# crudini --set /etc/dcv/dcv.conf "session-management/automatic-console-session" "owner" "ubuntu"

# session storage: https://docs.aws.amazon.com/dcv/latest/userguide/using-transfer.html
mkdir -p /home/${CARLA_OS_USER_NAME}/DCV-Storage
chown -R ${CARLA_OS_USER_NAME}:${CARLA_OS_USER_NAME} /home/ubuntu/DCV-Storage

# https://docs.aws.amazon.com/dcv/latest/adminguide/managing-sessions-start.html#managing-sessions-start-manual
tee /opt/dcv-virtual-session.sh > /dev/null << EOF
#!/bin/bash
dcvUser=${CARLA_OS_USER_NAME}
while true;
do
    if (/usr/bin/dcv list-sessions | grep \$dcvUser 1>/dev/null)
    then
    sleep 5
    else
    /usr/bin/dcv create-session demo --owner \$dcvUser --storage-root /home/\$dcvUser/DCV-Storage --type=virtual
    /usr/bin/dcv list-sessions
    fi
done
EOF

tee /etc/systemd/system/dcv-virtual-session.service > /dev/null << EOF
[Unit]
Description=Create DCV virtual session for user ${CARLA_OS_USER_NAME}
After=default.target network.target
[Service]
ExecStart=/opt/dcv-virtual-session.sh
[Install]
WantedBy=default.target
EOF

chmod +x /opt/dcv-virtual-session.sh

# text console: DCV virtual sessions only
systemctl daemon-reload
systemctl enable --now dcvserver
systemctl enable --now dcv-virtual-session

systemctl stop dcvserver
systemctl stop dcv-virtual-session

systemctl restart dcvserver
systemctl restart dcv-virtual-session

systemctl status dcvserver
systemctl status dcv-virtual-session