#!/bin/bash

if [ -f /usr/sbin/useradd ]; then
    COMMAND_ADD_USR='/usr/sbin/useradd'
    COMMAND_CHG_PWD='chpasswd'
elif [ -f /usr/sbin/adduser ]; then
    COMMAND_ADD_USR='/usr/sbin/adduser'
    COMMAND_CHG_PWD='chpasswd'
else
    exit 1
fi

SECRET=$(aws --region '${AWS::Region}' secretsmanager get-secret-value --secret-id '${CARLA_SECRET}' --query 'SecretString')
OS_PASSWORD=$(echo $SECRET | jq -r '. | fromjson | .password')
OS_USER=$(echo $SECRET | jq -r '. | fromjson | .username')

if [ $(getent group "$OS_USER") ]; then
    CREATE_USER_OPTS="-g $OS_USER"
else
    CREATE_USER_OPTS=''
fi

case "$OS_RELEASE" in
    amzn.2*)
        OS_ADMIN_GROUP='wheel'
        ;;
    *)
        # Catch all without the full path for untested platforms
        OS_ADMIN_GROUP='sudo'
esac

$COMMAND_ADD_USR $CREATE_USER_OPTS --comment "Local account for $OS_USER" "$OS_USER"

mkdir -p /home/$OS_USER
chown -R $OS_USER:$OS_USER /home/$OS_USER

echo "$OS_USER:$OS_PASSWORD" | $COMMAND_CHG_PWD

usermod -a -G $OS_ADMIN_GROUP "$OS_USER" -s /bin/bash