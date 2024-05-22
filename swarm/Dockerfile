FROM ubuntu:22.04

COPY /scripts/networking/localtime /etc/localtime

# Общие зависимости
RUN apt update \
    && apt install -y \
        network-manager=1.36.6-0ubuntu2 \
        iptables=1.8.7-1ubuntu5.2 \
        python3=3.10.6-1~22.04 \
        python3-pip=22.0.2+dfsg-1ubuntu0.4 \
    && apt clean

# Установка Python пакетов
RUN pip install --upgrade pip \
    && pip install jsonschema==4.22.0 \
    && pip install jsonrpcserver==5.0.9 \
    && pip install jsonrpcclient==4.0.3 requests==2.32.2

# Слои для каждого сервиса
COPY /scripts/networking/networking.py /networking.py
COPY /scripts/rpc/server.py /server.py
COPY /scripts/rpc/swarm_server.py /swarm_server.py
COPY /networks.json /networks.json


# Скрипт для запуска нужного сервиса на основе переменной среды
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
