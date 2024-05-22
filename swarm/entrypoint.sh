#!/bin/sh
case "$SERVICE_NAME" in
  networking)
    exec python3 /networking.py /networks.json
    ;;
  server)
    exec python3 /server.py
    ;;
  swarm_server)
    exec python3 /swarm_server.py
    ;;
  *)
    echo "Unknown service: $SERVICE_NAME"
    exit 1
    ;;
esac
