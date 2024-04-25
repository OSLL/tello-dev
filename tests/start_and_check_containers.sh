#!/bin/bash

# Запуск контейнеров
docker-compose up -d

# Проверка статуса контейнеров
if docker ps | grep -q "droner"; then
    echo "Контейнеры успешно запущены"
else
    echo "Произошла ошибка при запуске контейнеров"
    exit 1
fi
