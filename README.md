# Итерация 3
Зависимости:  
Версия ОС: Ubuntu 22.04.4 LTS  
Python 3.10.12

## Конфигурирование
Заполнить [конфигурационный файл](https://github.com/OSLL/tello-dev/blob/master/swarm/networks.json)
```json
{
    "ifaces" : {
        "iface-name" : {
            "ssid" : "",
            "password" : "",
            "ip": "172.18.0.3"
        },
        "iface-name": {
            "ssid" : "",
            "password" : "",
            "ip": "172.18.0.4"
        }
    }
}
```
- Необходимо выполнить команду `ip a` и вставить имена необходимых wlan-интерфейсов вместо `iface-name`.
- Установить пакет `network-manager`:
  `apt install network-manager=1.36.6-0ubuntu2`
- Выполнить команду `nmcli dev wifi list` и определить SSID wifi сетей причастных к дрону и вставить вместо `ssid`

**Пример сконфигурированного файла**
```json
{
    "ifaces" : {
        "wlo0" : {
            "ssid" : "DRONE 1",
            "password" : "",
            "ip": "172.18.0.3"
        },
        "wlo1": {
            "ssid" : "DRONE 2",
            "password" : "",
            "ip": "172.18.0.4"
        }
    }
}
```

## Запуск
Установить зависимости  
```bash
python3 -m pip install jsonrpcclient==4.0.3 requests==2.31.0
```
- Необходимо выполнить команду `docker compose -f ./swarm/docker-compose.yml up -d --build` (в одном месте монтируется с хоста папка и этого не избежать, иначе из контейнера нельзя будет управлять сетевыми устройствами)
- После запуска необходимо вызвать скрипт *client.py* передав флаги -c <command> --ip <ip>,
  где:
  
  --ip - P-адреса сервера JSON-RPC, к которому скрипт будет отправлять запросы, у нас это 127.0.0.1.
   
  -c - команда для дронов.  
  
Пример:
```bash
python3 ./swarm/scripts/rpc/client.py -c takeoff --ip 127.0.0.1
```
Результат:  
```bash
RESULT for 'http://127.0.0.1:65001' :
         {'jsonrpc': '2.0', 'result': 'OK', 'id': 1}
```
## Презентация и видео
[Видео запуска из лаборатории](https://drive.google.com/file/d/1AlVOG27zfj0EoQd7CfunbafDo9c3x55g/view?usp=sharing)  
[Логи запуска](https://drive.google.com/file/d/1QEGajXq3eJybhvswaCkATVJhRs7hSCsq/view?usp=sharing)  
Презентация:[ОУПРПО. Итерация 3. Проект 12](https://drive.google.com/file/d/1SChuYAM4CD_IQHj4QtwAoYbMkWu40w0_/view?usp=sharing)


#
#

# Итерация 2
## Версия 1
Чтобы запустить рой из одного дрона [(wiki скрипта)](https://github.com/OSLL/tello-dev/wiki/%D0%97%D0%B0%D0%BF%D1%83%D1%81%D0%BA-%D0%BE%D0%B4%D0%B8%D0%BD%D0%BE%D1%87%D0%BD%D0%BE%D0%B3%D0%BE-%D0%B4%D1%80%D0%BE%D0%BD%D0%B0), необходимо выполнить следующие команды:    
`docker build -t single_drone_swarm .`   
`sudo docker run single_drone_swarm`  
[Видео запуска из лаборатории](https://drive.google.com/file/d/1U_BEifhNsm7GAZkeG5-yhBLDRsZn3LQ_/view)
## Скрипты для создания физических сетевых интерфейсов в сети docker
Инструкция находится на [wiki-странице](https://github.com/OSLL/tello-dev/wiki/%D0%A4%D0%B8%D0%B7%D0%B8%D1%87%D0%B5%D1%81%D0%BA%D0%BE%D0%B5-%D1%83%D1%81%D1%82%D1%80%D0%BE%D0%B9%D1%81%D1%82%D0%B2%D0%BE). Работает по wifi-адаптер Realtek 802.11ac.
## Для создания контейнеров для каждого дрона, подключаемых к созданной сети  
`docker build -t single_drone_swarm .`    
`python run_swarm.py`     
Контейнеры отключаются через 21 секунду без дронов.
## Презентация и видео
[Видео запуска из лаборатории](https://drive.google.com/file/d/1U_BEifhNsm7GAZkeG5-yhBLDRsZn3LQ_/view)  
Презентация:[ОУПРПО. Итерация 2. Проект 12.pdf](https://github.com/OSLL/tello-dev/files/14780046/2.12.pdf)

#
#

# Обучение управлению дронами Tello

## Зависимости

Необходимые библиотеки лежат в `requirements.txt`.

Установка происходит командой `pip3 install -r requirements.txt`

## Запуск

Перед запуском решений необходимо подключиться к сети дрона.

Все решения запускаются через `main.py`: `python3 main.py -s <solution>`

Текущие решения:
* `default` -- решение по-умолчанию: после взлета поднимает дрона на 150см, поворачивает его на 360 градусов и приземляет его.
* `simple_lib_demo` -- простой пример использования дрона при помощи библиотеки djitellopy ([wiki страничка про библиотеки](./wiki/simple_using.md)). Программа дрона:
  * пролететь вперёд на 100см
  * подняться на 100см
  * повернуться вправо на 90 градусов
  * опуститься на 50см
  * пролететь влево на 100см
  * сдлеать сальто назад, вправо и вперёд
* `simple_socket_demo` -- использование дрона при помощи сокетов без каких-либо других библиотек. Решение максимально простое, поэтому нестабильное. Программа дрона:
  * пролететь вперёд на 100см
  * подняться на 100см
  * повернуться на 360 градусов вправо
* `test_all_sensors_demo` -- проверка всех сенсоров. Программа дрона как у `simple_lib_demo` решения. При этом на текущее видео с дрона наносятся данные с его сенсоров и датчиков.
* `follow_marker_with_coords` -- следование за маркером при помощи опредлении координат маркера относительно дрона. Видео с дрона записывается в `drone_stream.avi` файл. *Требуется откалиброванная камеры*
* `follow_marker_with_pixels` -- аналогично `follow_marker_with_coords`, только координаты вычисляются при помощи смещения маркера от центра изображения, перемещение дрона всегда на заданное расстояние по одной оси координат. Видео с дрона записывается в `drone_stream.avi` файл.
* `camera_calibration` -- калибровка камеры. Результаты будут в папке `calibration_results`

Если решение требует параметров камеры, то оно будет пытаться их считать из файла `camera_setting.npy`

## Структура папок

|     Папка         |        Описание         |
|:------------------|:------------------------|
|     `drone`       | Базовые классы для взаимодействия с дроном, которые используются решениями |
|     `utils`       | Общие полезные функции |
|     `wiki`        | Wiki-странички с полезной информацией |
|     `ros`         | Использование дрона при помощи ROS |
|   `solutions`     | Решения для запуска на дроне |
| `solutions/calibration` | Калибровка дрона: камера |
| `solutions/simple_use`  | Простое использование дрона. Решения `simple_lib_demo`, `simple_socket_demo`, `test_all_sensors_demo` |
| `solutions/following`   | Решения для следования за маркером: `follow_marker_with_coords`, `follow_marker_with_pixels` |


## ROS

Использование Tello дронов с ROS находится с [папке `ros`](./ros)

## Полезные ссылки

* [Оффициальные мануалы](https://www.ryzerobotics.com/tello-edu/downloads)
* [Оффициальные примеры кода](https://github.com/dji-sdk/Tello-Python)
* [Github репозиторий библиотеки DJITelloPy](https://github.com/damiafuentes/DJITelloPy)
