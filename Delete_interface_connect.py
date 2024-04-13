import subprocess

# ваш код для подключения контейнеров

# Данные о сетевых интерфейсах
network_info = """
7: docker0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default 
link/ether 02:42:77:d0:1c:38 brd ff:ff:ff:ff:ff:ff
inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0
valid_lft forever preferred_lft forever
inet6 fe80::42:77ff:fed0:1c38/64 scope link 
valid_lft forever preferred_lft forever
8: br-a04529330ebd: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default 
link/ether 02:42:79:98:57:42 brd ff:ff:ff:ff:ff:ff
inet 172.18.0.1/16 brd 172.18.255.255 scope global br-a04529330ebd
valid_lft forever preferred_lft forever
10: vethe32f082@if9: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue master br-3ae656ee29b7 state UP group default 
link/ether 16:bc:d1:b1:eb:2d brd ff:ff:ff:ff:ff:ff link-netnsid 1
inet6 fe80::14bc:d1ff:feb1:eb2d/64 scope link 
valid_lft forever preferred_lft forever
12: vetha2d4e99@if11: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue master docker0 state UP group default 
link/ether 42:1c:f3:87:c8:04 brd ff:ff:ff:ff:ff:ff link-netnsid 0
inet6 fe80::401c:f3ff:fe87:c804/64 scope link 
valid_lft forever preferred_lft forever
14: vethd0e3234@if13: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue master br-3ae656ee29b7 state UP group default 
link/ether 56:57:4a:1f:13:12 brd ff:ff:ff:ff:ff:ff link-netnsid 2
inet6 fe80::5457:4aff:fe1f:1312/64 scope link 
valid_lft forever preferred_lft forever
26: dm-f1fef7f465f8: <BROADCAST,NOARP,UP,LOWER_UP> mtu 1500 qdisc noqueue state UNKNOWN group default 
link/ether 52:aa:83:b0:a0:b9 brd ff:ff:ff:ff:ff:ff
inet6 fe80::80a0:7cff:fe8e:a9/64 scope link 
valid_lft forever preferred_lft forever
"""

# Разбиваем информацию на строки
lines = network_info.strip().split('\n')

# Проходим по каждой строке
for line in lines:
    # Разбиваем строку на части
    parts = line.split()
    # Проверяем, что строка содержит информацию о Docker-интерфейсе
    if parts[1].startswith('br-'):
        # Получаем имя интерфейса
        interface_name = parts[1][:-1]  # Убираем двоеточие в конце имени интерфейса
        # Удаляем интерфейс
        subprocess.run(["ip", "link", "delete", interface_name])
        print(f"Интерфейс {interface_name} удален")
