import subprocess

# ваш код для запуска Docker Swarm

# Данные о сетевых интерфейсах
network_info = """
3: br-cfe453156857: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default 
link/ether 02:42:a6:60:e9:36 brd ff:ff:ff:ff:ff:ff
inet 172.20.0.1/16 brd 172.20.255.255 scope global br-cfe453156857
valid_lft forever preferred_lft forever
4: br-3ae656ee29b7: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default 
link/ether 02:42:84:36:d7:f4 brd ff:ff:ff:ff:ff:ff
inet 172.23.0.1/16 brd 172.23.255.255 scope global br-3ae656ee29b7
valid_lft forever preferred_lft forever
inet6 fe80::42:84ff:fe36:d7f4/64 scope link 
valid_lft forever preferred_lft forever
5: br-55d74f66c2b8: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default 
link/ether 02:42:98:96:2c:86 brd ff:ff:ff:ff:ff:ff
inet 172.21.0.1/16 brd 172.21.255.255 scope global br-55d74f66c2b8
valid_lft forever preferred_lft forever
6: br-57a5652aba95: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default 
link/ether 02:42:89:ba:29:47 brd ff:ff:ff:ff:ff:ff
inet 172.19.0.1/16 brd 172.19.255.255 scope global br-57a5652aba95
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
