Создание образа из докер файла:
```bash
$ bash build.sh
```

Запуск конейнера:
```bash
$ bash start.sh
```

Подключение к контейнеру:
```bash
$ bash into.sh
```

Запуск узла сегментации:
```bash
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch segmentator main.launch \
      camera_ns:=/kitti/camera_color_left \
      image_topic:=image_raw \
      colorize:=true \
      rviz:=true
```
