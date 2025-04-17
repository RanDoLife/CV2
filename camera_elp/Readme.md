Задание 1.
Шаг 1. roslaunch camera_elp rqt_image_view.launch
Шаг 2. rosrun camera_elp image_raw_sub.py

Задание 2. 
Шаг 1. roslaunch camera_elp rqt_image_view_MJPEG.launch
Шаг 2. rosrun camera_elp move_detector.py
Шаг 3. Подвигайте рукой перед камерой
Шаг 4. В консоли выводится движение.

Задание 3. 
Шаг 1. roslaunch camera_elp rqt_image_view_MJPEG.launch
Шаг 2. rosrun camera_elp image_filter.py
Шаг 3. Задать rosparam set /target_color red/green/blue
Шаг 4. В rqt_image_view обновить топики
Шаг 5. Выбрать топик "/filtered/mask"