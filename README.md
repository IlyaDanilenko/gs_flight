# Описание пакета gs_flight

## Описание:
Данный пакет предоставляет инструменты для взаимодействия с автопилотом

## Состав пакета:
Классы:
* FlightController
* CallbackEvent

## Описание классов:

### 1. FlightController
Класс взаимодействия с автопилотом

#### Инициализация:
FlightController(callback) - callback функция реации на событие автопилота, должна иметь аргумент event

#### Поля:
* __alive - rospy.ServiceProxy: gs_interfaces.srv.Live
* __event_service - rospy.ServiceProxy: gs_interfaces.srv.Event
* __yaw_service - rospy.ServiceProxy: gs_interfaces.srv.Yaw
* __local_position_service- rospy.ServiceProxy: gs_interfaces.srv.Position
* __global_position_service - rospy.ServiceProxy: gs_interfaces.srv.PositionGPS
* __callback_event - rospy.Subscriber: std_msgs.msg.Int32

#### Методы:
* goToLocalPoint(x,y,z,time) - приказывает автопилоту лететь в локальные координаты, x - координата точки по оси x, в метрах , y - координата точки по оси y, в метрах, z- координата точки по оси z, в метрах, time - время, за которое коптер перейдет в следующую точку, в секундах. Если значение не указано, коптер стремится к точке с максимальной скоростью
* goToPoint(latitude,longitude,altitude) - приказывает автопилоту лететь в GPS координаты, latitude – задается широта в градусах, умноженных на 10^(-7), longitude – задается долгота в градусах, умноженных на 10^(−7), altitude – задается высота в метрах
* updateYaw(angle) - установливает рыскание, angle - угол в радианах
* preflight - приказывает сделать преполетную подготовку
* takeoff - приказывает выполнить взлет
* landing - приказывает выполнить посадку
* disarm - приказывает заглушить двигатели

#### Используемые сервисы:
* geoscan/alive (gs_interfaces/Live)
* geoscan/flight/set_event (gs_interfaces/Event)
* geoscan/flight/set_yaw (gs_interfaces/Yaw)
* geoscan/flight/set_local_position (gs_interfaces/Position)
* geoscan/flight/set_global_position (gs_interfaces/PositionGPS)

#### Используемые топики:
* geoscan/flight/callback_event (std_msgs/Int32)

### 2. CallbackEvent
Класс событий автопилота

#### Поля:
* ALL - пустое событие
* COPTER_LANDED - приземлился
* LOW_VOLTAGE1 - низкий заряд АКБ, но заряда хватит, чтобы вернуться домой
* LOW_VOLTAGE2 - низкий заряд АКБ, начата экстренная посадка
* POINT_REACHED - точка достигнута
* POINT_DECELERATION - близко к заданной точке
* TAKEOFF_COMPLETE - взлет выполнен
* ENGINES_STARTED - двигатели запущены
* SHOCK - сильный удар, возможна потеря управления

#### Инициализация:
Без параметров

## Необходимые пакеты:
ROS:
* gs_interfaces
* gs_core
* std_msgs
* geometry_msgs

## Примечание:
Все классы в данном пакете могут быть использованы только при запущеной ноде ros_plaz_node.py из пакета gs_core
