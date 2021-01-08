# Требования

1. листа металла под датчиком нет, датчик в "низком", Ардуина в 'ожидании';
1. лист металла зашёл под датчик, датчик перешёл в "высокое" (начало цикла);
1. Ардуина увидела "высокое" с датчика и ждёт определенное время  сек, (подлежит настройке)
1. если "высокое" сохраняется, Ардуина активирует реле (количество и длительность включения, выключения реле подлежат настройке
1. если "высокое" пропало, то выходим из цикла.


# Инструкция

## При использовании дисплея с кнопками
После включения на дисплее должно отобразится сообщение типа:
```
Controller
v0.1
```
Через время заданное в параметре 'Total time' появится собщение о состоянии сенсора, например:
```
Sensor OFF
```
'Sensor OFF' -- под сенсором нет листа железа.
Если сенсор определит лист железа, то на дисплее отобразится:
```
Sensor ON 0 s
```
И начнётся отсчёт времени. После достижения времени заданного параметром 'Datect time' появится надпись:
```
Sensor ON 0 s
Plate detected
```
И начнёт включаться реле с временными задержками, заданными параметром 'Relay time'.
Если истечёт время, заданное параметром 'Total time', а под сенсором будет лист железа, то появится надпись:
```
Sensor ON
Time elapsed
```
Реле будет отключено до тех пор, пока лист железа не будет убран.

### Настройка устройства
При нажатии на любую кнопку устройство перейдёт в режим конфигурирования. При этом устройство перестанет реагировать на сенсор.
На дисплее в первой строке будет отображаться параметр, а во второй его значение. При этом будет мигающий курсор, который говорит о том, что можно менять значение.
Для измнения параметра нужно нажимать кнопки 'UP' или 'DOWN'. Для сохранения кнопку 'OK'. Для отмены изменений -- 'CANCEL'.
Нажатие кнопок 'OK' или 'CANCEL' приведёт к переходу к редактированию следующего параметра или к выходу из режима конфигурирования, если все параметры будут пройдены.

#### Total time
Первым параметром будет праметр 'Total time'. На дисплее появится надпись вида:
```
Total time:
5 sec
```
Параметр отвечает за общее время нахождения листа железа под сенсором. Если лист железа будет находится под сенсором больше этого времени, то реле перестанет работать, а на дисплее появится надпись 'Time elapsed'.

#### Detect time
Второй параметр отвечает за время "детектирования" листа железа под таймером.

ВНИМНИЕ!!! Это параметр должен быть меньше параметра 'Total time', в противном случае реле срабатывать не будет. Устройство не проверяет корректность параметров.

На дисплее будет надпись вида:
```
Detect time:
3 sec
```

#### Relay time
Tретий параметр отвечает за время включения/выключения реле. Время задаётся в миллисекундах. После того как устройство "детектирует" лист железа (см. параметр 'Detect time'), включится реле и питание на него будет подаваться в течении времени, заданного этим параметром. Затем питание с реле будет снято, опять на время, заданное этим параметром. Потом реле опять будет включено и так далее, пока не истечёт время, заданное параметром 'Total time'.

ВНИМНИЕ!!! Это параметр должен быть меньше параметра 'Total time' минус время 'Detect time', в противном случае реле срабатывать не будет.

На дисплее будет надпись вида:
```
Relay time:
400 ms
```

#### Buttons time
Отвечает за время простоя в режиме конфигурации. Если в течении этого времени не нажать ни на одну кнопку, то устройство выйдет из режима конфигурирования и начнёт опрашивать сенсор и управлять реле.
На дисплее будет надпись вида:
```
Buttons time:
20 sec
```

#### Sensor level
Определяет логический уровень на выходе сенсора, когда под ним находится лист железа. Если под сенсором находится лист железа и сенсор выдаёт низкий логический уровень, то значение должно быть 'LOW'. Иначе -- 'HIGH'.
На дисплее будет надпись вида:
```
Sensor level:
LOW
```

#### Relay Level
Определяет логический уровень управления реле. Если для включения реле нужно подать высокий уровень, то значение должно быть -- HIGH, иначе -- LOW.
```
Relay level:
LOW
```

## Настройка устройства через последовательный порт
Каждая каманда, отправляемая в контроллер через Serail-интерфейс, должна завершаться символом '\r' ( CR ) или '\n' ( NL ).
Команды и значения регистро-зависимые.
Доступны следующие команды:

### total
`total = ?` -- узнать текущее значение параметра 'Total time'.
`total = <число>` -- задать значение параметра 'Total time', например `total = 5`.

### detect
`detect = ?` -- узнать текущее значение параметра 'Detect time'.
`detect = <число>` -- задать значение параметра 'Detect time', например `detect = 3`. Должно быть меньше 'Total time'.

### relay
`relay = ?` -- узнать текущее значение параметра 'Relay time'.
`relay = <число>` -- задать значение параметра 'Relay time', например `relay = 400`. Должно быть меньше 'Total time' - 'Detect time'.

### buttons
`buttons = ?` -- узнать текущее значение параметра 'Buttons time'.
`buttons = <число>` -- задать значение параметра 'Buttons time', например `buttons = 20`.

### sensorLevel
`sensorLevel = ?` -- узнать текущее значение параметра 'Sensor Level'.
`sensorLevel = <low или high>` -- задать значение параметра 'Sensor Level', например `sensorLevel = low`. 

### relayLevel
`relayLevel = ?` -- узнать текущее значение параметра 'Relay Level'.
`relayLevel = <low или high>` -- задать значение параметра 'Relay Level', например `relayLevel = high`. 
