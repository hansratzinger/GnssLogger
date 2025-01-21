# Ablaufbeschreibung

## Speicher

1. SD-Card .csv-file  

2. Memory

3. RTC Memory für deep sleep

## Ablauf

1. Es werden mit dem GNSS-Modul Positionen laufend an das Programm über die serielle Schnittstelle (UART) geschickt.

2. Wenn die übermittelte Position gültige Positionswerte, ein aktuelles Datum und HDOP kleiner 1 aufweist wird sie verarbeitet.

3. Modi:

    Mission-Modus: jede Position wird auf die Speicherkarte geschrieben und danach wird für 2 Sekunden in light sleep gewechselt. Dabei wird das Programm angehalten und nach aufwachen wieder fortgesetzt. Die Variablen bleiben im Speicher erhalten. Der Mission-Modus definiert sich dadurch, dass die neue Position ausserhalb eines definierten Abstandes, dem Positionskreis, liegt.

    Station-Modus: Liegt die neue Position innerhalb des Positionskreises der vorhergehenden Position, werden beide Positionen sowohl auf die SC-card wie auch in den Positionsspeicher geschrieben. Dieser umfasst 10 Positionen die alle gegenseitig innerhalb ihrer Positonskreise liegen müssen. Wenn der Positionsspeicher komplett gefüllt ist, wird in den Station-Modus gewechselt. Jeweils die letzte Position wird ins RTC-Memory geschrieben und danach für 5 Sekunden der deep sleep aktiviert. Dabei wird der ESP32 angehalten und der Speicher bis auf das RTC-Memory gelöscht. Nach 5 Sekunden wird der ESP vom wakupTimer aufgeweckt und startet neu. Durch die wakeupCause wird festgestellt, ob ein Reboot oder ein Wakeup vorliegt. Danach wird das RTC-Memory ausgelesen, worin die Daten der vorhergehenden Position gespeichert wurden. So kann verglichen werden, ob die neue Position innerhalb oder ausserhalb des Positonskreises der vorhergehenden Postition liegt und entsprechend verfahren werden. Liegt sie ausserhalb wird in den Missionmodus gewechselt und der Positionsspeicher gelöscht.