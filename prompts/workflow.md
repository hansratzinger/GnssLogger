# Ablaufbeschreibung

## Speicher

- SD-Card .csv-file  

- Arbeitsspeicher

- RTC Memory für deep sleep

## Ablauf

### Mission-Mode

Im Mission-Mode wird jede Position auf die Speicherkarte geschrieben und danach wird für 2 Sekunden in light sleep gewechselt. Dabei wird das Programm angehalten und nach aufwachen wieder fortgesetzt. Die Variablen bleiben im Speicher erhalten. Der Mission-Modus definiert sich dadurch, dass die neue Position ausserhalb eines definierten Abstandes, dem Positionskreis, zur vorhergehenden Position liegt. Liegt die neue Position innerhalb des Positionskreises der vorhergehenden Position, werden beide Positionen sowohl auf die SD-card wie auch in das Station-Mode-Memory (SMM) geschrieben. Dieses umfasst 10 Positionen, die alle gegenseitig innerhalb ihrer Positonskreise liegen müssen. Wenn das SMM komplett gefüllt ist, wird in den Station-Mode gewechselt.

### Station-Mode

Nach der Aktivierung des Station-Modus wird überprüft ob die aktuelle Position innerhalb der Positionskreise des SMM liegt. Wenn nicht wird in den Mission-Mode gewechselt und das SMM gelöscht. Wenn ja wird das SMM ins RTC-Memory geschrieben. Danach wird für 5 Sekunden der deep sleep aktiviert. Dabei wird der ESP32 angehalten und der Speicher bis auf das RTC-Memory gelöscht. Nach 5 Sekunden wird der ESP vom wakupTimer aufgeweckt und startet neu. Durch die wakeupCause wird festgestellt, ob ein Reboot oder ein Wakeup vorliegt. Bei wakeup vom Deep Sleep wird das RTC-Memory ausgelesen und die Überprüfung startet wieder wie oben beschrieben.  
