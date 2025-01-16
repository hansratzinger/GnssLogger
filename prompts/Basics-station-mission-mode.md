Um Strom der Batterie zu sparen soll eine deep-sleep Funktionalität eingebaut werden. Das System soll dabei in zwei Modi betrieben werden: 

- Station
- Mission

### Station
Im Station-Modus wird alle 4 Sekunden eine Positionsmessung durchgeführt und gespeichert. Wenn sich die Position ausserhalb eines bestimmten Bereiches befindet, wird in den Mission-Modus gewechselt.  

### Mission
Im Mission-Modus wird zusätzlich zu den Positionsmessungen ein .csv-File auf der SD-Card geschrieben, in dem alle Positionsmessungen gespeichert werden. Wenn sich alle Positionen der letzten 5 Minuten innerhalb des Bereiches der letzten 3 Positionen befinden, wird wieder in den Station-Modus gewechselt.  

In beiden Modi wird das System nach einer gewissen Zeit in den deep-sleep geschickt.  

### Anforderungen
- Das System soll in der Lage sein, die Position zu bestimmen und zu speichern.  
- Das System soll in der Lage sein, die Positionen der letzten 5 Minuten zu speichern und zu überprüfen.  
- Das System soll in der Lage sein, die Positionen in einem .csv-File zu speichern.  

### Hinweise
- Die Positionen sollen in einer geeigneten Datenstruktur gespeichert werden.  
- Die Daten sollen auf der SD-Card in einem .csv-File gespeichert werden.  
- Der Bereich wird durch einen Kreis mit Radius 5m um die jeweilige Position definiert.

