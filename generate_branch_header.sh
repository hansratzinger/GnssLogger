#!/bin/bash

# Debug-Ausgabe: Start des Skripts
echo "Starting generate_branch_header.sh script..."

# Ermitteln des aktuellen Git-Branch-Namens
BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD)
echo "Current Git Branch: $BRANCH_NAME"

# Überprüfen, ob der Branch-Name erfolgreich ermittelt wurde
if [ -z "$BRANCH_NAME" ]; then
  echo "Error: Could not determine the current Git branch."
  exit 1
fi

# Erstellen der Header-Datei im src-Verzeichnis
echo "Creating header file src/branch.h..."
echo "#ifndef BRANCH_H" > src/branch.h
echo "#define BRANCH_H" >> src/branch.h
echo "const String BRANCH = \"$BRANCH_NAME\";" >> src/branch.h
echo "#endif // BRANCH_H" >> src/branch.h
echo "Header file src/branch.h created successfully."
