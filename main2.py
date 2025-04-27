import math

# Globale Variablen und Flags
w = 0
f = ''

carryflag = 0
zeroflag = 0
digitcarryflag = 0
negativeflag = 0
overflowflag = 0

values = [0] * 256  # Speicher für Register
indexes = {f"R{i}": i for i in range(256)}  # Register-Adressen
curIndex = 0

pfad = r"C:\Users\Tuluhan\Desktop\PIC Programme\PIC Programme\Test01.src"

def programmAuswerten(dateipfad, breakpoints):
    try:
        with open(dateipfad, 'r') as file:
            programm = file.readlines()  # Jede Zeile in eine Liste speichern
        for command in programm:
            if command:  # Nur nicht-leere Zeilen verarbeiten
                print(command)
                commandExecute(command)
        printAllRegisters()
    except FileNotFoundError:
        print(f"Die Datei '{dateipfad}' wurde nicht gefunden.")
    except Exception as e:
        print(f"Ein Fehler ist aufgetreten: {e}")

def commandExecute(command):
    if command[0] == ";":
        return
    jonas = command.split(" ")
    match jonas[0]:
        case 'MOVF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            if d == '0':
                w = values[indexes[f]]
            else:
                values[indexes[f]] = values[indexes[f]]
            zeroflag = 1 if values[indexes[f]] == 0 else 0

        case 'MOVWF':
            f = jonas[1]
            values[indexes[f]] = w

        case 'MOVLW':
            k = int(jonas[1])
            w = k

        case 'CLRF':
            f = jonas[1]
            values[indexes[f]] = 0
            zeroflag = 1

        case 'CLRW':
            w = 0
            zeroflag = 1

        case 'BSF':
            f = jonas[1].split(",")[0]
            b = int(jonas[1].split(",")[1])
            values[indexes[f]] |= (1 << b)

        case 'BCF':
            f = jonas[1].split(",")[0]
            b = int(jonas[1].split(",")[1])
            values[indexes[f]] &= ~(1 << b)

        case 'RLF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            temp = (values[indexes[f]] << 1) | carryflag
            carryflag = 1 if temp > 255 else 0
            temp &= 0xFF
            if d == '0':
                w = temp
            else:
                values[indexes[f]] = temp

        case 'RRF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            temp = (carryflag << 8) | values[indexes[f]]
            carryflag = temp & 1
            temp >>= 1
            if d == '0':
                w = temp
            else:
                values[indexes[f]] = temp

        case 'ADDWF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            result = values[indexes[f]] + w
            carryflag = 1 if result > 255 else 0
            result &= 0xFF
            if d == '0':
                w = result
            else:
                values[indexes[f]] = result

        case 'ADDLW':
            k = int(jonas[1])
            result = w + k
            carryflag = 1 if result > 255 else 0
            w = result & 0xFF

        case 'SUBWF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            result = w - values[indexes[f]]
            carryflag = 1 if result < 0 else 0
            result &= 0xFF
            if d == '0':
                w = result
            else:
                values[indexes[f]] = result

        case 'SUBLW':
            k = int(jonas[1])
            result = k - w
            carryflag = 1 if result < 0 else 0
            w = result & 0xFF

        case 'ANDWF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            if d == '0':
                w = values[indexes[f]] & w
            else:
                values[indexes[f]] &= w

        case 'ANDLW':
            k = int(jonas[1])
            w &= k

        case 'IORWF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            if d == '0':
                w = values[indexes[f]] | w
            else:
                values[indexes[f]] |= w

        case 'IORLW':
            k = int(jonas[1])
            w |= k

        case 'XORWF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            if d == '0':
                w = values[indexes[f]] ^ w
            else:
                values[indexes[f]] ^= w

        case 'XORLW':
            k = int(jonas[1])
            w ^= k

        case 'DECF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            result = values[indexes[f]] - 1
            zeroflag = 1 if result == 0 else 0
            result &= 0xFF
            if d == '0':
                w = result
            else:
                values[indexes[f]] = result

        case 'DECFSZ':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            result = values[indexes[f]] - 1
            result &= 0xFF
            if result == 0:
                curIndex += 1
            if d == '0':
                w = result
            else:
                values[indexes[f]] = result

        case 'INCF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            result = values[indexes[f]] + 1
            zeroflag = 1 if result == 0 else 0
            result &= 0xFF
            if d == '0':
                w = result
            else:
                values[indexes[f]] = result

        case 'INCFSZ':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            result = values[indexes[f]] + 1
            result &= 0xFF
            if result == 0:
                curIndex += 1
            if d == '0':
                w = result
            else:
                values[indexes[f]] = result

        case 'NOP':
            pass

        case 'RETURN':
            pass  # Rücksprunglogik implementieren

        case 'RETFIE':
            pass  # Rücksprunglogik mit Interrupt implementieren

        case 'SLEEP':
            pass  # Schlafmodus implementieren

        case 'CLRWDT':
            pass  # Watchdog-Timer zurücksetzen

def printAllRegisters():
    for i in range(256):
        print(f"R{i}: {values[i]}\n")


programmAuswerten(pfad, [])