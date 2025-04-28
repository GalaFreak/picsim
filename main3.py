import math

program = []

w = 0
f = ''

carryflag = 0
zeroflag = 0

valuesBank0 = {}
valuesBank1 = {}
for i in range(0, 16 * 4):
    valuesBank0[i] = 0
    valuesBank1[i] = 0

filemap = { 
    'TMR0': 1,
    'OPTION': 1,
    'PCL': 2,
    'STATUS': 3,
    'FSR': 4,
    'PORTA': 5,
    'PORTB': 6,
    'TRISA': 5,
    'TRISB': 6,
    'EEDATA': 8,
    'EECON1': 8,
    'EEADR': 9,
    'EECON2': 9,
    'PCLATH': 10,
    'INTCON': 11,
}

functions = {}

bank = False
startadresse = 0
curIndex = 0

def programmAuswerten(dateipfad, breakpoints):
    global program
    with open(dateipfad, 'r') as file:
        program = file.readlines()
    
    curIndex2 = 0
    
    for line in program:
        finn = line.split(" ")
        if len(finn) == 1:
            functions.add(finn[0], curIndex2)
        curIndex2 += 1
    

def convertHex(hexString):
    if hexString.contains("h"):
        hexString = hexString.split("h")[0]
        return int(hexString, 16)
    else:
        return int(hexString)

def convertBin(binString):
    if binString.contains("b"):
        binString = binString.split("b")[0]
        return int(binString, 2)
    else:
        return int(binString)
    
def checkIndex(index):
    if index == "w" or index == "W":
        return -3
    if index.isnumeric():
        return index
    elif index.replace("h", "").isnumeric():
        return convertHex(index)
    elif index.replace("b", "").isnumeric():
        return convertBin(index)
    else:
        if filemap.__contains__(index.upper()):
            return filemap[index.upper()]
        else:
            return -1

def commandExecute(command):
    global w, f, carryflag, zeroflag, valuesBank0, valuesBank1, bank, startadresse, curIndex
    jonas = command.split(" ")
    if jonas[0] == ';':
        return
    
    if jonas.contains("ORG"):
        startadresse = jonas[1]
        
    if jonas.contains("EQU") and not filemap.__contains__(jonas[0].upper()):
        filemap.add(jonas[0].upper(), convertHex(jonas[2]))
    

    match jonas[0]:
        case 'GOTO':
            label = jonas[1]
            curIndex = functions[label]
            
        case 'MOVF':
            f = checkIndex(jonas[1].split(",")[0])
            # Hier weiter machen
            d = jonas[1].split(",")[1]
            if d == '0':
                w = f
            else:
                if d == '0':
                    zeroflag = 1
                else:
                    zeroflag = 0
                
        case 'MOVWF':
            f = jonas[1]
            values[indexes[f]] = w

        case 'MOVLW':
            k = jonas[1]
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
            b = jonas[1].split(",")[1]
            if not math.log2(values[indexes[f]] - pow(2, b)).is_integer():
                values[indexes[f]] += pow(2, b)
            
        case 'BCF':
            f = jonas[1].split(",")[0]
            b = jonas[1].split(",")[1]
            if not math.log2(values[indexes[f]] + pow(2, b)).is_integer():
                values[indexes[f]] -= pow(2, b)

        case 'RLF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            values[indexes[f]] *= 2
            if values[indexes[f]] > 255:
                values[indexes[f]] -= 256
                carryflag = 1
            if d == '0':
                w = values[indexes[f]]
            
        case 'RRF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            values[indexes[f]] //= 2
            if values[indexes[f]] % 2 == 1:
                carryflag = 1
            if d == '0':
                w = values[indexes[f]]

        # SWAPF geskippt

        # Ab hier hat Copilot gekocht

        case 'ADDWF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            if d == '0':
                w = values[indexes[f]] + w
                if w > 255:
                    carryflag = 1
                    w -= 256
            else:
                values[indexes[f]] += w
                if values[indexes[f]] > 255:
                    carryflag = 1
                    values[indexes[f]] -= 256
        
        case 'ADDLW':
            k = jonas[1]
            w = w + k
            if w > 255:
                carryflag = 1
                w -= 256
        
        case 'SUBWF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            if d == '0':
                w = w - values[indexes[f]]
                if w < 0:
                    carryflag = 1
                    w += 256
            else:
                values[indexes[f]] -= w
                if values[indexes[f]] < 0:
                    carryflag = 1
                    values[indexes[f]] += 256
            
        case 'SUBLW':
            k = jonas[1]
            w = k - w
            if w < 0:
                carryflag = 1
                w += 256

        case 'ANDWF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            if d == '0':
                w = values[indexes[f]] & w
            else:
                values[indexes[f]] &= w
        
        case 'ANDLW':
            k = jonas[1]
            w = w & k

        case 'IORWF':
            f = jonas[1].split(",")[0]
            d = jonas[1].split(",")[1]
            if d == '0':
                w = values[indexes[f]] | w
            else:
                values[indexes[f]] |= w

