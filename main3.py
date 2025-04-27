import math

w = 0
f = ''

carryflag = 0
zeroflag = 0

values = []
indexes = {

}
curIndex = 0

def programmAuswerten(dateipfad, breakpoints):
    a = 5

def commandExecute(command):
    jonas = command.split(" ")
    match jonas[0]:
        case 'MOVF':
            f = jonas[1].split(",")[0]
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
        