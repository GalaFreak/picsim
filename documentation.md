# PIC16F84 Simulator Dokumentation

Dieses Dokument bietet umfassende Informationen über den PIC16F84-Simulator und erläutert sowohl die Backend-Architektur als auch die Frontend-Benutzeroberfläche.

## Inhaltsverzeichnis

- [Systemübersicht](#systemübersicht)
- [Backend Dokumentation](#backend-dokumentation)
- [Frontend Dokumentation](#frontend-dokumentation)
- [Testprogramme](#testprogramme)
- [Bedienungsanleitung (Englisch)](#usage-guide)
- [Bedienungsanleitung (Deutsch)](#bedienungsanleitung-deutsch)
- [Speicher- und Registerbearbeitung](#speicher--und-registerbearbeitung)

## Systemübersicht

Der PIC16F84-Simulator ist ein umfassendes Entwicklungs- und Lehrmittel, das das Verhalten des PIC16F84-Mikrocontrollers simuliert. Er besteht aus zwei Hauptkomponenten:

1. **Backend (picsim_backend.py)**: Implementiert die Kernsimulation, einschließlich der Befehlsausführung, Speicherverwaltung und Peripheriesimulation.
2. **Frontend (picsim_frontend_pyqt.py)**: Bietet eine grafische Benutzeroberfläche mit PyQt5, die es Benutzern ermöglicht, mit dem Simulator zu interagieren, die Programmausführung zu visualisieren und I/O zu manipulieren.

## Backend Dokumentation

### Architektur

Der Backend-Simulator implementiert eine zyklusgenaue Simulation des PIC16F84-Mikrocontrollers, einschließlich:

- 14-Bit-Befehlsausführungs-Engine
- Speichermodell mit Banking
- Spezialfunktionsregister (SFRs)
- I/O-Ports und Peripheriegeräte
- Timer- und Interrupt-Verarbeitung
- EEPROM-Simulation

### Hauptkomponenten

#### Speichermodell

- **Programmspeicher**: 1K Wörter (14-Bit)
- **RAM**: 68 Byte allgemeine Zweckregister, 16 Spezialfunktionsregister
- **Banking**: Unterstützt Bank 0 und Bank 1 Umschaltung über STATUS.RP0 Bit
- **EEPROM**: 64 Byte Daten-EEPROM

#### Befehlsausführung

Die Befehlsausführung folgt dem PIC16F84-Datenblatt und implementiert:

- Byte-orientierte Dateiregisteroperationen
- Bit-orientierte Dateiregisteroperationen
- Literal- und Steueroperationen
- Sprung-, Aufruf- und Überspringoperationen

#### Peripheriesimulation

- **Timer0**: Mit Prescaler und internen/externen Taktquellenoptionen
- **I/O-Ports**: PORTA (5 Bit) und PORTB (8 Bit) mit konfigurierbarer Richtung
- **Interrupts**: Externer INT-Pin, Timer0-Überlauf, RB-Port-Änderung, EEPROM-Schreibvorgang abgeschlossen
- **EEPROM**: Lese- und Schreiboperationen mit korrekter Sequenzprüfung

### Wichtige Klassen und Methoden

#### PicSimulator

Hauptsimulationseinheit mit den folgenden wichtigen Methoden:

- `decode_execute(opcode)`: Dekodiert und führt einen einzelnen Befehl aus
- `get_ram(address)` / `set_ram(address, value)`: Speicherzugriff mit Banking-Unterstützung
- `update_timer0(cycles)`: Aktualisiert Timer0 basierend auf der Konfiguration
- `check_interrupts()`: Überprüft und verarbeitet Interrupt-Bedingungen
- `reset(por)`: Setzt den Simulator zurück (Power-On oder MCLR)
- `toggle_porta_pin(pin_index)` / `toggle_portb_pin(pin_index)`: Simuliert externe Eingangsänderungen

## Frontend Dokumentation

### GUI-Layout

Die GUI ist in mehrere Abschnitte unterteilt:

1. **Steuerfeld**: Programmladen, Ausführungskontrolle und Frequenzeinstellungen
2. **Laufzeitanzeige**: Zyklusanzahl und Ausführungszeitinformationen
3. **Spezialfunktionsregister (SFRs)**: Visualisierung und Bearbeitung der SFRs
4. **Stack-Anzeige**: Visualisierung des Stack-Inhalts
5. **I/O-Ports-Panel**: Interaktive Visualisierung von PORTA und PORTB
6. **Code-Listing**: Programmansicht mit aktueller Ausführungsposition und Breakpoint-Unterstützung
7. **Allzweckregister**: Ansicht und Bearbeitung der GPRs

### Hauptkomponenten

#### Steuerfeld
- Load LST-Datei, Testprogramm 14 und Testprogramm 15 Buttons
- Reset, Step, Run und Stop Buttons
- Frequenzsteuerung mit Schieberegler und Texteingabe

#### Code-Editor
- Zeigt LST-Datei mit Zeilennummern an
- Unterstützt Breakpoint-Umschaltung
- Hebt die aktuelle Ausführungszeile hervor

#### I/O-Visualisierung
- Interaktive Port-Pins, die für Eingänge umgeschaltet werden können
- Visuelle Indikatoren für Pin-Zustände (Eingang/Ausgang, High/Low)
- TRIS-Konfigurationsanzeige
- Farblich codierte Pin-Zustände (Eingänge/Ausgänge)

### Visualisierungsfunktionen

- **Stack**: Anzeige des 8-stufigen Hardware-Stacks mit Hervorhebung des TOS (Top of Stack)
- **SFRs**: Echtzeitaktualisierung der Anzeige aller Spezialfunktionsregister
- **Port-Pins**: Visuelle Darstellung der I/O-Pin-Zustände mit Tooltips, die detaillierte Informationen anzeigen
- **Ausführungshervorhebung**: Der aktuelle Befehl wird im Code-Listing hervorgehoben

## Testprogramme

### TPicSim14 - LED-Sequenzgenerator

Dieses Testprogramm demonstriert die grundlegende I/O-Manipulation, indem es ein "Leuchtband" (LED-Sequenz) Muster auf PORTB implementiert. Die Richtung der Sequenz wird durch den Zustand von RA0 bestimmt.

**Wichtige demonstrierte Funktionen:**
- Port-I/O-Konfiguration und -Manipulation
- Verwendung des Carry-Flags zur Sequenzsteuerung
- Bedingte Verzweigung basierend auf Eingaben
- Register-Rotationsoperationen (RRF/RLF)

### TPicSim15 - Port-Latch-Test

Dieses Programm testet die korrekte Funktion des Port-Latch-Mechanismus im PIC-Mikrocontroller. Es zeigt, wie der interne Latch-Wert erhalten bleibt, auch wenn ein Pin als Eingang konfiguriert ist, und wie dieser Wert am Pin erscheint, wenn der Pin wieder als Ausgang konfiguriert wird.

**Wichtige demonstrierte Funktionen:**
- Port-Konfiguration (TRIS-Register)
- Port-Latch-Verhalten
- Pin-Zustandspersistenz während I/O-Richtungsänderungen

## Usage Guide

### Loading and Running Programs

1. **Load a Program**: 
   - Click "Load LST" to select a custom program file
   - Click "Load Test 14" or "Load Test 15" to load one of the built-in test programs

2. **Execution Control**:
   - Click "Run" to start continuous execution
   - Click "Step" for single-instruction execution
   - Click "Stop" to pause execution
   - Click "Reset" to reset the microcontroller

3. **Setting Breakpoints**:
   - Click in the line number margin to toggle breakpoints
   - Execution will automatically pause when a breakpoint is hit

### I/O Manipulation

1. **Configuring I/O Direction**:
   - Modify TRISA/TRISB registers (1=input, 0=output)
   - Direction is displayed in the I/O panel as "i" (input) or "o" (output)

2. **Changing Input Values**:
   - Click on pin cells in the "PIN" row of the I/O tables
   - Only pins configured as inputs can be toggled
   - Pin values are color-coded (blue=high, red=low for inputs)

3. **Observing Output Values**:
   - Output pins show the value of the corresponding PORT register
   - Green indicates high output, dark gray indicates low output
   - Special case: RA4 shows "Z" when high (open-drain)

### Frequency Control

1. Set the oscillator frequency using either:
   - The text input field (0.1 MHz to 16.0 MHz)
   - The slider control
   
2. The execution time display ("Laufzeit") will update according to the set frequency

## Bedienungsanleitung (Deutsch)

Diese Anleitung erklärt die Bedienung der grafischen Benutzeroberfläche des PIC16F84 Simulators.

### Laden und Ausführen von Programmen

1.  **Laden einer LST-Datei**:
    *   Klicken Sie auf den "Load LST"-Button.
    *   Wählen Sie eine `.LST`-Datei aus dem Dateidialog aus. Der Code wird im Code-Editor angezeigt.
    *   Alternativ können die Testprogramme "TPicSim14.LST" und "TPicSim15.LST" über die dedizierten Buttons geladen werden.
2.  **Ausführungskontrolle**:
    *   **Reset**: Setzt den Simulator auf den Anfangszustand zurück (Programmzähler auf 0, Register zurücksetzen).
    *   **Step**: Führt den nächsten Befehl aus. Der aktuell ausgeführte Befehl wird im Code-Editor hervorgehoben.
    *   **Run**: Startet die kontinuierliche Ausführung des Programms mit der eingestellten Geschwindigkeit.
    *   **Stop**: Hält die kontinuierliche Ausführung an.

### Code-Editor und Breakpoints

1.  **Code-Anzeige**: Zeigt den geladenen Assembler-Code (`.LST`-Datei) mit Zeilennummern an.
2.  **Aktuelle Zeile**: Die als nächstes auszuführende Codezeile wird hervorgehoben.
3.  **Breakpoints setzen/entfernen**:
    *   Klicken Sie auf den Bereich links neben der Zeilennummer (im Zeilennummern-Bereich).
    *   Ein roter Punkt erscheint, um einen aktiven Breakpoint anzuzeigen.
    *   Ein erneuter Klick entfernt den Breakpoint.
    *   Wenn die Ausführung im "Run"-Modus einen Breakpoint erreicht, wird sie automatisch angehalten.

### Register und Speicher bearbeiten

1.  **Spezialfunktionsregister (SFRs)**:
    *   Die wichtigsten SFRs (W, PCL, STATUS, FSR, PCLATH, INTCON etc.) werden in Echtzeit angezeigt.
    *   Einige Register (z.B. W, PCLATH) können direkt bearbeitet werden. Klicken Sie in das Feld, geben Sie einen neuen Wert ein (hexadezimal mit `0x` oder dezimal) und drücken Sie Enter.
2.  **Allzweckregister (GPRs)**:
    *   Die Register im Speicherbereich (RAM) werden in einer Tabelle angezeigt.
    *   Klicken Sie auf einen Wert in der Tabelle, um ihn zu bearbeiten (hexadezimal mit `0x` oder dezimal). Drücken Sie Enter zur Bestätigung. Änderungen werden sofort wirksam.

### Stack-Anzeige

*   Zeigt den Inhalt des 8-stufigen Hardware-Stacks an.
*   Der aktuelle Top-of-Stack (TOS) wird farblich hervorgehoben.

### I/O-Ports (PORTA / PORTB)

1.  **Anzeige**:
    *   Die Tabellen zeigen den Zustand von PORTA und PORTB.
    *   **RA/RB Zeile**: Zeigt den Wert des jeweiligen Port-Registers (z.B. `PORTA`).
    *   **TRIS Zeile**: Zeigt die Datenrichtung an, die durch das TRIS-Register (TRISA/TRISB) festgelegt ist (`i` = Input, `o` = Output).
    *   **PIN Zeile**: Zeigt den tatsächlichen Zustand des Pins an.
2.  **Eingänge manipulieren**:
    *   Klicken Sie auf eine Zelle in der **PIN**-Zeile, um den Logikpegel eines **als Eingang konfigurierten** Pins zu ändern (zwischen 0 und 1).
    *   Dies simuliert ein externes Signal am Pin.
    *   Die Farbcodierung zeigt den Zustand an (z.B. Blau=High/1, Rot=Low/0 für Eingänge; Grün=High/1, Dunkelgrau=Low/0 für Ausgänge).
3.  **Ausgänge beobachten**:
    *   Der Zustand von als Ausgang konfigurierten Pins spiegelt den Wert im entsprechenden PORT-Register wider. Sie können nicht direkt durch Klicken geändert werden.
    *   Beachten Sie den Sonderfall RA4 (Open-Drain), der bei High-Pegel als 'Z' angezeigt wird.

### Frequenzsteuerung und Laufzeit

1.  **Frequenz einstellen**:
    *   Verwenden Sie den Schieberegler oder das Eingabefeld, um die Oszillatorfrequenz (in MHz) einzustellen (Bereich 0.1 bis 16.0 MHz).
2.  **Laufzeit-Anzeige**:
    *   **Cycles**: Zeigt die Anzahl der ausgeführten Befehlszyklen seit dem letzten Reset an.
    *   **Laufzeit**: Zeigt die simulierte Ausführungszeit in Mikrosekunden (µs) an, basierend auf der eingestellten Frequenz und der Anzahl der Zyklen. (Laufzeit = Zyklen * 4 / Frequenz).

## Speicher- und Registerbearbeitung

1. **Modifying Registers**:
   - Click on any editable register field and enter a new value
   - Values can be entered in hexadecimal (with 0x prefix) or decimal
   - Press Enter to confirm the change

2. **GPR Editing**:
   - All General Purpose Register values can be modified in a similar way
   - Changes take effect immediately
