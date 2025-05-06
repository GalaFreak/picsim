# coding: utf-8
import sys
import re
import time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton, QLabel, QLineEdit,
    QVBoxLayout, QHBoxLayout, QGridLayout, QSplitter, QFrame, QGroupBox,
    QFileDialog, QMessageBox, QPlainTextEdit, QListWidget, QListWidgetItem,
    QScrollArea, QTableWidget, QTableWidgetItem, QHeaderView, QTabWidget,
    QStyleFactory, QTextEdit, QSlider
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot, QRect, QSize
from PyQt5.QtGui import QFont, QColor, QTextCharFormat, QTextCursor, QPalette, QIcon, QPainter, QKeySequence

# Import our backend simulator
from picsim_backend import *

class LineNumberArea(QWidget):
    """Widget for displaying line numbers and breakpoint indicators."""
    breakpointToggled = pyqtSignal(int)
    
    def __init__(self, editor):
        super().__init__(editor)
        self.editor = editor
        self.setMouseTracking(True)
    
    def sizeHint(self):
        return QSize(self.editor.lineNumberAreaWidth(), 0)
    
    def paintEvent(self, event):
        self.editor.lineNumberAreaPaintEvent(event)
    
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            # Convert y position to line number (1-based)
            block = self.editor.firstVisibleBlock()
            block_number = block.blockNumber()
            top = self.editor.blockBoundingGeometry(block).translated(self.editor.contentOffset()).top()
            bottom = top + self.editor.blockBoundingRect(block).height()
            
            # Find which line contains the click position
            while block.isValid() and top <= event.pos().y():
                if event.pos().y() < bottom:
                    self.breakpointToggled.emit(block_number + 1)  # 1-based line numbers
                    break
                block = block.next()
                top = bottom
                bottom = top + self.editor.blockBoundingRect(block).height()
                block_number += 1

    def mouseMoveEvent(self, event):
        """Shows a tooltip when hovering over the breakpoint area."""
        # The breakpoint indicator area is in the left part of the line number area
        breakpoint_area_width = 16  # Match the width used in lineNumberAreaPaintEvent
        
        if event.pos().x() < breakpoint_area_width:
            # Check if mouse is over a valid line
            block = self.editor.firstVisibleBlock()
            block_number = block.blockNumber()
            top = self.editor.blockBoundingGeometry(block).translated(self.editor.contentOffset()).top()
            bottom = top + self.editor.blockBoundingRect(block).height()
            
            is_over_line = False
            
            # Find which line contains the mouse position
            while block.isValid() and top <= event.pos().y():
                if event.pos().y() < bottom:
                    is_over_line = True
                    break
                block = block.next()
                top = bottom
                bottom = top + self.editor.blockBoundingRect(block).height()
                block_number += 1
            
            if is_over_line:
                self.setToolTip("Click to toggle breakpoint")
            else:
                self.setToolTip("")  # Clear tooltip if not over a line
        else:
            self.setToolTip("")  # Clear tooltip if not in breakpoint area
            
        super().mouseMoveEvent(event)

class CodeEditorWithBreakpoints(QPlainTextEdit):
    """Enhanced code editor with line numbers and breakpoint toggles."""
    breakpointToggled = pyqtSignal(int)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.setFont(QFont("Courier New", 10))
        
        # Track breakpoints (line numbers with breakpoints)
        self.breakpoints = set()
        
        # Create line number area
        self.lineNumberArea = LineNumberArea(self)
        self.lineNumberArea.breakpointToggled.connect(self.toggleBreakpoint)
        
        # Connect signals for line number area updates
        self.blockCountChanged.connect(self.updateLineNumberAreaWidth)
        self.updateRequest.connect(self.updateLineNumberArea)
        self.cursorPositionChanged.connect(self.highlightCurrentLine)
        
        # Initialize line number area width
        self.updateLineNumberAreaWidth(0)
    
    def toggleBreakpoint(self, line):
        """Emits signal when breakpoint toggled."""
        self.breakpointToggled.emit(line)
    
    def lineNumberAreaWidth(self):
        """Calculate width of line number area."""
        digits = 1
        max_num = max(1, self.blockCount())
        while max_num >= 10:
            max_num //= 10
            digits += 1
        
        # Width = space for breakpoint indicator + spacing + space for digits + right margin
        space = 16 + self.fontMetrics().horizontalAdvance('9') * digits + 7
        return space
    
    def updateLineNumberAreaWidth(self, _):
        """Update viewport margins to accommodate line number area."""
        self.setViewportMargins(self.lineNumberAreaWidth(), 0, 0, 0)
    
    def updateLineNumberArea(self, rect, dy):
        """Handle updates to the line number area."""
        if dy:
            self.lineNumberArea.scroll(0, dy)
        else:
            self.lineNumberArea.update(0, rect.y(), self.lineNumberArea.width(), rect.height())
        
        if rect.contains(self.viewport().rect()):
            self.updateLineNumberAreaWidth(0)
    
    def resizeEvent(self, event):
        """Handle resize events."""
        super().resizeEvent(event)
        
        cr = self.contentsRect()
        self.lineNumberArea.setGeometry(QRect(cr.left(), cr.top(), self.lineNumberAreaWidth(), cr.height()))
    
    def lineNumberAreaPaintEvent(self, event):
        """Paint the line number area."""
        painter = QPainter(self.lineNumberArea)
        painter.fillRect(event.rect(), QColor("#2D2D30"))
        
        block = self.firstVisibleBlock()
        block_number = block.blockNumber()
        top = self.blockBoundingGeometry(block).translated(self.contentOffset()).top()
        bottom = top + self.blockBoundingRect(block).height()
        
        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                number = str(block_number + 1)
                line_number = block_number + 1  # 1-based line numbers
                
                # Draw breakpoint indicator to the left of line numbers
                if line_number in self.breakpoints:
                    indicator_size = 10
                    y_center = int(top) + self.fontMetrics().height() // 2
                    painter.setBrush(QColor(Qt.red))
                    painter.setPen(Qt.NoPen)
                    painter.drawEllipse(4, y_center - indicator_size // 2, indicator_size, indicator_size)
                
                # Draw line number with some spacing after breakpoint area
                painter.setPen(QColor(Qt.lightGray))
                text_x = 16  # Position after breakpoint area
                text_y = int(top)
                text_width = self.lineNumberArea.width() - text_x - 5  # Leave 5px right margin
                painter.drawText(text_x, text_y, text_width, 
                                self.fontMetrics().height(),
                                Qt.AlignRight, number)
            
            block = block.next()
            top = bottom
            bottom = top + self.blockBoundingRect(block).height()
            block_number += 1
    
    def highlightCurrentLine(self):
        """Highlight the current line."""
        extraSelections = []
        
        if not self.isReadOnly():
            selection = QTextEdit.ExtraSelection()
            selection.format.setBackground(QColor("#3E3E42"))
            selection.format.setProperty(QTextCharFormat.FullWidthSelection, True)
            selection.cursor = self.textCursor()
            selection.cursor.clearSelection()
            extraSelections.append(selection)
        
        self.setExtraSelections(extraSelections)

class HexSpinBox(QLineEdit):
    """Custom QLineEdit that displays and accepts hex values."""
    valueChanged = pyqtSignal(int)
    
    def __init__(self, parent=None, width=5, hex_prefix=True, read_only=False):
        super().__init__(parent)
        self.setFixedWidth(width * 10)  # Approximate width based on characters
        self.setAlignment(Qt.AlignRight)
        self.setFont(QFont("Courier New", 10))
        self.hex_prefix = hex_prefix
        self.read_only = read_only
        if read_only:
            self.setReadOnly(True)
        self.setMaxLength(width + (2 if hex_prefix else 0))
        self.returnPressed.connect(self.processInput)
        self._value = 0
    
    def setValue(self, value):
        """Set the value and update display."""
        self._value = value
        if self.hex_prefix:
            self.setText(f"0x{value:02X}")
        else:
            self.setText(f"{value:02X}")
    
    def processInput(self):
        if self.read_only:
            return
            
        try:
            text = self.text()
            if text.startswith("0x"):
                value = int(text, 16)
            else:
                value = int(text, 0)  # Auto-detect base
            
            self._value = value
            self.valueChanged.emit(value)
            # Format display
            if self.hex_prefix:
                self.setText(f"0x{value:02X}")
            else:
                self.setText(f"{value:02X}")
        except ValueError:
            # Restore previous value on error
            if self.hex_prefix:
                self.setText(f"0x{self._value:02X}")
            else:
                self.setText(f"{self._value:02X}")

class PicSimulatorGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Create backend simulator
        self.simulator = PicSimulator()
        
        # Use shorthand references for frequently accessed simulator attributes
        self.prog_mem = self.simulator.prog_mem
        self.ram = self.simulator.ram
        self.breakpoints = set()  # Keep breakpoints in GUI since they're related to UI
        self.step_mode = False
        self.last_pc = 0
        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.execute_cycle)
        
        self.setWindowTitle("PIC16F84 Simulator")
        self.setMinimumSize(1800, 900)
        
        # Create the central widget and main layout
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget)
        
        # Create main splitter
        self.main_splitter = QSplitter(Qt.Horizontal)
        self.main_layout.addWidget(self.main_splitter)
        
        # Left panel (Control, SFRs, Stack, I/O)
        self.left_panel = QWidget()
        self.left_layout = QVBoxLayout(self.left_panel)
        
        # Right panel (Code, GPRs)
        self.right_panel = QWidget()
        self.right_layout = QVBoxLayout(self.right_panel)
        
        self.main_splitter.addWidget(self.left_panel)
        self.main_splitter.addWidget(self.right_panel)
        
        # Adjust stretch factors: right panel (index 1) gets more stretch
        self.main_splitter.setStretchFactor(0, 1)  # Left panel stretch factor
        self.main_splitter.setStretchFactor(1, 2)  # Right panel stretch factor (larger)
        
        # Set initial sizes for the splitter based on 1800px minimum width
        # Increased left panel width from 600 to 680
        self.main_splitter.setSizes([610, 1190])
        
        # Initialize UI Components
        self.setup_control_panel()
        self.setup_runtime_panel()
        self.setup_sfr_panel()
        self.setup_stack_panel()
        self.setup_io_panel() # This now includes the I/O panel setup
        self.setup_code_panel()
        self.setup_gpr_panel()
        
        # Call reset AFTER all GUI elements are created
        self.reset(por=True)  # Power-on Reset
        
        # Initial GUI Update
        self.update_gui()
        
        # Apply dark mode theme
        self.apply_dark_theme()
    
    def apply_dark_theme(self):
        """Apply a dark theme to the application."""
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(53, 53, 53))
        palette.setColor(QPalette.WindowText, Qt.white)
        palette.setColor(QPalette.Base, QColor(25, 25, 25))
        palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ToolTipBase, Qt.white)
        palette.setColor(QPalette.ToolTipText, Qt.white)
        palette.setColor(QPalette.Text, Qt.white)
        palette.setColor(QPalette.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ButtonText, Qt.white)
        palette.setColor(QPalette.BrightText, Qt.red)
        palette.setColor(QPalette.Link, QColor(42, 130, 218))
        palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
        palette.setColor(QPalette.HighlightedText, Qt.black)
        
        QApplication.instance().setPalette(palette)
        QApplication.setStyle(QStyleFactory.create("Fusion"))
        
        # Custom stylesheets for specific components
        self.setStyleSheet("""
            QGroupBox {
                border: 1px solid gray;
                border-radius: 5px;
                margin-top: 0.5em;
                font-weight: bold;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 3px 0 3px;
            }
            QPushButton {
                background-color: #2D2D30;
                color: white;
                border: 1px solid #3E3E42;
                border-radius: 3px;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: #3E3E46;
            }
            QPushButton:pressed {
                background-color: #0078D7;
            }
            QPushButton:disabled {
                background-color: #2D2D30;
                color: #757575;
            }
            QLineEdit {
                background-color: #1E1E1E;
                color: #FFFFFF;
                border: 1px solid #3E3E42;
                border-radius: 2px;
                padding: 2px;
            }
            QTextEdit, QPlainTextEdit {
                background-color: #1E1E1E;
                color: #FFFFFF;
                border: 1px solid #3E3E42;
            }
            QListWidget {
                background-color: #1E1E1E;
                color: #FFFFFF;
                border: 1px solid #3E3E42;
            }
            QTableWidget {
                background-color: #1E1E1E;
                color: #FFFFFF;
                border: 1px solid #3E3E42;
                gridline-color: #3E3E42;
            }
        """)
    
    def setup_control_panel(self):
        """Set up the control panel with buttons and frequency input."""
        control_group = QGroupBox("Controls")
        control_layout = QGridLayout(control_group)
        
        # Control buttons
        self.load_button = QPushButton("Load LST (Ctrl+O)")
        self.load_button.clicked.connect(self.load_lst_file)
        self.load_button.setShortcut(QKeySequence("Ctrl+O"))
        control_layout.addWidget(self.load_button, 0, 0)
        
        self.reset_button = QPushButton("Reset (F2)")
        self.reset_button.clicked.connect(lambda: self.reset(por=True)) # Changed por=False to por=True
        self.reset_button.setShortcut(QKeySequence("F2"))
        control_layout.addWidget(self.reset_button, 0, 1)
        
        self.step_button = QPushButton("Step (F8)")
        self.step_button.clicked.connect(self.step_instruction)
        self.step_button.setShortcut(QKeySequence("F8"))
        control_layout.addWidget(self.step_button, 0, 2)
        
        self.run_button = QPushButton("Run (F9)")
        self.run_button.clicked.connect(self.run_program)
        self.run_button.setShortcut(QKeySequence("F9"))
        control_layout.addWidget(self.run_button, 0, 3)
        
        self.stop_button = QPushButton("Stop (F10)")
        self.stop_button.clicked.connect(self.stop_program)
        self.stop_button.setEnabled(False)
        self.stop_button.setShortcut(QKeySequence("F10"))
        control_layout.addWidget(self.stop_button, 0, 4)
        
        # Frequency setting
        freq_widget = QWidget()
        freq_layout = QHBoxLayout(freq_widget)
        freq_layout.setContentsMargins(0, 0, 0, 0)
        
        freq_label = QLabel("Quarz (MHz):")
        freq_layout.addWidget(freq_label)
        
        self.freq_edit = QLineEdit()
        self.freq_edit.setText(str(self.simulator.frequency_mhz))
        self.freq_edit.setFixedWidth(60)
        self.freq_edit.returnPressed.connect(self.update_frequency)
        freq_layout.addWidget(self.freq_edit)
        
        # Add slider for frequency control
        self.freq_slider = QSlider(Qt.Horizontal)
        self.freq_slider.setMinimum(1)  # Will be set to 0.1 using a scale factor
        self.freq_slider.setMaximum(160)  # Will represent 16.0 MHz (scaled by 10)
        self.freq_slider.setValue(int(float(self.simulator.frequency_mhz) * 10))  # Scale by 10
        self.freq_slider.setTickPosition(QSlider.TicksBelow)
        self.freq_slider.setTickInterval(10)  # Every 1 MHz
        # Make the slider expand to fill available space
        slider_policy = self.freq_slider.sizePolicy()
        slider_policy.setHorizontalStretch(1)
        self.freq_slider.setSizePolicy(slider_policy)
        self.freq_slider.setMinimumWidth(300)
        self.freq_slider.valueChanged.connect(self.update_frequency_from_slider)
        freq_layout.addWidget(self.freq_slider)
        
        # Remove stretch factor to allow slider to expand
        # freq_layout.addStretch(1)
        
        control_layout.addWidget(freq_widget, 1, 0, 1, 5)
        
        self.left_layout.addWidget(control_group)
    
    def setup_runtime_panel(self):
        """Set up the runtime display panel."""
        runtime_group = QGroupBox("Runtime")
        runtime_layout = QVBoxLayout(runtime_group)
        
        self.cycles_label = QLabel("Cycles: 0")
        runtime_layout.addWidget(self.cycles_label)
        
        self.laufzeit_label = QLabel("Laufzeit: 0.00 us")
        runtime_layout.addWidget(self.laufzeit_label)
        
        self.left_layout.addWidget(runtime_group)
    
    def setup_sfr_panel(self):
        """Set up the Special Function Registers panel."""
        sfr_group = QGroupBox("Special Function Registers (SFRs)")
        sfr_layout = QGridLayout(sfr_group)
        
        self.sfr_labels = {}
        self.sfr_values = {}
        
        sfr_to_display = {
            # Core
            "PC": self.simulator.pc, "W": self.simulator.w_reg, 
            "STATUS": 0, "PCL": 0, "PCLATH": 0, "FSR": 0,
            # Peripherals / Control
            "TMR0": 0, "OPTION": 0, "INTCON": 0,
            "PORTA": 0, "TRISA": 0, "PORTB": 0, "TRISB": 0,
            "EEDATA": 0, "EEADR": 0, "EECON1": 0,
            # Flags / Derived (Read-Only in GUI)
            "RP0": 0, "Z": 0, "C": 0, "DC": 0, "TO": 0, "PD": 0,
            "GIE": 0, "T0IE": 0, "T0IF": 0, "INTE": 0, "INTF": 0,
            "RBIE": 0, "RBIF": 0, "EEIE": 0, "EEIF": 0
        }
        
        # Order SFRs logically for display
        sfr_display_order = [
            "PC", "W", "STATUS", "PCL", "PCLATH", "FSR",
            "INTCON", "OPTION", "TMR0",
            "PORTA", "TRISA", "PORTB", "TRISB",
            "EEDATA", "EEADR", "EECON1",
            "C", "DC", "Z", "PD", "TO", "RP0",
            "GIE", "EEIE", "T0IE", "INTE", "RBIE",
            "EEIF", "T0IF", "INTF", "RBIF"
        ]
        
        # Find the longest label to determine consistent label width
        max_label_width = 0
        for name in sfr_display_order:
            if name in sfr_to_display:
                label_width = len(name) + 1  # +1 for colon
                max_label_width = max(max_label_width, label_width)
        
        row, col = 0, 0
        max_cols = 4  # Reduce columns to make more space for alignment
        
        for name in sfr_display_order:
            if name not in sfr_to_display:
                continue
                
            # Create label with fixed width and right alignment
            lbl = QLabel(f"{name}:")
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            lbl.setMinimumWidth(max_label_width * 10)  # Use font metrics for better sizing
            sfr_layout.addWidget(lbl, row, col * 2, 1, 1)  # Use col*2 to leave space between columns
            self.sfr_labels[name] = lbl
            
            # Determine entry configuration
            width = 5
            read_only = True
            
            if name in ["PC"]:
                width = 6
            elif len(name) > 3 or name in ["W", "STATUS", "PCL", "PCLATH", "FSR", "TMR0", 
                                          "OPTION", "INTCON", "PORTA", "TRISA", "PORTB", 
                                          "TRISB", "EEDATA", "EEADR", "EECON1"]:
                width = 5
                if name in ["W", "PCL", "PCLATH", "STATUS", "FSR", "TMR0", "OPTION", 
                           "INTCON", "PORTA", "TRISA", "PORTB", "TRISB", "EEDATA", 
                           "EEADR", "EECON1"]:
                    read_only = False
            
            # Create value display
            val_edit = HexSpinBox(width=width, read_only=read_only)
            val_edit.setValue(0)  # Initial value
            sfr_layout.addWidget(val_edit, row, col * 2 + 1, 1, 1)
            
            if not read_only:
                val_edit.valueChanged.connect(lambda v, n=name: self.edit_sfr(n, v))
            
            self.sfr_values[name] = val_edit
            
            col += 1
            if col >= max_cols:
                col = 0
                row += 1
        
        # Set column stretch factors to maintain alignment
        for i in range(max_cols * 2):
            if i % 2 == 0:  # Label columns
                sfr_layout.setColumnStretch(i, 1)
            else:  # Value columns
                sfr_layout.setColumnStretch(i, 0)
        
        self.left_layout.addWidget(sfr_group)

    def setup_stack_panel(self):
        """Set up the stack display panel."""
        # Create horizontal layout for stack and I/O
        stack_io_widget = QWidget()
        stack_io_layout = QHBoxLayout(stack_io_widget)
        stack_io_layout.setContentsMargins(0, 0, 0, 0)
        
        # Stack display
        stack_group = QGroupBox("Stack")
        stack_layout = QVBoxLayout(stack_group)
        
        self.stack_values = {}
        stack_widget = QWidget()
        stack_form_layout = QVBoxLayout(stack_widget)
        stack_form_layout.setSpacing(2)
        stack_form_layout.setContentsMargins(0, 0, 0, 0)
        
        for i in range(8):
            entry_widget = QWidget()
            entry_layout = QHBoxLayout(entry_widget)
            entry_layout.setContentsMargins(0, 0, 0, 0)
            entry_layout.setSpacing(0)  # Reduced spacing to 0
            
            # Stack index
            index_label = QLabel(f"{i}:")
            entry_layout.addWidget(index_label)
            
            # Stack value
            val_edit = HexSpinBox(width=6, read_only=True)
            val_edit.setValue(0)
            entry_layout.addWidget(val_edit)
            self.stack_values[i] = val_edit
            
            stack_form_layout.addWidget(entry_widget)
        
        stack_layout.addWidget(stack_widget)
        stack_io_layout.addWidget(stack_group, 0) # Stack group stretch factor to 0
        
        # Add the stack+IO panel to the main layout
        self.left_layout.addWidget(stack_io_widget)
        
        # I/O panel will be added in setup_io_panel()
        self.stack_io_widget = stack_io_widget
        self.stack_io_layout = stack_io_layout
    
    def setup_io_panel(self):
        """Set up the I/O ports panel with table view."""
        io_group = QGroupBox("I/O Ports")
        io_layout = QVBoxLayout(io_group)
        
        # Create Port A Table
        porta_group = QGroupBox("Port A")
        porta_layout = QVBoxLayout(porta_group)
        # Changed row count back to 3
        self.porta_table = QTableWidget(3, 9)  
        self.porta_table.setHorizontalHeaderLabels(["", "4", "3", "2", "1", "0", "", "", ""])
        # Removed "OUTPUT" label
        self.porta_table.setVerticalHeaderLabels(["RA", "TRIS", "PIN"]) 
        self.porta_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.porta_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        # Adjusted height back
        self.porta_table.setFixedHeight(120)  
        
        # Initialize PortA cells
        for col in range(9):
            # First column contains labels
            if col == 0:
                self.porta_table.setItem(0, col, QTableWidgetItem(""))
                self.porta_table.setItem(1, col, QTableWidgetItem(""))
                self.porta_table.setItem(2, col, QTableWidgetItem(""))
                continue
                
            # Disable unused pins (RA5-RA7 don't exist on PIC16F84)
            if col > 5:
                item1 = QTableWidgetItem("-")
                item2 = QTableWidgetItem("-")
                item3 = QTableWidgetItem("-")
                item1.setFlags(Qt.ItemIsEnabled)
                item2.setFlags(Qt.ItemIsEnabled)
                item3.setFlags(Qt.ItemIsEnabled)
                self.porta_table.setItem(0, col, item1)
                self.porta_table.setItem(1, col, item2)
                self.porta_table.setItem(2, col, item3)
            else:
                pin_idx = 5 - col  # Convert from column to pin number (RA4-RA0)
                # Set up items for TRIS and PIN with default values
                ra_item = QTableWidgetItem(f"RA{pin_idx}")
                tris_item = QTableWidgetItem("i")
                pin_item = QTableWidgetItem("0")
                
                ra_item.setTextAlignment(Qt.AlignCenter)
                tris_item.setTextAlignment(Qt.AlignCenter)
                pin_item.setTextAlignment(Qt.AlignCenter)
                
                # Make items read-only except PIN
                ra_item.setFlags(Qt.ItemIsEnabled)
                tris_item.setFlags(Qt.ItemIsEnabled)
                
                # Configure PIN cells to be clickable
                pin_item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
                
                # Add to table
                self.porta_table.setItem(0, col, ra_item)
                self.porta_table.setItem(1, col, tris_item)
                self.porta_table.setItem(2, col, pin_item)
        
        # Connect clicked signal for PIN row cells
        self.porta_table.cellClicked.connect(self.porta_pin_clicked)
        
        porta_layout.addWidget(self.porta_table)
        io_layout.addWidget(porta_group)
        
        # Create Port B Table
        portb_group = QGroupBox("Port B")
        portb_layout = QVBoxLayout(portb_group)
        # Changed row count back to 3
        self.portb_table = QTableWidget(3, 9) 
        self.portb_table.setHorizontalHeaderLabels(["", "7", "6", "5", "4", "3", "2", "1", "0"])
        # Removed "OUTPUT" label
        self.portb_table.setVerticalHeaderLabels(["RB", "TRIS", "PIN"]) 
        self.portb_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.portb_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        # Adjusted height back
        self.portb_table.setFixedHeight(120)  
        
        # Initialize PortB cells
        for col in range(9):
            # First column contains labels
            if col == 0:
                self.portb_table.setItem(0, col, QTableWidgetItem(""))
                self.portb_table.setItem(1, col, QTableWidgetItem(""))
                self.portb_table.setItem(2, col, QTableWidgetItem(""))
                continue
                
            pin_idx = 8 - col  # Convert from column to pin number (RB7-RB0)
            # Set up items for TRIS and PIN with default values
            rb_item = QTableWidgetItem(f"RB{pin_idx}")
            tris_item = QTableWidgetItem("i")
            pin_item = QTableWidgetItem("0")
            
            rb_item.setTextAlignment(Qt.AlignCenter)
            tris_item.setTextAlignment(Qt.AlignCenter)
            pin_item.setTextAlignment(Qt.AlignCenter)
            
            # Make items read-only except PIN
            rb_item.setFlags(Qt.ItemIsEnabled)
            tris_item.setFlags(Qt.ItemIsEnabled)
            
            # Configure PIN cells to be clickable
            pin_item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
            
            # Add to table
            self.portb_table.setItem(0, col, rb_item)
            self.portb_table.setItem(1, col, tris_item)
            self.portb_table.setItem(2, col, pin_item)
        
        # Connect clicked signal for PIN row cells
        self.portb_table.cellClicked.connect(self.portb_pin_clicked)
        
        portb_layout.addWidget(self.portb_table)
        io_layout.addWidget(portb_group)
        
        # Add IO group to the stack_io_layout
        self.stack_io_layout.addWidget(io_group, 1) # I/O group stretch factor to 1
    
    def porta_pin_clicked(self, row, col):
        """Handle clicks on Port A pins."""
        if row == 2 and col > 0 and col <= 5:  # PIN row (index 2) and valid pin columns (RA4-RA0)
            pin_idx = 5 - col  # Convert table column to pin number (RA4-RA0)
            # Attempt to toggle the pin state in the backend.
            # The backend toggle_porta_pin method checks if the pin is currently an input.
            if self.simulator.toggle_porta_pin(pin_idx):
                # If toggle was successful (pin was input), update the GUI.
                print(f"Toggled RA{pin_idx} input state via GUI click.")
                self.update_io_pins()
            else:
                # If toggle failed (pin was output), do nothing visually.
                print(f"Clicked RA{pin_idx}, but it's configured as output. No change.")
            # else: Clicked on an output pin, do nothing visually

    def portb_pin_clicked(self, row, col):
        """Handle clicks on Port B pins."""
        if row == 2 and col > 0:  # PIN row (index 2) and valid pin columns (RB7-RB0)
            pin_idx = 8 - col  # Convert table column to pin number (RB7-RB0)
            # Attempt to toggle the pin state in the backend.
            # The backend toggle_portb_pin method checks if the pin is currently an input.
            if self.simulator.toggle_portb_pin(pin_idx):
                # If toggle was successful (pin was input), update the GUI.
                print(f"Toggled RB{pin_idx} input state via GUI click.")
                self.update_io_pins()
            else:
                # If toggle failed (pin was output), do nothing visually.
                print(f"Clicked RB{pin_idx}, but it's configured as output. No change.")
            # else: Clicked on an output pin, do nothing visually

    def update_io_pins(self):
        """Updates the I/O pin displays based on current simulator state."""
        try:
            # Get current register values directly from simulator RAM
            # Note: PORT latches are read directly, pin states from simulator attributes
            porta_latch = self.simulator.ram[SFR_PORTA_ADDR] & 0x1F
            portb_latch = self.simulator.ram[SFR_PORTB_ADDR]
            # Need to read OPTION register correctly based on RP0
            status = self.simulator.ram[SFR_STATUS_ADDR]
            rp0 = (status >> STATUS_RP0) & 1
            option_addr = SFR_OPTION_REG_ADDR # OPTION is always at 0x81
            option = self.simulator.ram[option_addr] # For RBPU check
            rbpu_enabled = ((option >> OPTION_RBPU) & 1) == 0

            # Define colors for pin states
            color_high = QColor("#28A745")  # Green for High
            color_low = QColor("#D9534F")   # Red for Low
            color_hiz = QColor("#FFC107")   # Yellow for Hi-Z

            # Update Port A Table
            for i in range(5):  # RA0-RA4
                col = 5 - i  # Convert pin index to table column index
                # Use backend method to get TRIS state (correctly handles bank selection internally if needed)
                is_input = self.simulator.get_tris_a_bit(i) == 1
                pin_value = (self.simulator.porta_pins >> i) & 1 # Read simulated external pin level
                latch_value = (porta_latch >> i) & 1 # Read internal latch value

                # Update TRIS cell ('i' for input, 'o' for output)
                tris_cell = self.porta_table.item(1, col)
                tris_cell.setText('i' if is_input else 'o')
                tris_cell.setBackground(QColor("#4A4A4A")) # Neutral background for TRIS

                # Update PIN cell (shows external pin state)
                pin_cell = self.porta_table.item(2, col)
                if is_input:
                    # Pin is Input: Show the actual pin value (0 or 1)
                    pin_cell.setText('1' if pin_value else '0')
                    pin_cell.setBackground(color_high if pin_value else color_low)
                    pin_cell.setToolTip(f"RA{i}: Input Pin Value: {pin_value} (Click to toggle)")
                else: # Pin is Output
                    if i == 4: # RA4 is special (open-drain)
                        if latch_value: # Latch=1 -> Output is High-Impedance (Z)
                            pin_cell.setText('Z')
                            pin_cell.setBackground(color_hiz) 
                            pin_cell.setToolTip(f"RA{i}: Output Pin: Hi-Z (Open-drain high, Latch=1)")
                        else: # Latch=0 -> Output drives Low (0)
                            pin_cell.setText('0')
                            pin_cell.setBackground(color_low) 
                            pin_cell.setToolTip(f"RA{i}: Output Pin: Driving Low (Latch=0)")
                    else: # Normal Output (RA0-RA3)
                        # Show the value being driven (matches latch)
                        pin_cell.setText('1' if latch_value else '0')
                        pin_cell.setBackground(color_high if latch_value else color_low)
                        pin_cell.setToolTip(f"RA{i}: Output Pin: Driving {'High' if latch_value else 'Low'} (Latch={latch_value})")
            
            # Update Port B Table
            for i in range(8):  # RB0-RB7
                col = 8 - i  # Convert pin index to table column index
                # Use backend method to get TRIS state
                is_input = self.simulator.get_tris_b_bit(i) == 1
                pin_value = (self.simulator.portb_pins >> i) & 1 # Read simulated external pin level
                latch_value = (portb_latch >> i) & 1 # Read internal latch value
                pullup_active = is_input and rbpu_enabled # Pull-up only active if input and RBPU enabled

                # Update TRIS cell ('i' for input, 'o' for output, '↑' indicates pull-up enabled)
                tris_cell = self.portb_table.item(1, col)
                tris_text = 'i' + ('↑' if pullup_active else '') if is_input else 'o'
                tris_cell.setText(tris_text)
                tris_cell.setBackground(QColor("#4A4A4A")) # Neutral background for TRIS

                # Update PIN cell (shows external pin state)
                pin_cell = self.portb_table.item(2, col)
                if is_input:
                    # Pin is Input: Show the actual pin value (0 or 1)
                    pin_cell.setText('1' if pin_value else '0')
                    pin_cell.setBackground(color_high if pin_value else color_low)
                    tooltip = f"RB{i}: Input Pin Value: {pin_value} (Click to toggle)"
                    pin_cell.setToolTip(tooltip)
                else: # Pin is Output
                    # Show the value being driven (matches latch)
                    pin_cell.setText('1' if latch_value else '0')
                    pin_cell.setBackground(color_high if latch_value else color_low)
                    pin_cell.setToolTip(f"RB{i}: Output Pin: Driving {'High' if latch_value else 'Low'} (Latch={latch_value})")

            # Force repaint of the tables to ensure changes are visible
            self.porta_table.viewport().update()
            self.portb_table.viewport().update()

        except Exception as e:
            # Log error if updating fails
            print(f"Error updating I/O pins GUI: {e}")
            import traceback
            traceback.print_exc()

    def setup_code_panel(self):
        """Set up the code display panel."""
        code_group = QGroupBox("Code Listing (.LST)")
        code_layout = QVBoxLayout(code_group)
        
        # Use our enhanced code editor
        self.code_edit = CodeEditorWithBreakpoints()
        self.code_edit.breakpointToggled.connect(self.toggle_breakpoint)
        
        code_layout.addWidget(self.code_edit)
        
        self.right_layout.addWidget(code_group, 2)
    
    def setup_gpr_panel(self):
        """Set up the General Purpose Registers panel."""
        gpr_group = QGroupBox("General Purpose Registers (GPRs - Bank 0 View)")
        gpr_layout = QVBoxLayout(gpr_group)
        
        # Create a scroll area for GPRs
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        gpr_container = QWidget()
        gpr_grid = QGridLayout(gpr_container)
        gpr_grid.setHorizontalSpacing(5)  # Moderate horizontal spacing
        gpr_grid.setVerticalSpacing(2)
        
        self.gpr_labels = {}
        self.gpr_values = {}
        
        # Calculate the fixed width for address labels (all are 2 hex digits + colon)
        address_label_width = 30  # Reduced width for more compact display
        
        # Create GPR entries in a grid layout
        num_gpr_rows = (GPR_BANK0_END - GPR_BANK0_START + 1)
        gpr_cols = 10  # Back to original column count for compact display
        gpr_rows_per_col = (num_gpr_rows + gpr_cols - 1) // gpr_cols
        
        for i in range(num_gpr_rows):
            addr = GPR_BANK0_START + i
            addr_hex = f"{addr:02X}"
            
            col_num = (i // gpr_rows_per_col) * 2
            row_num = i % gpr_rows_per_col
            
            # Address label with right alignment and fixed width
            lbl = QLabel(f"{addr_hex}:")
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            lbl.setMinimumWidth(address_label_width)
            gpr_grid.addWidget(lbl, row_num, col_num)
            self.gpr_labels[addr] = lbl
            
            # Value entry
            val_edit = HexSpinBox(width=4, read_only=False)
            val_edit.setValue(0)
            val_edit.valueChanged.connect(lambda v, a=addr: self.edit_gpr(a, v))
            gpr_grid.addWidget(val_edit, row_num, col_num + 1)
            self.gpr_values[addr] = val_edit
        
        # Set column stretch factors - minimal values for tighter layout
        for i in range(gpr_cols * 2):
            if i % 2 == 0:  # Label columns
                gpr_grid.setColumnStretch(i, 0)  # Reduced stretch
            else:  # Value columns
                gpr_grid.setColumnStretch(i, 0)
        
        scroll_area.setWidget(gpr_container)
        gpr_layout.addWidget(scroll_area)
        
        self.right_layout.addWidget(gpr_group, 1)  # Less vertical space for GPRs
    
    # --- GUI Update Methods ---
    def update_gui(self):
        """Updates all GUI elements with current simulator state."""
        self.update_gui_stack()
        self.update_gui_runtime()
        self.update_gui_sfr_gpr()
        self.update_io_pins()
    
    def update_gui_sfr_gpr(self):
        """Updates the SFR and GPR displays."""
        # Update SFRs (including derived flags)
        status = self.ram[SFR_STATUS_ADDR]
        intcon = self.ram[SFR_INTCON_ADDR]
        porta_latch = self.ram[SFR_PORTA_ADDR]
        portb_latch = self.ram[SFR_PORTB_ADDR]
        
        # Read bank 1 regs only if RP0 is set
        rp0 = (status >> STATUS_RP0) & 1
        option = self.ram[SFR_OPTION_REG_ADDR] if rp0 == 1 else 0
        trisa = self.ram[SFR_TRISA_ADDR] if rp0 == 1 else 0xFF
        trisb = self.ram[SFR_TRISB_ADDR] if rp0 == 1 else 0xFF
        eecon1 = self.ram[SFR_EECON1_ADDR] if rp0 == 1 else 0
        
        sfr_map = {
            "PC": self.simulator.pc, "W": self.simulator.w_reg, 
            "STATUS": status, "PCL": self.ram[SFR_PCL_ADDR],
            "PCLATH": self.ram[SFR_PCLATH_ADDR], "FSR": self.ram[SFR_FSR_ADDR],
            "INTCON": intcon, "OPTION": option, "TMR0": self.ram[SFR_TMR0_ADDR],
            "PORTA": porta_latch, "TRISA": trisa, "PORTB": portb_latch, "TRISB": trisb,
            "EEDATA": self.ram[SFR_EEDATA_ADDR], "EEADR": self.ram[SFR_EEADR_ADDR],
            "EECON1": eecon1,
            # Flags / Derived
            "C": (status >> STATUS_C) & 1, "DC": (status >> STATUS_DC) & 1,
            "Z": (status >> STATUS_Z) & 1, "PD": (status >> STATUS_PD) & 1,
            "TO": (status >> STATUS_TO) & 1, "RP0": rp0,
            "GIE": (intcon >> INTCON_GIE) & 1, "EEIE": (intcon >> INTCON_EEIE) & 1,
            "T0IE": (intcon >> INTCON_T0IE) & 1, "INTE": (intcon >> INTCON_INTE) & 1,
            "RBIE": (intcon >> INTCON_RBIE) & 1,
            "EEIF": (eecon1 >> EECON1_EEIF) & 1,
            "T0IF": (intcon >> INTCON_T0IF) & 1, "INTF": (intcon >> INTCON_INTF) & 1,
            "RBIF": (intcon >> INTCON_RBIF) & 1
        }
        
        for name, val_edit in self.sfr_values.items():
            if name in sfr_map:
                current_val = sfr_map[name]
                val_edit.setValue(current_val)
        
        # Update GPRs
        for addr, val_edit in self.gpr_values.items():
            val_edit.setValue(self.ram[addr])
    
    def update_gui_stack(self):
        """Updates the stack display."""
        tos_index = (self.simulator.stack_ptr - 1 + 8) % 8
        
        for i in range(8):
            addr = self.simulator.stack[i]
            val_edit = self.stack_values[i]
            val_edit.setValue(addr)
            
            # Mark TOS with different color
            if i == tos_index:
                val_edit.setStyleSheet("background-color: #0078D7; color: white;")
            else:
                val_edit.setStyleSheet("")
    
    def update_gui_runtime(self):
        """Updates runtime counters."""
        self.cycles_label.setText(f"Cycles: {self.simulator.runtime_cycles}")
        
        if self.simulator.frequency_mhz > 0:
            # Runtime in microseconds = cycles * (instruction clock period)
            # For PIC, instruction clock period = 4 * oscillator period
            # So, laufzeit_us = cycles * (4 / frequency_MHz)
            self.simulator.laufzeit_us = self.simulator.runtime_cycles * (4.0 / self.simulator.frequency_mhz)
            self.laufzeit_label.setText(f"Laufzeit: {self.simulator.laufzeit_us:.2f} us")
        else:
            self.laufzeit_label.setText("Laufzeit: N/A (Freq=0)")
    
    def highlight_current_line(self):
        """Highlights the line corresponding to the current PC in the code view."""
        if hasattr(self, 'line_number_map') and self.simulator.pc in self.line_number_map:
            line_num = self.line_number_map[self.simulator.pc]
            
            # Get the cursor to scroll and highlight
            cursor = self.code_edit.textCursor()
            cursor.movePosition(QTextCursor.Start)
            cursor.movePosition(QTextCursor.Down, QTextCursor.MoveAnchor, line_num - 1)
            # Don't select the whole line, just position the cursor at the start
            cursor.movePosition(QTextCursor.StartOfLine) 
            
            # Set the cursor and scroll to it vertically, keeping horizontal scroll minimal
            self.code_edit.setTextCursor(cursor)
            self.code_edit.ensureCursorVisible() 
            
            # Reset horizontal scrollbar to the minimum (left) position
            h_scrollbar = self.code_edit.horizontalScrollBar()
            h_scrollbar.setValue(h_scrollbar.minimum())

            # Apply highlight format for current execution position
            highlight_format = QTextCharFormat()
            highlight_format.setBackground(QColor("yellow"))
            highlight_format.setForeground(QColor("black"))
            
            # Use a temporary cursor for selection to avoid moving the main cursor
            highlight_cursor = self.code_edit.textCursor()
            highlight_cursor.movePosition(QTextCursor.Start)
            highlight_cursor.movePosition(QTextCursor.Down, QTextCursor.MoveAnchor, line_num - 1)
            highlight_cursor.select(QTextCursor.LineUnderCursor) # Select the line for highlighting

            selection = QTextEdit.ExtraSelection()
            selection.format = highlight_format
            selection.cursor = highlight_cursor # Use the temporary cursor for the selection
            
            self.code_edit.setExtraSelections([selection])
        else:
             # Clear previous highlights if PC is not mappable
             self.code_edit.setExtraSelections([])
    
    # --- Control Methods ---
    def load_lst_file(self):
        """Opens a file dialog to load a LST file."""
        filepath, _ = QFileDialog.getOpenFileName(
            self, 
            "Open LST File",
            "",
            "LST files (*.LST *.lst);;Text files (*.txt);;All files (*.*)"
        )
        
        if not filepath:
            return
        
        try:
            self.prog_mem = [0x3FFF] * PROG_MEM_SIZE  # NOP
            self.code_edit.clear()
            self.breakpoints.clear()
            self.code_edit.breakpoints.clear()  # Clear editor's breakpoints
            self.line_number_map = {}  # Maps address to line number
            self.address_map = {}  # Maps line number to address
            max_addr = -1
            
            with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
                line_idx = 1
                for line in f:
                    # Add line to code editor
                    self.code_edit.appendPlainText(line.rstrip())
                    
                    # Parse line for address and opcode
                    match = re.match(r"^\s*([0-9A-Fa-f]{4})\s+([0-9A-Fa-f]{4})", line)
                    if match:
                        addr = int(match.group(1), 16)
                        opcode = int(match.group(2), 16)
                        
                        if addr < PROG_MEM_SIZE:
                            self.prog_mem[addr] = opcode
                            self.line_number_map[addr] = line_idx
                            self.address_map[line_idx] = addr
                            max_addr = max(max_addr, addr)
                        else:
                            print(f"Warning: Address {addr:04X} out of range in LST file.")
                    
                    line_idx += 1
            
            if max_addr == -1:
                QMessageBox.warning(self, "Load LST", "No valid code lines found in LST file.")
            else:
                QMessageBox.information(self, "Load LST", f"LST file loaded successfully. Max address: 0x{max_addr:03X}")
            
            self.reset(por=True)  # Reset processor after loading
            
        except Exception as e:
            QMessageBox.critical(self, "Load LST Error", f"Failed to load or parse LST file:\n{e}")
    
    def reset(self, por=True):
        """Resets the simulator state."""
        self.simulator.reset(por)
        self.simulator.running = False
        
        # Update GUI button states
        self.stop_button.setEnabled(False)
        self.run_button.setEnabled(True)
        self.step_button.setEnabled(True)
        
        self.update_gui()
        self.highlight_current_line()
        # Ensure I/O pins are updated on reset to reflect cleared registers
        self.update_io_pins() 
    
    def step_instruction(self):
        """Executes a single instruction cycle."""
        if self.simulator.running:
            return  # Don't step if running continuously
        
        try:
            self.step_mode = True
            self.last_pc = self.simulator.pc
            
            if self.simulator.pc >= PROG_MEM_SIZE:
                QMessageBox.warning(self, "Execution Halted", f"Program Counter (0x{self.simulator.pc:03X}) out of bounds.")
                return
            
            opcode = self.prog_mem[self.simulator.pc]
            
            # Check for NOP / unimplemented memory
            if opcode == 0x3FFF or opcode == 0x0000:
                print(f"PC=0x{self.simulator.pc:03X}: NOP (0x{opcode:04X})")
                cycles_taken = 1
                self.simulator.pc = (self.simulator.pc + 1) & 0x1FFF
            else:
                cycles_taken = self.simulator.decode_execute(opcode)
            
            self.simulator.runtime_cycles += cycles_taken
            
            # Simulate peripheral updates
            self.simulator.update_timer0(cycles_taken)
            self.simulator.check_interrupts()
            
            # Force I/O pin update after every instruction
            self.update_io_pins()
            self.update_gui()
            self.highlight_current_line()
            
        except ValueError as e:
            QMessageBox.critical(self, "Instruction Error", 
                            f"Error executing instruction at PC=0x{self.last_pc:03X} (Opcode: 0x{self.prog_mem[self.last_pc]:04X}):\n{e}")
            self.stop_program()
        except Exception as e:
            QMessageBox.critical(self, "Runtime Error", f"Unexpected error at PC=0x{self.last_pc:03X}:\n{e}")
            self.stop_program()
        finally:
            self.step_mode = False
    
    def run_program(self):
        """Runs the program continuously."""
        if self.simulator.running:
            return
        
        self.simulator.running = True
        self.run_button.setEnabled(False)
        self.step_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        
        # Use QTimer for execution cycles
        self.ui_timer.start(10)  # 10ms interval
    
    def stop_program(self):
        """Stops continuous execution."""
        self.simulator.running = False
        self.ui_timer.stop()
        
        self.run_button.setEnabled(True)
        self.step_button.setEnabled(True)
        self.stop_button.setEnabled(False)
    
    @pyqtSlot()
    def execute_cycle(self):
        """Executes instruction cycles in batches for better UI responsiveness."""
        if not self.simulator.running:
            self.stop_program()
            return
        
        try:
            # Execute a batch of instructions for better performance
            cycles_per_batch = 10
            for _ in range(cycles_per_batch):
                # Check for breakpoint
                if self.simulator.pc in self.breakpoints:
                    print(f"Breakpoint hit at address 0x{self.simulator.pc:03X}")
                    QMessageBox.information(self, "Breakpoint Hit", f"Breakpoint hit at address 0x{self.simulator.pc:03X}")
                    self.stop_program()
                    self.update_gui()
                    self.highlight_current_line()
                    return
                
                self.last_pc = self.simulator.pc
                
                if self.simulator.pc >= PROG_MEM_SIZE:
                    QMessageBox.warning(self, "Execution Halted", f"Program Counter (0x{self.simulator.pc:03X}) out of bounds.")
                    self.stop_program()
                    return
                
                opcode = self.prog_mem[self.simulator.pc]
                
                # NOP check
                if opcode == 0x3FFF or opcode == 0x0000:
                    cycles_taken = 1
                    self.simulator.pc = (self.simulator.pc + 1) & 0x1FFF
                else:
                    cycles_taken = self.simulator.decode_execute(opcode)
                
                self.simulator.runtime_cycles += cycles_taken
                self.simulator.update_timer0(cycles_taken)
                self.simulator.check_interrupts()
                
                # Check if we should exit the batch early (e.g., reached a breakpoint)
                if not self.simulator.running:
                    break
            
            # Update GUI after batch - ensure I/O pins are always updated
            self.update_io_pins()
            self.update_gui()
            self.highlight_current_line()
            
        except ValueError as e:
            QMessageBox.critical(self, "Instruction Error", 
                            f"Error executing instruction at PC=0x{self.last_pc:03X} (Opcode: 0x{self.prog_mem[self.last_pc]:04X}):\n{e}")
            self.stop_program()
            self.update_gui()
            self.highlight_current_line()
        except Exception as e:
            QMessageBox.critical(self, "Runtime Error", f"Unexpected error at PC=0x{self.last_pc:03X}:\n{e}")
            self.stop_program()
            self.update_gui()
            self.highlight_current_line()
    
    # --- GUI Interaction Methods ---
    def update_frequency(self):
        """Updates the simulator frequency from the entry field."""
        try:
            new_freq_str = self.freq_edit.text().replace(',', '.')  # Allow comma as decimal sep
            new_freq = float(new_freq_str)
            
            if new_freq >= 0.1 and new_freq <= 16.0:
                self.simulator.frequency_mhz = new_freq
                print(f"Frequency updated to {self.simulator.frequency_mhz} MHz")
                # Update slider without triggering valueChanged signal
                self.freq_slider.blockSignals(True)
                self.freq_slider.setValue(int(new_freq * 10))
                self.freq_slider.blockSignals(False)
                self.update_gui_runtime()
            else:
                QMessageBox.warning(self, "Invalid Frequency", "Frequency must be between 0.1 and 16.0 MHz.")
                self.freq_edit.setText(str(self.simulator.frequency_mhz))  # Revert
                # Update slider to match current value
                self.freq_slider.blockSignals(True)
                self.freq_slider.setValue(int(self.simulator.frequency_mhz * 10))
                self.freq_slider.blockSignals(False)
        except ValueError:
            QMessageBox.critical(self, "Invalid Input", "Please enter a valid number for frequency.")
            self.freq_edit.setText(str(self.simulator.frequency_mhz))  # Revert
            # Update slider to match current value
            self.freq_slider.blockSignals(True)
            self.freq_slider.setValue(int(self.simulator.frequency_mhz * 10))
            self.freq_slider.blockSignals(False)
        
        # Lose focus
        self.code_edit.setFocus()
    
    def update_frequency_from_slider(self, value):
        """Updates the simulator frequency from the slider."""
        new_freq = value / 10.0  # Convert from scaled value
        
        if new_freq >= 0.1 and new_freq <= 16.0:
            self.simulator.frequency_mhz = new_freq
            # Update text field to match
            self.freq_edit.setText(f"{new_freq:.1f}")
            self.update_gui_runtime()
            print(f"Frequency updated to {self.simulator.frequency_mhz} MHz")
        else:
            # Should not happen with proper slider range
            print(f"Warning: Invalid frequency from slider: {new_freq}")
    
    def toggle_breakpoint(self, line_number):
        """Toggles a breakpoint on the given line."""
        if not hasattr(self, 'address_map') or not self.address_map:
            print("Load LST file first to set breakpoints.")
            return
        
        # Check if line has an address
        if line_number in self.address_map:
            addr = self.address_map[line_number]
            
            # Toggle breakpoint
            if addr in self.breakpoints:
                # Remove breakpoint
                self.breakpoints.remove(addr)
                if line_number in self.code_edit.breakpoints:
                    self.code_edit.breakpoints.remove(line_number)
                print(f"Breakpoint removed at 0x{addr:03X}")
            else:
                # Add breakpoint
                self.breakpoints.add(addr)
                self.code_edit.breakpoints.add(line_number)
                print(f"Breakpoint set at 0x{addr:03X}")
            
            # Update the line number area
            self.code_edit.lineNumberArea.update()
        else:
            print(f"No address found on line {line_number} to set breakpoint.")
    
    # def toggle_porta_pin(self, pin_index):
    #     """Toggles the simulated input level of a PORTA pin."""
    #     if self.simulator.get_tris_a_bit(pin_index):
    #         self.simulator.porta_pins ^= (1 << pin_index)
    #         pin_level = (self.simulator.porta_pins >> pin_index) & 1
    #         print(f"Toggled RA{pin_index} input stimulus to {pin_level}")
    #         self.update_io_pins()
    
    # def toggle_portb_pin(self, pin_index):
    #     """Toggles the simulated input level of a PORTB pin."""
    #     if self.simulator.get_tris_b_bit(pin_index):
    #         prev_level = (self.simulator.portb_pins >> pin_index) & 1
    #         self.simulator.portb_pins ^= (1 << pin_index)
    #         current_level = (self.simulator.portb_pins >> pin_index) & 1
    #         print(f"Togged RB{pin_index} input stimulus to {current_level}")
            
    #         # Check for RB0/INT edge
    #         if pin_index == 0:
    #             option = self.ram[SFR_OPTION_REG_ADDR] if self.simulator.get_status_bit(STATUS_RP0) == 1 else 0xFF
    #             intedg = (option >> OPTION_INTEDG) & 1
    #             is_rising_edge = prev_level == 0 and current_level == 1
    #             is_falling_edge = prev_level == 1 and current_level == 0
                
    #             if (intedg == 1 and is_rising_edge) or (intedg == 0 and is_falling_edge):
    #                 if not self.simulator.get_intcon_bit(INTCON_INTF):
    #                     self.simulator.set_intcon_bit(INTCON_INTF)
    #                     print("INT/RB0 edge detected, INTF set.")
            
    #         self.update_io_pins()
    
    def edit_sfr(self, reg_name, value):
        """Updates a SFR when edited in the UI."""
        try:
            # Map register name to address
            address = -1
            is_bank_1_reg = False # Flag not strictly needed here but good practice

            # Handle W register separately as it's not memory-mapped
            if reg_name == "W":
                self.simulator.w_reg = value & 0xFF
            # Map other editable SFR names to their addresses
            elif reg_name == "PCL": address = SFR_PCL_ADDR
            elif reg_name == "PCLATH": address = SFR_PCLATH_ADDR
            elif reg_name == "STATUS": address = SFR_STATUS_ADDR
            elif reg_name == "FSR": address = SFR_FSR_ADDR
            elif reg_name == "INTCON": address = SFR_INTCON_ADDR
            elif reg_name == "TMR0": address = SFR_TMR0_ADDR
            elif reg_name == "PORTA": address = SFR_PORTA_ADDR
            elif reg_name == "PORTB": address = SFR_PORTB_ADDR
            elif reg_name == "EEDATA": address = SFR_EEDATA_ADDR
            elif reg_name == "EEADR": address = SFR_EEADR_ADDR
            # Bank 1 registers (use their fixed addresses)
            elif reg_name == "OPTION": address = SFR_OPTION_REG_ADDR; is_bank_1_reg = True
            elif reg_name == "TRISA": address = SFR_TRISA_ADDR; is_bank_1_reg = True
            elif reg_name == "TRISB": address = SFR_TRISB_ADDR; is_bank_1_reg = True
            elif reg_name == "EECON1": address = SFR_EECON1_ADDR; is_bank_1_reg = True

            if address != -1:
                # If a valid memory-mapped SFR was identified
                print(f"GUI Edit: Setting SFR {reg_name} (Addr: 0x{address:02X}) to 0x{value:02X}")
                # Use the backend's set_ram function to write the value.
                # This ensures any side effects (like calling handle_porta/b/trisa/b_write) are triggered.
                self.simulator.set_ram(address, value & 0xFF)

                # Crucially, after potentially changing PORT or TRIS registers,
                # immediately update the I/O pin display in the GUI to reflect the change.
                if reg_name in ["TRISA", "TRISB", "PORTA", "PORTB", "OPTION"]: # Added OPTION for RBPU changes
                    self.update_io_pins()
            elif reg_name != "W":
                # If the register name is not editable (e.g., PC, flags)
                print(f"GUI Edit: Cannot directly edit register '{reg_name}'.")

            # Refresh the entire SFR/GPR display after any edit attempt
            self.update_gui_sfr_gpr()
            # Also refresh stack and runtime in case STATUS/PCLATH etc. changed indirectly
            self.update_gui_stack()
            self.update_gui_runtime()

        except Exception as e:
            QMessageBox.critical(self, "SFR Edit Error", f"Error setting {reg_name}: {e}")
            # Revert display to current simulator state on error
            self.update_gui()

    def edit_gpr(self, address, value):
        """Updates a GPR when edited in the UI."""
        try:
            if GPR_BANK0_START <= address <= GPR_BANK0_END:
                print(f"Editing GPR 0x{address:02X} to 0x{value:02X}")
                self.simulator.set_ram(address, value & 0xFF)
            else:
                print(f"Invalid GPR address for edit: 0x{address:02X}")
            
        except Exception as e:
            QMessageBox.critical(self, "Edit Error", f"Error setting GPR 0x{address:02X}: {e}")
            self.update_gui_sfr_gpr()  # Revert display


# Needed for running directly
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PicSimulatorGUI()
    window.show()
    sys.exit(app.exec_())
