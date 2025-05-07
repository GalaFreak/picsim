# coding: utf-8
import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QIcon
from picsim_frontend_pyqt import PicSimulatorGUI

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PicSimulatorGUI()
    window.setWindowIcon(QIcon('picsim.ico'))
    window.show()
    sys.exit(app.exec_())
