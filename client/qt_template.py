import sys
from PyQt5 import QtWidgets as qtw
from PyQt5 import QtGui as qtg
from PyQt5 import QtCore as qtc


class MainWindow(qtw.QMainWindow):

    def __init__(self):
        """MainWindow constructor.

        This widget will be our main window.
        We'll define all the UI components in here.
        """
        super().__init__()
        # Main UI code goes here
        self.tabs = qtw.QTabWidget()
        # self.tabs.tabCloseRequested.connect(self.tabs.removeTab)
        # self.new = qtw.QPushButton('New')
        # self.tabs.setCornerWidget(self.new)
        self.setCentralWidget(self.tabs)

        # build tab 1
        container = qtw.QWidget()
        layout = qtw.QVBoxLayout(container)
        label = qtw.QLabel('Select One:', self)
        button1 = qtw.QRadioButton("None", checked=True)
        button2 = qtw.QRadioButton("Theta")
        button3 = qtw.QRadioButton("Omega")
        button4 = qtw.QRadioButton("Both")
        layout.addWidget(label)
        layout.addWidget(button1)
        layout.addWidget(button2)
        layout.addWidget(button3)
        layout.addWidget(button4)
        self.tabs.addTab(container, 'Motor Charging')

        # End main UI code
        self.show()


if __name__ == '__main__':
    app = qtw.QApplication(sys.argv)
    # it's required to save a reference to MainWindow.
    # if it goes out of scope, it will be destroyed.
    mw = MainWindow()
    sys.exit(app.exec())
