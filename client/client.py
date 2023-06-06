import sys
from PyQt5 import QtWidgets as qtw
# from PyQt5 import QtGui as qtg
# from PyQt5 import QtCore as qtc


class MainWindow(qtw.QMainWindow):
    def __init__(self):
        """MainWindow constructor.

        This widget will be our main window.
        We'll define all the UI components in here.
        """
        super().__init__()
        # Main UI code goes here

        # central widget will contain tabs and status box
        central_widget = qtw.QWidget()
        self.setCentralWidget(central_widget)
        central_container_layout = qtw.QVBoxLayout()
        central_widget.setLayout(central_container_layout)
        # build-out the central container
        self.tabs = qtw. QTabWidget()
        central_container_layout.addWidget(self.tabs)
        textEdit = qtw. QTextEdit()
        textEdit.setReadOnly(True)
        textEdit.setPlainText("nobody is home")
        central_container_layout.addWidget(textEdit)

        # #################################################
        # build tab 1
        level_1_layout = qtw.QHBoxLayout()
        level_2a_layout = qtw.QVBoxLayout()
        level_2b_layout = qtw.QFormLayout()

        # build tab 1, level 2a
        button1 = qtw.QRadioButton("None", checked=True)
        button2 = qtw.QRadioButton("Theta Motor")
        button3 = qtw.QRadioButton("Omega Motor")
        button4 = qtw.QRadioButton("Both")
        level_2a_layout.addWidget(button1)
        level_2a_layout.addWidget(button2)
        level_2a_layout.addWidget(button3)
        level_2a_layout.addWidget(button4)
        groupBox1 = qtw. QGroupBox(" Select One:")
        groupBox1.setLayout(level_2a_layout)

        # build tab 1, Level 2b
        lineEdit1 = qtw.QLineEdit()
        lineEdit2 = qtw.QLineEdit()
        level_2b_layout.addRow(
            qtw.QLabel('Theta Motor Target Voltage: '), lineEdit1)
        level_2b_layout.addRow(
            qtw. QLabel(' Omega Motor Target Voltage: '), lineEdit2)

        # build tab 1, Level 1
        level_1_layout.addWidget(groupBox1)
        level_1_layout.addLayout(level_2b_layout)

        # build. tab. 1
        zombo = qtw.QWidget()
        zombo.setLayout(level_1_layout)
        self.tabs.addTab(zombo, ' Speed Controls')

        # #################################################
        # build tab.2
        button21 = qtw.QRadioButton("None", checked=True)
        button22 = qtw.QRadioButton("Theta Motor")
        button23 = qtw.QRadioButton("Omega Motor")
        button24 = qtw.QRadioButton("Both")
        t211_layout = qtw.QVBoxLayout()
        t211_layout.addWidget(button21)
        t211_layout.addWidget(button22)
        t211_layout.addWidget(button23)
        t211_layout.addWidget(button24)
        groupBox2 = qtw.QGroupBox(" Select One:")
        groupBox2.setLayout(t211_layout)
        self.tabs.addTab(groupBox2, 'Balance Controls')
        # #################################################
        # End main UI code
        self.show()


if __name__ == '__main__':
    app = qtw.QApplication(sys.argv)
    # it's required to save a reference to Mainwindow.
    # if it goes out of scope, it will be destroyed.
    mw = MainWindow()
    sys.exit(app.exec())
